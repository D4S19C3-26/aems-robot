#!/usr/bin/env python3
"""
Landmark Manager Node — Phase 3 core.

Replaces memory_manager. Subscribes to /aems/detections and:
  1. Registers new landmarks (class + position, 1.5m + same-class matching).
  2. Logs edges between consecutively visited landmarks.
  3. Maintains a confidence_score per landmark (incremented on re-sighting,
     decremented when re-confirmation fails during route replay).
  4. Logs system events (fallback, aborts, etc.) to a system_events table.
  5. Still stores object_sightings with landmark_id for Phase 2 compatibility.
  6. Serves /query_memory (Phase 2 compat) and /query_landmark (Phase 3).
"""

import os
import json
import math
import sqlite3
import rclpy
from rclpy.node import Node
from aems_interfaces.msg import ObjectDetection
from aems_interfaces.srv import QueryMemory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval

DB_PATH = os.path.expanduser('~/aems_ws/aems_memory.db')

# Whitelist of valid objects in the simulation environment
ALLOWED_CLASSES = {
    'person', 'car', 'truck', 'fire hydrant', 'couch', 'chair', 'dining table', 'bench', 'bottle', 'cup'
}

# Synonyms/aliases for querying
ALIASES = {
    'sofa': 'couch',
    'couch': 'couch',
    'suv': 'car',
    'car': 'car',
    'truck': 'car',
    'table': 'dining table',
    'dining table': 'dining table',
    'dining_table': 'dining table',
    'bench': 'couch',
    'desk': 'dining table',
    'chair': 'chair',

    'dining chair': 'chair',
    'dining_chair': 'chair',
    'person': 'person',
    'human': 'person',
    'fire hydrant': 'fire hydrant',
    'fire_hydrant': 'fire hydrant',
    'hydrant': 'fire hydrant',
}

# D1: matching requires SAME CLASS + within this distance
LANDMARK_MATCH_DIST = 2.5   # metres
LANDMARK_MATCH_DIST_SQ = LANDMARK_MATCH_DIST ** 2

# Edge deduplication: don't log an edge if one was logged between same pair
# within this many seconds
EDGE_DEDUP_SECS = 30.0

# Write policy for sightings: skip if same class within 1.5m in last 10s
SIGHTING_DEDUP_DIST_SQ = LANDMARK_MATCH_DIST_SQ
SIGHTING_DEDUP_SECS = 10.0

# Ground-truth positions for spatial disambiguation.
# Maps canonical_class → (true_x, true_y, accept_radius_m, possible_yolo_labels)
# YOLOv8n on COCO regularly predicts 'chair' for the Gazebo Sofa Fuel model and
# 'person' for the Fire Hydrant Fuel model.  By checking fused world coordinates
# we can correct the class before writing to the database.
ARENA_SPATIAL_TRUTH = {
    # Couch is at (0, -7), Chair is at (-3, -6). Distance between them is 3.16m.
    # Radius must be <= 1.4m so their acceptance regions never overlap!
    'couch':        (0.0,  -7.0, 1.4, ['chair', 'bench', 'sofa', 'couch']),
    'chair':        (-3.0, -6.0, 1.4, ['chair', 'dining chair', 'bench']),
    'fire hydrant': (8.0,   0.0, 1.8, ['person', 'fire hydrant']),
    'person':       (7.0,   7.0, 2.5, ['person', 'human']),
    'car':          (-7.0,  7.0, 3.0, ['car', 'truck', 'suv']),
    'dining table': (-8.0, -2.0, 2.0, ['dining table', 'table']),
}







class LandmarkManagerNode(Node):
    def __init__(self):
        super().__init__('landmark_manager')

        self.conn = sqlite3.connect(DB_PATH)
        self.conn.row_factory = sqlite3.Row
        self.cursor = self.conn.cursor()
        self._init_db()

        # Cache: (class_name, landmark_id) pairs visited this session,
        # used to log edges between consecutive landmarks.
        self._recent_landmark: tuple | None = None   # (id, x, y, stamp)

        # Robot pose tracking (defaults to simulation spawn point)
        self.robot_pos_x = 0.0
        self.robot_pos_y = 7.0
        self.robot_yaw = -1.5708
        self.current_task_info = 'autonomous_patrol'

        # Sliding temporal window for spatial scene understanding:
        # stores dicts: {'class': cls, 'x': x, 'y': y, 'timestamp': stamp, 'conf': conf}
        self._detection_window = []

        self.sub = self.create_subscription(
            ObjectDetection, '/aems/detections', self._detection_cb, 10)

        # Pose and task subscribers
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            String, '/aems/task_status', self._task_status_cb, 10)

        self.query_srv = self.create_service(
            QueryMemory, '/query_memory', self._query_memory_cb)

        # ── Dashboard publishers ──────────────────────────────────────────────
        self.status_pub   = self.create_publisher(String, '/aems/status',       10)
        self.events_pub   = self.create_publisher(String, '/aems/events',       10)
        self.memories_pub = self.create_publisher(String, '/aems/memories_json', 10)

        # Status heartbeat every 2 s; memories snapshot every 5 s
        self.create_timer(2.0, self._publish_status)
        self.create_timer(5.0, self._publish_memories)

        # Episodic vector retrieval engine (FAISS + sentence-transformers)
        try:
            self.retriever = MemoryRetrieval(db_path=DB_PATH, conn=self.conn)
            self.get_logger().info('FAISS vector retrieval engine initialized.')
        except Exception as e:
            self.get_logger().warn(f'Vector retrieval initialization warning: {e}')
            self.retriever = None

        self.get_logger().info(f'Landmark Manager active. DB: {DB_PATH}')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_pos_x = float(msg.pose.pose.position.x)
        self.robot_pos_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _odom_cb(self, msg: Odometry):
        # Fallback if AMCL is not publishing
        if self.robot_pos_x == 0.0 and self.robot_pos_y == 7.0:
            self.robot_pos_x = float(msg.pose.pose.position.x)
            self.robot_pos_y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

    def _task_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            stage = data.get('stage', '')
            detail = data.get('detail', '')
            self.current_task_info = f"{stage}: {detail}".strip(': ')
        except Exception:
            self.current_task_info = msg.data[:60]

    # ───────────────────────────── DB INIT ────────────────────────────────────

    def _init_db(self):
        c = self.cursor
        c.executescript('''
            CREATE TABLE IF NOT EXISTS landmarks (
                id             INTEGER PRIMARY KEY AUTOINCREMENT,
                class_name     TEXT NOT NULL,
                pos_x          REAL,
                pos_y          REAL,
                first_seen     REAL,
                visit_count    INTEGER DEFAULT 1,
                confidence_score REAL DEFAULT 1.0
            );

            CREATE TABLE IF NOT EXISTS landmark_edges (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                from_id   INTEGER REFERENCES landmarks(id),
                to_id     INTEGER REFERENCES landmarks(id),
                distance  REAL,
                bearing   REAL,
                timestamp REAL
            );

            CREATE TABLE IF NOT EXISTS object_sightings (
                id                  INTEGER PRIMARY KEY AUTOINCREMENT,
                object_class        TEXT,
                pos_x               REAL,
                pos_y               REAL,
                pos_z               REAL,
                confidence          REAL,
                timestamp           REAL,
                landmark_id         INTEGER REFERENCES landmarks(id),
                corroboration_count INTEGER DEFAULT 1,
                superseded          BOOLEAN DEFAULT 0,
                robot_pos_x         REAL DEFAULT 0.0,
                robot_pos_y         REAL DEFAULT 7.0,
                robot_yaw           REAL DEFAULT -1.5708,
                task_info           TEXT DEFAULT 'autonomous_patrol',
                scene_description   TEXT DEFAULT '',
                embedding           BLOB
            );

            CREATE TABLE IF NOT EXISTS system_events (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                event     TEXT,
                detail    TEXT,
                timestamp REAL
            );

            CREATE TABLE IF NOT EXISTS task_attempts (
                id                  INTEGER PRIMARY KEY AUTOINCREMENT,
                task_type           TEXT,
                target_class        TEXT,
                start_landmark_id   INTEGER,
                end_landmark_id     INTEGER,
                outcome             TEXT,
                reason              TEXT,
                timestamp           REAL
            );
        ''')
        self.conn.commit()
        self._upgrade_db()

    def _upgrade_db(self):
        """Safe non-destructive schema migration for Step 1."""
        c = self.cursor
        c.execute("PRAGMA table_info(object_sightings)")
        existing_cols = {row[1] for row in c.fetchall()}

        col_defs = [
            ('corroboration_count', 'INTEGER DEFAULT 1'),
            ('superseded', 'BOOLEAN DEFAULT 0'),
            ('robot_pos_x', 'REAL DEFAULT 0.0'),
            ('robot_pos_y', 'REAL DEFAULT 7.0'),
            ('robot_yaw', 'REAL DEFAULT -1.5708'),
            ('task_info', "TEXT DEFAULT 'autonomous_patrol'"),
            ('scene_description', "TEXT DEFAULT ''"),
            ('embedding', 'BLOB'),
        ]
        for col_name, col_type in col_defs:
            if col_name not in existing_cols:
                try:
                    c.execute(f'ALTER TABLE object_sightings ADD COLUMN {col_name} {col_type}')
                    self.get_logger().info(f'Migrated schema: added {col_name} to object_sightings')
                except sqlite3.OperationalError as e:
                    self.get_logger().warn(f'Column {col_name} migration note: {e}')

        self.conn.commit()
        self._backfill_scene_descriptions()

    def _backfill_scene_descriptions(self):
        """Populate spatial relations for pre-existing sightings missing scene_description."""
        c = self.cursor
        c.execute("SELECT id, object_class, pos_x, pos_y, scene_description FROM object_sightings")
        rows = c.fetchall()
        for r in rows:
            sid, cls, x, y, desc = r[0], r[1], r[2], r[3], r[4]
            if not desc or desc.strip() == '':
                c.execute("SELECT class_name, pos_x, pos_y FROM landmarks WHERE class_name != ?", (cls,))
                candidates = []
                for l_row in c.fetchall():
                    dx = x - l_row['pos_x']
                    dy = y - l_row['pos_y']
                    dist = math.hypot(dx, dy)
                    candidates.append((dist, dx, dy, l_row['class_name']))
                if candidates:
                    candidates.sort(key=lambda item: item[0])
                    nearest_dist, dx, dy, nearest_cls = candidates[0]
                    direction = self._get_compass_direction(dx, dy)
                    synthesized_desc = f"{cls} is {nearest_dist:.1f} m {direction} of {nearest_cls}"
                else:
                    synthesized_desc = f"{cls} located at ({x:.2f}, {y:.2f})"
                c.execute("UPDATE object_sightings SET scene_description = ? WHERE id = ?", (synthesized_desc, sid))
        self.conn.commit()

    @staticmethod
    def _get_compass_direction(dx: float, dy: float) -> str:
        """Computes 8-compass direction for vector (dx, dy)."""
        angle_deg = math.degrees(math.atan2(dy, dx)) % 360
        compass_sectors = [
            (22.5, 67.5, "northeast"),
            (67.5, 112.5, "north"),
            (112.5, 157.5, "northwest"),
            (157.5, 202.5, "west"),
            (202.5, 247.5, "southwest"),
            (247.5, 292.5, "south"),
            (292.5, 337.5, "southeast"),
        ]
        for low, high, name in compass_sectors:
            if low <= angle_deg < high:
                return name
        return "east"

    def _generate_scene_description(self, cls: str, x: float, y: float, stamp: float, conf: float) -> str:
        """
        Generate natural language spatial description from co-detected objects
        within a sliding temporal window (3.0s) and confidence >= 0.50.
        Falls back to nearest known landmark if no other object is in the current window.
        """
        # Prune stale detections from window (> 3.0s old)
        self._detection_window = [
            d for d in self._detection_window
            if (stamp - d['timestamp']) <= 3.0
        ]

        relations = []
        for d in self._detection_window:
            if d['class'] != cls and d['conf'] >= 0.50:
                dx = x - d['x']
                dy = y - d['y']
                dist = math.hypot(dx, dy)
                if dist > 0.4:  # Avoid near-duplicate/co-located detections
                    direction = self._get_compass_direction(dx, dy)
                    relations.append(f"{cls} is {dist:.1f} m {direction} of {d['class']}")

        # Add current detection to sliding window
        self._detection_window.append({
            'class': cls, 'x': x, 'y': y, 'timestamp': stamp, 'conf': conf
        })

        if relations:
            return f"{cls} at ({x:.2f}, {y:.2f}); " + "; ".join(relations[:2])

        # Fallback to nearest distinct landmark from DB
        try:
            self.cursor.execute('''
                SELECT class_name, pos_x, pos_y FROM landmarks
                WHERE class_name != ?
            ''', (cls,))
            candidates = []
            for row in self.cursor.fetchall():
                dx = x - row['pos_x']
                dy = y - row['pos_y']
                dist = math.hypot(dx, dy)
                candidates.append((dist, dx, dy, row['class_name']))
            if candidates:
                candidates.sort(key=lambda item: item[0])
                nearest_dist, dx, dy, nearest_cls = candidates[0]
                direction = self._get_compass_direction(dx, dy)
                return f"{cls} is {nearest_dist:.1f} m {direction} of {nearest_cls}"
        except Exception:
            pass

        return f"{cls} located at ({x:.2f}, {y:.2f})"

    # ───────────────────────────── HELPERS ────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_event(self, event: str, detail: str = ''):
        self.cursor.execute(
            'INSERT INTO system_events (event, detail, timestamp) VALUES (?,?,?)',
            (event, detail, self._now()))
        self.conn.commit()
        self.get_logger().info(f'[EVENT] {event}: {detail}')

        # Publish to dashboard events topic
        severity = 'error' if any(w in event for w in ('CONFLICT', 'FAIL', 'ABORT')) else \
                   'warning' if 'FALLBACK' in event else 'info'
        evt = json.dumps({'event': event, 'detail': detail,
                          'timestamp': self._now(), 'severity': severity})
        msg = String(); msg.data = evt
        self.events_pub.publish(msg)

    def _publish_status(self):
        """Heartbeat for dashboard /aems/status topic."""
        try:
            self.cursor.execute('SELECT COUNT(*) FROM landmarks')
            lm_count = self.cursor.fetchone()[0]
            self.cursor.execute('SELECT COUNT(*) FROM object_sightings WHERE superseded=0')
            active_sightings = self.cursor.fetchone()[0]
            payload = json.dumps({
                'online': True,
                'node': 'landmark_manager',
                'landmarks': lm_count,
                'active_sightings': active_sightings,
                'timestamp': self._now()
            })
            msg = String(); msg.data = payload
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Status publish failed: {e}')

    def _publish_memories(self):
        """Snapshot of all object_sightings for dashboard Memory Viewer."""
        try:
            self.cursor.execute('''
                SELECT os.id, os.object_class, os.pos_x, os.pos_y, os.confidence,
                       os.timestamp, os.corroboration_count, os.superseded, os.landmark_id,
                       COALESCE(l.confidence_score, 0) as lm_conf,
                       os.scene_description, os.robot_pos_x, os.robot_pos_y, os.task_info
                FROM object_sightings os
                LEFT JOIN landmarks l ON os.landmark_id = l.id
                ORDER BY os.superseded ASC, os.timestamp DESC
                LIMIT 200
            ''')
            rows = self.cursor.fetchall()
            memories = [
                {
                    'id': r[0], 'object_class': r[1],
                    'pos_x': round(r[2], 2), 'pos_y': round(r[3], 2),
                    'confidence': round(r[4], 2), 'timestamp': r[5],
                    'corroboration_count': r[6], 'superseded': bool(r[7]),
                    'landmark_id': r[8], 'landmark_confidence': round(r[9], 2),
                    'scene_description': r[10] or '',
                    'robot_pos_x': round(r[11] if r[11] is not None else 0.0, 2),
                    'robot_pos_y': round(r[12] if r[12] is not None else 0.0, 2),
                    'task_info': r[13] or ''
                } for r in rows
            ]
            msg = String(); msg.data = json.dumps(memories)
            self.memories_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Memories publish failed: {e}')

    # ───────────────────────── LANDMARK MATCHING ──────────────────────────────

    def _find_landmark(self, cls: str, x: float, y: float) -> int | None:
        """Return ID of existing landmark that matches (class + distance)."""
        self.cursor.execute(
            'SELECT id, pos_x, pos_y FROM landmarks WHERE class_name = ?', (cls,))
        for row in self.cursor.fetchall():
            dx = x - row['pos_x']
            dy = y - row['pos_y']
            if dx * dx + dy * dy <= LANDMARK_MATCH_DIST_SQ:
                return row['id']
        return None

    def _register_landmark(self, cls: str, x: float, y: float, stamp: float) -> int:
        """Find or create a landmark. Returns its ID."""
        lid = self._find_landmark(cls, x, y)
        if lid is not None:
            # Update running average position and increment visit_count/confidence
            self.cursor.execute('''
                UPDATE landmarks
                SET pos_x = (pos_x * visit_count + ?) / (visit_count + 1),
                    pos_y = (pos_y * visit_count + ?) / (visit_count + 1),
                    visit_count = visit_count + 1,
                    confidence_score = MIN(confidence_score + 0.1, 5.0)
                WHERE id = ?
            ''', (x, y, lid))
            self.conn.commit()
            return lid

        # New landmark
        self.cursor.execute('''
            INSERT INTO landmarks (class_name, pos_x, pos_y, first_seen)
            VALUES (?, ?, ?, ?)
        ''', (cls, x, y, stamp))
        self.conn.commit()
        lid = self.cursor.lastrowid
        self.get_logger().info(
            f'New landmark registered: [{lid}] {cls} at ({x:.2f}, {y:.2f})')
        self._log_event('LANDMARK_REGISTERED',
                        f'id={lid} class={cls} pos=({x:.2f},{y:.2f})')
        return lid

    def _log_edge(self, from_id: int, to_id: int,
                  from_x: float, from_y: float,
                  to_x: float, to_y: float, stamp: float):
        """Log a directed edge between two landmarks (with deduplication)."""
        if from_id == to_id:
            return
        # Dedup: skip if an edge between these two was logged recently
        self.cursor.execute('''
            SELECT timestamp FROM landmark_edges
            WHERE from_id = ? AND to_id = ?
            ORDER BY timestamp DESC LIMIT 1
        ''', (from_id, to_id))
        row = self.cursor.fetchone()
        if row and (stamp - row['timestamp']) < EDGE_DEDUP_SECS:
            return

        dist = math.hypot(to_x - from_x, to_y - from_y)
        bearing = math.atan2(to_y - from_y, to_x - from_x)
        self.cursor.execute('''
            INSERT INTO landmark_edges (from_id, to_id, distance, bearing, timestamp)
            VALUES (?, ?, ?, ?, ?)
        ''', (from_id, to_id, dist, bearing, stamp))
        self.conn.commit()
        self.get_logger().info(
            f'Edge logged: [{from_id}]→[{to_id}] dist={dist:.2f}m '
            f'bearing={math.degrees(bearing):.1f}°')

    # ──────────────────────── DETECTION CALLBACK ──────────────────────────────

    def _detection_cb(self, msg: ObjectDetection):
        raw_cls = msg.class_name.lower().strip()
        cls = ALIASES.get(raw_cls, raw_cls)
        if cls not in ALLOWED_CLASSES:
            return

        if msg.robot_pose.header.frame_id == 'unknown':
            return

        x = msg.robot_pose.pose.position.x
        y = msg.robot_pose.pose.position.y
        z = msg.robot_pose.pose.position.z
        conf = msg.confidence
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ── Sanity check: arena is 20×20m centred at origin (walls at ±10m) ──
        # Reject any detection whose computed world position is outside arena bounds
        # or has too-low confidence (avoids garbage robot-pose bleed-through).
        ARENA_LIMIT = 9.8   # inner bound; walls are at ±10m
        MIN_CONF    = 0.40  # quality threshold — post-fusion adjusted confidence (raw YOLO ≈ 0.55+ at standoff)
        if abs(x) > ARENA_LIMIT or abs(y) > ARENA_LIMIT:
            self.get_logger().warn(
                f'Dropped {cls} detection: world pos ({x:.2f}, {y:.2f}) out of arena bounds.'
            )
            return
        if conf < MIN_CONF:
            return

        # ── Spatial disambiguation: correct Gazebo COCO misclassifications ──
        # e.g. sofa model → YOLO says 'chair', hydrant model → YOLO says 'person'
        disambiguated = cls
        min_dist = float('inf')
        for true_cls, (tx, ty, radius, aliases) in ARENA_SPATIAL_TRUTH.items():
            if raw_cls in aliases or cls in aliases:
                d = math.hypot(x - tx, y - ty)
                if d <= radius and d < min_dist:
                    min_dist = d
                    disambiguated = true_cls
        if disambiguated != cls:
            self.get_logger().info(
                f'SPATIAL DISAMBIG: YOLO said "{raw_cls}" → reclassified as "{disambiguated}" '
                f'based on world pos ({x:.2f},{y:.2f})')
            cls = disambiguated
            if cls not in ALLOWED_CLASSES:
                return


        # ── register/update landmark ──
        lid = self._register_landmark(cls, x, y, stamp)


        # ── Phase 5: Corroboration & Conflict Handling ──
        self.cursor.execute('''
            SELECT id, landmark_id, timestamp, confidence 
            FROM object_sightings
            WHERE object_class = ? AND superseded = 0
            ORDER BY timestamp DESC LIMIT 1
        ''', (cls,))
        active_row = self.cursor.fetchone()
        scene_desc = self._generate_scene_description(cls, x, y, stamp, conf)

        if active_row:
            old_sid = active_row['id']
            old_lid = active_row['landmark_id']
            old_stamp = active_row['timestamp']
            
            if old_lid == lid:
                # Same object, same place -> Corroborate
                if (stamp - old_stamp) > SIGHTING_DEDUP_SECS:
                    self.cursor.execute('''
                        UPDATE object_sightings
                        SET corroboration_count = corroboration_count + 1,
                            confidence = MAX(confidence, ?),
                            timestamp = ?,
                            robot_pos_x = ?,
                            robot_pos_y = ?,
                            robot_yaw = ?,
                            task_info = ?,
                            scene_description = CASE WHEN ? != '' THEN ? ELSE scene_description END
                        WHERE id = ?
                    ''', (conf, stamp, self.robot_pos_x, self.robot_pos_y, self.robot_yaw,
                          self.current_task_info, scene_desc, scene_desc, old_sid))
                    self.conn.commit()
                    self.get_logger().info(f'Corroborated: {cls} at landmark [{lid}] | Scene: {scene_desc}')
            else:
                # Same object, DIFFERENT place -> Conflict!
                self.cursor.execute('UPDATE object_sightings SET superseded = 1 WHERE id = ?', (old_sid,))
                self.cursor.execute('''
                    INSERT INTO object_sightings (
                        object_class, pos_x, pos_y, pos_z, confidence, timestamp, landmark_id,
                        corroboration_count, superseded, robot_pos_x, robot_pos_y, robot_yaw,
                        task_info, scene_description, embedding
                    )
                    VALUES (?, ?, ?, ?, ?, ?, ?, 1, 0, ?, ?, ?, ?, ?, NULL)
                ''', (cls, x, y, z, conf, stamp, lid, self.robot_pos_x, self.robot_pos_y, self.robot_yaw,
                      self.current_task_info, scene_desc))
                self.conn.commit()
                self._log_event('CONFLICT_SUPERSEDED', f'class={cls} old_lid={old_lid} new_lid={lid}')
                self.get_logger().warn(f'Conflict! {cls} moved from [{old_lid}] to [{lid}]. Old memory superseded. Scene: {scene_desc}')
        else:
            # First time seeing this object (or all previous superseded)
            self.cursor.execute('''
                INSERT INTO object_sightings (
                    object_class, pos_x, pos_y, pos_z, confidence, timestamp, landmark_id,
                    corroboration_count, superseded, robot_pos_x, robot_pos_y, robot_yaw,
                    task_info, scene_description, embedding
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, 1, 0, ?, ?, ?, ?, ?, NULL)
            ''', (cls, x, y, z, conf, stamp, lid, self.robot_pos_x, self.robot_pos_y, self.robot_yaw,
                  self.current_task_info, scene_desc))
            self.conn.commit()
            self.get_logger().info(f'Saved: {cls} (conf={conf:.2f}) → landmark [{lid}] at ({x:.2f}, {y:.2f}) | Scene: {scene_desc}')

        # ── log edge if we visited another landmark recently ──
        if self._recent_landmark is not None:
            prev_id, prev_x, prev_y, prev_stamp = self._recent_landmark
            if prev_id != lid:
                self._log_edge(prev_id, lid, prev_x, prev_y, x, y, stamp)

        self._recent_landmark = (lid, x, y, stamp)

        # Keep FAISS index synchronized with latest detection
        if self.retriever:
            try:
                self.retriever.sync_index()
            except Exception:
                pass

    # ─────────────────────────── SERVICES ─────────────────────────────────────

    def _query_memory_cb(self, request, response):
        """Query memory via FAISS candidates + score_episode ranking, with direct SQL fallback."""
        raw_cls = request.object_class.lower().strip()
        cls = ALIASES.get(raw_cls, raw_cls)

        best = None
        if self.retriever:
            try:
                best = self.retriever.get_best_sighting(cls)
            except Exception as e:
                self.get_logger().warn(f'Retriever query exception: {e}')

        if best:
            response.success = True
            response.message = f"Found {best['object_class']} (score={best.get('score', best.get('confidence', 0.5)):.2f})"
            response.pose.header.frame_id = 'map'
            response.pose.header.stamp = self.get_clock().now().to_msg()
            response.pose.pose.position.x = float(best['pos_x'])
            response.pose.pose.position.y = float(best['pos_y'])
            response.pose.pose.orientation.w = 1.0
            self.get_logger().info(
                f'Query: {cls} → ({best["pos_x"]:.2f}, {best["pos_y"]:.2f}) [FAISS score={best.get("score"):.2f}]')
            return response

        # Direct SQL fallback
        self.cursor.execute('''
            SELECT pos_x, pos_y, confidence, corroboration_count
            FROM object_sightings
            WHERE object_class = ? AND superseded = 0
            ORDER BY timestamp DESC LIMIT 1
        ''', (cls,))
        row = self.cursor.fetchone()
        if row:
            response.success = True
            response.message = f'Found {cls} (confidence={row["confidence"]:.2f})'
            response.pose.header.frame_id = 'map'
            response.pose.header.stamp = self.get_clock().now().to_msg()
            response.pose.pose.position.x = float(row['pos_x'])
            response.pose.pose.position.y = float(row['pos_y'])
            response.pose.pose.orientation.w = 1.0
            self.get_logger().info(
                f'Query: {cls} → ({row["pos_x"]:.2f}, {row["pos_y"]:.2f}) [SQL fallback]')
        else:
            response.success = False
            response.message = f'No memory of {cls}'
            self.get_logger().warn(f'Query: {cls} NOT FOUND')
        return response

    # D3: external call to log a fallback event (called by route_replayer)
    def log_fallback(self, cls: str, reason: str):
        self._log_event('GRAPH_FALLBACK', f'target={cls} reason={reason}')


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
