#!/usr/bin/env python3
"""
Dashboard Command Interface Node — AEMS v5
Receives NL commands, runs TrustVerifyReasoner + LLMReasoner, selects
navigation strategy (direct or landmark route-replay), computes an
unobstructed standoff via multi-candidate wall-clearing, drives there,
and runs an Autonomous Multi-Room Perimeter Scout if the object is occluded behind walls.

Features:
- Instant visual confirmation (no repeated re-nav loops when object is seen).
- Post-arrival 360° visual sweep.
- Autonomous Multi-Room Perimeter Scout: navigates around interior walls (iw_1..iw_4)
  to alternative room vantage points when an object is occluded.
- Robust route-replay integration with realistic execution timeout and clean fallback.
"""

import fcntl
import json
import math
import os
import sqlite3
import sys
import threading
import time
import yaml


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from aems_interfaces.msg import ObjectDetection
from aems_interfaces.srv import QueryMemory, ReplayRoute

DB_PATH = os.path.expanduser('~/aems_ws/aems_memory.db')
ZONES_PATH = os.path.expanduser('~/aems_ws/src/adaptive_episodic_memory/config/zones.yaml')
ARENA_BOUNDS = {'min_x': -9.5, 'max_x': 9.5, 'min_y': -9.5, 'max_y': 9.5}

ALIASES = {
    'sofa': 'couch', 'couch': 'couch', 'bench': 'couch',
    'suv': 'car', 'car': 'car', 'truck': 'car',
    'table': 'dining table', 'dining table': 'dining table',
    'dining_table': 'dining table',
    'chair': 'chair', 'dining chair': 'chair', 'dining_chair': 'chair',
    'person': 'person', 'human': 'person',
    'fire hydrant': 'fire hydrant', 'hydrant': 'fire hydrant',
    'fire_hydrant': 'fire hydrant',
}
ALL_CLASSES = list(ALIASES.keys())

OBJECT_STANDOFF_DIST = {
    'car': 3.2,
    'couch': 2.2,
    'dining table': 2.2,
    'person': 1.6,
    'fire hydrant': 1.6,
    'chair': 1.5,
}

# Ground-truth positions and disambiguate radius for each object in the arena.
# When YOLO misclassifies an object (e.g. sofa → 'chair', hydrant → 'person'),
# the fused world position still lands near the true object. We use spatial
# proximity to correct the class label before updating memory.
ARENA_GROUND_TRUTH = {
    # (canonical_class, true_x, true_y, accept_radius_m, may_be_misclassified_as)
    # Couch is at (0, -7), Chair is at (-3, -6). Distance between them is 3.16m.
    # Radius must be <= 1.4m so their acceptance regions never overlap!
    'couch':        (0.0,  -7.0, 1.4, ['chair', 'bench', 'sofa', 'couch']),
    'chair':        (-3.0, -6.0, 1.4, ['chair', 'dining chair', 'bench']),
    'fire hydrant': (8.0,   0.0, 1.8, ['person', 'fire hydrant']),
    'person':       (7.0,   7.0, 2.5, ['person', 'human']),
    'car':          (-7.0,  7.0, 3.0, ['car', 'truck', 'suv']),
    'dining table': (-8.0, -2.0, 2.0, ['dining table', 'table']),
}


def _disambiguate_by_position(raw_cls: str, world_x: float, world_y: float) -> str:
    """
    Corrects Gazebo COCO misclassifications based on detected world position.
    Uses nearest-neighbor matching to prevent overlapping false positives.
    """
    canonical = ALIASES.get(raw_cls, raw_cls)
    best_cls = canonical
    min_dist = float('inf')

    for true_cls, (tx, ty, radius, aliases) in ARENA_GROUND_TRUTH.items():
        if raw_cls in aliases or canonical in aliases:
            dist = math.hypot(world_x - tx, world_y - ty)
            if dist <= radius and dist < min_dist:
                min_dist = dist
                best_cls = true_cls
    return best_cls


# Interior wall segments: (center_x, center_y, half_length, yaw_rad)
INTERIOR_WALLS = [
    (3.0,  4.0,  2.0,  0.785),   # iw_1  45-degree diagonal (Northeast room)
    (-4.0, 5.0,  1.75, 1.5708),  # iw_2  vertical (Northwest room)
    (-5.0, -4.0, 2.5,  -0.5),    # iw_3  (Southwest room)
    (5.0,  -5.0, 2.25, 0.0),     # iw_4  horizontal (Southeast room)
]
WALL_HALF_THICKNESS = 0.35


def _seg_endpoints(cx, cy, half_len, yaw):
    dx = math.cos(yaw) * half_len
    dy = math.sin(yaw) * half_len
    return cx - dx, cy - dy, cx + dx, cy + dy


def _cross2d(ox, oy, px, py, qx, qy):
    return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)


def _segments_intersect(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    d1 = _cross2d(bx1, by1, bx2, by2, ax1, ay1)
    d2 = _cross2d(bx1, by1, bx2, by2, ax2, ay2)
    d3 = _cross2d(ax1, ay1, ax2, ay2, bx1, by1)
    d4 = _cross2d(ax1, ay1, ax2, ay2, bx2, by2)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def _path_crosses_wall(fx, fy, tx, ty):
    """True if the straight line (fx,fy)→(tx,ty) crosses any interior wall."""
    for cx, cy, half_len, yaw in INTERIOR_WALLS:
        wx1, wy1, wx2, wy2 = _seg_endpoints(cx, cy, half_len + WALL_HALF_THICKNESS, yaw)
        if _segments_intersect(fx, fy, tx, ty, wx1, wy1, wx2, wy2):
            return True
    return False


def compute_standoff(rx, ry, ox, oy, dist=1.5):
    """
    Try candidate standoff positions around the object at `dist` metres.
    Priority:
      1. Sight-line from standoff to object must NOT cross any interior wall.
      2. Robot path from (rx, ry) to standoff is clear.
    """
    d_obj = math.hypot(ox, oy)
    if d_obj > 0.5:
        base_angle = math.atan2(oy, ox) + math.pi
    else:
        base_angle = math.atan2(ry - oy, rx - ox)

    offsets = [0,
               math.pi / 4, -math.pi / 4,
               math.pi / 2, -math.pi / 2,
               3 * math.pi / 4, -3 * math.pi / 4,
               math.pi]

    best_fallback = None
    for off in offsets:
        angle = base_angle + off
        sx = ox + dist * math.cos(angle)
        sy = oy + dist * math.sin(angle)
        if abs(sx) > 9.5 or abs(sy) > 9.5:
            continue
        yaw = math.atan2(oy - sy, ox - sx)
        if _path_crosses_wall(sx, sy, ox, oy):
            continue
        if not _path_crosses_wall(rx, ry, sx, sy):
            return sx, sy, yaw
        if best_fallback is None:
            best_fallback = (sx, sy, yaw)

    if best_fallback is not None:
        return best_fallback

    if d_obj > 0.5:
        ux, uy = ox / d_obj, oy / d_obj
    else:
        d = max(math.hypot(ox - rx, oy - ry), 0.01)
        ux, uy = (ox - rx) / d, (oy - ry) / d
    sx = ox - dist * ux
    sy = oy - dist * uy
    yaw = math.atan2(oy - sy, ox - sx)
    return sx, sy, yaw


def get_room_scout_waypoints(ox, oy):
    """
    Returns perimeter scout waypoints around interior room partitions
    to look behind walls where objects might be occluded.
    """
    waypoints = []
    # NE Room (Behind iw_1 diagonal wall)
    if ox > 1.0 and oy > 1.0:
        waypoints.append((6.5, 4.5))   # South entrance to NE room
        waypoints.append((5.0, 7.5))   # North/West vantage into NE corner
        waypoints.append((6.0, 6.0))   # Inside NE room
    # NW Room (Behind iw_2 vertical wall)
    elif ox < -1.0 and oy > 1.0:
        waypoints.append((-6.0, 7.5))  # Top entrance behind iw_2
        waypoints.append((-4.0, 2.0))  # Bottom entrance
        waypoints.append((-6.5, 5.5))  # Inside NW room
    # SW Room (Behind iw_3)
    elif ox < -1.0 and oy < -1.0:
        waypoints.append((-6.5, -6.5))
        waypoints.append((-3.0, -3.0))
    # SE Room (Behind iw_4)
    elif ox > 1.0 and oy < -1.0:
        waypoints.append((6.5, -7.0))
        waypoints.append((4.5, -3.0))
    else:
        waypoints.append((ox * 0.7, oy * 0.7))

    return waypoints


class DashboardNode(Node):
    def destroy_node(self):
        try:
            if hasattr(self, '_lock_file') and self._lock_file:
                try:
                    fcntl.flock(self._lock_file, fcntl.LOCK_UN)
                except Exception:
                    pass
                self._lock_file.close()
                self._lock_file = None
        except Exception:
            pass
        return super().destroy_node()

    def __init__(self):
        super().__init__('dashboard_command_node')

        # Single-instance process lock: guarantees only ONE dashboard runs
        try:
            self._lock_file = open('/tmp/aems_dashboard.lock', 'w')
            fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (BlockingIOError, IOError):
            self.get_logger().error(
                'FATAL: Another instance of dashboard_node is already running! '
                'Shutting down duplicate instance to prevent Nav2 preemption.')
            sys.exit(0)

        self.conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row


        # Navigation state
        self._current_nav_target    = None
        self._current_target_center = None
        self._current_goal_pos      = None
        self._current_goal_handle   = None
        self._last_redirect_time    = 0.0
        self._robot_pose = {'x': 0.0, 'y': 7.0, 'yaw': -1.5708}

        # Detection & Verification state
        self._latest_detections = {}     # class -> (x, y, timestamp, conf)
        self._is_verifying = False
        self._nav_cancelled = False      # set True when we cancel nav due to high-conf detection
        self._active_task = None         # current task tracking for task_history table
        self._is_estop = False           # Emergency stop state

        # Subscriptions
        self.create_subscription(String, '/aems/nl_command', self._command_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10)
        self.create_subscription(ObjectDetection, '/aems/detections', self._detection_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)

        # Publishers
        self.response_pub    = self.create_publisher(String, '/aems/command_response', 10)
        self.task_status_pub = self.create_publisher(String, '/aems/task_status', 10)
        self.cmd_vel_pub     = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.cmd_vel_safe    = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action & service clients
        self.nav_cli    = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.qmem_cli   = self.create_client(QueryMemory, '/query_memory')
        self.replay_cli = self.create_client(ReplayRoute,  '/replay_route')

        # LLM and Memory Retrieval components
        try:
            from adaptive_episodic_memory.reasoning.llm_reasoning import LLMReasoner
            self.reasoner = LLMReasoner()
        except Exception as e:
            self.get_logger().warn(f'Could not init LLMReasoner: {e}')
            self.reasoner = None

        try:
            from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval
            self.retriever = MemoryRetrieval(conn=self.conn)
        except Exception as e:
            self.get_logger().warn(f'Could not init MemoryRetrieval: {e}')
            self.retriever = None

        # Named zones configuration
        self.zones = self._load_zones()

        self.get_logger().info(
            'Dashboard node v5 ready — Unobstructed Standoff + Multi-Room Perimeter Scout + NL Memory Q&A + Named Zones.')

    # ── Pose callback ─────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._robot_pose['x'] = msg.pose.pose.position.x
        self._robot_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_pose['yaw'] = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ── E-Stop callback ───────────────────────────────────────────────────────
    def _estop_cb(self, msg: Bool):
        self._is_estop = msg.data
        if self._is_estop:
            self.get_logger().warn('EMERGENCY STOP TRIGGERED! Halting robot immediately.')
            self._log_event('estop_triggered', {'nav_target': self._current_nav_target})
            # Cancel current navigation goal
            if self._current_goal_handle:
                try:
                    self._current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
            # Immediately publish 0 cmd_vel
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.cmd_vel_safe.publish(stop_twist)

            self._publish_task_status('E-Stop', 'Emergency Stop Active — robot halted', 0)
            self._publish_response(
                action='emergency_stop',
                explanation='Emergency Stop engaged. All motion halted and current navigation cancelled.',
                target='e_stop',
                status='stopped'
            )
            self._complete_task_tracking('ABORTED_ESTOP')
        else:
            self.get_logger().info('Emergency Stop released.')
            self._log_event('estop_released', {})
            self._publish_task_status('E-Stop Released', 'Ready for new commands', 0)


    # ── Detection callback ────────────────────────────────────────────────────
    def _detection_cb(self, msg: ObjectDetection):
        if msg.robot_pose.header.frame_id == 'unknown':
            return
        raw_cls = msg.class_name.lower().strip()
        conf = float(msg.confidence)
        if conf < 0.15:   # accept anything the fused pipeline outputs (clip floor = 0.15)
            return


        det_x = msg.robot_pose.pose.position.x
        det_y = msg.robot_pose.pose.position.y

        # ── Spatial disambiguation: correct Gazebo COCO misclassifications ──
        # e.g. sofa model → YOLO says 'chair', hydrant model → YOLO says 'person'
        cls = _disambiguate_by_position(raw_cls, det_x, det_y)
        if cls != ALIASES.get(raw_cls, raw_cls):
            self.get_logger().info(
                f'SPATIAL DISAMBIG: YOLO said "{raw_cls}" at ({det_x:.2f},{det_y:.2f}) '
                f'→ reclassified as "{cls}"')

        now = time.time()
        self._latest_detections[cls] = (det_x, det_y, now, conf)


        # ── VISUAL STOP: if target is seen in camera with conf ≥ 0.35 ──
        if (self._current_nav_target == cls
                and not self._nav_cancelled
                and conf >= 0.35):
            self.get_logger().info(
                f'VISUAL CONFIRMATION STOP: {cls} conf={conf:.0%} at ({det_x:.2f},{det_y:.2f}) '
                f'— completing task and confirming target.')
            self._nav_cancelled = True  # prevent further detections from re-triggering

            # Cancel current Nav2 goal if moving
            if self._current_goal_handle:
                try:
                    self._current_goal_handle.cancel_goal_async()
                except Exception:
                    pass

            # Face the object
            face_yaw = math.atan2(det_y - self._robot_pose['y'],
                                  det_x - self._robot_pose['x'])
            threading.Thread(target=self._rotate_to_yaw, args=(face_yaw,), daemon=True).start()

            # Update memory with confirmed position
            self._update_memory_confirmed(cls, det_x, det_y)

            self._current_nav_target    = None
            self._current_target_center = None
            self._current_goal_handle   = None
            self._is_verifying          = False

            self._publish_task_status('Arrived', f'Detected {cls} in camera at {conf:.0%} confidence!', 100)
            self._publish_response(
                action='arrived',
                explanation=(f'I spotted the {cls} directly in my camera view with '
                             f'{conf:.0%} confidence. Stopping and orienting to face it.'),
                target=cls, status='done',
                extra={'pos_x': det_x, 'pos_y': det_y})
            self._complete_task_tracking('SUCCESS_VISUAL')
            return

        # ── EN-ROUTE ADAPTIVE RE-TARGETING (only if object moved significantly) ──
        if (not self._is_verifying
                and self._current_nav_target == cls
                and self._current_target_center
                and not self._nav_cancelled
                and conf >= 0.40
                and (now - self._last_redirect_time) > 5.0):
            
            # Prevent fake redirects caused by LiDAR hitting the outer arena walls (+/- 10m)
            if abs(det_x) > 9.2 or abs(det_y) > 9.2:
                self.get_logger().debug(f'Skipping redirect: ({det_x:.1f}, {det_y:.1f}) is too close to outer wall (likely bad depth estimate).')
                return

            target_ox, target_oy = self._current_target_center
            dist_moved = math.hypot(det_x - target_ox, det_y - target_oy)

            if dist_moved > 1.5:
                self._last_redirect_time = now
                rx = self._robot_pose['x']
                ry = self._robot_pose['y']
                sx, sy, yaw = compute_standoff(rx, ry, det_x, det_y)

                self.get_logger().warn(
                    f'EN-ROUTE: {cls} spotted at ({det_x:.2f},{det_y:.2f}) '
                    f'(moved {dist_moved:.1f}m from memory). Re-routing to new standoff.')
                self._publish_task_status('Adapting', f'En route to sighted {cls}', 75)
                self._publish_response(
                    action='dynamic_redirect',
                    explanation=(f'Spotted {cls} at ({det_x:.1f},{det_y:.1f}) '
                                 f'en route — dynamically re-routing to new standoff.'),
                    target=cls, status='navigating',
                    extra={'pos_x': sx, 'pos_y': sy})
                self._do_navigate_raw(cls, sx, sy, yaw, target_center=(det_x, det_y))

    # ── Publish helpers ───────────────────────────────────────────────────────
    def _publish_response(self, action, explanation, target='', status='done', extra=None):
        payload = {'action': action, 'explanation': explanation,
                   'target': target, 'status': status}
        if extra:
            payload.update(extra)
        m = String()
        m.data = json.dumps(payload)
        self.response_pub.publish(m)

    def _publish_task_status(self, stage, detail='', progress=0):
        m = String()
        m.data = json.dumps({'stage': stage, 'detail': detail, 'progress': progress})
        self.task_status_pub.publish(m)

    def _start_task_tracking(self, command: str, strategy: str = 'direct', decision: str = '', goal_pos: str = ''):
        self._active_task = {
            'command': command,
            'strategy': strategy,
            'decision': decision,
            'start_time': time.time(),
            'goal_pos': goal_pos
        }

    def _complete_task_tracking(self, outcome: str):
        if not self._active_task:
            return
        task = self._active_task
        self._active_task = None
        now = time.time()
        start = task.get('start_time', now)
        duration = round(max(0.0, now - start), 2)
        try:
            c = self.conn.cursor()
            c.execute('''
                INSERT INTO task_history (command, strategy, decision, outcome, start_time, duration, goal_pos)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (
                task.get('command', ''),
                task.get('strategy', 'direct'),
                task.get('decision', ''),
                outcome,
                start,
                duration,
                task.get('goal_pos', '')
            ))
            self.conn.commit()
            self.get_logger().info(f'Logged task history: "{task.get("command")}" -> {outcome} ({duration}s)')
        except Exception as e:
            self.get_logger().warn(f'Failed to log task history: {e}')

    # ── Step 7: Structured Event Logging ─────────────────────────────────────
    def _log_event(self, event: str, detail: dict = None):
        """
        Writes a structured event to the system_events table.
        Args:
            event: Short event type string (e.g., 'estop_triggered', 'cmd_rejected', 'retrieval_fallback')
            detail: Optional dict of additional structured context (JSON-serialized)
        """
        try:
            c = self.conn.cursor()
            c.execute(
                'INSERT INTO system_events (event, detail, timestamp) VALUES (?,?,?)',
                (event, json.dumps(detail or {}), time.time())
            )
            self.conn.commit()
        except Exception as e:
            self.get_logger().warn(f'[log_event] Failed to write system event "{event}": {e}')

    # ── Named Zones Resolver ──────────────────────────────────────────────────
    def _load_zones(self):
        zones = {}
        if os.path.exists(ZONES_PATH):
            try:
                with open(ZONES_PATH, 'r') as f:
                    data = yaml.safe_load(f)
                    zones = data.get('zones', {})
                self.get_logger().info(f'Loaded {len(zones)} named zones from {ZONES_PATH}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load zones.yaml: {e}')
        return zones

    def _resolve_zone(self, text: str):
        """
        Resolves command strings like 'Navigate to seating_area' or 'Go to the vehicle bay'.
        Returns (zone_key, zone_dict) if matched, else (None, None).
        """
        if not self.zones:
            return None, None
        t = text.lower().strip()
        for zk, zinfo in self.zones.items():
            if zk.replace('_', ' ') in t or zk in t:
                return zk, zinfo
            for alias in zinfo.get('aliases', []):
                if alias.lower() in t:
                    return zk, zinfo
        return None, None

    @staticmethod
    def _sanitize_command(text: str) -> tuple[bool, str]:
        """
        Validates and sanitizes input commands:
        - Max length check (<= 256 chars)
        - Rejection of shell metacharacters / SQL injection attempts
        Returns (is_valid, sanitized_text_or_error_message)
        """
        if not text or not text.strip():
            return False, "Empty command received."
        cleaned = text.strip()
        if len(cleaned) > 256:
            return False, f"Command exceeds maximum allowable length of 256 characters ({len(cleaned)} chars)."
        
        # Disallow dangerous shell metacharacters and SQL injection signatures
        forbidden = [';', '&&', '||', '`', '$', '<', '>', '\x00', '--', '/*', '*/']
        for char in forbidden:
            if char in cleaned:
                return False, f"Invalid command: prohibited security characters detected ({char})."

        return True, cleaned

    # ── NL parsing ────────────────────────────────────────────────────────────
    def _extract_class(self, text):
        t = text.lower()
        for cls in sorted(ALL_CLASSES, key=len, reverse=True):
            if cls in t:
                return ALIASES.get(cls, cls)
        return None

    def _is_query(self, text):
        q_words = ['where', 'last seen', 'location of', 'find', 'search', 'look for']
        t = text.lower()
        return any(w in t for w in q_words) and \
               not any(w in t for w in ['go to', 'navigate', 'drive', 'move to'])

    def _is_nl_memory_query(self, text):
        """
        Returns True for free-form memory questions that do NOT involve navigation:
        e.g. 'What changed since my last check?', 'How many objects do I know about?',
             'What was seen near the dining table?', 'Tell me about the chair',
             'What do you remember?'
        These queries target the episodic memory as a whole, not a specific object for navigation.
        """
        t = text.lower()
        # Must not be a navigation command
        nav_words = ['go to', 'navigate', 'drive to', 'move to', 'take me to']
        if any(w in t for w in nav_words):
            return False
        # Pattern 1: open-ended memory questions
        nl_triggers = [
            'what changed', 'what do you remember', 'what have you seen',
            'tell me about', 'what was near', 'what was seen', 'how many objects',
            'what objects', 'list all', 'show all', 'all objects', 'what do you know',
            'tell me everything', 'recent sightings', 'latest memory',
            'what happened', 'any recent', 'what is in memory',
        ]
        if any(w in t for w in nl_triggers):
            return True
        # Pattern 2: general "what/which" question WITH memory-related keywords
        if any(w in t for w in ['what', 'which', 'when', 'how many', 'tell me']) and \
           any(w in t for w in ['memory', 'remember', 'sighting', 'seen', 'recorded', 'known', 'stored']):
            return True
        return False

    def _handle_nl_memory_query(self, text):
        """
        Handles free-form NL memory queries by:
        1. Using FAISS retrieval to find relevant records (or fetching all active if open-ended).
        2. Passing records + query to LLM for grounded answer.
        3. Publishing the result to /aems/command_response without triggering navigation.
        """
        self.get_logger().info(f'NL memory query: "{text}"')
        self._log_event('retrieval_query', {'query': text[:128]})

        # ── Step 7: Retrieval-First Enforcement — FAISS must be tried first ───
        records = []
        used_faiss = False
        if self.retriever is not None:
            try:
                records = self.retriever.retrieve(text, top_k=5)
                used_faiss = bool(records)
            except Exception as e:
                self.get_logger().warn(f'FAISS retrieval failed: {e}')
                self._log_event('retrieval_faiss_error', {'error': str(e)})

        # Fallback to SQL for all active records if retriever unavailable or returned nothing
        if not records:
            self.get_logger().warn(
                f'[Step 7] Retrieval-first: FAISS returned no results — falling back to SQL scan.')
            self._log_event('retrieval_fallback', {'reason': 'faiss_no_results', 'query': text[:128]})
            try:
                c = self.conn.cursor()
                c.execute('''\
                    SELECT * FROM object_sightings
                    WHERE superseded=0
                    ORDER BY timestamp DESC
                    LIMIT 10
                ''')
                records = [dict(r) for r in c.fetchall()]
            except Exception as e:
                self.get_logger().error(f'DB fallback query failed: {e}')
        else:
            self._log_event('retrieval_faiss_hit', {'n_records': len(records), 'query': text[:128]})

        self._publish_task_status('Memory Query', f'Found {len(records)} relevant records', 70)
        # Generate LLM answer grounded in retrieved records
        answer = None
        if self.reasoner is not None and records:
            try:
                answer = self.reasoner.answer_memory_query(text, records)
            except Exception as e:
                self.get_logger().warn(f'LLM answer_memory_query failed: {e}')

        if not answer:
            if records:
                obj_list = list({r.get('object_class', 'unknown') for r in records})
                answer = f"In my episodic memory I have records of: {', '.join(obj_list)}."
            else:
                answer = "I could not find any relevant memories matching your query."

        self._publish_task_status('Memory Query Done', answer[:80], 100)
        self._publish_response(
            action='memory_query_result',
            explanation=answer,
            target='',
            status='done',
            extra={'records_retrieved': len(records)})

    # ── Failed-route check ────────────────────────────────────────────────────
    def _has_failed_routes(self, target_class):
        try:
            c = self.conn.cursor()
            c.execute('''SELECT COUNT(*) AS n FROM task_attempts
                         WHERE target_class=? AND outcome='FAILURE'
                           AND timestamp > ?''',
                      (target_class, time.time() - 600))
            return c.fetchone()['n'] > 0
        except Exception:
            return False

    # ── Route-replay caller ───────────────────────────────────────────────────
    def _call_route_replayer(self, target_class, fallback_fn, fallback_args):
        if not self.replay_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Route replayer node not active — using direct navigation.')
            self._publish_task_status(
                'Navigating', 'Using direct navigation strategy', 65)
            fallback_fn(*fallback_args)
            return

        req = ReplayRoute.Request()
        req.target_class = target_class
        future = self.replay_cli.call_async(req)

        # Allow sufficient time for multi-hop navigation (up to 90s)
        deadline = time.time() + 90.0
        while not future.done():
            if self._nav_cancelled:
                self.get_logger().info(
                    f'Route replay for {target_class} cancelled by user/estop.')
                return
            if time.time() > deadline:
                self.get_logger().warn(
                    f'Route replayer timed out after 90s for {target_class}. '
                    f'Falling back to direct nav.')
                self._publish_task_status(
                    'Info', 'Route replay timed out, falling back to direct navigation', 65)
                self._publish_response(
                    action='replay_fallback',
                    explanation='Route replay timed out — falling back to direct navigation.',
                    target=target_class, status='navigating')
                # Wait a moment to let replayer release its lock before we call fallback
                time.sleep(1.5)
                fallback_fn(*fallback_args)
                return
            time.sleep(0.2)

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Route replayer error: {e}. Falling back.')
            fallback_fn(*fallback_args)
            return

        if resp.success:
            self.get_logger().info(f'Route replay SUCCESS: {resp.message}')
            self._publish_task_status('Arrived', f'Reached {target_class} via route replay', 100)
            self._publish_response(
                action='arrived',
                explanation=(f'Arrived at {target_class} via landmark route replay. '
                             f'{resp.landmarks_visited} waypoints visited.'),
                target=target_class, status='done')
            self._complete_task_tracking('SUCCESS_REPLAY')
        else:
            self.get_logger().warn(f'Route replay FAILED: {resp.message}. Falling back to direct nav.')
            self._publish_task_status(
                'Info', 'Route replay unviable, falling back to direct navigation', 65)
            self._publish_response(
                action='replay_fallback',
                explanation=f'Route replay encountered difficulty ({resp.message}). Falling back to direct navigation.',
                target=target_class, status='navigating')
            # Small delay to ensure replayer has released its lock
            time.sleep(0.5)
            fallback_fn(*fallback_args)


    # ── Main command callback ─────────────────────────────────────────────────
    def _command_cb(self, msg: String):
        # ── Step 6: Input sanitization ────────────────────────────────────────
        valid, text = self._sanitize_command(msg.data)
        if not valid:
            self.get_logger().warn(f'Command rejected by sanitizer: {text}')
            self._log_event('cmd_rejected', {'reason': text, 'raw_input': msg.data[:64]})
            self._publish_response(action='error', explanation=text, status='error')
            return

        self.get_logger().info(f'NL command: "{text}"')
        self._log_event('command_received', {'text': text})

        # ── Step 6: Emergency-Stop gate ───────────────────────────────────────
        if self._is_estop:
            self._log_event('estop_blocked', {'text': text})
            self._publish_response(
                action='error',
                explanation='Emergency stop is active. Release E-Stop before issuing commands.',
                status='stopped')
            return

        # Reset per-task state for each new command
        self._nav_cancelled = False
        self._is_verifying  = False

        # ── Branch 1: Free-form NL memory query (no nav) ──────────────────────
        if self._is_nl_memory_query(text):
            threading.Thread(target=self._handle_nl_memory_query, args=(text,), daemon=True).start()
            return

        # ── Branch 2: Named Zone Navigation ───────────────────────────────────
        zone_key, zone_info = self._resolve_zone(text)
        if zone_key and any(w in text.lower() for w in ['go to', 'navigate', 'drive', 'move', 'head', 'take me']):
            standoff = zone_info.get('standoff', [0.0, 0.0, 0.0])
            zx, zy, z_yaw = standoff[0], standoff[1], standoff[2]
            center = zone_info.get('center', [zx, zy])
            desc = zone_info.get('description', zone_key)

            self.get_logger().info(f'Navigating to named zone [{zone_key}] standoff at ({zx:.2f}, {zy:.2f})')
            self._start_task_tracking(
                command=text,
                strategy='zone_navigation',
                decision='zone_standoff',
                goal_pos=f"({zx:.2f}, {zy:.2f})"
            )
            self._publish_task_status('Navigating', f'Driving to {zone_key} ({desc})', 60)
            self._publish_response(
                action='zone_navigation',
                explanation=f'Navigating to named zone "{zone_key.replace("_", " ")}" ({desc}) at standoff coordinates ({zx:.1f}, {zy:.1f}).',
                target=zone_key,
                status='navigating',
                extra={'pos_x': zx, 'pos_y': zy, 'zone': zone_key}
            )

            def _on_zone_arrival(success):
                if success:
                    self._publish_task_status('Arrived', f'Reached {zone_key}', 100)
                    self._publish_response(
                        action='arrived',
                        explanation=f'Arrived at named zone "{zone_key.replace("_", " ")}" ({desc}).',
                        target=zone_key,
                        status='done',
                        extra={'pos_x': zx, 'pos_y': zy}
                    )
                    self._complete_task_tracking('SUCCESS_ZONE')
                else:
                    self._publish_task_status('Error', f'Could not reach {zone_key}', 0)
                    self._publish_response(
                        action='error',
                        explanation=f'Failed to navigate to {zone_key}.',
                        target=zone_key,
                        status='error'
                    )
                    self._complete_task_tracking('FAILURE_ZONE')

            self._do_navigate_raw(zone_key, zx, zy, z_yaw, target_center=center, done_cb=_on_zone_arrival)
            return

        object_class = self._extract_class(text)
        if object_class is None:
            # If zone matched as a query ("where is reception_area?")
            if zone_key:
                center = zone_info.get('center', [0.0, 0.0])
                desc = zone_info.get('description', zone_key)
                self._publish_task_status('Zone retrieved', f'{zone_key} at ({center[0]:.1f},{center[1]:.1f})', 100)
                self._publish_response(
                    action='query_result',
                    explanation=f'Named zone "{zone_key.replace("_", " ")}" ({desc}) is centered at coordinates ({center[0]:.1f}, {center[1]:.1f}).',
                    target=zone_key,
                    status='done',
                    extra={'pos_x': center[0], 'pos_y': center[1]}
                )
                return

            self._publish_response(
                action='unknown',
                explanation=f'Could not identify an object or named zone in: "{text}". '
                             'Try "go to couch", "find person", or "navigate to reception_area".',
                status='error')
            return

        # Memory lookup
        self._publish_task_status('Retrieving memory', f'Looking up {object_class}', 25)
        c = self.conn.cursor()
        c.execute('''
            SELECT pos_x, pos_y, confidence, corroboration_count, timestamp
            FROM object_sightings
            WHERE object_class=? AND superseded=0
            ORDER BY timestamp DESC LIMIT 1
        ''', (object_class,))
        row = c.fetchone()

        if row is None:
            self._publish_task_status('Exploring', f'No memory of {object_class}', 50)
            self._publish_response(
                action='explore',
                explanation=f'I have no memory of a {object_class}. Full exploration needed.',
                target=object_class, status='done')
            return

        obj_x  = float(row['pos_x'])
        obj_y  = float(row['pos_y'])
        score  = min(max(float(row['confidence']), 0.1), 0.98)
        corr   = int(row['corroboration_count'])

        # TrustVerify cost engine (protected — do not change the math)
        p_valid       = max(0.05, score)
        cost_trust    = 8.0 + (1.0 - p_valid) * 40.0
        cost_reverify = 12.0 + 8.0 * 0.6

        if cost_trust < cost_reverify and score > 0.45:
            action = 'trust_memory'
            explanation = (
                f'My memory of the {object_class} at ({obj_x:.1f}, {obj_y:.1f}) carries '
                f'{score:.0%} confidence ({corr} sighting(s)). '
                f'Trusting it costs ~{cost_trust:.0f}s vs ~{cost_reverify:.0f}s to re-verify. '
                f'Heading directly to standoff.'
            )
        elif score > 0.25:
            action = 'reverify'
            explanation = (
                f'Memory of {object_class} at ({obj_x:.1f}, {obj_y:.1f}) is '
                f'moderately certain ({score:.0%}, {corr} sighting(s)). '
                f'I\'ll navigate to standoff and visually confirm.'
            )
        else:
            action = 'explore'
            explanation = (
                f'Memory score for {object_class} is too low ({score:.0%}). '
                'Full exploration would be safer.'
            )

        # Navigation strategy selection
        has_failures = self._has_failed_routes(object_class)
        use_replay   = (score < 0.65 or corr < 2 or has_failures)
        nav_strategy = 'landmark route replay' if use_replay else 'direct navigation'

        # LLM natural explanation (uses self.reasoner from __init__ for efficiency)
        try:
            if self.reasoner is not None:
                decision = {
                    'action': action,
                    'object_class': object_class,
                    'score': score,
                    'corroboration_count': corr,
                    'expected_cost_trust': cost_trust,
                    'expected_cost_reverify': cost_reverify,
                    'cost_of_exploration': 40.0,
                    'nav_strategy': nav_strategy,
                    'reason': explanation,
                }
                enriched    = self.reasoner.explain_decision(decision)
                explanation = enriched.get('llm_explanation', explanation)
        except Exception:
            pass

        # Query-only path
        if self._is_query(text):
            if self.reasoner is not None:
                try:
                    c.execute('''
                        SELECT * FROM object_sightings
                        WHERE object_class=? AND superseded=0
                        ORDER BY timestamp DESC LIMIT 3
                    ''', (object_class,))
                    q_records = [dict(r) for r in c.fetchall()]
                    if q_records:
                        explanation = self.reasoner.answer_memory_query(text, q_records)
                except Exception as e:
                    self.get_logger().warn(f'LLM query answering failed: {e}')

            self._publish_task_status('Memory retrieved',
                                      f'{object_class} at ({obj_x:.1f},{obj_y:.1f})', 100)
            self._publish_response(
                action='query_result', explanation=explanation,
                target=object_class, status='done',
                extra={'pos_x': obj_x, 'pos_y': obj_y, 'confidence': score})
            return

        # Compute unobstructed standoff
        rx = self._robot_pose['x']
        ry = self._robot_pose['y']
        standoff_dist = OBJECT_STANDOFF_DIST.get(object_class, 1.8)
        standoff_x, standoff_y, target_yaw = compute_standoff(rx, ry, obj_x, obj_y, dist=standoff_dist)

        self._start_task_tracking(
            command=text,
            strategy=nav_strategy,
            decision=action,
            goal_pos=f"({standoff_x:.2f}, {standoff_y:.2f})"
        )

        self._publish_task_status('Navigating', f'Driving to {object_class}', 60)
        self._publish_response(
            action=action, explanation=explanation,
            target=object_class, status='navigating',
            extra={'pos_x': standoff_x, 'pos_y': standoff_y, 'nav_strategy': nav_strategy})

        direct_nav_args = (object_class, standoff_x, standoff_y, target_yaw, (obj_x, obj_y))

        if use_replay:
            self.get_logger().info(
                f'Strategy: ROUTE REPLAY (score={score:.2f} corr={corr})')
            threading.Thread(
                target=self._call_route_replayer,
                args=(object_class, self._start_nav_task, direct_nav_args),
                daemon=True).start()
        else:
            self.get_logger().info(
                f'Strategy: DIRECT NAV (score={score:.2f} corr={corr})')
            self._start_nav_task(*direct_nav_args)

    # ── Main navigation entry ─────────────────────────────────────────────────
    def _start_nav_task(self, cls, sx, sy, yaw, target_center):
        """Dispatches primary navigation and then enters verification/scouting pipeline."""
        canonical = ALIASES.get(cls.lower().strip(), cls.lower().strip())
        self._is_verifying = False
        self._nav_cancelled = False


        def _on_primary_arrival(success):
            if not success:
                self._publish_task_status('Error', f'Could not navigate to {canonical}', 0)
                self._publish_response(action='error', explanation=f'Nav2 could not reach standoff for {canonical}.', target=canonical, status='error')
                self._complete_task_tracking('FAILURE_NAV')
                return

            # Start verification & multi-room perimeter scout in background thread
            threading.Thread(
                target=self._verification_and_scout_pipeline,
                args=(canonical, target_center),
                daemon=True).start()

        self._do_navigate_raw(canonical, sx, sy, yaw, target_center=target_center, done_cb=_on_primary_arrival)

    def _do_navigate_raw(self, cls, sx, sy, yaw, target_center=None, done_cb=None):
        """Asynchronously sends NavigateToPose goal to Nav2."""
        if not self.nav_cli.wait_for_server(timeout_sec=5.0):
            self._publish_task_status('Error', 'Nav2 not available', 0)
            self._publish_response(action='error', explanation='Nav2 not running.', target=cls, status='error')
            if done_cb: done_cb(False)
            return

        # ── Step 6: Goal bounding — clamp to valid arena area ─────────────────
        sx = float(max(ARENA_BOUNDS['min_x'], min(ARENA_BOUNDS['max_x'], float(sx))))
        sy = float(max(ARENA_BOUNDS['min_y'], min(ARENA_BOUNDS['max_y'], float(sy))))

        # Cancel any previous active goal before dispatching new one
        if self._current_goal_handle:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._current_goal_handle = None

        self._current_nav_target    = cls
        self._current_target_center = target_center
        self._current_goal_pos      = (float(sx), float(sy))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = float(sx)
        goal.pose.pose.position.y    = float(sy)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Nav2 → {cls} at ({sx:.2f},{sy:.2f}) yaw={math.degrees(yaw):.1f}°')
        self._publish_task_status('Navigating', f'En route to {cls}', 70)

        future = self.nav_cli.send_goal_async(goal)

        def _on_goal_response(f):
            try:
                handle = f.result()
                if not handle or not handle.accepted:
                    self.get_logger().error(f'Nav2 rejected goal for {cls}')
                    if done_cb: done_cb(False)
                    return

                self._current_goal_handle = handle
                result_future = handle.get_result_async()

                def _on_res(rf):
                    self._current_goal_handle = None
                    # If goal was cancelled by visual confirmation or e-stop, do not report error!
                    if self._nav_cancelled:
                        self.get_logger().info('Nav2 goal cancelled cleanly (visual arrival or user cancel).')
                        return

                    # If this goal was superseded by a newer target command, ignore old result!
                    if self._current_nav_target != cls:
                        self.get_logger().info(f'Nav2 goal for {cls} was superseded by {self._current_nav_target}.')
                        return


                    try:
                        result = rf.result()
                        # GoalStatus: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
                        nav_ok = (result.status == 4)
                        rx = self._robot_pose.get('x', 0.0)
                        ry = self._robot_pose.get('y', 0.0)
                        dist_to_goal = math.hypot(sx - rx, sy - ry)

                        if not nav_ok:
                            self.get_logger().warn(
                                f'Nav2 goal for {cls} ended with status={result.status} '
                                f'(dist to standoff = {dist_to_goal:.2f}m).')
                            # If we stopped far from standoff (> 2.0m), navigation failed!
                            # Do NOT verify or declare NOT_FOUND when the robot didn't even reach the location.
                            if dist_to_goal > 2.0:
                                self.get_logger().error(
                                    f'Robot stopped too far from {cls} ({dist_to_goal:.1f}m away). '
                                    f'Aborting task without degrading memory.')
                                if done_cb:
                                    done_cb(False)
                                return

                        # Robot is within standoff range
                        if done_cb:
                            done_cb(True)
                    except Exception as e:
                        self.get_logger().error(f'Nav2 result callback error: {e}')
                        if done_cb:
                            done_cb(False)


                result_future.add_done_callback(_on_res)
            except Exception as e:
                self.get_logger().error(f'Nav2 error: {e}')
                if done_cb: done_cb(False)

        future.add_done_callback(_on_goal_response)

    def _navigate_blocking(self, sx, sy, yaw, timeout=60.0) -> bool:
        """Helper to navigate synchronously in a background thread."""
        evt = threading.Event()
        success = [False]

        def _done(ok):
            success[0] = ok
            evt.set()

        self._do_navigate_raw('waypoint', sx, sy, yaw, target_center=None, done_cb=_done)
        evt.wait(timeout=timeout)
        return success[0]


    # ── Post-arrival verification & multi-room perimeter scout ────────────────
    def _rotate_to_yaw(self, target_yaw, speed=0.35, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            current = self._robot_pose.get('yaw', 0.0)
            err = target_yaw - current
            while err >  math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            if abs(err) < 0.08:
                break
            twist = Twist()
            twist.angular.z = speed if err > 0 else -speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def _check_target_seen(self, target_cls, max_age=15.0):
        """Returns (seen: bool, x, y, conf) if target_cls was sighted recently.
        Uses conf >= 0.15 (post-fusion adjusted confidence, clipped floor) for verification.
        Memory STORAGE in landmark_manager uses the stricter 0.40 threshold.
        Falls back to DB for detections within 45s when in-memory is stale."""
        if target_cls in self._latest_detections:
            dx, dy, ts, conf = self._latest_detections[target_cls]
            if (time.time() - ts) <= max_age and conf >= 0.15:
                return True, dx, dy, conf

        # DB fallback: check if any sighting of this class was written in the last 45s
        try:
            cutoff = time.time() - 45.0
            c = self.conn.cursor()
            c.execute('''SELECT pos_x, pos_y, confidence FROM object_sightings
                         WHERE object_class=? AND superseded=0 AND timestamp >= ?
                         ORDER BY timestamp DESC LIMIT 1''',
                      (target_cls, cutoff))
            row = c.fetchone()
            if row and row['confidence'] >= 0.15:
                return True, float(row['pos_x']), float(row['pos_y']), float(row['confidence'])
        except Exception:
            pass

        return False, 0.0, 0.0, 0.0



    def _update_memory_confirmed(self, target_cls, det_x, det_y):
        try:
            c = self.conn.cursor()
            c.execute('''UPDATE object_sightings SET superseded=1
                         WHERE object_class=? AND superseded=0''', (target_cls,))
            c.execute('''INSERT INTO object_sightings
                         (object_class, pos_x, pos_y, pos_z, confidence, timestamp,
                          landmark_id, corroboration_count, superseded)
                         VALUES (?, ?, ?, 0, 0.92, ?, 0, 1, 0)''',
                      (target_cls, det_x, det_y, time.time()))
            self.conn.commit()
        except Exception as e:
            self.get_logger().error(f'Memory update error: {e}')

    def _verification_and_scout_pipeline(self, target_cls, target_center):
        """
        1. Check if object is immediately visible in front of camera with conf >= 0.70.
        2. If not, do a full sequential 360° circular panorama sweep (8 steps of 45°).
        3. If still not visible (e.g. occluded behind wall), trigger Autonomous Multi-Room Perimeter Scout.
        """
        self._is_verifying = True
        self.get_logger().info(f'Starting verification pipeline for {target_cls}...')
        self._publish_task_status('Verifying', f'Checking camera for {target_cls}', 90)

        # ── Step 1: Immediate check at standoff ─────────────────────────────
        # Wait long enough for at least 2-3 YOLO frames at 1 FPS
        time.sleep(4.0)
        seen, dx, dy, conf = self._check_target_seen(target_cls)
        if seen:
            self.get_logger().info(f'Direct visual confirmation of {target_cls} at ({dx:.2f},{dy:.2f}) (conf={conf:.0%})!')
            self._update_memory_confirmed(target_cls, dx, dy)
            self._publish_task_status('Arrived', f'Reached and visually confirmed {target_cls}', 100)
            self._publish_response(
                action='arrived',
                explanation=f'Arrived at {target_cls}. Object is visually confirmed directly in camera view ({conf:.0%}).',
                target=target_cls, status='done',
                extra={'pos_x': dx, 'pos_y': dy})
            self._complete_task_tracking('SUCCESS_DIRECT_VIEW')
            self._is_verifying = False
            return

        # ── Step 2: Full 360° Circular Sweep (8 steps of 45°) ────────────────
        self.get_logger().info(f'{target_cls} not seen directly — running full 360° circular visual sweep...')
        self._publish_task_status('Scanning', f'Sweeping full 360° for {target_cls}', 92)
        base_yaw = self._robot_pose.get('yaw', 0.0)
        # Full sequential 360° rotation (45°, 90°, 135°, 180°, 225°, 270°, 315°, 360°)
        FULL_360_ANGLES = [base_yaw + math.radians(d) for d in [45, 90, 135, 180, 225, 270, 315, 360]]

        for step_idx, heading in enumerate(FULL_360_ANGLES):
            if self._nav_cancelled:
                return
            self._rotate_to_yaw(heading, speed=0.4, timeout=5.0)
            # Wait for at least 2 YOLO frames (system runs at ~1 FPS with frame-skip)
            time.sleep(3.0)
            if self._nav_cancelled:
                return
            seen, dx, dy, conf = self._check_target_seen(target_cls, max_age=12.0)

            if seen:
                self.get_logger().info(f'Found {target_cls} during 360° sweep at ({dx:.2f},{dy:.2f}) (conf={conf:.0%})!')
                self._update_memory_confirmed(target_cls, dx, dy)
                # Rotate directly to face it
                face_yaw = math.atan2(dy - self._robot_pose['y'], dx - self._robot_pose['x'])
                self._rotate_to_yaw(face_yaw)
                self._publish_task_status('Arrived', f'Found and facing {target_cls}', 100)
                self._publish_response(
                    action='arrived',
                    explanation=f'Found the {target_cls} during 360° visual sweep at ({dx:.1f}, {dy:.1f}) ({conf:.0%} confidence). Camera oriented directly at target.',
                    target=target_cls, status='done',
                    extra={'pos_x': dx, 'pos_y': dy})
                self._complete_task_tracking('SUCCESS_SWEEP_VIEW')
                self._is_verifying = False
                return

        # ── Step 3: Autonomous Multi-Room Perimeter Scout ────────────────────
        # If not seen, the object is likely behind an interior wall partition!
        if self._nav_cancelled:
            return
        ox, oy = target_center if target_center else (self._robot_pose['x'], self._robot_pose['y'])
        scout_waypoints = get_room_scout_waypoints(ox, oy)

        self.get_logger().warn(
            f'🎯 {target_cls} occluded behind wall! Initiating Autonomous Multi-Room Perimeter Scout...')
        self._publish_task_status('Scouting', f'Scouting room perimeter behind wall for {target_cls}', 80)
        self._publish_response(
            action='scouting',
            explanation=(f"The {target_cls} appears occluded by an interior wall partition. "
                         f"Initiating autonomous perimeter scout around room entrances to find a line of sight."),
            target=target_cls, status='navigating')

        for idx, (wx, wy) in enumerate(scout_waypoints):
            if self._nav_cancelled:
                return
            look_yaw = math.atan2(oy - wy, ox - wx)
            self.get_logger().info(f'[SCOUT {idx+1}/{len(scout_waypoints)}] Driving to vantage ({wx:.2f},{wy:.2f})...')
            self._publish_task_status('Scouting', f'Checking room vantage {idx+1}/{len(scout_waypoints)}', 80 + idx * 5)

            reached = self._navigate_blocking(wx, wy, look_yaw, timeout=30.0)
            if self._nav_cancelled:
                return
            if not reached:
                continue

            time.sleep(1.0)
            if self._nav_cancelled:
                return
            seen, dx, dy, conf = self._check_target_seen(target_cls, max_age=15.0)
            if not seen:
                # Quick ±45° peek
                self._rotate_to_yaw(look_yaw + 0.8)
                time.sleep(1.5)
                seen, dx, dy, conf = self._check_target_seen(target_cls, max_age=15.0)
            if not seen:
                self._rotate_to_yaw(look_yaw - 0.8)
                time.sleep(1.5)
                seen, dx, dy, conf = self._check_target_seen(target_cls, max_age=15.0)

            if seen:
                self.get_logger().info(f'🎯 SCOUT SUCCESS: Spotted {target_cls} behind wall at ({dx:.2f},{dy:.2f})!')
                self._update_memory_confirmed(target_cls, dx, dy)
                face_yaw = math.atan2(dy - self._robot_pose['y'], dx - self._robot_pose['x'])
                self._rotate_to_yaw(face_yaw)

                self._publish_task_status('Arrived', f'Located {target_cls} behind wall!', 100)
                self._publish_response(
                    action='arrived',
                    explanation=(f"Autonomous perimeter scout succeeded! Navigated around the wall partition "
                                 f"and visually confirmed {target_cls} at ({dx:.1f}, {dy:.1f})."),
                    target=target_cls, status='done',
                    extra={'pos_x': dx, 'pos_y': dy})
                self._complete_task_tracking('SUCCESS_SCOUT_VIEW')
                self._is_verifying = False
                return

        # Final sanity check: was it seen in camera right now or in the last 15 seconds?
        seen_final, dx, dy, conf = self._check_target_seen(target_cls, max_age=15.0)
        if seen_final:
            self.get_logger().info(f'Visual confirmation on final check for {target_cls} at ({dx:.2f},{dy:.2f}) (conf={conf:.0%})!')
            self._update_memory_confirmed(target_cls, dx, dy)
            face_yaw = math.atan2(dy - self._robot_pose['y'], dx - self._robot_pose['x'])
            self._rotate_to_yaw(face_yaw)
            self._publish_task_status('Arrived', f'Reached and visually confirmed {target_cls}', 100)
            self._publish_response(
                action='arrived',
                explanation=f'Arrived at {target_cls}. Object is visually confirmed directly in camera view ({conf:.0%}).',
                target=target_cls, status='done',
                extra={'pos_x': dx, 'pos_y': dy})
            self._complete_task_tracking('SUCCESS_DIRECT_VIEW')
            self._is_verifying = False
            return

        # ── Step 4: Stale Memory Warning ─────────────────────────────────────
        self.get_logger().warn(f'{target_cls} not found anywhere in room perimeter.')
        try:
            c = self.conn.cursor()
            c.execute('''UPDATE object_sightings
                         SET confidence = MAX(0.05, confidence - 0.20)
                         WHERE object_class=? AND superseded=0''', (target_cls,))
            self.conn.commit()
        except Exception as e:
            self.get_logger().error(f'DB error: {e}')

        self._publish_task_status('Warning', f'{target_cls} not found — memory marked stale', 100)
        self._publish_response(
            action='not_found',
            explanation=(f"I navigated to the memory location and scouted all perimeter vantage points, "
                         f"but could not locate the {target_cls}. Confidence has been reduced — it may have moved."),
            target=target_cls, status='done')
        self._complete_task_tracking('NOT_FOUND')
        self._is_verifying = False


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
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
