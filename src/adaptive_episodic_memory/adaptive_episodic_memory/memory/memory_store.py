#!/usr/bin/env python3
"""
Adaptive Episodic Memory System - Memory Store
Implements the landmark-relative episodic memory + failure memory
as described in the project report (Tables 3 & 4).
"""

import sqlite3
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Dict, Optional, Tuple


class MemoryStore:
    def __init__(self, db_path: str = None):
        if db_path is None:
            # Default location inside the package
            db_path = str(Path.home() / "aems_ws" / "memory" / "episodic_memory.db")
        
        self.db_path = db_path
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self._create_tables()

    def _create_tables(self):
        cursor = self.conn.cursor()

        # ---------- Landmarks (graph nodes) ----------
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS landmarks (
                landmark_id     INTEGER PRIMARY KEY AUTOINCREMENT,
                name            TEXT,
                perceptual_signature TEXT,          -- JSON: feature fingerprint
                created_at      DATETIME DEFAULT CURRENT_TIMESTAMP,
                last_seen       DATETIME,
                confidence      REAL DEFAULT 1.0
            )
        """)

        # ---------- Transitions (graph edges) ----------
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS transitions (
                transition_id   INTEGER PRIMARY KEY AUTOINCREMENT,
                from_landmark_id INTEGER,
                to_landmark_id  INTEGER,
                direction       TEXT,               -- e.g. "north", "left", approximate
                approx_distance REAL,
                notes           TEXT,
                created_at      DATETIME DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (from_landmark_id) REFERENCES landmarks(landmark_id),
                FOREIGN KEY (to_landmark_id)   REFERENCES landmarks(landmark_id)
            )
        """)

        # ---------- Object Sightings (Table 3 in report) ----------
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS object_sightings (
                episode_id          INTEGER PRIMARY KEY AUTOINCREMENT,
                object_class        TEXT NOT NULL,
                landmark_id         INTEGER,
                relative_offset     TEXT,           -- JSON: {"x": 0.5, "y": 0.2, "yaw": 0.0}
                confidence          REAL,
                timestamp           DATETIME DEFAULT CURRENT_TIMESTAMP,
                corroboration_count INTEGER DEFAULT 1,
                status              TEXT DEFAULT 'active',  -- active / superseded / stale
                robot_pos_x         REAL DEFAULT 0.0,
                robot_pos_y         REAL DEFAULT 7.0,
                robot_yaw           REAL DEFAULT -1.5708,
                task_info           TEXT DEFAULT 'autonomous_patrol',
                scene_description   TEXT DEFAULT '',
                embedding           BLOB,
                FOREIGN KEY (landmark_id) REFERENCES landmarks(landmark_id)
            )
        """)

        # ---------- Attempt / Failure records (Table 4 in report) ----------
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS attempts (
                attempt_id          INTEGER PRIMARY KEY AUTOINCREMENT,
                task_type           TEXT NOT NULL,
                route_landmarks     TEXT,           -- JSON list of landmark_ids
                outcome             TEXT,           -- success / blocked / unreachable / timeout
                failure_landmark_id INTEGER,
                failure_reason      TEXT,
                timestamp           DATETIME DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (failure_landmark_id) REFERENCES landmarks(landmark_id)
            )
        """)

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS task_history (
                task_id             INTEGER PRIMARY KEY AUTOINCREMENT,
                command             TEXT NOT NULL,
                strategy            TEXT,
                decision            TEXT,
                outcome             TEXT,
                start_time          REAL,
                duration            REAL,
                goal_pos            TEXT,
                timestamp           DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)

        self.conn.commit()

    # ------------------------------------------------------------------
    # Landmark helpers
    # ------------------------------------------------------------------
    def add_landmark(self, name: str, signature: dict, confidence: float = 1.0) -> int:
        cursor = self.conn.cursor()
        cursor.execute("""
            INSERT INTO landmarks (name, perceptual_signature, last_seen, confidence)
            VALUES (?, ?, ?, ?)
        """, (name, json.dumps(signature), datetime.now(timezone.utc).isoformat(), confidence))
        self.conn.commit()
        return cursor.lastrowid

    def get_landmark(self, landmark_id: int) -> Optional[Dict]:
        cursor = self.conn.cursor()
        cursor.execute("SELECT * FROM landmarks WHERE landmark_id = ?", (landmark_id,))
        row = cursor.fetchone()
        return dict(row) if row else None

    # ------------------------------------------------------------------
    # Object sighting helpers (core of episodic memory)
    # ------------------------------------------------------------------
    def add_object_sighting(self, object_class: str, landmark_id: int,
                            relative_offset: dict, confidence: float) -> int:
        """
        Write policy (Section 7.4):
        - If a recent + corroborated record already exists → just increase corroboration
        - If location conflicts → mark old as 'superseded' and insert new
        """
        cursor = self.conn.cursor()

        # Check for existing active record of same object near same landmark
        cursor.execute("""
            SELECT * FROM object_sightings
            WHERE object_class = ? AND landmark_id = ? AND status = 'active'
            ORDER BY timestamp DESC LIMIT 1
        """, (object_class, landmark_id))
        existing = cursor.fetchone()

        if existing:
            # Same place → increase corroboration
            cursor.execute("""
                UPDATE object_sightings
                SET corroboration_count = corroboration_count + 1,
                    confidence = MAX(confidence, ?),
                    timestamp = ?
                WHERE episode_id = ?
            """, (confidence, datetime.now(timezone.utc).isoformat(), existing["episode_id"]))
            self.conn.commit()
            return existing["episode_id"]

        # Check for conflicting location (same object, different landmark)
        cursor.execute("""
            SELECT * FROM object_sightings
            WHERE object_class = ? AND status = 'active'
        """, (object_class,))
        conflicts = cursor.fetchall()

        for conf in conflicts:
            if conf["landmark_id"] != landmark_id:
                # Mark old as superseded
                cursor.execute("""
                    UPDATE object_sightings SET status = 'superseded'
                    WHERE episode_id = ?
                """, (conf["episode_id"],))

        # Insert new active record
        cursor.execute("""
            INSERT INTO object_sightings
            (object_class, landmark_id, relative_offset, confidence, timestamp, corroboration_count, status)
            VALUES (?, ?, ?, ?, ?, 1, 'active')
        """, (object_class, landmark_id, json.dumps(relative_offset),
              confidence, datetime.now(timezone.utc).isoformat()))
        self.conn.commit()
        return cursor.lastrowid

    def get_best_object_location(self, object_class: str) -> Optional[Dict]:
        """Return the highest-scoring active sighting for an object class."""
        cursor = self.conn.cursor()
        cursor.execute("""
            SELECT * FROM object_sightings
            WHERE object_class = ? AND status = 'active'
            ORDER BY (confidence * corroboration_count) DESC, timestamp DESC
            LIMIT 1
        """, (object_class,))
        row = cursor.fetchone()
        return dict(row) if row else None

    # ------------------------------------------------------------------
    # Attempt / Failure helpers
    # ------------------------------------------------------------------
    def log_attempt(self, task_type: str, route_landmarks: List[int],
                    outcome: str, failure_landmark_id: int = None,
                    failure_reason: str = None) -> int:
        cursor = self.conn.cursor()
        cursor.execute("""
            INSERT INTO attempts
            (task_type, route_landmarks, outcome, failure_landmark_id, failure_reason, timestamp)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (task_type, json.dumps(route_landmarks), outcome,
              failure_landmark_id, failure_reason, datetime.now(timezone.utc).isoformat()))
        self.conn.commit()
        return cursor.lastrowid

    def get_failed_routes(self, task_type: str) -> List[Dict]:
        """Return all previously failed routes for a given task type."""
        cursor = self.conn.cursor()
        cursor.execute("""
            SELECT * FROM attempts
            WHERE task_type = ? AND outcome != 'success'
            ORDER BY timestamp DESC
        """, (task_type,))
        return [dict(row) for row in cursor.fetchall()]

    def close(self):
        self.conn.close()


# Simple self-test when run directly
if __name__ == "__main__":
    store = MemoryStore("/tmp/test_aems.db")
    print("MemoryStore created successfully")
    
    # Quick test
    lid = store.add_landmark("kitchen_counter", {"type": "furniture", "color": "wood"})
    print(f"Added landmark id: {lid}")
    
    eid = store.add_object_sighting("bottle", lid, {"x": 0.3, "y": 0.1}, 0.92)
    print(f"Added object sighting id: {eid}")
    
    best = store.get_best_object_location("bottle")
    print("Best location:", best)
    
    store.close()
    print("Self-test passed.")
