#!/usr/bin/env python3
"""
Reset AEMS Memory Database to canonical ground truth positions from arena.sdf.
"""
import sqlite3
import os
import time
import math

DB_PATH = os.path.expanduser('~/aems_ws/aems_memory.db')

def reset_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()

    # Clear old data
    c.execute('DELETE FROM landmark_edges')
    c.execute('DELETE FROM object_sightings')
    c.execute('DELETE FROM landmarks')
    c.execute('DELETE FROM system_events')
    c.execute('DELETE FROM task_attempts')
    c.execute('DELETE FROM sqlite_sequence')

    now = time.time()

    # Canonical Gazebo arena objects
    ground_truth = [
        ('car', -7.0, 7.0, 0.0),
        ('person', 7.0, 7.0, 0.0),
        ('fire hydrant', 8.0, 0.0, 0.0),
        ('couch', 0.0, -7.0, 0.0),
        ('dining table', -8.0, -2.0, 0.0),
        ('chair', -3.0, -6.0, 0.0)
    ]

    landmark_ids = {}
    for cls, x, y, z in ground_truth:
        c.execute('''
            INSERT INTO landmarks (class_name, pos_x, pos_y, first_seen, visit_count, confidence_score)
            VALUES (?, ?, ?, ?, 3, 0.95)
        ''', (cls, x, y, now))
        lid = c.lastrowid
        landmark_ids[cls] = (lid, x, y)
        
        c.execute('''
            INSERT INTO object_sightings (object_class, pos_x, pos_y, pos_z, confidence, timestamp, landmark_id, corroboration_count, superseded)
            VALUES (?, ?, ?, ?, 0.95, ?, ?, 3, 0)
        ''', (cls, x, y, z, now, lid))

    # Connect topological edges
    edges = [
        ('car', 'person'),
        ('person', 'fire hydrant'),
        ('fire hydrant', 'couch'),
        ('couch', 'chair'),
        ('chair', 'dining table'),
        ('dining table', 'car'),
        ('car', 'chair'),
        ('person', 'couch')
    ]

    for from_cls, to_cls in edges:
        id1, x1, y1 = landmark_ids[from_cls]
        id2, x2, y2 = landmark_ids[to_cls]
        dist = math.hypot(x2 - x1, y2 - y1)
        bearing = math.atan2(y2 - y1, x2 - x1)
        
        c.execute('''
            INSERT INTO landmark_edges (from_id, to_id, distance, bearing, timestamp)
            VALUES (?, ?, ?, ?, ?)
        ''', (id1, id2, round(dist, 2), round(bearing, 3), now))
        c.execute('''
            INSERT INTO landmark_edges (from_id, to_id, distance, bearing, timestamp)
            VALUES (?, ?, ?, ?, ?)
        ''', (id2, id1, round(dist, 2), round(-bearing, 3), now))

    c.execute('''
        INSERT INTO system_events (event, detail, timestamp)
        VALUES (?, ?, ?)
    ''', ('DATABASE_RESET', 'Reset episodic memory database to canonical arena ground truth poses.', now))

    conn.commit()
    conn.close()
    print("✅ AEMS Memory DB reset successfully to original ground truth poses.")

if __name__ == '__main__':
    reset_db()
