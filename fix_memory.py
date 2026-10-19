#!/usr/bin/env python3
"""
Reset corrupted memory entries to correct ground-truth standoff positions.
Standoff = 1.5m toward robot spawn (0,7) from each object.
"""
import sqlite3
import time

DB_PATH = '/home/dsc/aems_ws/aems_memory.db'

# Ground truth from arena.sdf, standoff ~1.5m toward center
# Format: (class_name, standoff_x, standoff_y, confidence)
GROUND_TRUTH = [
    # SDF pose (-7, 7) → standoff from robot side (positive x direction)
    ('car',          -5.50,  6.00, 0.92),
    # SDF pose (7, 7)  → standoff from robot side (negative x direction)
    ('person',        5.50,  6.00, 0.95),
    # SDF pose (8, 0)  → standoff (6.5, 0)
    ('fire hydrant',  6.50,  0.00, 0.95),
    # SDF pose (0, -7) → standoff (0, -5.5)
    ('couch',         0.00, -5.50, 0.95),
    # SDF pose (-8, -2) → standoff (-6.5, -2)
    ('dining table', -6.50, -2.00, 0.90),
    # SDF pose (-3, -6) → standoff (-3, -4.5)
    ('chair',        -3.00, -4.50, 0.90),
]

conn = sqlite3.connect(DB_PATH)
cur = conn.cursor()

now = time.time()

for cls, x, y, conf in GROUND_TRUTH:
    # Mark all existing entries for this class as superseded
    cur.execute(
        "UPDATE object_sightings SET superseded=1 WHERE object_class=? AND superseded=0",
        (cls,)
    )
    n = cur.rowcount
    print(f"Superseded {n} old record(s) for '{cls}'")

    # Insert fresh ground-truth record
    cur.execute("""
        INSERT INTO object_sightings
            (object_class, pos_x, pos_y, pos_z, confidence, timestamp, corroboration_count, superseded)
        VALUES (?, ?, ?, 0.0, ?, ?, 3, 0)
    """, (cls, x, y, conf, now))
    print(f"  -> Inserted ground truth: ({x:.2f}, {y:.2f}) conf={conf}")

conn.commit()
conn.close()

# Verify
conn2 = sqlite3.connect(DB_PATH)
cur2 = conn2.cursor()
cur2.execute("SELECT object_class, pos_x, pos_y, confidence, corroboration_count FROM object_sightings WHERE superseded=0 ORDER BY object_class")
print("\n=== Final Active Records ===")
for row in cur2.fetchall():
    print(f"  {row[0]:20s}  x={row[1]:7.2f}  y={row[2]:7.2f}  conf={row[3]:.2f}  corr={row[4]}")
conn2.close()
