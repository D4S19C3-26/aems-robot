#!/usr/bin/env python3
import os
import sqlite3
import math
import rclpy
from rclpy.node import Node
from aems_interfaces.msg import ObjectDetection
from aems_interfaces.srv import QueryMemory

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
    'bench': 'dining table',
    'chair': 'chair',
    'dining chair': 'chair',
    'dining_chair': 'chair',
    'person': 'person',
    'human': 'person',
    'fire hydrant': 'fire hydrant',
    'fire_hydrant': 'fire hydrant',
    'hydrant': 'fire hydrant',
}

class MemoryManagerNode(Node):
    def __init__(self):
        super().__init__('memory_manager')
        
        # Connect to SQLite
        self.conn = sqlite3.connect(DB_PATH)
        self.cursor = self.conn.cursor()
        self.init_db()

        # Subscriber
        self.sub = self.create_subscription(ObjectDetection, '/aems/detections', self.detection_callback, 10)
        
        # Service
        self.srv = self.create_service(QueryMemory, '/query_memory', self.query_callback)
        
        self.get_logger().info(f'Memory Manager active. DB: {DB_PATH}')

    def init_db(self):
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS object_sightings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                object_class TEXT UNIQUE,
                pos_x REAL,
                pos_y REAL,
                pos_z REAL,
                confidence REAL,
                timestamp REAL
            )
        ''')
        self.conn.commit()

    def detection_callback(self, msg):
        raw_cls = msg.class_name.lower()

        # 1. Filter out unwanted phantom classes (like traffic light)
        canonical_cls = ALIASES.get(raw_cls, raw_cls)
        if canonical_cls not in ALLOWED_CLASSES:
            return

        x = msg.robot_pose.pose.position.x
        y = msg.robot_pose.pose.position.y
        z = msg.robot_pose.pose.position.z
        conf = msg.confidence
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 2. Only save if robot is properly localized (not uninitialized unknown pose)
        if msg.robot_pose.header.frame_id == 'unknown':
            return

        # 3. Always update to the LATEST position for this object class
        self.cursor.execute('''
            INSERT INTO object_sightings (object_class, pos_x, pos_y, pos_z, confidence, timestamp)
            VALUES (?, ?, ?, ?, ?, ?)
            ON CONFLICT(object_class) DO UPDATE SET
                pos_x = excluded.pos_x,
                pos_y = excluded.pos_y,
                pos_z = excluded.pos_z,
                confidence = excluded.confidence,
                timestamp = excluded.timestamp
        ''', (canonical_cls, x, y, z, conf, stamp))
        self.conn.commit()
        
        self.get_logger().info(f"Updated memory: {canonical_cls} latest position -> ({x:.2f}, {y:.2f}) [conf: {conf:.2f}]")

    def query_callback(self, request, response):
        raw_cls = request.object_class.lower().strip()
        cls = ALIASES.get(raw_cls, raw_cls)
        self.get_logger().info(f"Memory Query received for: '{raw_cls}' (resolved to '{cls}')")

        self.cursor.execute('''
            SELECT pos_x, pos_y, pos_z, timestamp, confidence FROM object_sightings 
            WHERE object_class = ? LIMIT 1
        ''', (cls,))
        row = self.cursor.fetchone()

        if row:
            response.success = True
            response.message = f"Found {cls} in episodic memory"
            response.pose.header.frame_id = 'map'
            response.pose.header.stamp = self.get_clock().now().to_msg()
            response.pose.pose.position.x = row[0]
            response.pose.pose.position.y = row[1]
            response.pose.pose.position.z = row[2]
            response.pose.pose.orientation.w = 1.0
            self.get_logger().info(f"Query resolved: found '{cls}' at ({row[0]:.2f}, {row[1]:.2f}) [conf: {row[4]:.2f}]")
        else:
            response.success = False
            response.message = f"No memory of '{raw_cls}' found."
            self.get_logger().warn(f"Query resolved: '{raw_cls}' NOT FOUND.")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MemoryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
