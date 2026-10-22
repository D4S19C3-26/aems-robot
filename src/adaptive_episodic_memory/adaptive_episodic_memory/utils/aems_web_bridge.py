#!/usr/bin/env python3
"""
AEMS Mission Control Web Bridge.
Serves HTTP on port 8080 and WebSocket on port 9090.
Compatible with websockets 14.x+ / 17.x+ (asyncio.run compatible).
"""

import asyncio
import base64
import http.server
import json
import math
import os
import socketserver
import sqlite3
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

import websockets

DB_PATH = os.path.expanduser('~/aems_ws/aems_memory.db')
DASHBOARD_DIR = os.path.expanduser('~/aems_ws/dashboard')
HTTP_PORT = 8080
WS_PORT = 9090


class AEMSWebBridgeNode(Node):
    def __init__(self):
        super().__init__('aems_web_bridge')
        self.connected_clients = set()
        self.loop = None
        self._last_img_time = 0.0

        # State caches (matches Gazebo spawn pose at 0.0, 7.0 facing South -1.571 rad)
        self.robot_pose = {'x': 0.0, 'y': 7.0, 'yaw': -1.571}
        self.current_plan = []
        self.latest_status = {'online': True, 'landmarks': 0, 'active_sightings': 0}

        # Subscriptions
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10)
        self.sub_plan = self.create_subscription(
            Path, '/plan', self._plan_cb, 10)
        self.sub_annotated_img = self.create_subscription(
            Image, '/aems/annotated_image', self._annotated_img_cb, 1)
        self.sub_raw_img = self.create_subscription(
            Image, '/camera/image_raw', self._raw_img_cb, 1)
        self.sub_status = self.create_subscription(
            String, '/aems/status', self._status_cb, 10)
        self.sub_events = self.create_subscription(
            String, '/aems/events', self._events_cb, 10)
        self.sub_memories = self.create_subscription(
            String, '/aems/memories_json', self._memories_cb, 10)
        self.sub_task = self.create_subscription(
            String, '/aems/task_status', self._task_cb, 10)
        self.sub_response = self.create_subscription(
            String, '/aems/command_response', self._response_cb, 10)

        # Publishers
        self.pub_nl_cmd = self.create_publisher(String, '/aems/nl_command', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Periodic memory snapshot & initial AMCL pose broadcast (only first 3s)
        self.create_timer(3.0, self._periodic_db_sync)
        self._initial_pose_count = 0
        self._init_pose_timer = self.create_timer(1.0, self._broadcast_initial_pose)

        # Periodic pose heartbeat — pushes cached robot_pose every second so
        # the dashboard always shows updated coordinates once AMCL starts flowing.
        self.create_timer(1.0, self._pose_heartbeat)

        self.get_logger().info(f'AEMS Web Bridge initialized. HTTP:{HTTP_PORT}, WS:{WS_PORT}')


    def _broadcast_initial_pose(self):
        """Broadcasts initial pose to AMCL — fires only 3 times on startup then stops."""
        if self._initial_pose_count >= 3:
            self._init_pose_timer.cancel()
            return
        self._initial_pose_count += 1
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 7.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = -0.7071068
        msg.pose.pose.orientation.w = 0.7071068
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.pub_initialpose.publish(msg)
        self.get_logger().info(f'Published /initialpose (attempt {self._initial_pose_count}/3)')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 3)}
        self.broadcast_json({
            'topic': '/amcl_pose',
            'msg': self.robot_pose
        })

    def _pose_heartbeat(self):
        """Re-broadcast the latest cached robot pose every second.
        Ensures the dashboard coordinate display stays live once AMCL starts publishing."""
        self.broadcast_json({
            'topic': '/amcl_pose',
            'msg': self.robot_pose
        })

    def _plan_cb(self, msg: Path):
        points = [{'x': round(p.pose.position.x, 3), 'y': round(p.pose.position.y, 3)} for p in msg.poses]
        self.current_plan = points
        self.broadcast_json({
            'topic': '/plan',
            'msg': {'poses': points}
        })

    def _annotated_img_cb(self, msg: Image):
        now = time.time()
        if now - self._last_img_time < 0.020:  # up to 50 FPS cap for smoother display
            return
        self._last_img_time = now

        try:
            if msg.encoding in ['bgr8', 'rgb8']:
                channels = 3
            elif msg.encoding == 'mono8':
                channels = 1
            else:
                channels = 3
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            h, w = frame.shape[:2]
            if w > 480:
                frame = cv2.resize(frame, (480, int(h * 480 / w)))

            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
            b64_str = base64.b64encode(jpeg).decode('utf-8')
            self.broadcast_json({
                'topic': '/aems/annotated_image',
                'msg': {'data': b64_str, 'format': 'jpeg'}
            })
        except Exception:
            pass


    def _raw_img_cb(self, msg: Image):
        if time.time() - self._last_img_time > 1.0:
            self._annotated_img_cb(msg)

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_status = data
            self.broadcast_json({'topic': '/aems/status', 'msg': data})
        except Exception:
            pass

    def _events_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.broadcast_json({'topic': '/aems/events', 'msg': data})
        except Exception:
            pass

    def _memories_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.broadcast_json({'topic': '/aems/memories_json', 'msg': data})
        except Exception:
            pass

    def _task_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.broadcast_json({'topic': '/aems/task_status', 'msg': data})
        except Exception:
            pass

    def _response_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.broadcast_json({'topic': '/aems/command_response', 'msg': data})
        except Exception:
            pass

    def _periodic_db_sync(self):
        try:
            if not os.path.exists(DB_PATH):
                return
            conn = sqlite3.connect(DB_PATH)
            conn.row_factory = sqlite3.Row
            c = conn.cursor()
            c.execute('''
                SELECT os.id, os.object_class, os.pos_x, os.pos_y, os.confidence,
                       os.timestamp, os.corroboration_count, os.superseded, os.landmark_id,
                       COALESCE(l.confidence_score, 1.0) as lm_conf
                FROM object_sightings os
                LEFT JOIN landmarks l ON os.landmark_id = l.id
                ORDER BY os.superseded ASC, os.timestamp DESC
                LIMIT 200
            ''')
            rows = c.fetchall()
            memories = [
                {
                    'id': r['id'], 'object_class': r['object_class'],
                    'pos_x': round(r['pos_x'], 2), 'pos_y': round(r['pos_y'], 2),
                    'confidence': round(r['confidence'], 2), 'timestamp': r['timestamp'],
                    'corroboration_count': r['corroboration_count'], 'superseded': bool(r['superseded']),
                    'landmark_id': r['landmark_id'], 'landmark_confidence': round(r['lm_conf'], 2)
                } for r in rows
            ]
            c.execute('SELECT id, class_name, pos_x, pos_y, confidence_score, visit_count FROM landmarks')
            lm_rows = c.fetchall()
            landmarks = [
                {
                    'id': r['id'], 'class_name': r['class_name'],
                    'pos_x': round(r['pos_x'], 2), 'pos_y': round(r['pos_y'], 2),
                    'confidence_score': round(r['confidence_score'], 2), 'visit_count': r['visit_count']
                } for r in lm_rows
            ]

            c.execute('SELECT from_id, to_id, distance FROM landmark_edges')
            edge_rows = c.fetchall()
            edges = [{'from_id': r['from_id'], 'to_id': r['to_id'], 'distance': round(r['distance'], 2)}
                     for r in edge_rows]

            # Task history query
            task_history = []
            try:
                c.execute('''
                    SELECT task_id, command, strategy, decision, outcome, start_time, duration, goal_pos, timestamp
                    FROM task_history
                    ORDER BY task_id DESC
                    LIMIT 50
                ''')
                th_rows = c.fetchall()
                task_history = [
                    {
                        'task_id': r['task_id'],
                        'command': r['command'],
                        'strategy': r['strategy'],
                        'decision': r['decision'],
                        'outcome': r['outcome'],
                        'start_time': r['start_time'],
                        'duration': r['duration'],
                        'goal_pos': r['goal_pos'],
                        'timestamp': r['timestamp']
                    } for r in th_rows
                ]
            except Exception:
                pass

            conn.close()

            self.broadcast_json({
                'topic': '/aems/db_snapshot',
                'msg': {
                    'memories': memories,
                    'landmarks': landmarks,
                    'edges': edges,
                    'task_history': task_history
                }
            })
        except Exception:
            pass

    def broadcast_json(self, payload):
        if not self.connected_clients or not self.loop or self.loop.is_closed():
            return
        msg_str = json.dumps(payload)
        asyncio.run_coroutine_threadsafe(self._async_broadcast(msg_str), self.loop)

    async def _async_broadcast(self, msg_str):
        dead_clients = set()
        for ws in list(self.connected_clients):
            try:
                await ws.send(msg_str)
            except Exception:
                dead_clients.add(ws)
        self.connected_clients.difference_update(dead_clients)

    def handle_client_message(self, message_str):
        try:
            req = json.loads(message_str)
            op = req.get('op')
            
            if op == 'publish' or 'command' in req:
                topic = req.get('topic', '')
                if topic == '/aems/nl_command' or 'command' in req:
                    cmd_text = req.get('msg', {}).get('data', '') if 'msg' in req else req.get('command', '')
                    if cmd_text:
                        msg = String()
                        msg.data = cmd_text
                        self.pub_nl_cmd.publish(msg)
                        self.get_logger().info(f'Forwarded NL command: {cmd_text}')

                elif topic == '/cmd_vel':
                    t = req.get('msg', {})
                    twist = Twist()
                    twist.linear.x = float(t.get('linear', {}).get('x', 0.0))
                    twist.angular.z = float(t.get('angular', {}).get('z', 0.0))
                    self.pub_cmd_vel.publish(twist)

            elif op == 'subscribe':
                topic = req.get('topic')
                if topic == '/amcl_pose':
                    self.broadcast_json({'topic': '/amcl_pose', 'msg': self.robot_pose})
                elif topic == '/aems/status':
                    self.broadcast_json({'topic': '/aems/status', 'msg': self.latest_status})
        except Exception as e:
            self.get_logger().warn(f'Error parsing WS message: {e}')


async def ws_handler(websocket, node: AEMSWebBridgeNode):
    node.connected_clients.add(websocket)
    node.get_logger().info(f'Dashboard client connected. Active: {len(node.connected_clients)}')
    
    await websocket.send(json.dumps({'topic': '/amcl_pose', 'msg': node.robot_pose}))
    await websocket.send(json.dumps({'topic': '/aems/status', 'msg': node.latest_status}))
    node._periodic_db_sync()

    try:
        async for message in websocket:
            node.handle_client_message(message)
    except Exception:
        pass
    finally:
        node.connected_clients.discard(websocket)
        node.get_logger().info(f'Dashboard client disconnected. Active: {len(node.connected_clients)}')


def run_http_server():
    os.chdir(DASHBOARD_DIR)
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass
    try:
        with socketserver.TCPServer(('', HTTP_PORT), QuietHandler) as httpd:
            httpd.serve_forever()
    except Exception as e:
        print(f"HTTP Server warning: {e}")


async def main_async(node: AEMSWebBridgeNode):
    node.loop = asyncio.get_running_loop()
    async with websockets.serve(lambda ws: ws_handler(ws, node), '0.0.0.0', WS_PORT):
        node.get_logger().info(f'WebSocket server active on ws://0.0.0.0:{WS_PORT}')
        node.get_logger().info(f'Mission Control Dashboard live at http://localhost:{HTTP_PORT}')
        while rclpy.ok():
            await asyncio.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = AEMSWebBridgeNode()

    # Start HTTP static file server in daemon thread
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    # Run ROS2 spin in background thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    # Run asyncio WebSocket in main thread
    try:
        asyncio.run(main_async(node))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
