#!/usr/bin/env python3
"""
Deterministic waypoint sweeper for SLAM mapping.
Waits for slam_toolbox to publish a valid map AND for the
map->base_footprint TF chain to exist before sending any goal.
Each waypoint has a 90-second timeout; failed ones are skipped.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# Per-waypoint timeout in seconds.
WAYPOINT_TIMEOUT_SEC = 90.0

# Wait up to this long for SLAM to be ready on startup.
SLAM_READY_TIMEOUT_SEC = 120.0

WAYPOINTS = [
    # Arena is 20x20m. Robot starts at (0,0).
    # Lawnmower sweep: 3 vertical columns, top-to-bottom and bottom-to-top alternating.
    # Staying 1.5m inside the outer walls (max ±8.5) to avoid tight corners.
    # Column 1: left side, sweep top to bottom
    (-7.0,  7.0),
    (-7.0,  0.0),
    (-7.0, -7.0),
    # Column 2: centre-left, sweep bottom to top
    (-3.0, -7.0),
    (-3.0,  0.0),
    (-3.0,  7.0),
    # Column 3: centre-right, sweep top to bottom
    ( 3.0,  7.0),
    ( 3.0,  0.0),
    ( 3.0, -7.0),
    # Column 4: right side, sweep bottom to top
    ( 7.0, -7.0),
    ( 7.0,  0.0),
    ( 7.0,  7.0),
    # Return home
    ( 0.0,  0.0),
]


class WaypointSweeper(Node):
    def __init__(self):
        super().__init__('waypoint_sweeper')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._idx = 0
        self._goal_handle = None
        self._timeout_timer = None
        self._lock = threading.Lock()

        # TF listener to check map → base_footprint
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Map readiness flag
        self._map_ready = False
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 1
        )

    # ─────────────────────────── startup checks ───────────────────────────
    def _map_callback(self, msg: OccupancyGrid):
        if not self._map_ready:
            # Check that the map actually has some data (not all -1/unknown)
            non_unknown = sum(1 for v in msg.data if v >= 0)
            if non_unknown > 100:
                self.get_logger().info(
                    f'Map received with {non_unknown} known cells. SLAM is producing data.'
                )
                self._map_ready = True

    def wait_for_slam_ready(self):
        """Block until SLAM has a map AND the TF chain is valid."""
        self.get_logger().info('Waiting for slam_toolbox to produce a map...')
        start = time.time()

        while time.time() - start < SLAM_READY_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.5)

            if not self._map_ready:
                continue

            # Check TF chain: map → odom → base_footprint
            try:
                self._tf_buffer.lookup_transform(
                    'map', 'base_footprint',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1)
                )
                self.get_logger().info(
                    'TF map → base_footprint is valid! '
                    'Waiting 5 extra seconds for Nav2 costmaps to inflate...'
                )
                time.sleep(5.0)
                return True
            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().info(
                    'Map ready, waiting for TF map → base_footprint...', throttle_duration_sec=5.0
                )

        self.get_logger().error(
            f'SLAM not ready after {SLAM_READY_TIMEOUT_SEC}s! '
            'Is slam_toolbox running and lidar data flowing?'
        )
        return False

    # ─────────────────────────── goal sending ────────────────────────────
    def send_next_goal(self):
        with self._lock:
            if self._idx >= len(WAYPOINTS):
                self.get_logger().info('╔══════════════════════════════════════╗')
                self.get_logger().info('║  SWEEP COMPLETE! All waypoints done. ║')
                self.get_logger().info('╚══════════════════════════════════════╝')
                self.get_logger().info('Now run: ros2 run nav2_map_server map_saver_cli -f ~/aems_ws/maps/arena_map')
                rclpy.shutdown()
                return

            x, y = WAYPOINTS[self._idx]

        self.get_logger().info(
            f'[{self._idx + 1}/{len(WAYPOINTS)}] → ({x:.2f}, {y:.2f})  '
            f'[timeout {WAYPOINT_TIMEOUT_SEC:.0f}s]'
        )

        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    # ─────────────────────────── callbacks ───────────────────────────────
    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'WP {self._idx + 1} rejected by Nav2. Skipping.')
            self._advance()
            return

        with self._lock:
            self._goal_handle = handle

        self._timeout_timer = self.create_timer(WAYPOINT_TIMEOUT_SEC, self._on_timeout)
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._cancel_timeout()
        status = future.result().status  # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if status == 4:
            self.get_logger().info(f'WP {self._idx + 1} reached ✓')
        else:
            self.get_logger().warn(
                f'WP {self._idx + 1} ended with status {status}. Skipping.'
            )
        self._advance()

    def _on_timeout(self):
        self.get_logger().warn(
            f'WP {self._idx + 1} TIMED OUT after {WAYPOINT_TIMEOUT_SEC:.0f}s. Cancelling & skipping.'
        )
        self._cancel_timeout()
        with self._lock:
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        self._advance()

    def _cancel_timeout(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _advance(self):
        self.get_logger().info('Pausing 3s for SLAM to settle...')
        time.sleep(3.0)
        with self._lock:
            self._idx += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSweeper()

    # Block until SLAM is genuinely ready before sending any goals
    if not node.wait_for_slam_ready():
        node.destroy_node()
        rclpy.shutdown()
        return

    node.send_next_goal()
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
