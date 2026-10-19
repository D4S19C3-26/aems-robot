#!/usr/bin/env python3
"""
Smart Auto Explorer
- Uses correct laser scan angle math (not a naive thirds split)
- Follows walls to systematically cover rooms
- Detects when stuck and performs recovery spin
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class SmartAutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # State
        self.state = 'forward'     # forward | turn_left | turn_right | recover
        self.last_x = None
        self.last_y = None
        self.last_moved_time = time.time()
        self.recovery_start = None
        self.recovery_duration = 0.0

        # Tuning
        self.LINEAR_SPEED  = 0.4    # m/s forward
        self.ANGULAR_SPEED = 0.8    # rad/s turning
        self.FRONT_CLEAR   = 0.55   # metres — obstacle threshold ahead
        self.SIDE_CLEAR    = 0.30   # metres — side wall threshold
        self.STUCK_TIMEOUT = 4.0    # seconds before declaring stuck

        self.get_logger().info(
            'Smart Auto Explorer running! '
            'Robot will explore and map the house automatically.'
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:
            self.last_x, self.last_y = x, y
            return

        moved = math.hypot(x - self.last_x, y - self.last_y)
        if moved > 0.05:                     # moved at least 5 cm
            self.last_x, self.last_y = x, y
            self.last_moved_time = time.time()

    def get_sector_min(self, ranges, angle_min, angle_inc,
                       sector_start_deg, sector_end_deg):
        """Return the minimum valid range in the given angular sector."""
        start_idx = max(0, int((math.radians(sector_start_deg) - angle_min) / angle_inc))
        end_idx   = min(len(ranges) - 1,
                        int((math.radians(sector_end_deg) - angle_min) / angle_inc))
        if start_idx >= end_idx:
            return float('inf')
        sector = [r for r in ranges[start_idx:end_idx]
                  if not math.isnan(r) and not math.isinf(r) and r > 0.05]
        return min(sector) if sector else float('inf')

    def scan_callback(self, msg: LaserScan):
        # ── Sector distances ─────────────────────────────────────────────────
        a_min = msg.angle_min
        a_inc = msg.angle_increment
        r     = msg.ranges

        front      = self.get_sector_min(r, a_min, a_inc, -30,  30)
        front_left = self.get_sector_min(r, a_min, a_inc,  30,  60)
        front_right= self.get_sector_min(r, a_min, a_inc, -60, -30)
        left       = self.get_sector_min(r, a_min, a_inc,  60,  90)
        right      = self.get_sector_min(r, a_min, a_inc, -90, -60)

        cmd = Twist()
        now = time.time()

        # ── Stuck detection / recovery ────────────────────────────────────────
        if self.state != 'recover':
            if now - self.last_moved_time > self.STUCK_TIMEOUT:
                self.get_logger().warn('STUCK! Starting recovery spin...')
                self.state = 'recover'
                self.recovery_start = now
                # Spin for 2-3 seconds, direction toward the more open side
                self.recovery_duration = 2.5
                self._recovery_dir = 1.0 if left > right else -1.0

        if self.state == 'recover':
            elapsed = now - self.recovery_start
            if elapsed < self.recovery_duration:
                # Spin in place
                cmd.linear.x  = 0.0
                cmd.angular.z = self.ANGULAR_SPEED * self._recovery_dir
            else:
                self.get_logger().info('Recovery done, resuming forward.')
                self.state = 'forward'
                self.last_moved_time = now
            self.pub.publish(cmd)
            return

        # ── Normal navigation ─────────────────────────────────────────────────
        if front < self.FRONT_CLEAR:
            # Obstacle ahead — turn toward the more open side
            if front_left > front_right:
                self.state = 'turn_left'
            else:
                self.state = 'turn_right'

        elif front_left < self.SIDE_CLEAR * 0.8:
            # Wall too close on the front-left, nudge right
            self.state = 'turn_right'

        elif front_right < self.SIDE_CLEAR * 0.8:
            # Wall too close on the front-right, nudge left
            self.state = 'turn_left'

        else:
            self.state = 'forward'

        if self.state == 'forward':
            cmd.linear.x  = self.LINEAR_SPEED
            # Gentle right-wall following: keep right wall at ~0.4 m
            if right < 0.4:
                cmd.angular.z =  0.15   # drift left a little
            elif right > 0.7 and front > 1.0:
                cmd.angular.z = -0.15   # drift right toward the wall
            else:
                cmd.angular.z =  0.0

        elif self.state == 'turn_left':
            cmd.linear.x  = 0.05
            cmd.angular.z = self.ANGULAR_SPEED

        elif self.state == 'turn_right':
            cmd.linear.x  = 0.05
            cmd.angular.z = -self.ANGULAR_SPEED

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SmartAutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot cleanly
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
