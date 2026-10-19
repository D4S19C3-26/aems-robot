#!/usr/bin/env python3
"""
Twist → TwistStamped relay node.
Bridges BOTH:
  - /cmd_vel_raw (teleop) → /cmd_vel_nav (TwistStamped, for Gazebo bridge)
  - /cmd_vel     (Nav2)   → /cmd_vel_nav (TwistStamped, for Gazebo bridge)

Nav2's controller_server publishes plain Twist on /cmd_vel.
The Gazebo Harmonic bridge expects TwistStamped on /cmd_vel.
This relay intercepts Nav2's Twist, wraps it in a stamp, and republishes
as TwistStamped on /cmd_vel_nav which the Gazebo bridge forwards to Gazebo.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.declare_parameter('max_linear_x', 0.50)
        self.declare_parameter('max_angular_z', 1.00)
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value

        # Publisher: TwistStamped on /cmd_vel_nav.
        # custom_spawn.launch.py remaps the TurtleBot3 bridge's /cmd_vel input
        # to this topic, so this is the command path Gazebo actually listens to.
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)

        # Source 1: teleop publishes to /cmd_vel_raw
        self.sub_raw = self.create_subscription(
            Twist, '/cmd_vel_raw', self.callback, 10)

        # Source 2: Nav2 controller_server publishes Twist to /cmd_vel
        self.sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10)

        self.get_logger().info(
            'TwistRelay: /cmd_vel_raw + /cmd_vel (Twist) -> /cmd_vel_nav '
            f'(TwistStamped), clamped to {self.max_linear_x:.2f} m/s and '
            f'{self.max_angular_z:.2f} rad/s')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist.linear.x = clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        stamped.twist.linear.y = 0.0
        stamped.twist.linear.z = 0.0
        stamped.twist.angular.x = 0.0
        stamped.twist.angular.y = 0.0
        stamped.twist.angular.z = clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)
        self.pub.publish(stamped)



def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
