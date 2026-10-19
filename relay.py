#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay_manual')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.get_logger().info("Relay started: /cmd_vel -> /cmd_vel_nav")

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)

rclpy.init()
node = TwistRelay()
rclpy.spin(node)
