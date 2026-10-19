#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

msg = """
WASD Teleop Controller Active!
---------------------------
Moving around:
        w
   a    s    d

w : move forward
s : move backward
a : turn left
d : turn right
SPACE : force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'd': (0, 0, 0, -1),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.speed = 0.15
        self.turn = 0.30
        self.x = 0.0
        self.th = 0.0

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            print(msg)
            while rclpy.ok():
                key = getKey(settings)
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][3]
                elif key == ' ':
                    self.x = 0.0
                    self.th = 0.0
                else:
                    self.x = 0.0
                    self.th = 0.0
                    if (key == '\x03'):
                        break

                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.header.frame_id = 'base_link'
                twist.twist.linear.x = self.x * self.speed
                twist.twist.angular.z = self.th * self.turn
                self.pub.publish(twist)
        finally:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopWASD()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
