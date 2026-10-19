#!/usr/bin/env python3
"""Pass pose-aware perception events into the episodic-memory command stream."""

import json
import math

import rclpy
from aems_interfaces.msg import ObjectDetection
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class PerceptionBridgeNode(Node):
    def __init__(self):
        super().__init__('perception_bridge')
        self.sub_detections = self.create_subscription(
            ObjectDetection, '/aems/detections', self.detections_callback, 10)
        self.pub_command = self.create_publisher(String, '/aems/command', 10)
        self.landmark_grid_size = 2.0
        self.get_logger().info('Perception bridge ready for pose-aware detections.')

    def landmark_id(self, x, y):
        return ((int(math.floor(x / self.landmark_grid_size)) + 1000) * 10000
                + int(math.floor(y / self.landmark_grid_size)) + 1000)

    def detections_callback(self, detection):
        pose = detection.robot_pose
        if pose.header.frame_id == 'unknown':
            self.get_logger().warn('Ignoring detection: no localization pose received yet.')
            return
        x, y = pose.pose.position.x, pose.pose.position.y
        command = {
            'type': 'log_sighting',
            'object_class': detection.class_name,
            'landmark_id': self.landmark_id(x, y),
            'offset': {'x': 1.0, 'y': 0.0},
            'confidence': detection.confidence,
            'detection_stamp': {
                'sec': detection.header.stamp.sec,
                'nanosec': detection.header.stamp.nanosec,
            },
            'robot_pose': {'frame_id': pose.header.frame_id, 'x': x, 'y': y},
        }
        output = String()
        output.data = json.dumps(command)
        self.pub_command.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
