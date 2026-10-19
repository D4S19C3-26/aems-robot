#!/usr/bin/env python3
import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = [
            (3.0, 0.0), (3.0, 3.0), (-3.0, 3.0),
            (-3.0, -3.0), (3.0, -3.0), (0.0, 0.0)
        ]
        
    def run(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        
        # Give SLAM and Nav2 a few seconds to fully initialize the TF tree
        self.get_logger().info('Nav2 connected. Waiting 5 seconds for SLAM TF tree to settle...')
        time.sleep(5.0)

        for x, y in self.waypoints:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(f'---> Navigating to ({x}, {y})')
            
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error('Goal was REJECTED by Nav2 (possibly map frame not ready).')
                time.sleep(2)
                continue

            get_result_future = goal_handle.get_result_async()
            
            # Proper blocking wait for the result
            self.get_logger().info('Goal accepted, driving...')
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)
            
            if get_result_future.done():
                status = get_result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f'Success! Reached ({x}, {y})')
                else:
                    self.get_logger().warn(f'Navigation failed or aborted with status: {status}')
            else:
                self.get_logger().warn('Timeout reached (60s), skipping to next waypoint.')
                goal_handle.cancel_goal_async()
                time.sleep(2)
            
        self.get_logger().info('Sweep complete. Creating maps directory and saving map...')
        os.makedirs('/home/dsc/aems_ws/maps', exist_ok=True)
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', '/home/dsc/aems_ws/maps/arena_map'])
        self.get_logger().info('Map saved to /home/dsc/aems_ws/maps/arena_map.yaml')

def main():
    rclpy.init()
    node = AutoMapper()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
