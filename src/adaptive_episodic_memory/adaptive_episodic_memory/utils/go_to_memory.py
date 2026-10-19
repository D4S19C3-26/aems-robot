#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from aems_interfaces.srv import QueryMemory

class GoToMemoryClient(Node):
    def __init__(self, target_object):
        super().__init__('go_to_memory_client')
        self.target_object = target_object
        
        self.cli = self.create_client(QueryMemory, '/query_memory')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /query_memory service...')
            
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def query_and_go(self):
        req = QueryMemory.Request()
        req.object_class = self.target_object
        self.get_logger().info(f"Asking memory manager where '{self.target_object}' is...")
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if not response.success:
            self.get_logger().error(response.message)
            return

        self.get_logger().info(f"Memory found! Object is at ({response.pose.pose.position.x:.2f}, {response.pose.pose.position.y:.2f}). Navigating there...")
        
        # Now send goal to Nav2
        while not self.nav_cli.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('waiting for Nav2 navigate_to_pose action server...')
            rclpy.spin_once(self, timeout_sec=0.5)
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = response.pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        
        self.get_logger().info(f'Sending navigation goal to ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})...')
        send_goal_future = self.nav_cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted, navigating...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        status = get_result_future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run adaptive_episodic_memory go_to_memory <object_class>")
        return
        
    target = sys.argv[1]
    rclpy.init(args=args)
    node = GoToMemoryClient(target)
    node.query_and_go()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
