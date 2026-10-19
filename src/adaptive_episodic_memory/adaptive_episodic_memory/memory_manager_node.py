#!/usr/bin/env python3
"""
Memory Manager ROS 2 Node
Central service that other nodes will call to store / retrieve / decide.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from adaptive_episodic_memory.memory.memory_store import MemoryStore
from adaptive_episodic_memory.memory.retrieval import MemoryRetrieval
from adaptive_episodic_memory.reasoning.trust_verify import TrustVerifyReasoner


class MemoryManagerNode(Node):
    def __init__(self):
        super().__init__('memory_manager')

        # Initialize the memory system
        self.store = MemoryStore()          # uses default path ~/aems_ws/memory/...
        self.retrieval = MemoryRetrieval(self.store)
        self.reasoner = TrustVerifyReasoner(self.store, self.retrieval)

        # Simple topic interface for now (we will add proper services later)
        self.decision_pub = self.create_publisher(String, '/aems/decision', 10)
        self.command_sub = self.create_subscription(
            String,
            '/aems/command',
            self.command_callback,
            10
        )

        self.get_logger().info('Adaptive Episodic Memory Manager started')
        self.get_logger().info(f'Memory database: {self.store.db_path}')

    def command_callback(self, msg: String):
        """
        Expected command format (JSON string):
        {
            "type": "decide",
            "object_class": "bottle"
        }
        or
        {
            "type": "log_sighting",
            "object_class": "bottle",
            "landmark_id": 1,
            "offset": {"x": 0.3, "y": 0.1},
            "confidence": 0.9
        }
        """
        try:
            data = json.loads(msg.data)
            cmd_type = data.get("type")

            if cmd_type == "decide":
                object_class = data.get("object_class", "unknown")
                decision = self.reasoner.decide(object_class)
                self.get_logger().info(f'Decision for {object_class}: {decision["action"]}')
                
                out = String()
                out.data = json.dumps(decision)
                self.decision_pub.publish(out)

            elif cmd_type == "log_sighting":
                eid = self.store.add_object_sighting(
                    object_class=data["object_class"],
                    landmark_id=data["landmark_id"],
                    relative_offset=data.get("offset", {}),
                    confidence=data.get("confidence", 0.8)
                )
                self.get_logger().info(f'Logged sighting, episode_id={eid}')

            elif cmd_type == "log_attempt":
                aid = self.store.log_attempt(
                    task_type=data.get("task_type", "navigate-to-object"),
                    route_landmarks=data.get("route_landmarks", []),
                    outcome=data.get("outcome", "success"),
                    failure_landmark_id=data.get("failure_landmark_id"),
                    failure_reason=data.get("failure_reason")
                )
                self.get_logger().info(f'Logged attempt, attempt_id={aid}')

            else:
                self.get_logger().warn(f'Unknown command type: {cmd_type}')

        except Exception as e:
            self.get_logger().error(f'Failed to process command: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MemoryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.store.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
