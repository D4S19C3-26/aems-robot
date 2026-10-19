import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.bridge = CvBridge()
        self.get_logger().info('Waiting for /camera/image_raw...')

    def cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Save to the artifacts directory so the AI can embed it
            out_path = '/home/dsc/.gemini/antigravity/brain/b53eaf2b-0ee7-4e0e-b0e5-bf83b37c44b6/scratch/gazebo_camera_frame.jpg'
            cv2.imwrite(out_path, cv_img)
            self.get_logger().info(f'Saved frame to {out_path}')
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            rclpy.shutdown()
            sys.exit(0)

rclpy.init()
node = FrameCapture()
rclpy.spin(node)
