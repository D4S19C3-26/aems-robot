import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import sys

class FrameCapture(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.get_logger().info('Waiting for /camera/image_raw (sensor data QoS)...')

    def cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            out_path = '/home/dsc/.gemini/antigravity/brain/b53eaf2b-0ee7-4e0e-b0e5-bf83b37c44b6/scratch/gazebo_camera_frame.jpg'
            cv2.imwrite(out_path, cv_img)
            print("CAPTURE_SUCCESS")
        except Exception as e:
            print("CAPTURE_ERROR")
        finally:
            rclpy.shutdown()
            sys.exit(0)

rclpy.init()
node = FrameCapture()
rclpy.spin(node)
