#!/usr/bin/env python3
"""
Quick diagnostic: saves every 30th camera frame as a JPEG + runs YOLO on it.
Use this while teleoping to confirm the camera-to-YOLO pipeline is working.
Saved images go to ~/aems_ws/debug_frames/
"""
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

SAVE_DIR = os.path.expanduser('~/aems_ws/debug_frames')

class YoloDiag(Node):
    def __init__(self):
        super().__init__('yolo_diag')
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.model = YOLO(os.path.expanduser('~/aems_ws/yolov8n.pt'))
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb, sensor_qos)
        self.count = 0
        self.get_logger().info('Diagnostic YOLO running — drive the robot around!')

    def cb(self, msg):
        self.count += 1
        if self.count % 30 != 0:  # process every 30th frame (~1/sec)
            return
        ch = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding, 3)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        results = self.model(img, verbose=False, conf=0.30)[0]  # lower threshold

        # Draw boxes and save
        for box in results.boxes:
            conf = float(box.conf[0])
            cls = self.model.names[int(box.cls[0])]
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f'{cls} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self.get_logger().info(f'DETECTED: {cls} ({conf:.2f})')

        fpath = os.path.join(SAVE_DIR, f'frame_{self.count:05d}.jpg')
        cv2.imwrite(fpath, img)
        if not results.boxes:
            self.get_logger().info(f'Frame {self.count}: no detections (saved to {fpath})')
        else:
            self.get_logger().info(f'Frame {self.count}: {len(results.boxes)} det(s) saved to {fpath}')

def main():
    rclpy.init()
    node = YoloDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
