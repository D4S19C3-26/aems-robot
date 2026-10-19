#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os, time, collections

MODEL_PATH   = os.path.expanduser('~/aems_ws/yolov8s.pt')
CAMERA_TOPIC = '/camera/image_raw'
CAPTURE_SECS = 12
CONF_THRESH  = 0.20

class YOLOProbeNode(Node):
    def __init__(self):
        super().__init__('yolo_probe')
        self.declare_parameter('target_class', 'unknown')
        self.target_class = self.get_parameter('target_class').value.lower()
        self.bridge   = CvBridge()
        self.model    = YOLO(MODEL_PATH)
        self.model.fuse()
        self.detections = collections.defaultdict(list)
        self.frame_count = 0
        self.start_time  = time.time()
        self.done        = False
        self._sub = self.create_subscription(Image, CAMERA_TOPIC, self._cb, 5)
        self.get_logger().info(f'Probing "{self.target_class}" for {CAPTURE_SECS}s ...')

    def _cb(self, msg):
        if self.done:
            return
        if time.time() - self.start_time > CAPTURE_SECS:
            self._print_summary()
            self.done = True
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return
        results = self.model.predict(cv_img, conf=CONF_THRESH, verbose=False)[0]
        self.frame_count += 1
        for box in results.boxes:
            cls_name = results.names[int(box.cls)].lower()
            self.detections[cls_name].append(float(box.conf))

    def _print_summary(self):
        print('\n' + '='*64)
        print(f'TARGET: "{self.target_class}"  |  Frames: {self.frame_count}')
        print('-'*64)
        print(f'{"Predicted class":<24} {"Hits":>6} {"MaxConf":>9} {"AvgConf":>9}')
        print('-'*64)
        if not self.detections:
            print('   *** NOTHING DETECTED ***')
        else:
            for cls, confs in sorted(self.detections.items(), key=lambda kv: -len(kv[1])):
                flag = '  <- TARGET' if cls == self.target_class else ''
                print(f'{cls:<24} {len(confs):>6} {max(confs):>9.2f} {sum(confs)/len(confs):>9.2f}{flag}')
        print('='*64)
        if self.target_class in self.detections:
            best = max(self.detections[self.target_class])
            verdict = 'PASS' if best >= 0.5 else f'FAIL (weak, max={best:.2f})'
        elif self.detections:
            top_cls, top_confs = max(self.detections.items(), key=lambda kv: len(kv[1]))
            verdict = f'FAIL: misclassified as "{top_cls}" (max={max(top_confs):.2f})'
        else:
            verdict = 'FAIL: invisible to YOLO'
        print(f'  VERDICT: {verdict}\n')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOProbeNode()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
