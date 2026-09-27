"""Offline baseline person detector using OpenCV's bundled HOG classifier."""
import time
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.last_frame, self.last_run, self.hits = None, -1.0, 0
        self.received = -1.0
        self.pub = self.create_publisher(Bool, '/person_detected', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image, qos_profile_sensor_data)
        self.create_timer(.33, self.detect)

    def image(self, msg):
        self.last_frame, self.received = msg, time.monotonic()

    def detect(self):
        if self.last_frame is None or time.monotonic()-self.received > 1:
            self.hits = 0
            self.pub.publish(Bool(data=False))
            return
        # Consume each image at most once; duplicated frames do not confirm a person.
        msg, self.last_frame = self.last_frame, None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            if width > 640:
                frame = cv2.resize(frame, (640, round(height*640/width)))
            if frame.shape[0] < 128 or frame.shape[1] < 64:
                raise ValueError('Image too small for HOG detector')
            boxes, weights = self.hog.detectMultiScale(frame, winStride=(8, 8),
                                                      padding=(8, 8), scale=1.05)
            found = len(boxes) > 0 and any(float(w) >= .5 for w in weights)
            self.hits = self.hits+1 if found else 0
            self.pub.publish(Bool(data=self.hits >= 3))
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self.hits = 0
            self.pub.publish(Bool(data=False))
            self.get_logger().warning(f'Camera detection failed: {exc}')


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
