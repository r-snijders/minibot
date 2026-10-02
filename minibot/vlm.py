"""ROS camera adapter for an Ollama vision model; one request in flight."""
import base64
import time
from concurrent.futures import ThreadPoolExecutor
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
from minibot.ai_protocol import infer


class VLM(Node):
    def __init__(self):
        super().__init__('vlm')
        for name, value in [('endpoint', 'http://127.0.0.1:11434'),
                            ('model', 'qwen2.5vl:3b'), ('interval', 2.0),
                            ('request_timeout', 30.0), ('max_age', 15.0)]:
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.latest, self.future, self.started, self.last_request = None, None, 0., -1e9
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pub = self.create_publisher(String, '/perception/observations', 10)
        self.status = self.create_publisher(String, '/perception/status', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image, qos_profile_sensor_data)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def image(self, msg):
        self.latest = (msg, time.monotonic())

    def param(self, name):
        return self.get_parameter(name).value

    def tick(self):
        now = time.monotonic()
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result = self.future.result()
                age = now - self.started
                if age < self.param('max_age'):
                    result.update(age_seconds=age, stamp=self.frame_stamp, model=self.param('model'))
                    self.pub.publish(String(data=json.dumps(result)))
                    self.status.publish(String(data='ready'))
                else:
                    self.status.publish(String(data='discarded stale inference'))
            except Exception as exc:
                self.status.publish(String(data=f'inference unavailable: {type(exc).__name__}'))
                self.get_logger().warning(f'VLM request failed: {exc}')
            self.future = None
        if self.latest is None or now - self.last_request < self.param('interval'):
            return
        msg, received = self.latest
        self.latest = None
        if now - received > 1.0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            if w > 640:
                frame = cv2.resize(frame, (640, max(1, round(h * 640 / w))))
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise ValueError('JPEG encoding failed')
            image = base64.b64encode(encoded.tobytes()).decode('ascii')
            self.frame_stamp = {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}
            self.started, self.last_request = received, now
            self.future = self.worker.submit(infer, self.param('endpoint'), self.param('model'),
                                             image, self.param('request_timeout'))
        except Exception as exc:
            self.get_logger().warning(f'VLM image conversion failed: {exc}')

    def destroy_node(self):
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = VLM()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
