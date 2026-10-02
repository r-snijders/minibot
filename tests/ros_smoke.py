"""Container integration check: real Gazebo/ROS, mock HTTP model (no weights)."""
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


class ModelHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        payload = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        assert base64.b64decode(payload['images'][0]).startswith(b'\xff\xd8')
        data = json.dumps({'done': True, 'response': json.dumps({
            'person_visible': True, 'description': 'Synthetic integration test result'})}).encode()
        self.send_response(200)
        self.end_headers()
        self.wfile.write(data)
    def log_message(self, *args):
        pass


def main():
    server = HTTPServer(('127.0.0.1', 11434), ModelHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rclpy.init()
    node = rclpy.create_node('minibot_smoke')
    seen, positions = set(), []
    subscriptions = []
    for topic, msg_type in [('/camera/image_raw', Image), ('/scan', LaserScan), ('/clock', Clock)]:
        subscriptions.append(node.create_subscription(msg_type, topic,
            lambda msg, t=topic: seen.add(t), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Odometry, '/odom',
        lambda msg: positions.append(msg.pose.pose.position.x), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Bool, '/person_detected',
        lambda msg: seen.add('person') if msg.data else None, 10))
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    start = time.monotonic()
    try:
        while time.monotonic() - start < 90:
            rclpy.spin_once(node, timeout_sec=.05)
            command = Twist()
            if positions and '/scan' in seen:
                command.linear.x = .08
            pub.publish(command)
            if {'/camera/image_raw', '/scan', '/clock', 'person'} <= seen and positions and max(positions)-min(positions) > .03:
                print('PASS: clock, camera, lidar, motion, HTTP VLM and reasoning')
                return
        raise RuntimeError(f'Smoke timeout: seen={seen}, odometry samples={len(positions)}')
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
