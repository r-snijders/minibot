"""The only publisher to /drive/cmd_vel; fail closed on stale commands."""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.enabled = False
        self.heartbeat = -1.0
        self.values = {}
        self.pub = self.create_publisher(Twist, '/drive/cmd_vel', 10)
        self.create_subscription(Bool, '/behavior/enabled', self.mode, 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.record('manual', m), 10)
        self.create_subscription(Twist, '/behavior/cmd_vel', lambda m: self.record('auto', m), 10)
        self.create_timer(.05, self.tick)

    def mode(self, msg):
        if msg.data != self.enabled:
            self.values.clear()
        self.enabled, self.heartbeat = msg.data, time.monotonic()

    def record(self, source, msg):
        self.values[source] = (msg, time.monotonic())

    def tick(self):
        now = time.monotonic()
        msg, stamp = self.values.get('auto' if self.enabled else 'manual', (Twist(), -1.0))
        out = Twist()
        if now-stamp < .3 and (not self.enabled or now-self.heartbeat < .5):
            if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
                out.linear.x = max(-.15, min(.15, msg.linear.x))
                out.angular.z = max(-.6, min(.6, msg.angular.z))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VelocityGate()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
