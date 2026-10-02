"""Deterministic semantic supervisor; navigation owns movement and greetings.

No model text is executed. Valid, fresh observations produce a person signal,
which the existing autonomy supervisor uses to stop, greet and wait ten seconds.
"""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from minibot.ai_protocol import ObservationMemory


class Reasoning(Node):
    def __init__(self):
        super().__init__('reasoning')
        self.declare_parameter('observation_ttl', 15.0)
        self.memory = ObservationMemory(self.get_parameter('observation_ttl').value)
        self.behavior = 'waiting for navigation'
        self.people = self.create_publisher(Bool, '/person_detected', 10)
        self.decisions = self.create_publisher(String, '/reasoning/status', 10)
        self.create_subscription(String, '/perception/observations', self.observe, 10)
        self.create_subscription(String, '/behavior/status', self.on_behavior, 10)
        self.create_timer(.2, self.tick)

    def observe(self, msg):
        try:
            self.memory.accept(json.loads(msg.data), time.monotonic())
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'Rejected observation: {exc}')

    def on_behavior(self, msg):
        self.behavior = msg.data

    def tick(self):
        visible = self.memory.visible(time.monotonic())
        self.people.publish(Bool(data=visible))
        self.decisions.publish(String(data=json.dumps({
            'intent': 'greet_person' if visible else 'continue_mission',
            'observation_fresh': time.monotonic() < self.memory.until,
            'description': self.memory.description,
            'navigation': self.behavior})))


def main():
    rclpy.init()
    node = Reasoning()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
