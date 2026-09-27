"""Speak on the machine running this node using its configured audio output."""
import shutil
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Speech(Node):
    def __init__(self):
        super().__init__('speech')
        self.executable = shutil.which('espeak-ng')
        self.process = None
        if not self.executable:
            self.get_logger().warning('Install espeak-ng for audible greetings; text remains on /speech/text')
        self.create_subscription(String, '/speech/text', self.say, 10)

    def say(self, msg):
        self.get_logger().info(msg.data)
        if self.executable and (self.process is None or self.process.poll() is not None):
            self.process = subprocess.Popen([self.executable, '--', msg.data[:200]],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def destroy_node(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = Speech()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
