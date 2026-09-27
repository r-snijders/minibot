import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import serial

from minibot.geometry import TICKS_PER_REV, WHEEL_RADIUS, WHEEL_SEPARATION


class Hardware(Node):
    def __init__(self):
        super().__init__('rover_hardware')
        self.declare_parameter('port', '/dev/ttyACM0')
        port = self.get_parameter('port').value
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.ser.reset_input_buffer()
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.poll)
        self.x = self.y = self.yaw = 0.0
        self.last_ticks = None
        self.last_sample = None
        self.last_command = 0.0
        self.target = (0.0, 0.0)
        self.buffer = bytearray()

    def command(self, msg):
        v = max(-0.25, min(0.25, msg.linear.x))
        w = max(-1.5, min(1.5, msg.angular.z))
        self.target = ((v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS,
                       (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS)
        self.last_command = time.monotonic()

    def poll(self):
        target = self.target if time.monotonic() - self.last_command < 0.4 else (0.0, 0.0)
        try:
            self.ser.write(f'V {target[0]:.3f} {target[1]:.3f}\n'.encode())
            self.buffer.extend(self.ser.read(min(self.ser.in_waiting, 1024)))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial link failed: {exc}')
            rclpy.shutdown()
            return
        if len(self.buffer) > 2048:
            self.buffer.clear()
        while b'\n' in self.buffer:
            line, _, self.buffer = self.buffer.partition(b'\n')
            fields = line.split()
            if len(fields) != 3 or fields[0] != b'T':
                continue
            try:
                self.update_odom(int(fields[1]), int(fields[2]))
            except ValueError:
                continue

    def update_odom(self, left, right):
        now = self.get_clock().now()
        if self.last_ticks is None:
            self.last_ticks, self.last_sample = (left, right), now
            return
        dt = (now - self.last_sample).nanoseconds / 1e9
        if dt <= 0:
            return
        dl = (left - self.last_ticks[0]) * 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV
        dr = (right - self.last_ticks[1]) * 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV
        ds, dyaw = (dr + dl) / 2, (dr - dl) / WHEEL_SEPARATION
        self.x += ds * math.cos(self.yaw + dyaw / 2)
        self.y += ds * math.sin(self.yaw + dyaw / 2)
        self.yaw += dyaw
        self.last_ticks, self.last_sample = (left, right), now
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        stamp = now.to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x = ds / dt
        msg.twist.twist.angular.z = dyaw / dt
        self.odom_pub.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z, transform.transform.rotation.w = qz, qw
        self.tf.sendTransform(transform)

    def destroy_node(self):
        try:
            self.ser.write(b'V 0 0\n')
            self.ser.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = Hardware()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
