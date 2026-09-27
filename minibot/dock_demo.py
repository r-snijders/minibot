"""Simulation-only charging and final approach to a fixed, unobstructed dock.

This deliberately does not represent electrical charging or general navigation.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String


class DockDemo(Node):
    def __init__(self):
        super().__init__('dock_demo')
        self.declare_parameter('initial_soc', 0.8)
        self.soc = float(self.get_parameter('initial_soc').value)
        self.soc = max(0.0, min(1.0, self.soc))
        self.pose = None
        self.scan = None
        self.state = 'IDLE'
        self.started = None
        self.last_tick = time.monotonic()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery = self.create_publisher(BatteryState, '/battery_state', 10)
        self.status = self.create_publisher(String, '/dock/status', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Bool, '/dock/request', self.on_request, 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = p.x, p.y, yaw

    def on_scan(self, msg):
        self.scan = msg

    def on_request(self, msg):
        if msg.data and self.state in ('IDLE', 'FAULT'):
            self.state = 'APPROACH'
            self.started = time.monotonic()
        elif not msg.data and self.state == 'APPROACH':
            self.stop()
            self.state = 'IDLE'

    def stop(self):
        self.cmd.publish(Twist())

    def obstacle(self, expected_dock_range):
        if self.scan is None:
            return False
        scan = self.scan
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) < 0.3 and math.isfinite(distance):
                if scan.range_min <= distance < expected_dock_range - 0.12:
                    return True
        return False

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        if self.state == 'IDLE' and self.soc < 0.25 and self.pose is not None:
            self.state, self.started = 'APPROACH', now

        if self.state == 'APPROACH' and self.pose is not None:
            x, y, yaw = self.pose
            # The world has a dock face at x=0.96; the robot front is ~0.09 m
            # ahead of base_link. The mock contact pose is x=0.85, y=0.
            if now - self.started > 30 or x > 0.9 or abs(y) > 0.35 or self.obstacle(0.87 - x):
                self.stop()
                self.state = 'FAULT'
            elif x >= 0.84 and abs(y) < 0.035 and abs(yaw) < 0.1:
                self.stop()
                self.state = 'CHARGING'
            else:
                command = Twist()
                if abs(yaw) > 0.3:
                    command.angular.z = -0.45 if yaw > 0 else 0.45
                else:
                    command.angular.z = max(-0.4, min(0.4, -2.5 * y - 0.8 * yaw))
                    command.linear.x = min(0.10, max(0.025, 0.5 * (0.85 - x)))
                self.cmd.publish(command)

        if self.state == 'CHARGING':
            self.soc = min(1.0, self.soc + dt / 120.0)  # accelerated mock charge
        elif self.state != 'FAULT':
            self.soc = max(0.0, self.soc - dt / 3600.0)
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.percentage = self.soc
        battery.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                       if self.state == 'CHARGING' else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        battery.present = True
        self.battery.publish(battery)
        self.status.publish(String(data=self.state))


def main():
    rclpy.init()
    node = DockDemo()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
