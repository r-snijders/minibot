"""Gazebo-only battery fixture: mock contact, independent of behavior state."""
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class SimBattery(Node):
    def __init__(self):
        super().__init__('sim_battery')
        self.declare_parameter('initial_soc', .8)
        self.declare_parameter('discharge_per_second', .002)
        self.declare_parameter('charge_per_second', .02)
        self.soc = float(self.get_parameter('initial_soc').value)
        self.odom = None
        self.received = -1.0
        self.last_time = self.get_clock().now().nanoseconds/1e9
        self.contact_since = None
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_subscription(Odometry, '/odom', self.update, 10)
        self.create_timer(.1, self.tick)

    def update(self, msg):
        self.odom, self.received = msg, time.monotonic()

    def tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        dt, self.last_time = max(0, min(.2, now-self.last_time)), now
        contact = False
        if self.odom is not None and time.monotonic()-self.received < .5:
            p, q, v = self.odom.pose.pose.position, self.odom.pose.pose.orientation, self.odom.twist.twist
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            contact = math.hypot(p.x-.85, p.y) < .025 and abs(yaw) < .12 and abs(v.linear.x) < .012 and abs(v.angular.z) < .05
        if not contact:
            self.contact_since = None
        elif self.contact_since is None:
            self.contact_since = now
        charging = self.contact_since is not None and now-self.contact_since > 1.0
        rate = self.get_parameter('charge_per_second' if charging else 'discharge_per_second').value
        self.soc = max(0.0, min(1.0, self.soc + rate*dt*(1 if charging else -1)))
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self.soc
        msg.present = True
        msg.voltage = float('nan')
        msg.current = float('nan')
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if charging and self.soc >= .999
                                  else BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging
                                  else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimBattery()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
