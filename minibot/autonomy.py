"""Mapping patrol, dock return, charging, and greeting supervisor.

Owns all autonomous movement. Uses a conservative grid planner for this small
prototype; it is not a replacement for a validated navigation stack.
"""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from minibot.planning import Grid, Greeting, wrap


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Autonomy(Node):
    def __init__(self):
        super().__init__('autonomy')
        for key, value in [('simulation', True), ('low_battery', .25),
                           ('critical_battery', .10), ('resume_battery', .90),
                           ('dock_file', '~/.local/state/minibot/dock.json'),
                           ('restore_dock', False)]:
            self.declare_parameter(key, value)
        self.enabled, self.state, self.detail = True, 'EXPLORING', 'Waiting for inputs'
        self.grid = self.scan = self.battery = None
        self.scan_at = self.battery_at = self.map_at = -1.0
        self.dock = None
        self.dock_file = Path(self.get_parameter('dock_file').value).expanduser()
        if self.get_parameter('restore_dock').value:
            try:
                saved = json.loads(self.dock_file.read_text())
                values = [float(saved[k]) for k in ('x', 'y', 'yaw')]
                if saved['frame'] != 'map' or not all(math.isfinite(v) for v in values):
                    raise ValueError('Invalid dock coordinates')
                self.dock = tuple(values)
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().error(f'Cannot restore dock: {exc}')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.greeting = Greeting()
        self.person = False
        self.person_at = -1.0
        self.resume_state = 'EXPLORING'
        self.path, self.visited = [], []
        self.goal = None
        self.next_plan = 0.0
        self.entered = time.monotonic()
        self.blocked_since = None
        self.last_progress = self.entered
        self.progress_pose = None
        self.dock_retries = 0
        self.retry_docking = False
        self.resume_after_charge = 'EXPLORING'
        self.cmd_pub = self.create_publisher(Twist, '/behavior/cmd_vel', 10)
        self.enabled_pub = self.create_publisher(Bool, '/behavior/enabled', 10)
        self.status_pub = self.create_publisher(String, '/behavior/status', 10)
        self.say_pub = self.create_publisher(String, '/speech/text', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/dock/markers', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
        self.create_subscription(Bool, '/person_detected', self.on_person, 10)
        self.create_service(Trigger, '/behavior/mark_dock', self.mark_dock)
        self.create_service(SetBool, '/behavior/enable', self.enable)
        self.create_timer(.1, self.tick)

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or msg.info.resolution <= 0:
            return
        p, q = msg.info.origin.position, msg.info.origin.orientation
        self.grid = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                         (p.x, p.y, yaw_of(q)), msg.data)
        self.map_at = time.monotonic()
        self.next_plan = 0.0

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def on_battery(self, msg):
        if msg.present and math.isfinite(msg.percentage) and 0 <= msg.percentage <= 1:
            self.battery, self.battery_at = msg, time.monotonic()

    def on_person(self, msg):
        self.person, self.person_at = msg.data, time.monotonic()
        self.greeting.observe(msg.data, self.person_at)

    def enable(self, request, response):
        self.enabled = request.data
        if self.enabled:
            self.transition('EXPLORING')
        self.cmd_pub.publish(Twist())
        self.enabled_pub.publish(Bool(data=self.enabled))
        response.success = True
        response.message = 'Autonomy enabled' if self.enabled else 'Autonomy disabled; manual input enabled'
        return response

    def pose(self, source='base_link'):
        transform = self.tf.lookup_transform('map', source, Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
        if self.get_clock().now().nanoseconds/1e9-stamp > 1.0:
            raise ValueError('Stale transform')
        t, q = transform.transform.translation, transform.transform.rotation
        return t.x, t.y, yaw_of(q)

    def save_dock(self):
        x, y, yaw = self.dock
        self.dock_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.dock_file.with_suffix('.tmp')
        tmp.write_text(json.dumps({'frame': 'map', 'x': x, 'y': y, 'yaw': yaw}, indent=2)+'\n')
        tmp.replace(self.dock_file)
        arrow = Marker()
        arrow.header.frame_id, arrow.ns, arrow.id = 'map', 'dock', 0
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.pose.position.x, arrow.pose.position.y = x, y
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .25, .06, .06
        arrow.color.r, arrow.color.g, arrow.color.a = 1.0, .4, 1.0
        label = Marker()
        label.header.frame_id, label.ns, label.id = 'map', 'dock', 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, .3
        label.pose.orientation.w = 1.0
        label.scale.z, label.color.r, label.color.g, label.color.b, label.color.a = .12, 1.0, 1.0, 1.0, 1.0
        label.text = 'Charging base'
        self.markers.publish(MarkerArray(markers=[arrow, label]))

    def mark_dock(self, request, response):
        del request
        if self.enabled:
            response.success, response.message = False, 'Disable autonomy and place robot at its charging pose first'
            return response
        try:
            self.dock = self.pose()
            self.save_dock()
            response.success, response.message = True, 'Saved current map pose as charging pose'
        except (TransformException, OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def transition(self, state, detail=''):
        self.state, self.detail, self.entered = state, detail, time.monotonic()
        self.goal, self.path, self.next_plan = None, [], 0.0
        self.last_progress, self.progress_pose = self.entered, None
        self.blocked_since = None

    def staging(self):
        x, y, yaw = self.dock
        return x-.35*math.cos(yaw), y-.35*math.sin(yaw), yaw

    def navigate(self, pose, target, now):
        x, y, yaw = pose
        if now >= self.next_plan:
            self.path = self.grid.route(self.grid.cell(x, y), self.grid.cell(*target))
            self.next_plan = now+2
        while self.path and math.hypot(self.path[0][0]-x, self.path[0][1]-y) < .065:
            self.path.pop(0)
        if not self.path:
            return 0.0, 0.0
        px, py = self.path[0]
        error = wrap(math.atan2(py-y, px-x)-yaw)
        return (.10 if abs(error) < .3 else 0.0), max(-.5, min(.5, 1.5*error))

    def scan_clear(self, v, w, dock_range=None):
        if v == 0 and w == 0:
            return True
        scan = self.scan
        if scan is None or scan.header.frame_id != 'laser_frame':
            return False
        valid = 0
        for i, distance in enumerate(scan.ranges):
            angle = wrap(scan.angle_min+i*scan.angle_increment)
            relevant = abs(w) > .1 or abs(wrap(angle-(math.pi if v < 0 else 0))) < .65
            if not relevant:
                continue
            if math.isinf(distance) and distance > 0:
                valid += 1
            elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                valid += 1
                # Dock face is intentionally approached below general clearance.
                # Final approach must additionally be guarded by real bump/contact
                # sensors on hardware; this lidar has a close-range blind zone.
                if dock_range is not None and v > 0 and abs(angle) < .65:
                    expected = dock_range / max(.1, math.cos(angle))
                    if abs(distance-expected) < .045:
                        continue
                threshold = .16 if v == 0 else .23
                if distance < threshold:
                    return False
        return valid >= 3

    def tick(self):
        command = Twist()
        self.enabled_pub.publish(Bool(data=self.enabled))
        try:
            if self.enabled:
                command.linear.x, command.angular.z = self.step(time.monotonic())
        except (TransformException, ValueError) as exc:
            self.detail = f'Waiting: {exc}'
        except Exception as exc:
            self.transition('FAULT', str(exc))
            self.get_logger().error(f'Behavior stopped: {exc}')
        self.cmd_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps({'state': self.state if self.enabled else 'MANUAL',
                                                        'detail': self.detail})))

    def step(self, now):
        if self.state == 'FAULT':
            return 0.0, 0.0
        if self.battery is None or now-self.battery_at > 2:
            self.detail = 'Waiting for fresh battery telemetry'
            return 0.0, 0.0
        if self.grid is None or now-self.map_at > 30 or now-self.scan_at > .75:
            self.detail = 'Waiting for fresh map and laser scan'
            return 0.0, 0.0
        pose = self.pose()
        x, y, yaw = pose
        if self.dock is None:
            if not self.get_parameter('simulation').value:
                self.detail = 'Mark the physical charging pose before enabling autonomy'
                return 0.0, 0.0
            ox, oy, oyaw = self.pose('odom')
            self.dock = (ox+.85*math.cos(oyaw), oy+.85*math.sin(oyaw), oyaw)
            self.save_dock()
        elif not hasattr(self, 'marker_sent'):
            self.save_dock()
        self.marker_sent = True
        soc = self.battery.percentage
        charging = self.battery.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING, BatteryState.POWER_SUPPLY_STATUS_FULL)
        # Critical battery interrupts greetings; ordinary low battery waits until
        # the 10 s greeting ends, then takes priority over exploration.
        if self.state == 'GREETING' and soc > self.get_parameter('critical_battery').value:
            if now < self.greeting.until:
                return 0.0, 0.0
            self.transition(self.resume_state)
        elif self.state == 'GREETING':
            self.transition('RETURNING')
        if soc <= self.get_parameter('low_battery').value and self.state == 'EXPLORING':
            self.dock_retries = 0
            self.transition('RETURNING', 'Battery low; returning to marked dock')
        if (self.state in ('EXPLORING', 'RETURNING') and soc > self.get_parameter('critical_battery').value
                and self.person and now-self.person_at < 1 and self.greeting.start(now)):
            self.resume_state = self.state
            self.transition('GREETING', 'Hi! Pausing for 10 seconds')
            self.say_pub.publish(String(data='Hi!'))
            return 0.0, 0.0
        if self.state == 'CHARGING':
            if not charging:
                self.transition('FAULT', 'Charging contact lost')
            elif soc >= self.get_parameter('resume_battery').value:
                self.retry_docking = False
                self.transition('UNDOCKING')
            return 0.0, 0.0
        if self.state == 'VERIFY_CHARGE':
            if charging:
                self.transition('CHARGING', 'Charger feedback confirmed')
            elif now-self.entered > 10:
                self.dock_retries += 1
                if self.dock_retries >= 3:
                    self.transition('FAULT', 'Charger did not confirm charging after 3 attempts')
                else:
                    self.retry_docking = True
                    self.transition('UNDOCKING', 'Backing out before retry')
            return 0.0, 0.0
        v = w = 0.0
        if self.state == 'EXPLORING':
            if self.goal is None:
                cell = self.grid.explore(self.grid.cell(x, y), self.visited)
                if cell is None:
                    self.visited = []
                    self.detail = 'No new reachable target; waiting for map update'
                    return 0.0, 0.0
                self.goal = self.grid.point(cell)
                self.next_plan = 0.0
            if math.hypot(self.goal[0]-x, self.goal[1]-y) < .10:
                self.visited.append(self.goal)
                self.goal = None
                return 0.0, 0.0
            v, w = self.navigate(pose, self.goal, now)
        elif self.state == 'RETURNING':
            sx, sy, syaw = self.staging()
            if math.hypot(sx-x, sy-y) < .075:
                self.transition('DOCKING', 'Aligning for final approach')
            else:
                v, w = self.navigate(pose, (sx, sy), now)
        elif self.state == 'DOCKING':
            dx, dy, dyaw = self.dock
            forward = (dx-x)*math.cos(dyaw)+(dy-y)*math.sin(dyaw)
            lateral = -(dx-x)*math.sin(dyaw)+(dy-y)*math.cos(dyaw)
            if now-self.entered > 30 or abs(lateral) > .12 or forward < -.035:
                self.transition('FAULT', 'Dock approach exceeded alignment or time limit')
                return 0.0, 0.0
            error = wrap(dyaw-yaw)
            if abs(error) > .08:
                w = max(-.25, min(.25, error))
            elif forward < .015 and abs(lateral) < .025:
                self.transition('VERIFY_CHARGE', 'Stopped; waiting for charger feedback')
            else:
                v, w = min(.035, max(.012, forward*.4)), max(-.2, min(.2, error+2*lateral))
        elif self.state == 'UNDOCKING':
            sx, sy, _ = self.staging()
            if math.hypot(sx-x, sy-y) < .07:
                self.transition('DOCKING' if self.retry_docking else 'EXPLORING')
                return 0.0, 0.0
            if now-self.entered > 20:
                self.transition('FAULT', 'Undocking timed out')
                return 0.0, 0.0
            v, w = -.04, 0.0
        if self.progress_pose is None or math.hypot(x-self.progress_pose[0], y-self.progress_pose[1]) > .05 or abs(wrap(yaw-self.progress_pose[2])) > .2:
            self.progress_pose, self.last_progress = pose, now
        elif now-self.last_progress > 20:
            if self.state == 'EXPLORING':
                if self.goal:
                    self.visited.append(self.goal)
                self.transition('EXPLORING', 'Trying another target after no progress')
            else:
                self.transition('FAULT', 'No progress toward dock')
            return 0.0, 0.0
        dock_range = None
        if self.state == 'DOCKING':
            dx, dy, dyaw = self.dock
            dock_range = (dx-x)*math.cos(dyaw)+(dy-y)*math.sin(dyaw)+.11
        if not self.scan_clear(v, w, dock_range):
            self.detail = 'Stopped for laser obstacle; awaiting a clear path'
            return 0.0, 0.0
        return v, w


def main():
    rclpy.init()
    node = Autonomy()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
