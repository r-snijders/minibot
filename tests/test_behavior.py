"""Exercise supervisor decisions with stand-in ROS types (no ROS runtime).

These tests are not transport, physics, camera, or launch integration tests.
"""
import importlib
import math
import sys
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from minibot.planning import Greeting, Grid


class Twist:
    def __init__(self):
        self.linear, self.angular = NS(x=0.0), NS(z=0.0)


def load_without_ros():
    stubs = {}
    def stub(name, **members):
        mod = types.ModuleType(name)
        mod.__dict__.update(members)
        stubs[name] = mod
        return mod
    rclpy = stub('rclpy')
    stub('rclpy.node', Node=object)
    stub('rclpy.qos', QoSProfile=NS, DurabilityPolicy=NS(TRANSIENT_LOCAL=1), qos_profile_sensor_data=object())
    stub('rclpy.time', Time=object)
    for parent in ('geometry_msgs', 'nav_msgs', 'sensor_msgs', 'std_msgs', 'std_srvs', 'visualization_msgs'):
        stub(parent)
    stub('geometry_msgs.msg', Twist=Twist)
    stub('nav_msgs.msg', OccupancyGrid=NS)
    battery = type('BatteryState', (), {'POWER_SUPPLY_STATUS_CHARGING': 1, 'POWER_SUPPLY_STATUS_DISCHARGING': 2, 'POWER_SUPPLY_STATUS_FULL': 4})
    stub('sensor_msgs.msg', BatteryState=battery, LaserScan=NS)
    stub('std_msgs.msg', Bool=NS, String=NS)
    stub('std_srvs.srv', SetBool=NS, Trigger=NS)
    stub('tf2_ros', Buffer=object, TransformListener=object, TransformException=type('TransformException', (Exception,), {}))
    stub('visualization_msgs.msg', Marker=NS, MarkerArray=NS)
    with patch.dict(sys.modules, stubs):
        return importlib.import_module('minibot.autonomy'), importlib.import_module('minibot.velocity_gate')


autonomy, gate_module = load_without_ros()


class BehaviorTests(unittest.TestCase):
    def setUp(self):
        self.node = autonomy.Autonomy.__new__(autonomy.Autonomy)
        n = self.node
        n.enabled, n.state, n.detail = True, 'EXPLORING', ''
        n.dock, n.marker_sent = (.85, 0, 0), True
        n.greeting, n.person, n.person_at = Greeting(), False, 100
        n.resume_state = 'EXPLORING'
        n.battery = NS(percentage=.8, present=True, power_supply_status=2)
        n.scan = NS(header=NS(frame_id='laser_frame'), angle_min=-math.pi,
                    angle_increment=math.pi/180, range_min=.15, range_max=8., ranges=[8.]*360)
        n.grid = Grid(100, 100, .05, (-2.5, -2.5, 0), [0]*10000)
        n.pose = lambda: (0., 0., 0.)
        n.path, n.visited, n.goal = [], [], None
        n.next_plan, n.entered, n.last_progress, n.progress_pose = 0., 100., 100., None
        n.dock_retries, n.retry_docking = 0, False
        n.get_parameter = lambda k: NS(value={'low_battery': .25, 'critical_battery': .1,
                                             'resume_battery': .9, 'simulation': True}[k])
        self.said = []
        n.say_pub = NS(publish=lambda msg: self.said.append(msg.data))

    def step(self, now=100.):
        self.node.battery_at = self.node.scan_at = self.node.map_at = now
        with patch.object(autonomy.time, 'monotonic', return_value=now):
            return self.node.step(now)

    def test_person_stops_for_ten_seconds_then_resumes(self):
        n = self.node
        n.person = True
        self.assertEqual(self.step(), (0., 0.))
        self.assertEqual(n.state, 'GREETING')
        self.assertEqual(self.said, ['Hi!'])
        self.assertEqual(self.step(109.9), (0., 0.))
        self.step(110.)
        self.assertEqual(n.state, 'EXPLORING')
        self.assertEqual(self.said, ['Hi!'])

    def test_low_battery_after_greeting_returns_home(self):
        n = self.node
        n.person = True
        self.step()
        n.battery.percentage = .2
        self.step(105.)
        self.assertEqual(n.state, 'GREETING')
        self.step(110.)
        self.assertEqual(n.state, 'RETURNING')

    def test_critical_battery_interrupts_greeting(self):
        n = self.node
        n.person = True
        self.step()
        n.battery.percentage = .09
        self.step(101.)
        self.assertEqual(n.state, 'RETURNING')

    def test_pose_alone_does_not_confirm_charging(self):
        n = self.node
        n.state = 'DOCKING'
        n.pose = lambda: (.84, 0, 0)
        self.assertEqual(self.step(), (0., 0.))
        self.assertEqual(n.state, 'VERIFY_CHARGE')
        self.step(105.)
        self.assertEqual(n.state, 'VERIFY_CHARGE')
        n.battery.power_supply_status = 1
        self.step(106.)
        self.assertEqual(n.state, 'CHARGING')
        n.battery.percentage = .91
        self.step(107.)
        self.assertEqual(n.state, 'UNDOCKING')

    def test_charge_attempt_limit(self):
        n = self.node
        n.state, n.dock_retries = 'VERIFY_CHARGE', 2
        self.step(111.)
        self.assertEqual(n.state, 'FAULT')

    def test_contact_loss_stops(self):
        n = self.node
        n.state = 'CHARGING'
        self.assertEqual(self.step(), (0., 0.))
        self.assertEqual(n.state, 'FAULT')

    def test_stale_battery_stops(self):
        n = self.node
        n.battery_at = 90
        self.assertEqual(n.step(100), (0., 0.))

    def test_laser_blocks_path_and_allows_only_expected_dock(self):
        n = self.node
        n.scan.ranges[180] = .155
        self.assertFalse(n.scan_clear(.03, 0))
        self.assertTrue(n.scan_clear(.03, 0, .16))
        self.assertFalse(n.scan_clear(.03, 0, .4))

    def test_no_movement_with_nan_scan(self):
        self.node.scan.ranges = [float('nan')]*360
        self.assertFalse(self.node.scan_clear(.1, 0))

    def test_gate_blocks_manual_during_autonomy_and_stale_heartbeat(self):
        gate = gate_module.VelocityGate.__new__(gate_module.VelocityGate)
        manual, auto = Twist(), Twist()
        manual.linear.x, auto.linear.x = .15, 0.
        gate.enabled, gate.heartbeat = True, 100.
        gate.values = {'manual': (manual, 100.), 'auto': (auto, 100.)}
        sent = []
        gate.pub = NS(publish=sent.append)
        with patch.object(gate_module.time, 'monotonic', return_value=100.1):
            gate.tick()
        self.assertEqual(sent[-1].linear.x, 0.)
        auto.linear.x = .1
        gate.values['auto'] = (auto, 101.)
        with patch.object(gate_module.time, 'monotonic', return_value=101.):
            gate.tick()
        self.assertEqual(sent[-1].linear.x, 0.)

    def test_complete_return_charge_and_resume_with_kinematic_fixture(self):
        # Motion fixture, not Gazebo: start away from the dock and integrate the
        # actual supervisor's velocity commands through a conservative grid.
        n = self.node
        n.battery.percentage = .22
        pose = [-.8, .7, 1.0]
        n.pose = lambda: tuple(pose)
        seen = set()
        contacted = None
        for tick in range(2200):
            now = 100+tick*.1
            v, w = self.step(now)
            seen.add(n.state)
            pose[2] = math.atan2(math.sin(pose[2]+w*.1), math.cos(pose[2]+w*.1))
            pose[0] += v*math.cos(pose[2])*.1
            pose[1] += v*math.sin(pose[2])*.1
            contact = math.hypot(pose[0]-.85, pose[1]) < .025 and abs(pose[2]) < .12 and v == 0 and w == 0
            contacted = (now if contacted is None else contacted) if contact else None
            if contacted is not None and now-contacted > 1:
                n.battery.power_supply_status = 1
                n.battery.percentage = min(1., n.battery.percentage+.002)
            else:
                n.battery.power_supply_status = 2
            if 'UNDOCKING' in seen and n.state == 'EXPLORING':
                break
        self.assertNotIn('FAULT', seen, (n.detail, pose))
        self.assertTrue({'RETURNING', 'DOCKING', 'VERIFY_CHARGE', 'CHARGING', 'UNDOCKING', 'EXPLORING'} <= seen, (seen, pose, n.detail))


if __name__ == '__main__':
    unittest.main()
