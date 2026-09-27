# Map, recharge, and greet

## Run

After pulling, rebuild with `colcon build --symlink-install` and source the workspace.
Run **one** `ros2 launch minibot autonomy.launch.py`. This includes Gazebo, its
sensor bridges, `slam_toolbox`, the controller, a command gate, a battery fixture,
the camera detector, and local speech. Do not launch `sim.launch.py` separately.

The robot follows grid paths through known free space. Obstacles and unknown areas
are inflated by 160 mm for its footprint. It prefers reachable targets near unknown
cells, then patrols unvisited free areas when the map is already covered. This is
a small conservative planner for the prototype, with a laser stop check, not Nav2.
It does not promise complete coverage of inaccessible spaces or handle arbitrary
dynamic crowds. A blocked dock route or failed final alignment stops with `FAULT`.

### Behavior priorities

1. Missing/stale scan, map, battery or TF: stop and wait. Runtime errors: stop.
2. Battery at or below 10%: interrupt a greeting to return to the dock.
3. Battery at or below 25%: return to the saved dock instead of continuing exploration.
4. A person detected during exploration or return: say “Hi!”, remain still for
   10 seconds, then resume. A new greeting needs both a 60-second cooldown and
   at least three seconds of person absence. Greetings do not interrupt contact
   alignment, charging, or reverse undocking.
5. After feedback confirms charging, wait for 90%, reverse to the staging pose,
   and resume exploring. Failed charge confirmation allows at most three attempts.

Thresholds are ROS parameters `low_battery`, `critical_battery`, `resume_battery`
on `/autonomy`. They are fractions from 0 to 1. Tune reserve against real runtime.

## Dock location and map

In this one provided Gazebo world, the robot knows the dock charging pose in odom
coordinates `(0.85, 0, 0)`. Once SLAM provides `map -> odom`, the controller converts
it into map coordinates, saves it to `~/.local/state/minibot/dock.json`, and publishes
an orange arrow and “Charging base” label on `/dock/markers`. Navigation targets a
staging pose 0.35 m behind the charge pose before the slow final approach.

This is a registered station location, not visual dock discovery. The marker is
saved alongside the map, not burned into occupancy pixels. Save a SLAM pose graph
with slam_toolbox's serialization service and retain the matching dock JSON. A
fresh mapping session creates a fresh coordinate system, so `restore_dock` defaults
to false. Only restore a saved dock with the corresponding map/localization setup;
the bundled launch starts a fresh mapping session and does not load an old map.
Loop closure or wheel drift may shift an old dock estimate; physical final docking
needs a local marker/contact sensor to correct that offset.

## Camera and greeting

The detector uses OpenCV's bundled HOG full-body people classifier, at about 3 Hz,
and requires three positive frames. It runs offline without downloading weights.
It is a baseline, not reliable recognition from a low robot camera: partial people,
nearby legs, unusual viewpoints and lighting can be missed or falsely detected.
Use a stronger person detector later behind the same `/person_detected` Bool topic.
No identity recognition, image recording, or cloud upload is included.

Speech uses `espeak-ng` on the computer running the speech node. For physical
greetings, run it on the Pi with a working USB/I2S speaker and audio output.
Missing audio leaves text on `/speech/text` but cannot produce audible speech.

The default Gazebo world has no human mesh. To test the **event handling only**:

```bash
# Start with the camera detector disabled so it cannot overwrite this test signal:
ros2 launch minibot autonomy.launch.py vision:=false
# In another sourced terminal:
ros2 topic pub --once /person_detected std_msgs/msg/Bool '{data: true}'
ros2 topic echo /behavior/status
```

Expect `GREETING`, a zero command for 10 seconds, then the previous task. To re-arm,
publish false for at least 3 seconds and wait out the 60-second cooldown. Injecting
a Bool does not test camera recognition. Test the actual detector using a real
camera or recorded camera topic with people in view.

## Charging feedback and physical use

The Gazebo fixture independently checks proximity, heading and stationary odometry
at the dock, waits one second, then publishes mock `BatteryState` charging status.
It does not consult the controller state. Simulated discharge is 0.2 percentage
points/second; charging is 2 percentage points/second. Both use simulation time.
The fixture neither switches electrical power nor models real battery chemistry.

On hardware, run `hardware.launch.py` and your lidar/camera drivers, and publish a
real `/battery_state` with `present=true`, a valid fraction `percentage`, and charger
status derived from actual electrical telemetry. Never run `sim_battery` on hardware.
Then launch `autonomy.launch.py simulation:=false`, disable autonomy, place the robot
at its charging pose facing into the dock, mark it, and move it back to the staging
pose before re-enabling:

```bash
ros2 service call /behavior/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /behavior/mark_dock std_srvs/srv/Trigger '{}'
# Manually reverse about 0.35 m from the dock using /cmd_vel, then stop.
ros2 service call /behavior/enable std_srvs/srv/SetBool '{data: true}'
```

The physical charging circuit, charge monitor, bumper interlock and local dock
alignment sensor are not implemented here. The current final approach relies on
map pose; the lidar has a blind zone near the charging contacts. Integrate those
components before allowing unattended physical docking. See `sensors-and-dock.md`.

## Command ownership and troubleshooting

Only `velocity_gate` publishes `/drive/cmd_vel`, which the Gazebo bridge and physical
driver consume. `/cmd_vel` is manual input, `/behavior/cmd_vel` is autonomous input.
When enabled, autonomy blocks manual commands, even during its greeting stop.
Commands older than 0.3 s and missing autonomy heartbeats older than 0.5 s become
zero. Disable autonomy with the service to switch back to manual operation.

`/behavior/status` is JSON text with state and detail. `/battery_state` shows charge;
`/speech/text` shows greetings. RViz can display `/map`, `/scan`, TF, and the
MarkerArray `/dock/markers` with fixed frame `map`.

Validation in the development workspace covers Python/XML/YAML structure and
ROS-independent planning/state tests. ROS, Gazebo, audio hardware and a camera
are unavailable there; end-to-end simulation and perception remain unverified.

From the repository root, run `PYTHONPATH=. python3 -m unittest discover -s tests -v`.
The tests use stand-in ROS message types and a kinematic motion fixture. They cover
obstacle/unknown-space planning, greeting timing, battery priority, charger
confirmation, retry limits, and movement-command timeouts.
