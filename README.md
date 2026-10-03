# Minibot: one ROS interface for Gazebo and a tabletop build

A two-wheel differential-drive rover (180 mm long, 140 mm wide), with 65 mm wheels and a trailing ball caster. Gazebo includes a 2D lidar, forward camera, and a simulation-only charging dock. See [sensors and charging design](docs/sensors-and-dock.md) before buying parts or wiring a charger.

## Docker Compose (recommended)

Run ROS 2 Lyrical, Gazebo Jetty, platform control, navigation, a VLM adapter and
reasoning as separate services.

**Start here: [Install and run Minibot with Docker Compose](docs/compose.md).**

The guide covers Docker installation, the NVIDIA Container Toolkit, CPU and GPU
startup, model download, the optional Gazebo GUI, verification and troubleshooting.
It also explains how `compose.yaml` and `compose.nvidia.yaml` fit together.

No ROS or Gazebo installation on the host is required. The reasoning policy is
currently deterministic; the VLM performs real inference through Ollama.

## Native software (Ubuntu 26.04, optional)

Install [ROS 2 Lyrical](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html) from the official instructions, then:

```bash
sudo apt update
sudo apt install ros-lyrical-ros-gz ros-lyrical-teleop-twist-keyboard ros-lyrical-slam-toolbox ros-lyrical-cv-bridge ros-lyrical-visualization-msgs python3-opencv python3-serial python3-colcon-common-extensions espeak-ng
mkdir -p ~/rover_ws/src
git clone https://github.com/r-snijders/minibot.git ~/rover_ws/src/minibot
cd ~/rover_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Simulation

```bash
ros2 launch minibot sim.launch.py
# In another terminal, source ROS and ~/rover_ws/install/setup.bash, then:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# Observe:
ros2 topic echo /odom
```

Click the Gazebo play button if the world is paused. Keep keyboard commands below 0.25 m/s and 1.5 rad/s. The Gazebo DiffDrive system consumes `/cmd_vel`, publishes `/odom`, and stops on a 0.5 s command timeout. The bridge maps these same topics to ROS.

The simulated lidar publishes `/scan`; the camera publishes `/camera/image_raw` and `/camera/camera_info`. To make a 2D map, start `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true` in another sourced terminal, then drive around the room slowly. Inspect the sensors with `ros2 topic echo /scan --once` and `ros2 topic hz /camera/image_raw`.

### Autonomous mapping, charging, and greetings

Run this instead of the standalone simulation command above; it starts Gazebo,
SLAM, exploration, person detection, speech, and the battery fixture together:

```bash
ros2 launch minibot autonomy.launch.py
# Or start low to exercise return-to-dock immediately:
ros2 launch minibot autonomy.launch.py initial_soc:=0.22
ros2 topic echo /behavior/status
ros2 topic echo /battery_state
```

The robot explores reachable parts of `/map`, favors places near unknown space,
returns to a saved map location below 25% battery, verifies charger feedback, and
resumes after 90%. A confirmed camera detection triggers “Hi!” and a 10-second
stop, followed by the previous task. It does not identify who the person is.
The map marker `/dock/markers` labels the charging base in RViz.

See [behavior instructions and limits](docs/behavior.md) for the greeting test,
physical setup, map saving, and battery thresholds. The plain Gazebo room has no
human model, so it does not by itself exercise camera person recognition.

### Physical build

Parts: 180 x 140 mm rigid base plate, two matched 6 V geared DC motors **with quadrature encoders** (roughly 100–200 rpm at the wheel), two 65 mm wheels, one low ball caster, Arduino Uno/Nano compatible 5 V board, TB6612FNG dual motor driver, battery holder and suitable 6 V motor supply, a regulated 5 V supply for the computer, wiring, switch, and a Raspberry Pi 5 or laptop with USB connection to the Arduino. Check the motor's stall current against the driver's limits; choose a more capable driver if necessary. Include a fuse and a physical power switch. Do not power motors from the Pi/Arduino 5 V pin.

Place wheel centers 160 mm apart. The axle is 90 mm behind the front edge of the plate. Place the caster behind the axle; adjust caster height so both drive wheels touch the floor. Measure the *effective* wheel radius and wheel separation after assembly and edit `minibot/geometry.py` **and** `worlds/rover.sdf` to match. The two files deliberately show the same numeric values so they are easy to compare; they are not automatically synchronized.

Arduino -> TB6612FNG: D9 PWMA, D7 AIN1, D8 AIN2; D10 PWMB, D11 BIN1, D12 BIN2; D6 STBY. Left encoder A/B -> D2/D4; right encoder A/B -> D3/D5. Connect motor power to VM, regulated 5 V logic to VCC, and share ground between motor driver and Arduino. USB connects Arduino to the ROS computer. The Arduino sketch is in `firmware/rover_motors/rover_motors.ino`. Reversing motor wires or encoder channels may be necessary to make positive wheel commands drive forward; lift the wheels off the ground for this check.

The sketch assumes **360 quadrature counts per wheel revolution**. This is a placeholder: determine the actual wheel-side count (including gearbox and both edges of channel A) and update `TICKS_PER_REV` in the sketch and `minibot/geometry.py` together. The firmware uses a simple proportional speed loop, capped PWM, and a 500 ms loss-of-command stop. Tune its `KP` and minimum PWM for your motor.

```bash
# Upload the sketch with Arduino IDE. On the ROS computer:
ros2 launch minibot hardware.launch.py port:=/dev/serial/by-id/YOUR_ARDUINO
# Second terminal, source ROS/workspace and drive as above.
```

Use a stable `/dev/serial/by-id/...` path (find it with `ls /dev/serial/by-id/`). The physical node publishes encoder-integrated `/odom` and `odom -> base_link` TF. Its position drifts; this is wheel odometry, not localization. Start with the robot raised, send a brief forward command, verify both wheels and encoder signs, then test on the floor at low speed. The Arduino USB serial protocol is `V <left_rad_s> <right_rad_s>` toward the MCU and `T <left_ticks> <right_ticks>` back. Sim and hardware must be launched in separate ROS domains or at separate times to avoid duplicate `/odom` and competing `/cmd_vel` consumers.

## Layout and assumptions

- `worlds/rover.sdf`: chassis, wheels, caster, Gazebo DiffDrive plugin, room, simulated sensors and dock.
- `launch/sim.launch.py`: Gazebo and topic bridges.
- `minibot/autonomy.py`: mapping patrol, map-based dock return, charge confirmation, greetings.
- `minibot/velocity_gate.py`: command selection and timeout; outputs `/drive/cmd_vel`.
- `minibot/sim_battery.py`: Gazebo-only battery and mock contact feedback.
- `minibot/person_detector.py` and `speech.py`: offline baseline people detector and speech.
- `minibot/hardware.py`: velocity conversion and wheel-encoder odometry.
- `firmware/rover_motors`: low-level speed control and watchdog.

The hardware launch connects the physical driver to `/drive/cmd_vel` and publishes
encoder `/odom` at 20 Hz. Manual commands still go to `/cmd_vel`; autonomous commands
have exclusive control while the behavior is enabled. Physical charging and battery
telemetry remain hardware integration work. The camera and lidar require their own
drivers on the physical robot. Keep its wheels clear of people and objects while testing.

Official references: [ROS 2 Lyrical installation](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html), [ROS and Gazebo pairing](https://gazebosim.org/docs/latest/ros_installation/), [Gazebo ROS bridge](https://gazebosim.org/docs/latest/ros2_integration/).
