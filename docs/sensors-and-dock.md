# Sensors and autonomous charging roadmap

## Sensor choices

| Purpose | Suggested hardware | ROS data | Mount |
| --- | --- | --- | --- |
| 2D mapping | Slamtec RPLIDAR A1 with USB adapter | `/scan` (`sensor_msgs/LaserScan`) | On top, 360° clear sight; scan center 90 mm above chassis center in simulation |
| RGB perception | Raspberry Pi Camera Module 3 (standard or wide) | `/camera/image_raw`, `/camera/camera_info` | Forward, 90 mm ahead and 80 mm above chassis center in simulation |

The simulation samples 360 lidar rays at 8 Hz with 0.15–8 m range and 640×480 RGB frames at 10 Hz. These are simplified values, not an exact performance model of either product. Adjust physical mounting transforms in both launch files and the simulation world to match measurements. A 2D scan sees furniture legs at its height; it can miss low objects and overhangs, so it is not a sole collision sensor.

For a physical A1, follow [Slamtec's ROS 2 driver](https://github.com/Slamtec/sllidar_ros2), build it in the same workspace, and launch `sllidar_a1_launch.py` with the serial port and `frame_id:=laser_frame` parameters. Verify `/scan` before starting [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox). With the robot's `/odom` and TF tree, start mapping using `ros2 launch slam_toolbox online_async_launch.py`. The A1 driver is maintained independently from this repository; check its instructions for the actual device path and baud rate.

For the Pi camera, first confirm it works with `rpicam-hello` on the chosen Pi OS/image. [camera_ros](https://github.com/christianrauch/camera_ros) can publish a libcamera camera into ROS. Its README notes that the standard ROS libcamera package may lack full support for newer Pi modules, so test it with the actual Camera Module 3 and Pi 5 before relying on a binary install. The camera can later detect a high-contrast dock marker or AprilTag for final alignment. The simulation has a visible orange dock, **not** a functional fiducial detector.

## Charging architecture

Use a **protected battery pack and a charger explicitly matched to its chemistry and cell count**. The physical robot needs a charger/power-path solution specified for operating the Pi and motor electronics while charging. Choose the pack and power-path hardware before finalizing a wiring diagram. The original motor supply suggestion is not a charging design.

The dock can be a low-voltage, wall-supplied station with a wide funnel and recessed, spring-loaded contacts. Use a keyed contact layout, a fuse/current limit, and a controller that enables the dock output only after mechanical engagement is detected. Add independent measurement of charging current or a charger status signal. A contact switch alone does not prove that the battery is charging. Place the electrical charge controller on the robot or in a matched protected pack, following its manufacturer's wiring instructions. Do not connect dock contacts directly to bare battery cells or to the Pi's 5 V rail.

Suggested control sequence:

1. A battery monitor publishes `/battery_state`; below a threshold, request return-to-dock while retaining enough reserve to drive home.
2. A map/localization stack (Nav2) takes the robot to a pose near the dock. The lidar and encoder odometry support mapping; localization and obstacle-aware navigation are additional work.
3. A dock marker or short-range alignment sensor guides the final slow approach. Add a bumper/contact sensor and a way to detect a stalled wheel.
4. Stop the motors. Confirm contact **and actual positive charging current** within a timeout; otherwise back away and retry only a limited number of times.
5. Remain stopped while charging; undock after the charge controller reports completion. A motor controller watchdog and charger protection must remain effective independently of ROS.

[Nav2's Docking Server](https://docs.nav2.org/tutorials/docs/using_docking.html) is the natural later integration point for approach and retries. Its charging confirmation should come from the actual charger telemetry. In the present repository, `dock_demo.py` only demonstrates the last straight-line approach in Gazebo and publishes an artificial battery percentage. It does not control real charge hardware and must not be launched as a physical charging controller.

### Mechanical and electrical decisions still needed

- Battery chemistry/cell count, capacity, pack protection, charger model, and whether its power path supports loads while charging.
- Dock contact voltage/current, fuse, connector polarity and spacing, switched output logic, and a real current/charge-status feedback signal.
- Dock shape, spring travel, bump sensing, and a marker pose that remains visible from the final approach.
- Motor stall current, Pi peak power, camera/lidar draw, and the margin needed to reach the dock at low battery.

Bench test the chosen charger and pack to its datasheet before allowing autonomous docking. Then make the robot prove charging state from electrical telemetry, not just position.
