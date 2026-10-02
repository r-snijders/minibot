# Container simulation and AI

Requires native Docker Engine on Linux and the Compose plugin. ROS and Gazebo
are installed only in the image. All ROS services use host networking and domain
42, with discovery restricted to localhost by default. Gazebo uses partition
`minibot`. Set a distinct ROS_DOMAIN_ID and GZ_PARTITION for concurrent projects.
For a real robot on another computer, explicitly change discovery/network settings.

## Start on your NVIDIA desktop

Install the NVIDIA Container Toolkit on the host using its official instructions:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
The existing NVIDIA host driver is also required. Do not install a driver in the image.

```bash
git clone https://github.com/r-snijders/minibot.git
cd minibot
# Starts simulation, platform watchdog, battery, navigation, VLM adapter,
# deterministic reasoning and a local Ollama server.
docker compose -f compose.yaml -f compose.nvidia.yaml --profile ai up --build -d
# One-time model download; persisted in the models volume.
docker compose --profile ai exec model ollama pull qwen2.5vl:3b
docker compose logs -f vlm reasoning navigation
```

The VLM retries at its sampling interval while the server/model is unavailable.
You can instead use an existing Ollama server by setting VLM_ENDPOINT and omitting
`--profile ai`. Set VLM_MODEL to change the vision model. OLLAMA_IMAGE overrides
the server image (default latest); pin a tested tag or digest for reproducibility.
The small default model is a starting point, not a detection accuracy guarantee.
Gazebo and inference share the GPU. Reduce image sampling frequency if needed.

CPU-only alternative (slower inference):

```bash
docker compose --profile ai up --build -d
docker compose --profile ai exec model ollama pull qwen2.5vl:3b
```

The default server runs with Xvfb so simulated cameras and GPU LiDAR can render
using software graphics without a desktop display. The NVIDIA override switches
to EGL headless rendering. This still renders sensors; it only removes the GUI.

## Optional Gazebo GUI on Ubuntu / Hyprland

Use the current XWayland DISPLAY and its Xauthority cookie. If XAUTHORITY is not
already set, obtain the correct cookie file from your session before starting.
Do not use `xhost +`. The cookie mount grants this trusted GUI access to your display.

```bash
# DISPLAY and XAUTHORITY must refer to your running X/XWayland session.
test -n "$DISPLAY" && test -f "$XAUTHORITY"
docker compose -f compose.yaml -f compose.nvidia.yaml --profile gui up -d gui
```

Closing/stopping the GUI leaves the server running. If graphics fail, check the
NVIDIA runtime and XWayland credentials separately from ROS networking.

## Observe and drive

```bash
docker compose exec reasoning ros2 topic echo /reasoning/status
docker compose exec vlm ros2 topic echo /perception/observations
docker compose exec gazebo ros2 topic hz /camera/image_raw
# Pause autonomous control before manually driving:
docker compose exec platform ros2 service call /behavior/enable std_srvs/srv/SetBool '{data: false}'
docker compose run --rm platform ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Services:

| Service | Responsibility |
|---|---|
| gazebo | Gazebo Jetty server, ROS bridge, sensor transforms |
| platform | Sole publisher to /drive/cmd_vel; limits and stale-command stop |
| battery | Simulated state of charge and dock contact feedback |
| navigation | Existing mapping, grid planner, battery/docking and greeting execution |
| vlm | Latest-frame camera adapter; bounded asynchronous Ollama requests |
| reasoning | Validates observations, publishes person signal and semantic intent |
| model | Optional local Ollama model runtime |
| gui | Optional independent Gazebo client |

The reasoning node is a deterministic policy, not an LLM planner. It delegates
exploration, low-battery return, stop/greet/wait-10s and resume to the existing
autonomy node. The existing prototype grid planner is retained; this is not Nav2.
VLM text is never executed or converted directly to motor commands. Only a
validated boolean person signal affects greetings. The VLM is not an obstacle
safety sensor: LiDAR and the controller remain responsible for motion checks.

`/perception/observations` is JSON in std_msgs/String with person_visible,
description, model, source simulation stamp and age_seconds. Observations older
than 15 wall seconds are discarded/expired. The adapter keeps one request in
flight and the newest camera frame, so model latency cannot create a frame queue.
Inference timeouts use wall time even if simulation pauses. Navigation and
battery use simulation time. The platform watchdog uses wall time.

The existing room has no human asset. Real VLM person detection requires adding a
recognizable human mesh/actor; it cannot be demonstrated with this empty room.
You can test the reasoning/greeting wiring independently (synthetic observation):

```bash
docker compose exec reasoning ros2 topic pub --once /perception/observations std_msgs/msg/String \
  '{data: "{\"person_visible\": true, \"description\": \"test person\", \"age_seconds\": 0}"}'
docker compose exec navigation ros2 topic echo /behavior/status
```

Speech text is published to /speech/text. Audible speech is disabled in Compose
until host audio is explicitly connected; no audio socket is mounted by default.
Physical camera/motor/charger drivers are not started by this simulation stack.
The simulated battery/contact model does not validate real charger electronics.

## Verification

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
docker compose config --quiet
docker compose build
# End-to-end transport/physics test, using a mock model server (no model download):
# Stop local model/navigation first so port 11434 and manual control are free.
docker compose --profile ai stop model navigation
docker compose up -d gazebo platform battery vlm reasoning
docker compose run --rm --no-deps platform python3 /ws/src/minibot/tests/ros_smoke.py
```

The smoke test verifies actual simulated camera, scan, clock, odometry movement,
JPEG-to-HTTP inference and reasoning publication. Its mock response is not a VLM
accuracy test. GitHub CI runs unit tests, Compose validation, image build and this
smoke test with software rendering. Hardware GPU/GUI validation remains local.

```bash
docker compose --profile ai --profile gui down
```

Maps/dock state and model weights persist in named volumes. `down -v` deletes them.
Keep source mounts in development overrides; deployed code is baked into the
image so these same images can later be used in Kubernetes.

References: https://gazebosim.org/docs/latest/ros_installation/
https://docs.ollama.com/api/generate and https://docs.ollama.com/docker
