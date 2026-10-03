# Install and run Minibot with Docker Compose

This guide starts from a fresh Ubuntu 26.04 desktop. Install Docker on the host;
ROS 2 Lyrical, Gazebo Jetty and the Minibot code are built inside the container
image. You do not need a native ROS or Gazebo installation.

## Which files do what?

| File | Purpose |
|---|---|
| `docker/Dockerfile` | Builds the shared ROS image with Gazebo and Minibot nodes |
| `compose.yaml` | Complete service definitions, networking, volumes and software-rendered Gazebo |
| `compose.nvidia.yaml` | Override that enables NVIDIA GPUs for Gazebo, Ollama and the optional GUI |

For NVIDIA, pass both Compose files in that order. Compose merges their settings;
the override replaces Gazebo's software-rendering startup command with headless
GPU rendering. The NVIDIA file is not a standalone stack.

## 1. Install prerequisites

Install Git if it is not already available:

```bash
sudo apt update
sudo apt install git
```

Install [Docker Engine and the Compose plugin](https://docs.docker.com/engine/install/ubuntu/)
using Docker's official APT repository instructions. If Docker is already
installed and works, keep it. Use native Docker Engine on Linux for this setup.
The NVIDIA override requires Docker Compose 2.30.0 or newer.

Check:

```bash
docker version
docker compose version
docker run --rm hello-world
```

The commands below assume your user can access Docker. If you receive a permission
error, follow Docker's [Linux post-installation instructions](https://docs.docker.com/engine/install/linux-postinstall/).
Membership of the Docker group grants root-level access to the host.

### NVIDIA users only

Your NVIDIA host driver must work first:

```bash
nvidia-smi
```

Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
using its official Ubuntu/Debian instructions, then configure the runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Restarting Docker can interrupt existing containers. No NVIDIA driver installation
is needed inside the Minibot image.

Verify container GPU access:

```bash
docker run --rm --gpus all ubuntu:26.04 nvidia-smi
```

CPU-only users can skip the NVIDIA steps.

## 2. Clone the repository

```bash
git clone https://github.com/r-snijders/minibot.git
cd minibot
```

If you already have a checkout, run `git pull origin main` inside it instead.

## 3. Select NVIDIA or CPU mode

Define **one** of these Bash functions while inside the repository. It keeps all
commands using the same Compose configuration. Define it again in each new
terminal before using `dc`.

**NVIDIA mode:**

```bash
dc() {
  docker compose -f compose.yaml -f compose.nvidia.yaml --profile ai "$@"
}
```

**CPU-only mode:**

```bash
dc() {
  docker compose -f compose.yaml --profile ai "$@"
}
```

The `ai` profile enables the local Ollama model server. CPU inference and software
sensor rendering can be substantially slower. NVIDIA mode uses EGL headless
rendering; CPU mode uses Xvfb and software rendering. Both modes render camera
and LiDAR data without requiring an open desktop window.

## 4. Build and download the vision model

```bash
# Validate the selected configuration.
dc config --quiet

# Build the shared ROS/Gazebo/Minibot image.
dc build

# Start the model server and download weights once.
dc up -d model
dc exec model ollama pull qwen2.5vl:3b
```

The first build and model download can take a while. Model weights persist in a
named volume. The default model is a starting point, not an accuracy guarantee.

## 5. Start the complete stack

```bash
dc up -d
dc ps
dc logs -f gazebo vlm reasoning navigation
```

This starts Gazebo, the platform watchdog, simulated battery, navigation, VLM
adapter, deterministic reasoning and Ollama. The GUI is optional and starts
separately below. Pressing Ctrl+C exits the log viewer without stopping containers.

Check camera frames and model observations in separate terminals:

```bash
dc exec gazebo ros2 topic hz /camera/image_raw
```

```bash
dc exec vlm ros2 topic echo /perception/observations
```

You should receive camera frames and structured scene descriptions. The room has
no human asset yet, so a person greeting is not expected without adding one or
injecting a synthetic observation as described below.

## 6. Open the optional Gazebo GUI

From your desktop terminal, check the X/XWayland display and credentials:

```bash
test -n "$DISPLAY" && test -f "$XAUTHORITY"
```

If that succeeds:

```bash
dc --profile gui up -d gui
```

This also applies to Hyprland through XWayland. If the credential check fails,
obtain the Xauthority cookie file for your running session and set XAUTHORITY
before starting the GUI. Do not use `xhost +`. The cookie mount grants this trusted
GUI access to your display. The headless simulation can run without GUI credentials.

Closing/stopping the GUI leaves the server running. If graphics fail, check the
NVIDIA runtime and XWayland credentials separately from ROS networking.

## 7. Stop, restart or rebuild

```bash
# Stop and remove containers, retaining model weights and robot state.
dc --profile gui down

# Start again using the existing image and weights.
dc up -d

# After changing application code, rebuild and recreate affected containers.
dc up --build -d
```

Adding `-v` to `down` deletes the named volumes, including downloaded model weights.

## Configuration and networking

All ROS services use host networking and domain 42, with discovery restricted to
localhost by default. Gazebo uses partition `minibot`. Set distinct ROS_DOMAIN_ID
and GZ_PARTITION values for concurrent projects. Communicating with a physical
robot on another computer requires explicitly changing discovery/network settings.

Gazebo and inference share GPU resources in NVIDIA mode. Reduce camera sampling
frequency if necessary. The current override exposes all NVIDIA GPUs.

The local model server binds port 11434 on the host's loopback interface. If an
existing Ollama server already uses that port, either stop it or use that server:
set VLM_ENDPOINT and omit `--profile ai` in your chosen dc function. In that mode,
skip `dc up -d model` and download the vision model through your existing server.

Set VLM_MODEL to change the vision model. OLLAMA_IMAGE overrides the model-server
image (default latest); pin a tested tag or digest for reproducible deployments.
The VLM adapter retries while the server/model is unavailable.

## Troubleshooting

| Symptom | First check |
|---|---|
| Docker permission denied | Docker post-installation instructions and your user's access |
| Compose rejects `gpus` | Compose version must be at least 2.30.0 |
| No GPU in container | Host `nvidia-smi`, NVIDIA toolkit configuration and container GPU check |
| Port 11434 already in use | Existing Ollama process or another model container |
| VLM has no observations | Camera topic, model download, `dc logs vlm model` |
| No GUI window | DISPLAY/XAUTHORITY check and `dc logs gui` |
| No camera or scan data | `dc logs gazebo` and GPU/software rendering setup |

## Observe and drive

```bash
dc exec reasoning ros2 topic echo /reasoning/status
dc exec vlm ros2 topic echo /perception/observations
dc exec gazebo ros2 topic hz /camera/image_raw
# Pause autonomous control before manually driving:
dc exec platform ros2 service call /behavior/enable std_srvs/srv/SetBool '{data: false}'
dc run --rm platform ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
dc exec reasoning ros2 topic pub --once /perception/observations std_msgs/msg/String \
  '{data: "{\"person_visible\": true, \"description\": \"test person\", \"age_seconds\": 0}"}'
dc exec navigation ros2 topic echo /behavior/status
```

Speech text is published to /speech/text. Audible speech is disabled in Compose
until host audio is explicitly connected; no audio socket is mounted by default.
Physical camera/motor/charger drivers are not started by this simulation stack.
The simulated battery/contact model does not validate real charger electronics.

## Verification

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
dc config --quiet
dc build
# End-to-end transport/physics test, using a mock model server (no model download):
# Stop local model/navigation first so port 11434 and manual control are free.
dc stop model navigation
dc up -d gazebo platform battery vlm reasoning
dc run --rm --no-deps platform python3 /ws/src/minibot/tests/ros_smoke.py
```

The smoke test verifies actual simulated camera, scan, clock, odometry movement,
JPEG-to-HTTP inference and reasoning publication. Its mock response is not a VLM
accuracy test. The GitHub CI workflow is configured to run unit tests, Compose validation,
image build and this smoke test with software rendering. At the time this guide
was written, local unit tests passed, but full Gazebo/GPU execution had not been
verified by the implementation author. Hardware GPU/GUI validation remains local.

```bash
dc --profile gui down
```

Maps/dock state and model weights persist in named volumes. `down -v` deletes them.
Keep source mounts in development overrides; deployed code is baked into the
image so these same images can later be used in Kubernetes.

References: https://gazebosim.org/docs/latest/ros_installation/
https://docs.ollama.com/api/generate and https://docs.ollama.com/docker
