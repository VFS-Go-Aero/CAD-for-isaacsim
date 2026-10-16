# Go Aero eVTOL — Isaac Sim + PX4 & ArduPilot SITL Simulation

Simulate the Go Aero quadrotor eVTOL in NVIDIA Isaac Sim with PX4 or ArduPilot (ArduCopter) autopilot firmware.

## What This Does

- Loads the Go Aero CAD model (STL meshes) into Isaac Sim with rigid-body physics, uneven terrain, 3D obstacles (3 m, 5 m, 10 m), and onboard RTX LiDAR + dual ZED X stereo/depth camera mounts
- Runs **PX4 Autopilot** or **ArduPilot (ArduCopter)** in Software-In-The-Loop (SITL) mode
- Supports both **Mode 1 (Visualizer)** (`px4_visualizer.py`) and **Mode 2 (Full PhysX HIL/SIL Bridge)** (`px4_bridge.py` / `ardupilot_bridge.py`)
- Supports autonomous flight commands (takeoff, waypoints, PosHold, 3D obstacle avoidance) via MAVSDK (`fly_demo.py`) and PyMAVLink (`ardu_poshold.py`)
- Streams ROS 2 topics (`/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`, `/lidar/points`, `/clock`, `/tf`, `/tf_static`) to the Go Aero Jetson obstacle detection & avoidance stack (`point_cloud` → `/merged_cloud` → `obstacle_detection` → `/merged_cloud/obstacles` → `obstacle_to_mavlink` → `/mavros/obstacle_distance_3d/send` & `/mavros/obstacle/send`; see [`simulation/JETSON_HIL.md`](./simulation/JETSON_HIL.md) and [`docker/jetson-sil/`](./docker/jetson-sil/))

## Requirements

- **Ubuntu 24.04**
- **NVIDIA GPU** (8GB+ VRAM) with CUDA drivers
- **~30GB disk** (Isaac Sim + PX4/ArduPilot + conda env)

## Install

### 1. Miniconda

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
bash /tmp/miniconda.sh -b -p $HOME/miniconda3
eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
conda init bash
source ~/.bashrc
```

### 2. Conda Environment with Isaac Sim

```bash
# Accept conda terms of service (required before creating envs)
source ~/miniconda3/etc/profile.d/conda.sh
conda config --set auto_activate_base false
conda init bash
source ~/.bashrc

conda create -n env_isaaclab_jazzy python=3.11 -y
conda activate env_isaaclab_jazzy
pip install --upgrade pip setuptools
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com

# flatdict==4.0.1 (required by isaaclab) has a broken setup.py that calls pkg_resources,
# which is absent in pip's isolated build envs on modern setuptools. This script patches
# it and pre-builds a wheel that pip will use instead of rebuilding from source.
python - << 'EOF'
import subprocess, sys, tarfile, textwrap, urllib.request
from pathlib import Path

src  = Path("/tmp/flatdict_src")
whl  = Path("/tmp/flatdict_whl")
src.mkdir(exist_ok=True)
whl.mkdir(exist_ok=True)

url = "https://files.pythonhosted.org/packages/source/f/flatdict/flatdict-4.0.1.tar.gz"
tgz = src / "flatdict-4.0.1.tar.gz"
print("Downloading flatdict source...")
urllib.request.urlretrieve(url, tgz)

with tarfile.open(tgz) as t:
    t.extractall(src)

(src / "flatdict-4.0.1" / "setup.py").write_text(
    "import setuptools\nsetuptools.setup()\n"
)

print("Building patched wheel...")
subprocess.check_call([sys.executable, "-m", "pip", "wheel",
    str(src / "flatdict-4.0.1"), "--no-deps", "-w", str(whl)])
print("Done:", list(whl.iterdir()))
EOF

pip install 'isaaclab[isaacsim,all]==2.3.2' --extra-index-url https://pypi.nvidia.com --find-links /tmp/flatdict_whl/
pip install trimesh pymavlink 'mavsdk<4'
```

### Updated Conda IsaacLab Build
```bash
#!/bin/bash
set -e  # stop immediately if any command fails, instead of silently continuing

# --- Conda setup ---
source ~/miniconda3/etc/profile.d/conda.sh
conda config --set auto_activate_base false

# Accept conda terms of service (this was missing before, which is why
# env_isaaclab_jazzy never actually got created and every pip install
# below ran against the system Python instead)
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

conda init bash
source ~/.bashrc

# Remove any partial/broken env from a previous failed attempt
conda env remove -n env_isaaclab_jazzy -y || true

conda create -n env_isaaclab_jazzy python=3.11 -y
conda activate env_isaaclab_jazzy

# Sanity check: make sure activation actually worked before installing anything
if [[ "$(which python)" != *"env_isaaclab_jazzy"* ]]; then
    echo "ERROR: conda environment did not activate correctly. Aborting."
    echo "python resolved to: $(which python)"
    exit 1
fi
echo "Using python: $(which python)"

pip install --upgrade pip setuptools
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com

# flatdict==4.0.1 (required by isaaclab) has a broken setup.py that calls pkg_resources,
# which is absent in pip's isolated build envs on modern setuptools. This patches it
# and pre-builds a wheel that pip will use instead of rebuilding from source.
python - << 'EOF'
import subprocess, sys, tarfile, urllib.request
from pathlib import Path

src = Path("/tmp/flatdict_src")
whl = Path("/tmp/flatdict_whl")
src.mkdir(exist_ok=True)
whl.mkdir(exist_ok=True)

url = "https://files.pythonhosted.org/packages/source/f/flatdict/flatdict-4.0.1.tar.gz"
tgz = src / "flatdict-4.0.1.tar.gz"
print("Downloading flatdict source...")
urllib.request.urlretrieve(url, tgz)

with tarfile.open(tgz) as t:
    t.extractall(src)

(src / "flatdict-4.0.1" / "setup.py").write_text(
    "import setuptools\nsetuptools.setup()\n"
)

print("Building patched wheel...")
subprocess.check_call([sys.executable, "-m", "pip", "wheel",
    str(src / "flatdict-4.0.1"), "--no-deps", "-w", str(whl)])
print("Done:", list(whl.iterdir()))
EOF

pip install 'isaaclab[isaacsim,all]==2.3.2' --extra-index-url https://pypi.nvidia.com --find-links /tmp/flatdict_whl/
pip install trimesh pymavlink 'mavsdk<4'

# --- Final verification ---
echo "=== conda envs ==="
conda info --envs

echo "=== import check ==="
python -c "import isaaclab; print('isaaclab imported OK')"
```

### 3. ROS2 Jazzy (system packages for rosbridge)

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt-get update
sudo apt-get install -y ros-jazzy-rosbridge-server ros-jazzy-rosapi ros-jazzy-ros-base
```

### 4. PX4 Autopilot (SITL)

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --depth 1
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx
```

If the setup script fails with an apt error about `ros2.list`, fix it:
```bash
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update
bash Tools/setup/ubuntu.sh --no-nuttx
```

### 5. Create Go Aero Airframe

```bash
cat > ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero << 'EOF'
#!/bin/sh
#
# @name Go Aero eVTOL Quadrotor SITL
# @type Quadrotor X
# @class Copter

. ${R}etc/init.d/rc.mc_defaults

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

# Motor 1: Front-Right (CCW)
param set-default CA_ROTOR0_PX 0.515
param set-default CA_ROTOR0_PY 0.468
param set-default CA_ROTOR0_KM 0.05

# Motor 2: Rear-Right (CW)
param set-default CA_ROTOR1_PX -0.515
param set-default CA_ROTOR1_PY 0.468
param set-default CA_ROTOR1_KM -0.05

# Motor 3: Rear-Left (CCW)
param set-default CA_ROTOR2_PX -0.515
param set-default CA_ROTOR2_PY -0.468
param set-default CA_ROTOR2_KM 0.05

# Motor 4: Front-Left (CW)
param set-default CA_ROTOR3_PX 0.515
param set-default CA_ROTOR3_PY -0.468
param set-default CA_ROTOR3_KM -0.05

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

param set-default SYS_HITL 1
param set-default SENS_EN_GPSSIM 0
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 0
EOF

chmod +x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero
```

Register it in CMakeLists:
```bash
sed -i '/10016_none_iris/a\\t10050_none_go_aero' \
    ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

### 6. Build PX4

```bash
cd ~/PX4-Autopilot
make px4_sitl
```

### 7. Build the Isaac Sim Scene

Clone this repo and build the USD scene from the STL meshes:

```bash
git clone https://github.com/VFS-Go-Aero/CAD-for-isaacsim.git ~/go_aero_repo
cd ~/go_aero_repo
git checkout sgoss/isaac-sim-import

# Copy meshes, scripts, and ArduPilot parameters to working directory
mkdir -p ~/go_aero/meshes ~/go_aero/params
cp meshes/* ~/go_aero/meshes/
cp simulation/*.py ~/go_aero/
cp simulation/params/* ~/go_aero/params/

# Build the USD scene with 3m/5m/10m obstacles (headless, takes ~60s)
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
export ROS_DISTRO=jazzy
python ~/go_aero/build_scene.py --headless --obstacles
```

This creates `~/go_aero/vehicle_scene.usd` with the Go Aero model, uneven terrain, 3 m / 5 m / 10 m calibration obstacles, sensor mounts, ground plane, and lighting.

## Run the Demo (PX4 SITL)

You need **three terminals** (or use `nohup` / `tmux` for background processes).

### Terminal 1: Start PX4

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d
```

Wait for `Ready for takeoff!`

### Terminal 2: Start Isaac Sim Visualizer

```bash
export DISPLAY=:1  # or your display
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
export LD_LIBRARY_PATH=$HOME/miniconda3/envs/env_isaaclab_jazzy/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib:$LD_LIBRARY_PATH

python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550
```

Wait for `Visualizer running!` (~90 seconds for Isaac Sim to boot)

### Terminal 3: Fly

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda deactivate 2>/dev/null
conda activate env_isaaclab_jazzy

# Fly a square pattern (3 loops at 5m altitude)
python ~/go_aero/fly_demo.py --loops 3 --altitude 5 --size 8

# Or manual control via MAVSDK:
python -c "
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address='udpin://0.0.0.0:14540')
    async for state in drone.core.connection_state():
        if state.is_connected: break
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(10)
    await drone.action.land()

asyncio.run(run())
"
```

## Alternate Flow: ArduPilot (Copter SITL)

Instead of PX4, both the **Visualizer (Mode 1)** and **PhysX Bridge (Mode 2)** work with **ArduPilot's Copter SITL**:
- **Mode 1 (`px4_visualizer.py`)** — ArduPilot SITL (`-f quad`) runs its internal physics model and streams MAVLink `GLOBAL_POSITION_INT` + `ATTITUDE` over UDP `14550` to Isaac Sim.
- **Mode 2 (`ardupilot_bridge.py`)** — Isaac Sim PhysX owns the physics and exchanges binary servo PWM packets (`0x481A` / `18458`) and JSON sensor state (`\n{...}\n`) with ArduPilot SITL (`-f JSON`) over UDP `9002`.

### One-Time Install (on the instance)

```bash
# Clone ArduPilot + submodules
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot

# Install ArduPilot prereqs (skipped on Ubuntu 24.04 where the installer script lags;
# install the pieces SITL actually needs directly into the conda env):
conda activate env_isaaclab_jazzy
pip install empy==3.3.4 pexpect future pymavlink MAVProxy

# Pre-compile ArduCopter SITL binary (takes ~3 min the first time)
./waf configure --board sitl
./waf copter
```

### Mode 1: ArduPilot SITL Internal Physics + `px4_visualizer.py` (3 Terminals)

**Terminal 1 — ArduPilot Copter SITL:**
```bash
conda activate env_isaaclab_jazzy
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild \
  --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14540 \
  --out=udp:127.0.0.1:14551
```
- `14550` feeds `px4_visualizer.py`
- `14540` feeds your control script (`ardu_poshold.py`, MAVProxy, etc.)
- `14551` (plus `SERIAL1` TCP `5762`) feeds the Jetson / SIL obstacle detection stack (`obstacle_to_mavlink` + `latency_relay.py` + MAVROS sending `OBSTACLE_DISTANCE_3D`)
- Wait ~15 s until the console prints `EKF3 IMU0 is using GPS` before arming.

**Terminal 2 — Isaac Sim Visualizer:**
```bash
export DISPLAY=:1
conda activate env_isaaclab_jazzy
# (--port 14550 is also supported as an alias for --mavlink-port 14550)
python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550
```

**Terminal 3 — PosHold or Obstacle Avoidance Flight Test:**
```bash
conda activate env_isaaclab_jazzy
# PosHold hover test: takes off in GUIDED to 5 m, sets RC3 throttle override to 50%
# (1500 PWM = hover mid-stick), switches to POSHOLD for 30 s, then LANDs:
python ~/go_aero/ardu_poshold.py --altitude 5 --throttle-pct 50 --hold 30

# Or GUIDED forward obstacle avoidance test (commands 8 m North toward the 5 m obstacle,
# verifying ArduPilot stops at AVOID_MARGIN = 2.0 m in front of the obstacle):
python ~/go_aero/ardu_poshold.py --mode guided-forward --target-north 8 --altitude 2.5 --hold 20
```

### Mode 2: Isaac Sim PhysX + ArduPilot SITL JSON Bridge (`ardupilot_bridge.py`)

**Terminal 1 — Isaac Sim ArduPilot PhysX Bridge (start first):**
```bash
export DISPLAY=:1
conda activate env_isaaclab_jazzy
python ~/go_aero/ardupilot_bridge.py --device cuda --port 9002 --lidar --zed --ros2
```
Wait for `Waiting for ArduPilot SITL (-f JSON) on UDP 9002...`.

**Terminal 2 — ArduPilot Copter SITL (`-f JSON`):**
```bash
conda activate env_isaaclab_jazzy
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --no-rebuild \
  --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14540 \
  --out=udp:127.0.0.1:14551
```

**Terminal 3 — Flight Test (`ardu_poshold.py`):**
```bash
conda activate env_isaaclab_jazzy
python ~/go_aero/ardu_poshold.py --altitude 5 --throttle-pct 50 --hold 30
```

## Ground Control Station (GCS) Setup

- **Mission Planner (for ArduPilot SITL over SSH TCP tunnel):**
  ArduPilot's `sim_vehicle.py` exposes MAVLink on TCP port `5763` (`SERIAL2`) by default. Forward it over SSH from your local machine so Mission Planner never steals UDP `14550` from `px4_visualizer.py`:
  ```bash
  ssh -i key.pem -L 5763:127.0.0.1:5763 ubuntu@<ELASTIC_IP>
  ```
  In Mission Planner, select **TCP**, host `127.0.0.1`, port `5763`, and click **Connect**.
- **QGroundControl (for PX4 SITL on dedicated UDP port `14601`):**
  Do **not** point QGroundControl at UDP `14550` on the same machine as `px4_visualizer.py` — otherwise QGC will steal `14550` from `px4_visualizer.py`. Instead, open a dedicated GCS MAVLink stream on UDP `14601` (in the PX4 shell: `mavlink start -x -u 14600 -o 14601 -r 4000000`, using local source port `14600` to avoid conflicting with PX4's onboard link on `14580`) and configure QGroundControl with a UDP Comm Link listening on port `14601`.

## Jetson HIL & Docker SIL Obstacle Detection (`OBSTACLE_DISTANCE_3D` & `OBSTACLE_DISTANCE`)

To run the full **Go Aero Obstacle Detection & Avoidance** stack (`VFS-Go-Aero/visual-obstacle-detection` nodes `point_cloud`, `obstacle_detection`, and `obstacle_to_mavlink` + `latency_relay.py` + MAVROS):
- See [`simulation/JETSON_HIL.md`](./simulation/JETSON_HIL.md) for the end-to-end architecture, ROS 2 topic wiring (`/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`, `/merged_cloud`, `/merged_cloud/obstacles`, `/mavros/obstacle_distance_3d/send`, `/mavros/obstacle/send`), and step-by-step Hardware-In-The-Loop (physical Jetson Orin NX) & Software-In-The-Loop instructions.
- See [`docker/jetson-sil/`](./docker/jetson-sil/) (`docker compose -f docker/jetson-sil/docker-compose.yml --profile test run --rm jetson-sil-test`) to run the entire Jetson obstacle detection pipeline in a container on macOS or Linux **without a physical Jetson**.

## Files

| File | Description |
|------|-------------|
| `meshes/` | STL mesh files (`frame.stl`, `FR.stl`, `FL.stl`, `BR.stl`, `BL.stl`) |
| `go_aero_vehicle.urdf` | Fixed URDF (reference, not used for rendering) |
| `XML2_fixed.urdf` | Alternate URDF reference |
| `simulation/` | Simulation scripts + dedicated docs (see [`simulation/README.md`](./simulation/README.md)) |
| `simulation/_frames.py` | Shared NED/ENU + mesh-fix quaternion helpers |
| `simulation/_compat.py` | Isaac Sim 5.1 / 6.0 compatibility layer & shared USD prim path constants |
| `simulation/build_scene.py` | Builds `vehicle_scene.usd` from STL meshes (with obstacles, uneven terrain, LiDAR & ZED mounts) |
| `simulation/open_scene.py` | Opens scene with standalone PID hover controller |
| `simulation/px4_visualizer.py` | Mode 1 visualizer: mirrors PX4 or ArduPilot state via MAVLink into Isaac Sim |
| `simulation/px4_bridge.py` | Mode 2 PhysX HIL bridge for PX4 SITL (`HIL_SENSOR` / `HIL_ACTUATOR_CONTROLS`) |
| `simulation/ardupilot_bridge.py` | Mode 2 PhysX bridge for ArduPilot SITL (`-f JSON`, UDP 9002) |
| `simulation/fly_demo.py` | MAVSDK script: takeoff, fly square, land (PX4) |
| `simulation/ardu_poshold.py` | ArduPilot flight & obstacle avoidance test script (`poshold` and `guided-forward` modes) |
| `simulation/joystick_control.py` | Xbox controller → `MANUAL_CONTROL` over UDP to PX4 / ArduPilot |
| `simulation/add_lidar_tutorial.py` | Standalone tutorial script attaching RTX LiDAR at `/World/Vehicle/chassis/lidar_mount` |
| `simulation/add_zed_tutorial.py` | Standalone tutorial script attaching ZED X cameras at `zed1_mount` and `zed2_mount` |
| `simulation/jetson_sil_launch.py` | ROS 2 Humble SIL launch file for `visual_obstacle_detection` + MAVROS + `latency_relay.py` |
| `simulation/synthetic_zed_publisher.py` | Synthetic ZED X `PointCloud2` publisher & standalone pure-Python obstacle pipeline self-test |
| `simulation/latency_relay.py` | 80 ms Orin NX compute latency buffer & 72-sector PX4 2D `OBSTACLE_DISTANCE` bridge |
| `simulation/params/go_aero_ardupilot.param` | ArduCopter parameters (`PRX1_TYPE=2`, `AVOID_ENABLE=7`, `AVOID_BEHAVE=1`, `AVOID_DIST_MAX=5`, `AVOID_MARGIN=2`) |
| `simulation/JETSON_HIL.md` | Jetson Orin HIL & Docker SIL obstacle detection + avoidance guide |
| `simulation/LIDAR_TUTORIAL.md` | Step-by-step RTX LiDAR attachment & verification guide |
| `simulation/ZED_TUTORIAL.md` | Step-by-step ZED X stereo/depth camera attachment & ROS 2 guide |
| `simulation/ARCHITECTURE.md` | Process topology, data flow, coordinate frames |
| `simulation/PX4_API.md` | MAVLink + MAVSDK reference for the simulation |
| `docker/jetson-sil/` | Containerized Jetson ROS 2 Humble + `visual_obstacle_detection` + MAVROS stack |
| `tests/test_simulation_suite.py` | Pure-Python unit & integration test suite covering frames, bridges, obstacle pipeline, and SIL launch |

## fly_demo.py Options

```
--loops N       Number of square loops (default: 3)
--altitude M    Flight altitude in meters (default: 5)
--size M        Square size in meters (default: 8)
--speed M       Flight speed in m/s (default: 3)
--port P        MAVSDK UDP port (default: 14540)
```

## ardu_poshold.py Options

```
--altitude M        Takeoff altitude in meters (default: 5.0)
--throttle-pct P    RC3 throttle percentage for POSHOLD hover (default: 50 -> 1500 PWM)
--hold S            Hold duration in seconds (default: 30.0)
--port P            MAVLink UDP port (default: 14540)
--mode MODE         poshold (default) or guided-forward (obstacle stop verification)
--target-north M    Target North distance in GUIDED mode (implies --mode guided-forward)
--obstacle-north M  Known North distance of scene obstacle (default: 5.0)
--avoid-margin M    Expected ArduPilot AVOID_MARGIN stopping distance (default: 2.0)
```

## Vehicle Specs

| Property | Value |
|----------|-------|
| Type | Quadrotor X |
| Chassis | ~1.06m × 0.60m × 1.15m |
| Prop span | ~0.81m per blade |
| Arm length | ~0.70m (center to motor) |
| Track width | 0.94m |
| Wheelbase | 1.03m |
| Flight controller | Pixhawk 6C |
| Firmware | PX4 Autopilot / ArduPilot ArduCopter |

## Simulation Modes & Capabilities

- **Mode 1 — Visualizer (`px4_visualizer.py`)**: PX4 SIH or ArduPilot Copter SITL (`-f quad`) handles flight dynamics internally while Isaac Sim mirrors the 6-DoF vehicle pose (`LOCAL_POSITION_NED` or `GLOBAL_POSITION_INT` + `ATTITUDE`), spins propellers, and optionally streams RTX LiDAR, ZED RGB+depth, and ROS 2 topics (`--lidar --zed --ros2`).
- **Mode 2 — Full PhysX Bridge (`px4_bridge.py` / `ardupilot_bridge.py`)**: Isaac Sim PhysX is the source of truth for rigid-body dynamics, collisions, ground effect, and onboard sensors (`IMUSensor`, RTX LiDAR, ZED camera), closing the loop with PX4 (`HIL_SENSOR` / `HIL_ACTUATOR_CONTROLS` on TCP 4560) or ArduPilot (`SIM_JSON` on UDP 9002).
- **3D Obstacle Avoidance (`JETSON_HIL.md` / `docker/jetson-sil`)**: Streams synthetic ZED X point clouds (`/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`) over ROS 2 into the unmodified `VFS-Go-Aero/visual-obstacle-detection` nodes (`point_cloud`, `obstacle_detection`, `obstacle_to_mavlink`) and `latency_relay.py`, publishing `OBSTACLE_DISTANCE_3D` (`/mavros/obstacle_distance_3d/send`) for ArduPilot (`PRX1_TYPE=2`, `AVOID_ENABLE=7`, stopping at `AVOID_MARGIN=2.0 m`) and 72-sector `OBSTACLE_DISTANCE` (`/mavros/obstacle/send`) for PX4 (`CP_DIST=2.0 m`).

## Troubleshooting

### `ModuleNotFoundError: No module named 'mavsdk'` (or pymavlink, trimesh, etc.)

The conda env isn't properly activated. Check with `which python` — it should show:
```
/home/ubuntu/miniconda3/envs/env_isaaclab_jazzy/bin/python
```

If it shows `env_isaaclab` (without `_jazzy`) or `/usr/bin/python`, deactivate and re-activate:
```bash
conda deactivate
conda deactivate
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
which python  # verify correct path
```

### `conda activate env_isaaclab_jazzy` fails / env doesn't exist

The conda environment hasn't been created yet. Create it:
```bash
conda create -n env_isaaclab_jazzy python=3.11 -y
conda activate env_isaaclab_jazzy
pip install --force-reinstall setuptools wheel
pip install isaacsim==5.1.0.0 isaaclab==2.3.2
pip install trimesh pymavlink 'mavsdk<4'
```

Isaac Sim is ~15GB and takes several minutes to install via pip.

### `Error while loading conda entry point: conda-anaconda-tos`

This is a harmless warning from conda plugins, not an error. Ignore it.

### `_rclpy_pybind11.cpython-311` import error

System ROS2 Jazzy (Python 3.12) is conflicting with the conda env (Python 3.11). Do NOT `source /opt/ros/jazzy/setup.bash` in the same shell as the conda env. Rosbridge should run in a separate terminal with system Python.

### PX4 says `Insufficient capacity` on AWS

GPU instances sometimes have no available capacity. Retry:
```bash
# Retry every 10 seconds for 10 minutes
for i in $(seq 1 60); do
  aws ec2 start-instances --instance-ids <ID> --region us-east-1 && break
  sleep 10
done
```

### Isaac Sim viewport is white/black

- White: camera is pointing at empty space. Press **F** in the viewport to frame the scene.
- Black: no lighting. Check the scene was built with `build_scene.py`.
- Ensure `DISPLAY=:1` is set and DCV is running.

### PX4 `COMMAND_DENIED` on arm

PX4 may be in a bad state from a previous flight. Kill and restart:
```bash
pkill -9 -f "bin/px4"
rm -f /tmp/px4_lock_*
cd ~/PX4-Autopilot/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d
```

## Cloud Deployment (AWS)

For running on AWS with GPU:

```bash
# g6e.2xlarge (NVIDIA L40S, 46GB VRAM) works well
# Use NVIDIA Isaac Sim AMI for pre-installed drivers

# Start instance
aws ec2 start-instances --instance-ids <INSTANCE_ID> --region us-east-1

# Connect via DCV for GUI
# https://<ELASTIC_IP>:8443

# SSH for terminals
ssh -i key.pem ubuntu@<ELASTIC_IP>
```
