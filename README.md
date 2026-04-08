# Go Aero eVTOL — Isaac Sim + PX4 SITL Simulation

Simulate the Go Aero quadrotor eVTOL in NVIDIA Isaac Sim with PX4 autopilot firmware.

## What This Does

- Loads the Go Aero CAD model (STL meshes) into Isaac Sim
- Runs PX4 autopilot firmware in Software-In-The-Loop (SITL) mode
- Visualizes the drone flying in real-time with spinning propellers
- Supports autonomous flight commands (takeoff, waypoints, land) via MAVSDK

## Requirements

- **Ubuntu 24.04**
- **NVIDIA GPU** (8GB+ VRAM) with CUDA drivers
- **~30GB disk** (Isaac Sim + PX4 + conda env)

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
pip install isaacsim==5.1.0.0 isaaclab==2.3.2
pip install trimesh pymavlink mavsdk
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

# Motor 1: Front-Right (CW)
param set-default CA_ROTOR0_PX 0.515
param set-default CA_ROTOR0_PY 0.468
param set-default CA_ROTOR0_KM 0.05

# Motor 2: Rear-Right (CCW)
param set-default CA_ROTOR1_PX -0.515
param set-default CA_ROTOR1_PY 0.468
param set-default CA_ROTOR1_KM -0.05

# Motor 3: Rear-Left (CW)
param set-default CA_ROTOR2_PX -0.515
param set-default CA_ROTOR2_PY -0.468
param set-default CA_ROTOR2_KM 0.05

# Motor 4: Front-Left (CCW)
param set-default CA_ROTOR3_PX 0.515
param set-default CA_ROTOR3_PY -0.468
param set-default CA_ROTOR3_KM -0.05

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 1
param set-default SENS_EN_MAGSIM 1
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

# Copy meshes and scripts to working directory
mkdir -p ~/go_aero/meshes
cp meshes/* ~/go_aero/meshes/
cp build_scene.py open_scene.py px4_visualizer.py fly_demo.py px4_bridge.py ~/go_aero/

# Build the USD scene (headless, takes ~60s)
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
export ROS_DISTRO=jazzy
python ~/go_aero/build_scene.py --headless
```

This creates `~/go_aero/vehicle_scene.usd` with the Go Aero model, ground plane, and lighting.

## Run the Demo

You need **three terminals** (or use `nohup` for background processes).

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

## Files

| File | Description |
|------|-------------|
| `meshes/` | STL mesh files (frame + 4 propellers) |
| `go_aero_vehicle.urdf` | Fixed URDF (reference, not used for rendering) |
| `build_scene.py` | Builds `vehicle_scene.usd` from STL meshes |
| `open_scene.py` | Opens scene with optional hover controller |
| `px4_visualizer.py` | Reads PX4 state via MAVLink, updates Isaac Sim |
| `fly_demo.py` | MAVSDK script: takeoff, fly square, land |
| `px4_bridge.py` | MAVLink bridge (for future HIL mode) |

## fly_demo.py Options

```
--loops N       Number of square loops (default: 3)
--altitude M    Flight altitude in meters (default: 5)
--size M        Square size in meters (default: 8)
--speed M       Flight speed in m/s (default: 3)
--port P        MAVSDK UDP port (default: 14540)
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
| Firmware | PX4 Autopilot |

## Current Limitations

- **Visualization only** — PX4 SIH handles all physics internally with a generic quadrotor model. Isaac Sim only mirrors the position.
- **Generic physics** — Not using the real Go Aero mass, inertia, motor specs, or aerodynamics.
- **No collision** — The 3D model has no rigid bodies or colliders in Isaac Sim.
- **No sensors** — No simulated camera, LiDAR, or other sensors on the drone.

## Next Steps for Accurate Simulation

To switch from visualization to full Hardware-In-The-Loop (HIL) simulation:

1. Get real vehicle specs (mass, motor KV, prop diameter/pitch, battery voltage)
2. Add rigid body physics + collision to the Isaac Sim model
3. Create motor thrust model (thrust = Ct × ρ × n² × D⁴)
4. Switch PX4 to HIL mode (Isaac Sim sends sensor data, PX4 sends motor commands)
5. Tune PX4 parameters to match real vehicle dynamics

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
pip install trimesh pymavlink mavsdk
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
