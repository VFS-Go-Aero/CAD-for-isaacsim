# Jetson Companion Computer Simulation (SIL & HIL Guide)

This guide explains how to test the **Go Aero eVTOL companion computer stack** — **Ubuntu 22.04 + ROS 2 Humble + ZED X stereo point clouds + [`VFS-Go-Aero/visual-obstacle-detection`](https://github.com/VFS-Go-Aero/visual-obstacle-detection) (`feature/visual_obstacle_detection_2` @ `cba2f42`) + MAVROS + ArduPilot/PX4 obstacle avoidance** — across three progressive stages:

1. **Mode A — Local SIL on macOS / Linux (No GPU & No Physical Jetson Needed):**
   Runs in pure Python or inside the multi-arch `docker/jetson-sil` container (`linux/arm64` on Apple Silicon Macs — the exact `aarch64` ISA of the Jetson Orin NX!).
2. **Mode B — Cloud GPU SIL on AWS EC2 (`g6e.2xlarge`):**
   Runs NVIDIA Isaac Sim + the `jetson-sil` container + ArduPilot/PX4 SITL + Mission Planner / QGroundControl on a single cloud GPU host.
3. **Mode C — Physical Jetson HIL (ZED Box Mini Orin NX 16GB on the Bench):**
   Streams simulated sensor data and clock sync over LAN/VPN to a real Stereolabs ZED Box Mini running L4T R36.4.x (JetPack 6) and ZED SDK 5.1.0.

---

## 1. Architecture Overview

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ MODE A: Local macOS / Linux SIL (No GPU, No Physical Jetson)                           │
│                                                                                        │
│  ┌──────────────────────────────┐     ROS 2 Humble (docker/jetson-sil, arm64/amd64)    │
│  │ synthetic_zed_publisher.py   │──── /zed1 & /zed2/zed_node/point_cloud/cloud_reg ───┐│
│  │  - coherent obstacle patches │                                                     ▼│
│  │  - excluded_boxes self-pts   │     ┌───────────────────────────────────────────────┐│
│  │  - disconnected noise        │     │ VFS-Go-Aero/visual-obstacle-detection         ││
│  └──────────────────────────────┘     │  1. point_cloud (TF + 15 exclusion boxes)     ││
│                                       │  2. obstacle_detection (32x8 sectors, 2-of-3) ││
│  ┌──────────────────────────────┐     │  3. obstacle_to_mavlink (RFU -> FRD frame=12) ││
│  │ tf2_ros static_transform_pub │────►└──────────────────────┬────────────────────────┘│
│  │  base_link -> zed1/2_cam_lnk │                            │                         │
│  └──────────────────────────────┘                            ▼                         │
│                                       ┌───────────────────────────────────────────────┐│
│                                       │ latency_relay.py (--delay-ms 80)              ││
│                                       │  - buffers ObstacleDistance3D for ArduPilot   ││
│                                       │  - bins 72x5° OBSTACLE_DISTANCE for PX4       ││
│                                       └──────────────────────┬────────────────────────┘│
│                                                              │ MAVROS / pymavlink      │
│                                                              ▼                         │
│                                       ┌───────────────────────────────────────────────┐│
│                                       │ ArduPilot SITL (TCP 5762, PRX1_TYPE=2)        ││
│                                       │   or PX4 SITL (UDP 14580, CP_DIST=2.0)        ││
│                                       └───────────────────────────────────────────────┘│
└────────────────────────────────────────────────────────────────────────────────────────┘
```

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ MODE B & C: Isaac Sim GPU Host (EC2)  <===>  jetson-sil Container OR Physical Jetson   │
│                                                                                        │
│  ┌────────────────────────────────────┐        ┌──────────────────────────────────────┐│
│  │ Isaac Sim Host (Ubuntu 24.04)      │        │ Companion Stack (Ubuntu 22.04 Humble)││
│  │  - px4_bridge.py / ardupilot_bridge│        │  (jetson-sil container OR ZED Box)   ││
│  │  - Dual ZED X depth cameras        │  DDS   │  - point_cloud                       ││
│  │    (/World/Vehicle/chassis/zed_*)  ├───────►│  - obstacle_detection                ││
│  │  - Publishes /clock + PointCloud2  │        │  - obstacle_to_mavlink               ││
│  │  - Runs ArduPilot or PX4 SITL      │◄───────┤  - latency_relay.py (80 ms SIL mode) ││
│  └────────────────────────────────────┘MAVLink │  - MAVROS (apm.launch -> SITL:5762)  ││
│                                                └──────────────────────────────────────┘│
└────────────────────────────────────────────────────────────────────────────────────────┘
```

### Why Direct `PointCloud2` Bypasses the ZED SDK Version Lock
On the physical Go Aero vehicle, the ZED Box Mini runs **ZED SDK 5.1.0** on L4T R36.4 (`Ubuntu 22.04`). Meanwhile:
- The `zed-isaac-sim` streamer extension for Isaac Sim 6.0 requires **ZED SDK ≥ 5.4.1**.
- However, [`visual_obstacle_detection`](https://github.com/VFS-Go-Aero/visual-obstacle-detection) does **not** call `pyzed` / `sl.Camera` directly — its `point_cloud.py` node (verified in [`synthetic_zed_publisher.py`](./synthetic_zed_publisher.py)) only subscribes to standard `sensor_msgs/msg/PointCloud2` messages on:
  - `/zed1/zed_node/point_cloud/cloud_registered`
  - `/zed2/zed_node/point_cloud/cloud_registered`
  plus the TF transforms `base_link -> zed1_camera_link` and `base_link -> zed2_camera_link` (`0.1524, 0.0, -0.127`).

By publishing `PointCloud2` directly on those two topics (from [`synthetic_zed_publisher.py`](./synthetic_zed_publisher.py) in Mode A or Isaac Sim in Mode B), the **exact, unmodified** `visual_obstacle_detection` package runs end-to-end without any ZED SDK version mismatch. When ZED IPC streaming is desired, [`jetson_sil_launch.py`](./jetson_sil_launch.py) also supports `use_zed_sim_stream:=true`.

### QoS and Frame ID Requirements for Isaac Sim Publishers
Because `visual_obstacle_detection` (`point_cloud.py`) subscribes to both ZED point cloud topics using the default ROS 2 Humble QoS profile (`depth=10`, **`RELIABLE`**, `VOLATILE`) and transforms clouds into `base_link` via TF2:
1. **QoS Reliability (`RELIABLE`):** Any Isaac Sim `ROS2CameraHelper` or synthetic publisher sending `PointCloud2` on `/zed1/zed_node/point_cloud/cloud_registered` and `/zed2/zed_node/point_cloud/cloud_registered` **must** use **`RELIABLE`** QoS (not `BEST_EFFORT`), otherwise ROS 2 Humble's DDS QoS compatibility check will drop the subscription match.
2. **Frame IDs (`zed1_camera_link` / `zed2_camera_link`):** Set `frameId` on the Isaac Sim camera point-cloud publishers to `zed1_camera_link` and `zed2_camera_link` (or `zed1_camera_optical_frame` / `zed2_camera_optical_frame`). [`jetson_sil_launch.py`](./jetson_sil_launch.py) publishes static TFs for both `base_link -> zedN_camera_link` (`0.1524, 0.0, -0.127`, RPY `0, 0, 0`) and `zedN_camera_link -> zedN_camera_optical_frame` (RPY `-pi/2, 0, -pi/2`).

---

## 2. Mode A — Local SIL on macOS / Linux (Zero Hardware Required)

### Step A1: Instant Pure-Python Verification (No Docker, No ROS 2 Needed)

You can test the entire obstacle detection math pipeline (`excluded_boxes.xml` airframe filtering, 26-neighbor 10 cm voxel coherence, 32×8 sector map, 2-of-3 frame `CandidateConfirmation`, RFU→FRD `ObstacleDistance3D` conversion, and 72-sector PX4 `OBSTACLE_DISTANCE` binning) directly on macOS or Linux:

```bash
# 1. Verify the visual_obstacle_detection math pipeline on synthetic point clouds
python3 simulation/synthetic_zed_publisher.py --self-test

# 2. Verify the 80 ms Orin NX latency buffer and 72-sector PX4 2D binning
python3 simulation/latency_relay.py --self-test

# 3. Run the full unit test suite
python3 tests/test_simulation_suite.py
```

### Step A2: Build & Run the Virtual Jetson Container (`docker/jetson-sil`)

On an Apple Silicon Mac (`M1`/`M2`/`M3`/`M4`), Docker builds and runs the `linux/arm64` image natively — exercising the exact **`aarch64` Linux architecture** of the Jetson Orin NX.

```bash
# Run the one-shot container self-test (builds visual-obstacle-detection with colcon
# inside ROS 2 Humble and runs both our self-tests and upstream pytest checks):
docker compose -f docker/jetson-sil/docker-compose.yml --profile test run --rm jetson-sil-test
```

### Step A3: Run the Full ROS 2 Pipeline Inside `jetson-sil`

```bash
# Launch the virtual Jetson with synthetic ZED point clouds (obstacle at 4.0 m ahead),
# system clock (use_sim_time:=false), and an 80 ms compute latency relay,
# without needing an external autopilot:
docker compose -f docker/jetson-sil/docker-compose.yml run --rm \
  --name go-aero-jetson-sil \
  -e USE_SIM_TIME=false \
  -e ENABLE_MAVROS=false \
  -e USE_SYNTHETIC_ZED=true \
  -e OBSTACLE_DIST=4.0 \
  -e LATENCY_MS=80.0 \
  jetson-sil \
  ros2 launch /ros2_ws/sim_scripts/jetson_sil_launch.py \
    use_sim_time:=false \
    enable_mavros:=false \
    use_synthetic_zed:=true \
    obstacle_dist:=4.0 \
    latency_ms:=80.0
```

In a second terminal, inspect the live ROS 2 topics inside the container:

```bash
# Verify merged point cloud (after 15-box airframe exclusion)
docker exec -it go-aero-jetson-sil bash -c \
  "source /entrypoint.sh && ros2 topic hz /merged_cloud"

# Verify confirmed 3D obstacle representatives
docker exec -it go-aero-jetson-sil bash -c \
  "source /entrypoint.sh && ros2 topic echo --once /merged_cloud/obstacles"

# Verify delayed MAVROS ObstacleDistance3D output (MAV_FRAME_BODY_FRD = 12)
docker exec -it go-aero-jetson-sil bash -c \
  "source /entrypoint.sh && ros2 topic echo --once /mavros/obstacle_distance_3d/send"

# Verify 72-sector 2D LaserScan output for PX4 Collision Prevention
docker exec -it go-aero-jetson-sil bash -c \
  "source /entrypoint.sh && ros2 topic echo --once /mavros/obstacle/send"
```

---

## 3. Mode B — Cloud GPU SIL on AWS EC2 (`g6e.2xlarge`)

On the EC2 simulation instance, Isaac Sim, the autopilot SITL (ArduPilot or PX4), and the `jetson-sil` container run side-by-side.

### Terminal 1: Start Autopilot SITL (ArduPilot or PX4)

#### Option 1 — ArduPilot SITL (with Go Aero Avoidance Params)
```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --no-rebuild \
  --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14540 \
  --out=udp:127.0.0.1:14551
```
ArduPilot SITL automatically opens `tcp:127.0.0.1:5762` (SERIAL1) and `tcp:127.0.0.1:5763` (SERIAL2) for companion-computer MAVROS and Mission Planner connections, while forwarding UDP `14550` (visualizer), `14540` (`ardu_poshold.py`), and `14551` (extra telemetry/obstacle link).

#### Option 2 — PX4 SITL
```bash
cd ~/PX4-Autopilot/build/px4_sitl_default
PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d
```

### Terminal 2: Start Isaac Sim Bridge (with ZED Depth Cameras + ROS 2)

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# ArduPilot Physics Bridge:
python ~/go_aero/ardupilot_bridge.py --device cuda --zed --ros2

# OR PX4 Physics Bridge:
python ~/go_aero/px4_bridge.py --device cuda --zed --ros2
```

### Terminal 3: Start the `jetson-sil` Companion Container

```bash
cd ~/go_aero_repo
docker compose -f docker/jetson-sil/docker-compose.yml build jetson-sil
docker run --rm -it \
  --name go-aero-jetson-sil \
  --network host \
  -v "$(pwd)/simulation:/ros2_ws/sim_scripts:ro" \
  vfs-go-aero/jetson-sil:humble \
  ros2 launch /ros2_ws/sim_scripts/jetson_sil_launch.py \
    use_sim_time:=true \
    use_synthetic_zed:=false \
    latency_ms:=80.0 \
    fcu_url:=tcp://127.0.0.1:5762
```
*(For PX4 SITL, pass `fcu_url:=udp://:14540@127.0.0.1:14580` and `enable_px4_2d_bridge:=true`, leaving UDP `14550` free for QGroundControl / `px4_visualizer.py`.)*

### Terminal 4: Connect Ground Control Station (Mission Planner / QGC)

Forward the GCS port over SSH (or use UDP 14550 for QGroundControl) from your local laptop:

```bash
# Forward ArduPilot SITL TCP port 5763 to local port 5763 for Mission Planner:
ssh -i isaac-sim-keypair.pem -L 5763:127.0.0.1:5763 ubuntu@<EC2_IP>
```
In **Mission Planner**, select **TCP** → `127.0.0.1` → port `5763`, then press `Ctrl+F` → **Proximity** to view the live 3D obstacle radar ring around the vehicle. For **QGroundControl** with PX4, connect to UDP port `14550`.

---

## 4. Mode C — Physical Jetson HIL (ZED Box Mini Orin NX on the Bench)

When the physical **Stereolabs ZED Box Mini (Jetson Orin NX 16GB)** is available on the bench:

### Hardware & Software Baseline
| Component | Specification |
|---|---|
| Device | Stereolabs ZED Box Mini (NVIDIA Jetson Orin NX 16GB, `aarch64`) |
| OS / BSP | Ubuntu 22.04 LTS / L4T R36.4.x (JetPack 6.1) |
| ZED SDK | 5.1.0 (`/usr/local/zed/include/sl/Camera.hpp`) |
| ROS 2 | ROS 2 Humble (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) |
| Detection Repo | `VFS-Go-Aero/visual-obstacle-detection` (`feature/visual_obstacle_detection_2` @ `cba2f42`) |

### Step C1: Clock Synchronization (`chrony`)
Because `point_cloud.py` looks up TF transforms and `obstacle_to_mavlink.py` timestamps `ObstacleDistance3D` messages, keep clocks aligned between the sim host and the Jetson (or pass `use_sim_time:=true` when publishing `/clock`):

```bash
# On the physical Jetson:
sudo apt-get install -y chrony
sudo chronyc makestep
```

### Step C2: Fast DDS Unicast Discovery Across LAN / VPN
When the Isaac Sim host and physical Jetson are on separate subnets or connected over WireGuard/Tailscale where multicast UDP is blocked, configure `ROS_STATIC_PEERS` or a Fast DDS XML profile on both machines:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Step C3: Launch on the Physical Jetson
Copy [`simulation/jetson_sil_launch.py`](./jetson_sil_launch.py) and [`simulation/latency_relay.py`](./latency_relay.py) to the Jetson workspace and launch with `latency_ms:=0.0` (since the physical Orin NX exhibits real hardware compute latency). If flying PX4 and you want the 72-sector 2D `OBSTACLE_DISTANCE` bridge active at `0.0` ms delay, pass `enable_px4_2d_bridge:=true`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch jetson_sil_launch.py \
  use_sim_time:=true \
  use_synthetic_zed:=false \
  latency_ms:=0.0 \
  fcu_url:=tcp://<SIM_HOST_IP>:5762
```

---

## 5. ArduPilot vs PX4 Obstacle Avoidance Reference (Issue #1 & #4)

`visual_obstacle_detection` natively emits `mavros_msgs/msg/ObstacleDistance3D` (`MAV_FRAME_BODY_FRD = 12`). To support **both** ArduPilot and PX4 in the simulation comparison (Issue #1), [`latency_relay.py`](./latency_relay.py) provides both representations simultaneously:

| Feature | ArduPilot (`ArduCopter`) | PX4 (`PX4-Autopilot`) |
|---|---|---|
| **MAVLink Message** | `OBSTACLE_DISTANCE_3D` (`#393`, 3D `[x, y, z]` in `MAV_FRAME_BODY_FRD`) | `OBSTACLE_DISTANCE` (`#330`, 72-element 2D array in `cm`, `5°` increments) |
| **ROS 2 Topic** | `/mavros/obstacle_distance_3d/send` (`mavros_msgs/msg/ObstacleDistance3D`) | `/mavros/obstacle/send` (CCW FLU `sensor_msgs/msg/LaserScan`) or direct `pymavlink` UDP |
| **Key Autopilot Params** | `PRX1_TYPE = 2` (MAVLink)<br>`AVOID_ENABLE = 7` (All sources)<br>`AVOID_DIST_MAX = 5.0`<br>`AVOID_MARGIN = 2.0`<br>`AVOID_BEHAVE = 0` (Slide) or `1` (Stop) | `CP_DIST = 2.0` (Collision Prevention distance in m)<br>`CP_DELAY = 0.4`<br>`CP_GO_NO_DATA = 0`<br>`COM_OBS_AVOID = 1` |
| **Flight Modes** | `LOITER`, `POSHOLD`, `GUIDED`, `AUTO` | `Position` mode (and Offboard with `COM_OBS_AVOID=1`) |
| **Stale Data Timeout** | `obstacle_to_mavlink.py` stops publishing after `0.75 s` if `/merged_cloud/obstacles` stalls | `latency_relay.py` prunes obstacles older than `0.60 s` and sets sectors to `max_distance_cm + 1` (`501`) |
