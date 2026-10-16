# Go Aero Isaac Sim Simulation

This folder contains the Python scripts that drive the Go Aero eVTOL simulation in NVIDIA Isaac Sim, with **PX4 SITL** or **ArduPilot (ArduCopter) SITL** in the loop.

For installation and one-time setup, see the [root README](../README.md).
For internals, see [ARCHITECTURE.md](./ARCHITECTURE.md), [PX4_API.md](./PX4_API.md), and [JETSON_HIL.md](./JETSON_HIL.md).

---

## What This Is

A real-time visual and physical simulation of the Go Aero quadrotor flying under PX4 or ArduPilot autopilot control. Two modes are supported for both autopilots:

| Mode | Autopilot | Script | Physics | Use case |
|------|-----------|--------|---------|----------|
| **Mode 1 — Visualizer** | PX4 SIH (`sihsim_quadx`) or ArduCopter (`-f quad`) | `px4_visualizer.py` | Autopilot internal | Smooth visualization, demos, autonomy & obstacle avoidance testing |
| **Mode 2 — Bridge** | PX4 (`none_go_aero`) | `px4_bridge.py` | Isaac Sim PhysX | Hardware-In-The-Loop (HIL) — full PhysX dynamics via MAVLink `HIL_*` |
| **Mode 2 — Bridge** | ArduCopter (`-f JSON`) | `ardupilot_bridge.py` | Isaac Sim PhysX | Full PhysX dynamics via ArduPilot `SIM_JSON` over UDP 9002 |

Use the Visualizer mode for quick demos and autonomy work; use the Bridge when you need PhysX to be the source of truth (airframe tuning, real motor/prop characterisation, collision and ground-effect experiments).

---

## Where It Lives

### In this repo (GitHub)

```
CAD-for-isaacsim/
├── simulation/                ← you are here
│   ├── README.md
│   ├── ARCHITECTURE.md
│   ├── PX4_API.md
│   ├── JETSON_HIL.md              Jetson Orin HIL + Docker SIL obstacle detection guide
│   ├── LIDAR_TUTORIAL.md          RTX LiDAR mount & point-cloud verification tutorial
│   ├── ZED_TUTORIAL.md            Dual ZED X stereo/depth camera tutorial
│   ├── _frames.py                 shared NED/ENU + mesh-fix quaternion helpers
│   ├── _compat.py                 Isaac Sim 5.1 / 6.0 compatibility layer & mount path constants
│   ├── build_scene.py             build vehicle_scene.usd from STL meshes
│   ├── open_scene.py              open scene with PID hover controller (no PX4)
│   ├── px4_visualizer.py          Mode 1: mirror PX4 or ArduPilot pose into Isaac Sim
│   ├── px4_bridge.py              Mode 2: full PX4 HIL bridge (sensors out, motors in)
│   ├── ardupilot_bridge.py        Mode 2: full ArduPilot SITL JSON bridge (UDP 9002)
│   ├── ardu_poshold.py            ArduPilot flight & obstacle avoidance test script
│   ├── fly_demo.py                MAVSDK script: takeoff, fly square, land (PX4)
│   ├── joystick_control.py        (Phase 2) Xbox controller → MANUAL_CONTROL to PX4/ArduPilot
│   ├── add_lidar_tutorial.py      standalone RTX LiDAR tutorial script
│   ├── add_zed_tutorial.py        standalone ZED X camera tutorial script
│   ├── jetson_sil_launch.py       ROS 2 Humble SIL launch file for visual_obstacle_detection
│   ├── synthetic_zed_publisher.py synthetic ZED X PointCloud2 publisher & self-test harness
│   ├── latency_relay.py           80 ms Orin NX latency buffer & PX4 72-sector 2D bridge
│   └── params/
│       └── go_aero_ardupilot.param   ArduCopter Quad-X + 3D proximity/avoidance params
├── docker/
│   └── jetson-sil/            Containerized Jetson ROS 2 + MAVLink obstacle stack (no physical Jetson needed)
│       ├── Dockerfile
│       ├── docker-compose.yml
│       └── entrypoint.sh
├── tests/
│   └── test_simulation_suite.py   Pure-Python unit & integration test suite
├── meshes/                    STL files: frame.stl, FR.stl, FL.stl, BR.stl, BL.stl
├── go_aero_vehicle.urdf       reference URDF (not used at runtime)
├── XML2_fixed.urdf            alternate URDF
└── README.md                  install + setup guide
```

### On the EC2 instance (`isaac-sim-massrobotics-01`, IP `52.91.203.66`)

```
/home/ubuntu/
├── miniconda3/                                   conda + env_isaaclab_jazzy env
├── PX4-Autopilot/                                PX4 source + built SITL binary
├── ardupilot/                                    ArduPilot source + built ArduCopter SITL binary
└── go_aero/                                      runtime working dir
    ├── meshes/                                   copy of repo meshes
    ├── params/
    │   └── go_aero_ardupilot.param
    ├── vehicle_scene.usd                         built by build_scene.py
    ├── _frames.py
    ├── _compat.py
    ├── build_scene.py
    ├── open_scene.py
    ├── px4_visualizer.py
    ├── px4_bridge.py
    ├── ardupilot_bridge.py
    ├── ardu_poshold.py
    ├── fly_demo.py
    ├── joystick_control.py
    ├── add_lidar_tutorial.py
    ├── add_zed_tutorial.py
    ├── jetson_sil_launch.py
    ├── synthetic_zed_publisher.py
    └── latency_relay.py
```

The scripts read the scene from `~/go_aero/vehicle_scene.usd` (a hard-coded path inside each script — see the *Updating* section if you need to change this).

---

## How to Run (PX4 SITL)

Three terminals (or `tmux` panes). Activate the conda env in each first:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
```

### Visualizer mode (PX4 SIH physics)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — PX4 | `cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d` | `Ready for takeoff!` |
| 2 — Isaac Sim | `python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550` | `Visualizer running!` (~90 s) |
| 3 — Mission | `python ~/go_aero/fly_demo.py --loops 3 --altitude 5 --size 8` | takeoff → square → land |

### HIL bridge mode (Isaac Sim PhysX is the physics)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — Bridge | `python ~/go_aero/px4_bridge.py --device cuda --px4-port 4560` | `Waiting for PX4 on TCP 4560...` |
| 2 — PX4 | `cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d` | bridge prints `PX4 connected. Bridge running.`; PX4 then prints `Ready for takeoff!` after EKF converges (~3 s) |
| 3 — Mission | `python ~/go_aero/fly_demo.py --loops 2 --altitude 5 --size 8` | takeoff → square → land |

The bridge requires the `chassis_IMU` prim to exist in the USD — rebuild the scene first (`python ~/go_aero/build_scene.py --headless --obstacles`) if you're upgrading from an earlier checkout.

Connect a viewer with NICE DCV at `https://52.91.203.66:8443` to see the Isaac Sim viewport.

---

## How to Run (ArduPilot Copter SITL)

### Mode 1 — Visualizer mode (ArduPilot `-f quad` internal physics + `px4_visualizer.py`)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — ArduPilot | `cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --no-rebuild --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14551` | `EKF3 IMU0 is using GPS` (~15 s) |
| 2 — Isaac Sim | `export DISPLAY=:1 && python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550` *(note: `--port 14550` is also supported as an alias)* | `Visualizer running!` |
| 3 — Flight / Obstacle Test | `python ~/go_aero/ardu_poshold.py --altitude 5 --throttle-pct 50 --hold 30` *(or `--mode guided-forward --target-north 8 --altitude 2.5`)* | GUIDED takeoff → PosHold / obstacle stop → LAND |

### Mode 2 — PhysX Bridge mode (ArduPilot `-f JSON` + `ardupilot_bridge.py`)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — Bridge | `export DISPLAY=:1 && python ~/go_aero/ardupilot_bridge.py --device cuda --port 9002 --lidar --zed --ros2` | `Waiting for ArduPilot SITL (-f JSON) on UDP 9002...` |
| 2 — ArduPilot | `cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --no-rebuild --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14551` | Bridge prints `ArduPilot SITL connected. Bridge running.`; ArduPilot prints `EKF3 IMU0 is using GPS` |
| 3 — Flight / Obstacle Test | `python ~/go_aero/ardu_poshold.py --altitude 5 --throttle-pct 50 --hold 30` | GUIDED takeoff → PosHold / obstacle stop → LAND |

---

## Ground Control Station (GCS) Setup

- **Mission Planner (ArduPilot SITL over TCP 5763 SSH tunnel):**
  ArduPilot SITL's `SERIAL2` listens on TCP port `5763`. Tunnel TCP `5763` over SSH from your workstation so Mission Planner never touches UDP `14550`:
  ```bash
  ssh -i isaac-sim-keypair.pem -L 5763:127.0.0.1:5763 ubuntu@52.91.203.66
  ```
  In Mission Planner, choose **TCP** → `127.0.0.1` → port `5763`.
- **QGroundControl (PX4 SITL on dedicated UDP port `14601`):**
  Never point QGroundControl at UDP `14550` while `px4_visualizer.py` is running on `14550`. Instead, start a separate MAVLink stream on UDP `14601` (`mavlink start -x -u 14600 -o 14601 -r 4000000`, using local port `14600` to avoid conflicting with PX4's onboard link on `14580`) and set QGroundControl's Comm Link to UDP port `14601`.

---

## Jetson HIL & Docker SIL Obstacle Detection

See **[JETSON_HIL.md](./JETSON_HIL.md)** and **[`../docker/jetson-sil/`](../docker/jetson-sil/)** for running the Go Aero 3D obstacle detection & avoidance stack:
- **Software-In-The-Loop (no physical Jetson required):** `docker compose -f docker/jetson-sil/docker-compose.yml up --build` runs the ROS 2 Humble `VFS-Go-Aero/visual-obstacle-detection` pipeline (`point_cloud` → `/merged_cloud` → `obstacle_detection` → `/merged_cloud/obstacles` → `obstacle_to_mavlink`), `latency_relay.py`, and MAVROS (`/mavros/obstacle_distance_3d/send` and `/mavros/obstacle/send`) on the same host.
- **Hardware-In-The-Loop (physical Jetson Orin NX):** Streams `/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`, `/lidar/points`, `/clock`, `/tf`, and `/tf_static` from Isaac Sim (`--lidar --zed --ros2`) to the Jetson over Fast DDS (`rmw_fastrtps_cpp`).

---

## How to Update

The scripts on the EC2 instance live in `~/go_aero/` — that's a **copy** of the repo files, not a clone. Three workflows:

### Workflow A — Edit on EC2, push back to git (typical)

```bash
# On the EC2 instance
cd ~/go_aero
nano px4_visualizer.py         # or vim/code-server

# When happy, copy back into the repo clone
cd ~/go_aero_repo               # the cloned CAD-for-isaacsim
git checkout sgoss/isaac-sim-import
cp ~/go_aero/{build_scene,open_scene,px4_visualizer,px4_bridge,ardupilot_bridge,ardu_poshold,fly_demo,joystick_control,add_lidar_tutorial,add_zed_tutorial,jetson_sil_launch,synthetic_zed_publisher,latency_relay,_frames,_compat}.py simulation/

# Commit and push
git add simulation/
git commit -m "Tweak visualizer: <what changed>"
git push origin sgoss/isaac-sim-import
```

### Workflow B — Edit locally, sync to EC2 via scp

```bash
# Local clone
git clone -b sgoss/isaac-sim-import git@github.com:VFS-Go-Aero/CAD-for-isaacsim.git
cd CAD-for-isaacsim
# ...edit simulation/*.py...

# Push edits to EC2
scp -i isaac-sim-keypair.pem simulation/*.py ubuntu@52.91.203.66:~/go_aero/

# Then commit + push as usual
git add simulation/ && git commit -m "..." && git push
```

### Workflow C — Pull from git on EC2

```bash
ssh -i isaac-sim-keypair.pem ubuntu@52.91.203.66
cd ~/go_aero_repo && git pull
cp simulation/*.py ~/go_aero/
cp simulation/params/* ~/go_aero/params/
```

### When you change the scene structure (`build_scene.py`)

You must rebuild the USD scene before the visualizer can see the change:

```bash
conda activate env_isaaclab_jazzy
python ~/go_aero/build_scene.py --headless --obstacles     # ~60 s
```

### When you change `px4_visualizer.py` or `fly_demo.py`

Just restart the corresponding terminal — no rebuild needed.

### When you change PX4 airframe parameters

```bash
cd ~/PX4-Autopilot
make px4_sitl                                   # rebuild PX4 (~3 min first time, ~30 s incremental)
```

Then restart the PX4 terminal.

---

## File Map

| File | Reads | Writes | Other side |
|---|---|---|---|
| `_frames.py` | — | Pure-Python NED/ENU/mesh quaternion conversions | Imported by `px4_visualizer.py`, `px4_bridge.py`, `ardupilot_bridge.py` |
| `_compat.py` | — | Isaac Sim 5.1 / 6.0 extension & `AppLauncher`/`SimulationApp` shims + prim paths | Imported by scene, bridge, visualizer, and tutorial scripts |
| `build_scene.py` | `~/go_aero/meshes/*.stl` | `~/go_aero/vehicle_scene.usd` (terrain, obstacles, sensor mounts) | — |
| `open_scene.py` | `~/go_aero/vehicle_scene.usd` | viewport | — (standalone, no PX4) |
| `px4_visualizer.py` | USD scene + MAVLink UDP `:14550` | viewport + optional ROS 2 sensor topics | PX4 SIH or ArduCopter SITL (`-f quad`) |
| `px4_bridge.py` | USD scene + MAVLink TCP/UDP `:4560` (bidirectional) | `HIL_SENSOR` / `HIL_GPS` out, `HIL_ACTUATOR_CONTROLS` in + optional ROS 2 topics | PX4 SITL (HIL mode) |
| `ardupilot_bridge.py` | USD scene + SIM_JSON UDP `:9002` (bidirectional) | JSON sensor state out (`\n{...}\n`), binary 16-ch PWM servos in + optional ROS 2 topics | ArduCopter SITL (`-f JSON`) |
| `fly_demo.py` | — | MAVSDK UDP `:14540` | PX4 (offboard) |
| `ardu_poshold.py` | MAVLink UDP `:14540` | GUIDED takeoff, RC3 override / POSHOLD, or GUIDED forward obstacle test | ArduCopter SITL |
| `joystick_control.py` | Xbox controller (USB) | MAVLink `MANUAL_CONTROL` over UDP `:14550` | PX4 / ArduPilot GCS endpoint |
| `add_lidar_tutorial.py` | `~/go_aero/vehicle_scene.usd` | Attaches RTX LiDAR at `/World/Vehicle/chassis/lidar_mount/lidar_sensor` | Standalone / ROS 2 verification |
| `add_zed_tutorial.py` | `~/go_aero/vehicle_scene.usd` | Attaches ZED X cameras at `zed1_mount` and `zed2_mount` | Standalone / ROS 2 verification |
| `jetson_sil_launch.py` | Launch args (`use_sim_time`, `fcu_url`, `latency_ms`, etc.) | Launches static TFs, `visual_obstacle_detection`, `latency_relay.py`, MAVROS | `docker/jetson-sil` or physical Jetson |
| `synthetic_zed_publisher.py` | CLI flags (`--obstacle-dist`, `--publish-clock`, `--self-test`) | `/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/...`, optional `/clock` | `point_cloud` node in `visual_obstacle_detection` |
| `latency_relay.py` | `/mavros/obstacle_distance_3d/send_raw` | Delayed `/mavros/obstacle_distance_3d/send` + 72-sector `/mavros/obstacle/send` | MAVROS / ArduPilot (`OBSTACLE_DISTANCE_3D`) / PX4 (`OBSTACLE_DISTANCE`) |
| `params/go_aero_ardupilot.param` | — | Loaded by `sim_vehicle.py --add-param-file` | ArduCopter SITL (`PRX1_TYPE=2`, `AVOID_ENABLE=7`) |

See [PX4_API.md](./PX4_API.md) for the MAVLink message and MAVSDK method details, [ARCHITECTURE.md](./ARCHITECTURE.md) for the full data flow, and [JETSON_HIL.md](./JETSON_HIL.md) for the Jetson / Docker SIL obstacle avoidance pipeline.

---

## Common Tasks

### Change the demo flight pattern
Edit `fly_demo.py` — the `waypoints` list and the `for loop in range(args.loops)` body.

### Add a new MAVLink message handler in the visualizer
Edit `px4_visualizer.py` `mavlink_receiver()` — add an `elif msg_type == 'YOUR_MSG':` branch and store fields into `vehicle_state`.

### Move the simulated home location
Edit `px4_bridge.py` / `ardupilot_bridge.py` constants `HOME_LAT`, `HOME_LON`, `HOME_ALT` (currently set to Boston for the Mass Robotics demo).

### Change vehicle mass / inertia / motor layout
Edit `build_scene.py` — `mass_api.CreateMassAttr(...)`, the `props` dict (motor positions), and prop mass. Then **rebuild the scene**.

### Change the camera angle
Edit the `cam.set_position_world(...)` and `cam.set_target_world(...)` calls near the top of `px4_visualizer.py` or `open_scene.py`.

---

## HIL Bridge — Airframe Setup

The `none_go_aero` airframe (`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero`) must tell PX4 to run in HIL mode and disable its internal sensor sims. The PX4 rc script `px4-rc.mavlinksim` already starts `simulator_mavlink` on TCP port 4560 when `PX4_SIM_MODEL` is not `sihsim`/`gz`, so no extra `simulator_mavlink` line is needed — we just need the param flips:

```sh
# HIL mode: simulator provides sensors, PX4 provides actuators
param set-default SYS_HITL 1
param set-default SENS_EN_GPSSIM 0
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 0
```

Re-apply after a fresh PX4 checkout with:

```bash
sed -i '/^set AUTOCNF/a param set-default SYS_HITL 1\nparam set-default SENS_EN_GPSSIM 0\nparam set-default SENS_EN_BAROSIM 0\nparam set-default SENS_EN_MAGSIM 0' \
  ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero
cd ~/PX4-Autopilot && make px4_sitl
```

The bridge defaults to `--protocol tcp` to match PX4's out-of-the-box TCP client. If you patch `px4-rc.mavlinksim` to launch `simulator_mavlink` with `-u`, also pass `--protocol udp` to the bridge.

The SIH visualizer mode is unaffected — it uses a different airframe (`10040_sihsim_quadx`) that leaves `SYS_HITL=0`.

---

## HIL Bridge — Verification

After a rebuild + PX4 restart, work through these steps to confirm the loop is actually closed:

1. **Rebuild scene** — `python ~/go_aero/build_scene.py --headless --obstacles`. Grep the resulting USD for `chassis_IMU` to confirm the sensor prim exists.
2. **Start bridge first** — `python ~/go_aero/px4_bridge.py --device cuda --px4-port 4560`. The bridge binds the TCP port and prints `Waiting for PX4 on TCP 4560...`. It must be up before PX4, because PX4's `simulator_mavlink` is the TCP *client*.
3. **Start PX4** — `PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d`. PX4 connects into the bridge; the bridge prints `PX4 connected. Bridge running.` and ~3 s later PX4 prints `Ready for takeoff!` (EKF converged on our sensors).
4. **Fly offboard** — `python ~/go_aero/fly_demo.py --altitude 5 --size 8 --loops 2`. Expect the bridge's status line to show motors hovering around ~0.5–0.6 during hover, not pegged at 0 or 1. In the DCV viewport the vehicle should visibly pitch forward/back while moving.
5. **Sanity kill** — stop the bridge mid-flight. PX4 must enter failsafe within `COM_OF_LOSS_T` seconds and print `Sensor timeout`. If it doesn't, PX4 is still using its internal SIH and the HIL airframe changes above didn't take effect.
6. **Regression** — the old visualizer path (`px4_visualizer.py`) should still work unchanged against the SIH airframe.

## Xbox Manual Flight (`joystick_control.py`)

`joystick_control.py` runs on your **local Windows machine** (not on EC2) and feeds an Xbox controller's sticks into PX4 as `MANUAL_CONTROL` MAVLink messages over UDP. PX4 already binds 14550 for its default GCS instance, so no PX4-side changes are needed — just open UDP 14550 inbound on the VividCloud security group to your home IP.

### Setup

1. Plug Xbox controller into the Windows machine, confirm Windows sees it under *Set up USB game controllers*.
2. `pip install pygame pymavlink`
3. Open the EC2 security group `isaac-sim-sg` (in the VividCloud `us-east-1` account) and add an inbound UDP rule for port 14550 from your current IP (`curl ifconfig.me`). Rule source: `your.ip.here/32`.
4. Start PX4 on EC2 (either SIH or HIL mode works):
   ```bash
   cd ~/PX4-Autopilot/build/px4_sitl_default
   PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d    # SIH path
   # or the HIL path — also start px4_bridge.py in a second terminal
   ```
5. From Windows: `python simulation/joystick_control.py --ec2-ip 52.91.203.66`
6. **A** = arm, **B** = disarm, **Y** = cycle mode (STABILIZED → ALTITUDE → POSITION), sticks = control.

### Stick layout

| Xbox axis | PX4 field | Range |
|---|---|---|
| Left stick X | roll | [-1000, 1000] |
| Left stick Y | pitch (inverted) | [-1000, 1000] |
| Right stick X | yaw | [-1000, 1000] |
| Right stick Y | throttle (remapped) | [0, 1000] |

5 % deadzone on each axis by default; pass `--deadzone 0.1` to widen.

### Failsafe behaviour

If the controller disconnects mid-flight, the pygame axes return zero and the script keeps streaming neutral sticks. If the script itself dies, PX4's `COM_RC_LOSS_T` failsafe (default 0.5 s) triggers and the autopilot auto-lands / RTLs depending on the param.

On graceful shutdown (Ctrl-C) the script sends 1 s of neutral sticks followed by a disarm command, so the vehicle settles rather than latching the last stick input.

---

## HIL Bridge — Tuning

The bridge exposes four tuning knobs at the top of `px4_bridge.py` and `ardupilot_bridge.py`:

| Constant | Default | Notes |
|---|---|---|
| `MAX_THRUST_PER_MOTOR` | `2 × hover ≈ 24.5 N` | Bump to 3× hover for acro-style response |
| `K_TAU` | `0.016 m` | Moment-per-unit-thrust. Adjust if yaw response is too soft/twitchy |
| `ACC_NOISE_STD`, `GYRO_NOISE_STD`, ... | small | Pass `--no-noise` to disable entirely while debugging |
| `SENSOR_RATE` | `250` Hz | Physics step rate + HIL_SENSOR / SIM_JSON send rate. Keep them equal |

Inertia and mass live in `build_scene.py` — edit `mass_api.CreateMassAttr` / `CreateDiagonalInertiaAttr` and rebuild the scene.

---

## Stopping & Costs

The instance is `g6e.2xlarge` (NVIDIA L40S, 46 GB VRAM) — about **\$2/hr on-demand** when running. An EC2 instance scheduler tag (`Schedule = isaacsim`) auto-stops it nightly, but stop it manually when done:

```bash
aws ec2 stop-instances --profile vividcloud-exp --region us-east-1 --instance-ids i-00f42bf1ca1f91f99
```
