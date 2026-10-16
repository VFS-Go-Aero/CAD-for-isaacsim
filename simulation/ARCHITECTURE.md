# Simulation Architecture

How the Go Aero simulation hangs together — processes, data flow, coordinate frames, and threading.

## Two Simulation Modes

### Mode 1 — PX4 SIH (or ArduPilot SITL) + Isaac Sim Visualizer  *(default, working)*

PX4's built-in **Simulation-In-Hardware (SIH)** module (or ArduPilot SITL's built-in physics) simulates the entire vehicle internally (generic quadrotor model). Isaac Sim is a viewer that receives the autopilot's pose over MAVLink and animates the 3D model, optionally publishing `/clock`, `/tf`, and simulated ZED / LiDAR point clouds to ROS 2 (`--ros2`).

```
 ┌──────────────────────┐                ┌──────────────────────┐                ┌──────────────────────┐
 │   fly_demo.py        │                │   PX4 SITL (sihsim)  │                │  px4_visualizer.py   │
 │   (MAVSDK client)    │   UDP 14540    │                      │   UDP 14550    │  (Isaac Sim)         │
 │   ──────────────     ├───────────────►│   ──────────────     ├───────────────►│   ──────────────     │
 │   - arm/disarm       │  setpoints     │   - SIH physics      │  LOCAL_POS_NED │   - open USD scene   │
 │   - takeoff/land     │  offboard cmds │   - EKF              │  ATTITUDE      │   - mirror pose      │
 │   - position NED     │                │   - controller       │  SERVO_OUTPUT  │   - spin props       │
 │                      │                │   - actuator output  │  HEARTBEAT     │   - render / --ros2  │
 └──────────────────────┘                └──────────────────────┘                └──────────────────────┘
```

**Properties:** No physics in Isaac Sim. Visualization (and optional ROS 2 sensor rendering with `--ros2`) only. PX4's or ArduPilot's internal quadrotor model handles dynamics, EKF, and control internally.

### Mode 2A — Isaac Sim PhysX + PX4 SITL HIL  *(`px4_bridge.py`)*

Isaac Sim is the source of truth for physics. It simulates IMU/GPS/baro/mag from an `IMUSensor` attached to the chassis rigid body plus velocity/position from the same rigid body, sends them to PX4 over the MAVLink **HIL_SENSOR** / **HIL_GPS** path, and applies PX4's motor commands as per-prop thrust forces and a net yaw torque on the chassis.

```
 ┌──────────────────────┐                                      ┌──────────────────────┐
 │   px4_bridge.py      │   HIL_SENSOR  (250 Hz)               │                      │
 │   (Isaac Sim PhysX)  │   HIL_GPS     (10 Hz)                │   PX4 SITL           │
 │   ──────────────     ├─────────────────────────────────────►│   (none_go_aero)     │
 │   - rigid bodies     │   HEARTBEAT   (1 Hz)                 │   ──────────────     │
 │   - PhysX @ 250 Hz   │                                      │   - EKF              │
 │   - IMUSensor        │       TCP 4560 (bidirectional)       │   - controller       │
 │   - force + torque   │◄─────────────────────────────────────┤   - mixer            │
 │     on chassis       │   HIL_ACTUATOR_CONTROLS (motor cmds) │                      │
 └──────────────────────┘                                      └──────────────────────┘
```

PX4's `simulator_mavlink` module is the one that opens the connection — it's the TCP *client* (default) and dials out to the bridge's listening socket. The bridge uses a single `tcpin:0.0.0.0:4560` pymavlink connection for both directions. Pass `--protocol udp` to the bridge (and patch `px4-rc.mavlinksim` to use `-u`) if you'd rather run UDP.

### Mode 2B — Isaac Sim PhysX + ArduPilot SITL JSON Backend  *(`ardupilot_bridge.py`)*

When running with **ArduPilot (`arducopter`)** instead of PX4, Isaac Sim connects to ArduPilot SITL's native `-f JSON` backend (`sim_vehicle.py -v ArduCopter -f JSON`) over **UDP port 9002**:

```
 ┌──────────────────────┐                                      ┌──────────────────────┐
 │ ardupilot_bridge.py  │   JSON state (gyro, accel, pos,      │   ArduCopter SITL    │
 │   (Isaac Sim PhysX)  │   attitude, velocity, timestamp)     │   (-f JSON)          │
 │   ──────────────     ├─────────────────────────────────────►│   ──────────────     │
 │   - rigid bodies     │                                      │   - EKF3             │
 │   - PhysX @ 250 Hz   │       UDP 9002 (lockstep / JSON)     │   - attitude/pos ctl │
 │   - IMUSensor        │◄─────────────────────────────────────┤   - motor mixer      │
 │   - ZED / LiDAR /    │   16-ch binary servo PWM packet      │   - TCP 5760/5762/   │
 │     ROS 2 (--ros2)   │   (magic 18458 / 29569)              │     5763 (MAVLink)   │
 └──────────────────────┘                                      └──────────────────────┘
```

ArduPilot SITL binds **UDP 9002** and sends 16-channel PWM motor frames to `ardupilot_bridge.py`, which steps PhysX, applies motor forces/torques, and replies with newline-terminated JSON state. ArduPilot simultaneously exposes serial-over-TCP MAVLink ports (`5760` primary / MAVProxy, `5762` MAVROS, `5763` secondary GCS / Mission Planner).

**Properties:** Real airframe parameters (mass, inertia tensor, motor placement, prop characteristics) drive dynamics. Required for tuning the autopilot against the actual Go Aero airframe.

---

## Process Topology

| Process | Conda env? | GPU? | Network |
|---|---|---|---|
| `bin/px4 -d` (PX4 SITL) | No | No | binds local UDP `:18570` and **sends to** UDP `14550` (GCS / visualizer); binds local UDP `:14580` and **sends to** UDP `14540` (MAVSDK / MAVROS); HIL mode additionally dials out as a TCP client to `4560` |
| `arducopter --model JSON` (ArduPilot SITL) | No | No | binds UDP `9002` (JSON physics backend) and listens on TCP `5760` / `5762` / `5763` (MAVLink serial ports; MAVProxy can fan out UDP `14550` / `14551`) |
| `python px4_visualizer.py` | **yes** (`env_isaaclab_jazzy`) | **yes** (`--device cuda`) | **binds** UDP `0.0.0.0:14550` (`udpin`) |
| `python fly_demo.py` | **yes** (`env_isaaclab_jazzy`) | No | **binds** UDP `0.0.0.0:14540` (`udpin`) |
| `python px4_bridge.py` | **yes** (`env_isaaclab_jazzy`) | **yes** | **binds** TCP `0.0.0.0:4560` (`tcpin`, bidirectional; PX4 connects in) |
| `python ardupilot_bridge.py` | **yes** (`env_isaaclab_jazzy`) | **yes** | exchanges binary PWM / JSON state with ArduPilot SITL on UDP `9002` |
| `python joystick_control.py` *(Phase 2, local Windows machine)* | No | No | sends `MANUAL_CONTROL` over UDP |

For the **Visualizer** path: PX4 (or ArduPilot + MAVProxy) must start first so its MAVLink output is already sending to UDP `14550` when the visualizer binds `0.0.0.0:14550`. MAVSDK clients (`fly_demo.py`) can start any time after PX4.

For the **PX4 HIL Bridge** path: start order is **bridge first, PX4 second**. PX4's `simulator_mavlink` module connects out to the bridge's listening TCP socket (`4560`); if the bridge isn't up yet, PX4 prints `Waiting for simulator to connect…` and halts sensor wiring until it sees us.

---

## Coordinate Frames

This is the single biggest source of bugs. Three frames are involved:

| Frame | Convention | Used by |
|---|---|---|
| **PX4 / ArduPilot NED** | x = north, y = east, z = **down** (negative = up) | All PX4 / ArduPilot telemetry + MAVSDK setpoints |
| **Isaac Sim ENU (world)** | x = east, y = north, z = **up** | The USD stage |
| **STL mesh / vehicle local** | y = up (mesh native) → rotated +90° X to match world Z-up | Inside `/World/Vehicle` |

### Conversions used in the code

`px4_visualizer.py` mavlink_receiver:
```python
# PX4 LOCAL_POSITION_NED -> Isaac Sim ENU (X=east, Y=north, Z=up)
vehicle_state['x'] =  msg.y     # ned.east -> enu.x
vehicle_state['y'] =  msg.x     # ned.north -> enu.y
vehicle_state['z'] = -msg.z     # ned.down -> -down = up
```

`fly_demo.py` waypoints — already in **NED** (north, east, down), no conversion needed (passed directly to MAVSDK):
```python
alt = -args.altitude            # NED: negative = up
PositionNedYaw(north, east, alt, yaw)
```

`px4_bridge.py` reverse conversion (Isaac Sim ENU pose → GPS):
```python
lat_offset = float(pos[1]) / 111139.0                                     # Y(=north) -> lat
lon_offset = float(pos[0]) / (111139.0 * math.cos(math.radians(HOME_LAT)))  # X(=east) -> lon
```

---

## Components in Detail

### `build_scene.py`  →  `vehicle_scene.usd`

Pure-USD scene assembly. Run once per scene structure change.

1. **Physics scene** — gravity (0, 0, −9.81)
2. **Ground plane + terrain + lighting** — distant + dome lights + procedural uneven terrain mesh
3. **Vehicle Xform** at `/World/Vehicle`, `RotateXOp(90)` to convert mesh Y-up to world Z-up
4. **Chassis** — rigid body, `mass = 5 kg`, simplified box collider, marked as `ArticulationRootAPI`
5. **Sensor mounts on `/World/Vehicle/chassis`** (rigidly attached to the chassis so they do not spin with the propellers):
   - `chassis_IMU` at `(0, 0, 0)`
   - `lidar_mount` at `(0.468, -0.04, -0.515)` with `RotateX(180)` (downward-facing under front-right arm)
   - `zed1_mount` at `(0.0, -0.127, -0.1524)` with identity rotation (forward-facing ZED camera, 5 in below & 6 in forward of center)
   - `zed2_mount` at `(0.0, -0.127, -0.1524)` with identity rotation (second forward-facing ZED camera, matching `multi_zed.launch.py`)
6. **4 propellers** — each: STL mesh, rigid body (`0.1 kg`), revolute joint to chassis around Y axis, `DriveAPI('angular')` with velocity drive (stiffness=0, damping=0.5)

Output: `~/go_aero/vehicle_scene.usd`. The visualizer and bridges read this read-only.

### `open_scene.py`  *(standalone, no PX4 — for testing the scene + physics)*

Opens the scene and runs a **PID altitude controller** on the prop joint drives. PX4 is not involved. Useful for:
- Verifying the scene loads (including in `--headless` mode)
- Tuning the prop velocity → thrust scaling
- Testing PhysX behavior in isolation

PID gains are constants in the file (`KP=15`, `KD=8`, `KI=2`).

### `px4_visualizer.py`  *(default mode)*

The main visualizer. Two threads:

| Thread | Loop | Owns |
|---|---|---|
| **MAVLink RX** (`mavlink_receiver`) | `mav.recv_match(blocking=True, timeout=1.0)` | reads UDP 14550, writes `vehicle_state` dict |
| **Main / Isaac Sim** | `while simulation_app.is_running(): app.update()` | reads `vehicle_state` (under `state_lock`), updates USD attrs |

`vehicle_state` is a plain dict guarded by a `threading.Lock`. The main loop:
1. Snapshot the dict (under lock)
2. Set `xformOp:translate` on `/World/Vehicle` to ENU position
3. For each prop, integrate `prop_angles[name] += motors[i] * 30.0 * spin_dir` and set `xformOp:rotateY`

Pass `--ros2` (and optionally `--zed` / `--lidar`) to enable `isaacsim.ros2.bridge` in Mode 1 so `/clock`, `/tf`, `/tf_static`, and simulated ZED / LiDAR point clouds are published for SIL testing.

**Important:** The visualizer requests a 50 Hz data stream from PX4 with `request_data_stream_send(MAV_DATA_STREAM_ALL, 50, 1)` after the first heartbeat — without this you'll only get heartbeats and won't see the drone move.

### `fly_demo.py`  *(MAVSDK mission script)*

Uses **MAVSDK-Python** (high-level async API). Connects to PX4 on UDP 14540, runs an `asyncio` event loop:

1. `connect()` — wait for connection_state.is_connected
2. `telemetry.health()` — wait for `is_global_position_ok` and `is_home_position_ok`
3. `offboard.set_position_ned(...)` once to seed the setpoint
4. `action.arm()`
5. `offboard.start()`
6. Loop waypoints with `offboard.set_position_ned(...)` + `wait_for_position()` polling
7. `offboard.stop()` → `action.land()` → `action.disarm()`

**Why offboard mode and not a mission upload?** Offboard gives smooth real-time control (good for visualization). For real autonomous testing later you'd switch to `mission` mode with uploaded waypoints.

### `px4_bridge.py`  *(HIL — closed loop)*

The full HIL bridge. Two threads, same shape as the visualizer, but the data flows the *other* way: Isaac Sim is the source of truth.

**Receiver thread** — drains the single bidirectional `tcpin:0.0.0.0:4560` connection (or `udpin:` if `--protocol udp`):
- `HIL_ACTUATOR_CONTROLS` → clamps 4 motor commands into `motor_commands[]` under a `threading.Lock`
- `HEARTBEAT` → updates `armed` from `MAV_MODE_FLAG_SAFETY_ARMED`

**Main / physics loop** — one `app.update()` per PhysX tick at `SENSOR_RATE` (250 Hz):
1. `chassis.get_world_pose()` and `chassis.get_linear_velocity()` — ENU world pose and velocity
2. `imu.get_current_frame()` — mesh-body-frame `lin_acc` (with gravity) and `ang_vel`
3. Build the sensor packet:
   - Accelerometer + gyro: convert mesh-body → FRD via `mesh_body_to_frd()`
   - Magnetometer: rotate the NED world field `B_WORLD_NED` into the mesh-body frame via the chassis quaternion's conjugate, then FRD-convert
   - Barometer: ISA altitude formula on `pos.z + HOME_ALT`
4. `hil_sensor_send(fields_updated=0x1FFF)` — all 13 fields
5. `hil_gps_send(…)` at 10 Hz with converted ENU→NED velocity (cm/s)
6. `heartbeat_send(MAV_TYPE_GENERIC)` at 1 Hz
7. Read the latest clamped motor commands, compute body-frame thrust `F_body += (0, T_i, 0)`, cross-product moment `M_body += r_body × F_i`, and reaction yaw `M_body.y -= K_TAU * T_i * spin_dir`
8. Rotate body-frame force/torque into world frame with the chassis quaternion and push via `chassis.apply_forces_and_torques_at_pos(is_global=True, positions=None)`
9. Separately set each prop joint's `DriveAPI:angular` target velocity for the visual spin — decoupled from the force model so joint behaviour doesn't bleed into flight dynamics

Optional flags:
- `--lidar`: attaches an RTX LiDAR at `/World/Vehicle/chassis/lidar_mount/lidar_sensor`
- `--zed`: attaches simulated ZED stereo/depth cameras at `/World/Vehicle/chassis/zed1_mount` and `zed2_mount`
- `--ros2`: enables `isaacsim.ros2.bridge` and publishes `/clock`, `/tf`, `/tf_static`, `/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`, and `/lidar/points`

Timing: MAVLink timestamps use `timeline.get_current_time()` (sim seconds) rather than wall-clock so the sensor stream stays consistent even when the viewport stutters. The bridge blocks on `px4_connected.wait()` before the first sensor packet is sent — this is the "connect-ready gate" that replaces `wait_heartbeat()`.

Frame conversions are centralised in `_frames.py` (shared with the visualizer).

---

## Network Ports

| Port | Direction | Protocol | Used by |
|---|---|---|---|
| **14540** | PX4 (`:14580`) → MAVSDK / MAVROS clients (`:14540`) | UDP | `fly_demo.py` or MAVROS binds `0.0.0.0:14540` (`udpin`), PX4 sends to `14540` |
| **14550** | PX4 (`:18570`) ↔ primary GCS / visualizer (`:14550`) | UDP | Upstream PX4 SITL binds local port `18570` and **sends to** UDP `14550`; `px4_visualizer.py` is what **binds** `0.0.0.0:14550` (`udpin`) |
| **14601** *(optional)* | PX4 (`:14600`) ↔ 2nd GCS (`:14601`) | UDP | Secondary GCS (QGroundControl / Mission Planner) via `mavlink start -x -u 14600 -o 14601` so it doesn't conflict with `px4_visualizer.py` on `14550` |
| **4560** | bidirectional (PX4 client → bridge server) | **TCP** (default) or UDP | `px4_bridge.py` binds and listens (`tcpin:0.0.0.0:4560`), PX4's `simulator_mavlink` dials out to it (HIL mode only). Use `--protocol udp` if you patch `px4-rc.mavlinksim` |
| **9002** | bidirectional (ArduPilot SITL ↔ physics bridge) | UDP | `ardupilot_bridge.py` (Mode 2B JSON physics backend: 16-ch PWM out from ArduPilot, JSON sensor state in from Isaac Sim) |
| **5760 / 5762 / 5763** | ArduPilot SITL serial0 / serial1 / serial2 | TCP | ArduPilot SITL MAVLink TCP ports for MAVProxy (`5760`), MAVROS (`5762`, e.g. `fcu_url:=tcp://127.0.0.1:5762`), and Mission Planner / QGC (`5763`) |
| **8443** | DCV remote desktop | TCP/TLS | NICE DCV viewer ↔ EC2 |
| **22** | SSH | TCP | terminal access |

### Why a Second GCS Needs Its Own Port (Avoiding UDP 14550 Conflicts)

Upstream PX4 SITL does **not** bind `14550` — it binds local UDP port `18570` and sends packets **to** destination port `14550`. Meanwhile, `px4_visualizer.py` binds `0.0.0.0:14550` (`udpin:0.0.0.0:14550`). If you open QGroundControl or Mission Planner on the same host (or forward `14550`), both the GCS and `px4_visualizer.py` try to bind/consume UDP `14550`, causing either `Address already in use` or stolen telemetry packets.

- **With PX4 SITL:** Start a dedicated MAVLink instance in PX4 (`mavlink start -x -u 14600 -o 14601 -r 4000000`) and point QGroundControl / Mission Planner at UDP `14601`.
- **With ArduPilot SITL:** Either connect Mission Planner / QGroundControl directly to ArduPilot's `SERIAL2` TCP port (`5763`), or pass multiple `--out` flags to MAVProxy (`--out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551`) so `px4_visualizer.py` gets `14550` and the GCS gets `14551`.

---

## Threading & Timing

**Visualizer mode.** The main loop runs as fast as Isaac Sim updates (~60 fps). MAVLink messages arrive on a background thread and just write into a dict — they're never blocked by the renderer. The lock is held only for the snapshot copy in the main loop, so contention is minimal. PX4 SIH internally runs at 1 kHz. The 50 Hz data stream we request gives us ~17 ms-old pose data on average — fine for visualization.

**HIL bridge mode.** The main loop is the physics loop — each `app.update()` call advances PhysX by `1/SENSOR_RATE = 4 ms`. Sensor packets are sent every physics tick (250 Hz); GPS every 25 ticks; heartbeats every 250 ticks. The receiver thread is a thin drain loop: `recv_match(blocking=True, timeout=1.0)` → write into `motor_commands[]` under a `threading.Lock`. The main loop snapshots the list once per tick so lock contention is one short copy per 4 ms.

Viewport rendering is decoupled from physics: `--render-hz` (default 30) throttles how often the main loop forces a viewport refresh. Physics never waits on rendering.

If you see jittery motion, check:
1. Visualizer: the data stream rate is actually being sent (`request_data_stream_send` succeeded)
2. The conda env didn't get the wrong pymavlink (only the one in `env_isaaclab_jazzy` should be active)
3. Isaac Sim's actual frame rate via the `[INFO] t=…` status line (bridge) or `step % 300 == 0` (visualizer)
4. HIL only: PX4 is not *also* trying to drive its own simulator — `SYS_HITL=1` and `SENS_EN_{GPSSIM,BAROSIM,MAGSIM}=0` in the airframe file must be set

---

## Why This Design

- **PX4 SIH for the visualizer mode** — zero-config, zero physics setup, runs anywhere PX4 builds. Trades realism for simplicity.
- **MAVLink + MAVSDK split** — MAVLink (`pymavlink`) for low-level/sensor traffic in the visualizer/bridge; MAVSDK for high-level mission scripts. This matches how real ground stations and onboard companions are typically structured.
- **USD scene built once, reloaded fast** — `build_scene.py` runs ~60 s; `px4_visualizer.py` opens the cached USD in ~10 s after the first time Isaac Sim warms up.
- **Pose-only update path in the visualizer** — sidesteps PhysX entirely so the renderer never fights PX4 for control. When we move to HIL via `px4_bridge.py` (or `ardupilot_bridge.py`), the trade flips: PhysX wins, the autopilot commands forces.
