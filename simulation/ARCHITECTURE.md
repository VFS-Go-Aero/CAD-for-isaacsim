# Simulation Architecture

How the Go Aero simulation hangs together — processes, data flow, coordinate frames, and threading.

## Two Simulation Modes

### Mode 1 — PX4 SIH + Isaac Sim Visualizer  *(default, working)*

PX4's built-in **Simulation-In-Hardware (SIH)** module simulates the entire vehicle internally (generic quadrotor model). Isaac Sim is a passive viewer that receives PX4's pose over MAVLink and animates the 3D model.

```
 ┌──────────────────────┐                ┌──────────────────────┐                ┌──────────────────────┐
 │   fly_demo.py        │                │   PX4 SITL (sihsim)  │                │  px4_visualizer.py   │
 │   (MAVSDK client)    │   UDP 14540    │                      │   UDP 14550    │  (Isaac Sim)         │
 │   ──────────────     ├───────────────►│   ──────────────     ├───────────────►│   ──────────────     │
 │   - arm/disarm       │  setpoints     │   - SIH physics      │  LOCAL_POS_NED │   - open USD scene   │
 │   - takeoff/land     │  offboard cmds │   - EKF              │  ATTITUDE      │   - mirror pose      │
 │   - position NED     │                │   - controller       │  SERVO_OUTPUT  │   - spin props       │
 │                      │                │   - actuator output  │  HEARTBEAT     │   - render viewport  │
 └──────────────────────┘                └──────────────────────┘                └──────────────────────┘
```

**Properties:** No physics in Isaac Sim. Visualization only. PX4's generic quadrotor model handles dynamics, EKF, controller — all internally.

### Mode 2 — Isaac Sim PhysX + PX4 SITL HIL  *(`px4_bridge.py`)*

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

**Properties:** Real airframe parameters (mass, inertia tensor, motor placement, prop characteristics) drive dynamics. Required for tuning the autopilot against the actual Go Aero airframe.

---

## Process Topology

| Process | Conda env? | GPU? | Network |
|---|---|---|---|
| `bin/px4 -d` (PX4 SITL) | No | No | binds UDP 14550 (MAVLink GCS), 14540 (MAVSDK offboard); HIL mode additionally dials out to the bridge on TCP 4560 |
| `python px4_visualizer.py` | **yes** (`env_isaaclab_jazzy`) | **yes** (`--device cuda`) | binds UDP 14550 input |
| `python fly_demo.py` | **yes** (`env_isaaclab_jazzy`) | No | binds UDP 14540 input |
| `python px4_bridge.py` | **yes** (`env_isaaclab_jazzy`) | **yes** | binds TCP 4560 (bidirectional; PX4 connects in) |
| `python joystick_control.py` *(Phase 2, local Windows machine)* | No | No | sends UDP to EC2:14550 |

For the **Visualizer** path: PX4 must start first so its MAVLink output is already publishing to 14550 when the visualizer binds. MAVSDK clients (`fly_demo.py`) can start any time after PX4.

For the **HIL Bridge** path: start order is **bridge first, PX4 second**. PX4's `simulator_mavlink` module connects out to the bridge's listening socket; if the bridge isn't up yet, PX4 prints `Waiting for simulator to connect…` and halts sensor wiring until it sees us.

---

## Coordinate Frames

This is the single biggest source of bugs. Three frames are involved:

| Frame | Convention | Used by |
|---|---|---|
| **PX4 NED** | x = north, y = east, z = **down** (negative = up) | All PX4 telemetry + MAVSDK setpoints |
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
2. **Ground plane + lighting** — distant + dome lights
3. **Vehicle Xform** at `/World/Vehicle`, `RotateXOp(90)` to convert mesh Y-up to world Z-up
4. **Chassis** — rigid body, `mass = 5 kg`, simplified box collider, marked as `ArticulationRootAPI`
5. **4 propellers** — each: STL mesh, rigid body (`0.1 kg`), revolute joint to chassis around Y axis, `DriveAPI('angular')` with velocity drive (stiffness=0, damping=0.5)

Output: `~/go_aero/vehicle_scene.usd`. The visualizer reads this read-only.

### `open_scene.py`  *(standalone, no PX4 — for testing the scene + physics)*

Opens the scene and runs a **PID altitude controller** on the prop joint drives. PX4 is not involved. Useful for:
- Verifying the scene loads
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

Timing: MAVLink timestamps use `timeline.get_current_time()` (sim seconds) rather than wall-clock so the sensor stream stays consistent even when the viewport stutters. The bridge blocks on `px4_connected.wait()` before the first sensor packet is sent — this is the "connect-ready gate" that replaces `wait_heartbeat()`.

Frame conversions are centralised in `_frames.py` (shared with the visualizer).

---

## Network Ports

| Port | Direction | Protocol | Used by |
|---|---|---|---|
| 14540 | PX4 → MAVSDK clients | UDP | `fly_demo.py` listens, PX4 publishes |
| 14550 | PX4 ↔ ground stations | UDP | `px4_visualizer.py` listens, PX4 publishes; `joystick_control.py` sends `MANUAL_CONTROL` in |
| 4560 | bidirectional | TCP (default) or UDP | `px4_bridge.py` binds and listens, PX4's `simulator_mavlink` dials out to it (HIL mode only). TCP is PX4's default transport; use `--protocol udp` if you patch `px4-rc.mavlinksim` |
| 8443 | DCV remote desktop | TCP/TLS | NICE DCV viewer ↔ EC2 |
| 22 | SSH | TCP | terminal access |

PX4's default SITL config opens 14540 + 14550 simultaneously, so MAVSDK and the visualizer can run together.

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
- **Pose-only update path in the visualizer** — sidesteps PhysX entirely so the renderer never fights PX4 for control. When we move to HIL via `px4_bridge.py`, the trade flips: PhysX wins, PX4 commands forces.
