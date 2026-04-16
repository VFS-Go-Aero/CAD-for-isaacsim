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

### Mode 2 — Isaac Sim PhysX + PX4 SITL HIL  *(work in progress, `px4_bridge.py`)*

Isaac Sim is the source of truth for physics. It simulates IMU/GPS/baro/mag, sends them to PX4 over the MAVLink **HIL_SENSOR** / **HIL_GPS** path, and applies PX4's motor commands as physical forces in the scene.

```
 ┌──────────────────────┐                                      ┌──────────────────────┐
 │   px4_bridge.py      │   HIL_SENSOR  (250 Hz)               │                      │
 │   (Isaac Sim PhysX)  │   HIL_GPS     (10 Hz)                │   PX4 SITL           │
 │   ──────────────     ├─────────────────────────────────────►│   (none_go_aero)     │
 │   - rigid bodies     │   HEARTBEAT   (1 Hz)                 │   ──────────────     │
 │   - PhysX            │       UDP 14560                      │   - EKF              │
 │   - sensor sim       │                                      │   - controller       │
 │   - motor force      │◄─────────────────────────────────────┤                      │
 │     application      │   HIL_ACTUATOR_CONTROLS (motor cmds) │                      │
 └──────────────────────┘                                      └──────────────────────┘
```

**Properties:** Real airframe parameters (mass, motor placement, prop characteristics) drive dynamics. Required for tuning autopilot for the actual Go Aero airframe.

---

## Process Topology

| Process | Conda env? | GPU? | Network |
|---|---|---|---|
| `bin/px4 -d` (PX4 SITL) | No | No | binds UDP 14550 (MAVLink), 14540 (MAVSDK), 14560 (HIL) |
| `python px4_visualizer.py` | **yes** (`env_isaaclab_jazzy`) | **yes** (`--device cuda`) | binds UDP 14550 input |
| `python fly_demo.py` | **yes** (`env_isaaclab_jazzy`) | No | binds UDP 14540 input |
| `python px4_bridge.py` | **yes** (`env_isaaclab_jazzy`) | **yes** | UDP 14560 in/out |

PX4 must start *before* the visualizer/bridge so the sockets are open when the viewer first connects. MAVSDK clients (`fly_demo.py`) can start any time after PX4.

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

### `px4_bridge.py`  *(HIL — work in progress)*

The full HIL bridge (vs. the visualizer's read-only mode). Two threads, same shape as the visualizer, but the data flows the *other* way: Isaac Sim is the source of truth.

Periodic sender in main loop:
- `HIL_SENSOR` at 250 Hz (`SENSOR_RATE`) — IMU + baro + mag
- `HIL_GPS` at 10 Hz (`GPS_RATE`)
- `HEARTBEAT` at 1 Hz

Receiver thread parses `HIL_ACTUATOR_CONTROLS` from PX4 and stores 4 normalized motor commands. The main loop should apply these as forces on the prop rigid bodies — **this is the WIP part**: currently the bridge sends sensors but doesn't yet apply motor forces back into PhysX. See *Next Steps* in the [root README](../README.md#next-steps-for-accurate-simulation).

---

## Network Ports

| Port | Direction | Protocol | Used by |
|---|---|---|---|
| 14540 | PX4 → MAVSDK clients | UDP | `fly_demo.py` listens, PX4 publishes |
| 14550 | PX4 → ground stations | UDP | `px4_visualizer.py` listens, PX4 publishes |
| 14560 | bidirectional | UDP | `px4_bridge.py` ↔ PX4 (HIL mode only) |
| 8443 | DCV remote desktop | TCP/TLS | NICE DCV viewer ↔ EC2 |
| 22 | SSH | TCP | terminal access |

PX4's default SITL config opens 14540 + 14550 simultaneously, so MAVSDK and the visualizer can run together.

---

## Threading & Timing

The visualizer's main loop runs as fast as Isaac Sim updates (~60 fps). MAVLink messages arrive on a background thread and just write into a dict — they're never blocked by the renderer. The lock is held only for the snapshot copy in the main loop, so contention is minimal.

PX4 SIH internally runs at 1 kHz. The 50 Hz data stream we request gives us ~17 ms-old pose data on average — fine for visualization.

If you see jittery motion, check:
1. The data stream rate is actually being sent (`request_data_stream_send` succeeded)
2. The conda env didn't get the wrong pymavlink (only the one in `env_isaaclab_jazzy` should be active)
3. Isaac Sim's actual frame rate via `step % 300 == 0` log line

---

## Why This Design

- **PX4 SIH for the visualizer mode** — zero-config, zero physics setup, runs anywhere PX4 builds. Trades realism for simplicity.
- **MAVLink + MAVSDK split** — MAVLink (`pymavlink`) for low-level/sensor traffic in the visualizer/bridge; MAVSDK for high-level mission scripts. This matches how real ground stations and onboard companions are typically structured.
- **USD scene built once, reloaded fast** — `build_scene.py` runs ~60 s; `px4_visualizer.py` opens the cached USD in ~10 s after the first time Isaac Sim warms up.
- **Pose-only update path in the visualizer** — sidesteps PhysX entirely so the renderer never fights PX4 for control. When we move to HIL via `px4_bridge.py`, the trade flips: PhysX wins, PX4 commands forces.
