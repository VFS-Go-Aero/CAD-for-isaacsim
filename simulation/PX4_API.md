# PX4 API Reference

Everything the simulation talks to PX4 with — MAVLink messages, MAVSDK methods, ports, parameters, and patterns for adding new commands.

For the bigger picture, see [ARCHITECTURE.md](./ARCHITECTURE.md).

---

## Two APIs, Two Levels

| Library | Layer | Used by | Use for |
|---|---|---|---|
| **`pymavlink`** (`mavutil`) | low — raw MAVLink frames | `px4_visualizer.py`, `px4_bridge.py` | Sensor sim, raw telemetry, HIL |
| **`mavsdk`** (Python async) | high — fluent commands | `fly_demo.py` | Mission scripts, autonomous behaviors |

Both can run against the same PX4 instance simultaneously — PX4 SITL exposes them on different UDP ports.

---

## Network Endpoints (PX4 SITL defaults)

| Port | Direction | Protocol | Purpose | Code |
|---|---|---|---|---|
| **14540** | PX4 → client | UDP | MAVSDK / offboard control | `fly_demo.py` connects with `udpin://0.0.0.0:14540` |
| **14550** | PX4 → ground station | UDP | Telemetry stream | `px4_visualizer.py` listens with `udpin:0.0.0.0:14550` |
| **14560** | bidirectional | UDP | HIL (sensors in, motors out) | `px4_bridge.py` uses both `udpout` and `udpin` on 14560 |

PX4 publishes to all enabled outputs simultaneously, so MAVSDK + visualizer can run together without conflict.

---

## Part 1 — `pymavlink` (low-level MAVLink)

### Connect

```python
from pymavlink import mavutil

# Listen for telemetry from PX4 (visualizer mode)
mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Send sensors to PX4 (HIL mode, bridge)
mav = mavutil.mavlink_connection('udpout:127.0.0.1:14560', source_system=1, source_component=1)
```

After connecting, **always wait for a heartbeat first** to learn the target system/component IDs:

```python
mav.wait_heartbeat(timeout=30)
print(f'PX4 system={mav.target_system}, component={mav.target_component}')
```

### Request a higher message rate

PX4 only sends heartbeats by default. To get position/attitude streams, request them:

```python
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    50,    # Hz
    1,     # 1 = enable, 0 = disable
)
```

Available `MAV_DATA_STREAM_*` constants:

| Constant | Includes |
|---|---|
| `MAV_DATA_STREAM_ALL` | everything below |
| `MAV_DATA_STREAM_RAW_SENSORS` | RAW_IMU, SCALED_PRESSURE |
| `MAV_DATA_STREAM_EXTENDED_STATUS` | SYS_STATUS, GPS_RAW_INT |
| `MAV_DATA_STREAM_POSITION` | LOCAL_POSITION_NED, GLOBAL_POSITION_INT |
| `MAV_DATA_STREAM_EXTRA1` | ATTITUDE |
| `MAV_DATA_STREAM_EXTRA2` | VFR_HUD |
| `MAV_DATA_STREAM_EXTRA3` | SERVO_OUTPUT_RAW, BATTERY_STATUS |

### Receive loop

```python
while True:
    msg = mav.recv_match(blocking=True, timeout=1.0)   # or filter: type='ATTITUDE'
    if msg is None:
        continue
    msg_type = msg.get_type()
    # dispatch on msg_type
```

### Messages we currently consume (from PX4 → us)

These are handled in `px4_visualizer.py` `mavlink_receiver()`:

#### `LOCAL_POSITION_NED` — vehicle position
```python
msg.x   # north, m  (NED)
msg.y   # east, m   (NED)
msg.z   # down, m   (NED — negative = up)
msg.vx, msg.vy, msg.vz   # velocity (m/s)
msg.time_boot_ms
```
Visualizer converts NED → ENU with `(x_enu, y_enu, z_enu) = (msg.y, msg.x, -msg.z)`.

#### `ATTITUDE` — orientation in radians
```python
msg.roll, msg.pitch, msg.yaw   # rad, NED frame
msg.rollspeed, msg.pitchspeed, msg.yawspeed   # rad/s
```

#### `SERVO_OUTPUT_RAW` — motor PWM outputs
```python
msg.servo1_raw  …  msg.servo8_raw   # microseconds, typically 1000–2000
```
Visualizer normalizes to 0–1 with `max(0, (s - 1000) / 1000.0)` and uses it to spin propellers proportionally.

#### `HEARTBEAT` — armed state, mode, etc.
```python
msg.base_mode      # bitfield
msg.custom_mode    # PX4-specific main mode
msg.system_status  # MAV_STATE
is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
```

### Messages we currently send (us → PX4)

These are sent by `px4_bridge.py` for HIL mode:

#### `HIL_SENSOR` — IMU + baro + mag (250 Hz)
```python
mav.mav.hil_sensor_send(
    timestamp_us,
    xacc, yacc, zacc,            # m/s^2 (body frame)
    xgyro, ygyro, zgyro,         # rad/s
    xmag, ymag, zmag,            # Gauss
    abs_pressure,                # hPa
    diff_pressure,               # hPa
    pressure_altitude,           # m
    temperature,                 # °C
    fields_updated,              # bitmask: 0x1FF = all sensors
    id,                          # sensor instance (0)
)
```

#### `HIL_GPS` — GPS fix (10 Hz)
```python
mav.mav.hil_gps_send(
    timestamp_us,
    fix_type,                    # 3 = 3D fix
    lat, lon,                    # int, degrees * 1e7
    alt,                         # int, mm
    eph, epv,                    # HDOP/VDOP * 100, 65535=unknown
    vel,                         # cm/s
    vn, ve, vd,                  # cm/s
    cog,                         # course over ground, cdeg
    satellites_visible,
    id, yaw,
)
```

#### `HEARTBEAT` — keep-alive (1 Hz)
```python
mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0,
)
```

### Adding a new MAVLink handler

```python
# In px4_visualizer.py mavlink_receiver(), add:
elif msg_type == 'BATTERY_STATUS':
    with state_lock:
        vehicle_state['voltage'] = msg.voltages[0] / 1000.0   # mV → V
        vehicle_state['current'] = msg.current_battery / 100.0  # cA → A
        vehicle_state['battery_remaining'] = msg.battery_remaining  # %
```

Then read `vehicle_state['voltage']` from the main loop and display/visualize it.

---

## Part 2 — `mavsdk` (high-level async)

### Connect

```python
import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address='udpin://0.0.0.0:14540')

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

asyncio.run(main())
```

The connection_state stream yields once on connect — break out of it.

### Pre-flight checks (`telemetry`)

```python
async for health in drone.telemetry.health():
    if health.is_global_position_ok and health.is_home_position_ok:
        break
# Other useful health flags:
#   is_gyrometer_calibration_ok
#   is_accelerometer_calibration_ok
#   is_magnetometer_calibration_ok
#   is_local_position_ok
#   is_armable
```

### Action commands (`action`)

```python
await drone.action.arm()
await drone.action.disarm()
await drone.action.takeoff()
await drone.action.land()
await drone.action.return_to_launch()
await drone.action.kill()                      # emergency disarm in flight
await drone.action.set_takeoff_altitude(5.0)   # m
await drone.action.set_maximum_speed(5.0)      # m/s
```

### Offboard mode (`offboard`)

Used for real-time setpoint streaming. `fly_demo.py` is built around this:

```python
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError

# 1. Send an initial setpoint BEFORE starting offboard
await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

# 2. Arm
await drone.action.arm()

# 3. Switch into offboard mode
try:
    await drone.offboard.start()
except OffboardError as e:
    print(f"Offboard start failed: {e}")
    await drone.action.disarm()
    return

# 4. Stream setpoints (call as often as desired; PX4 ignores stale setpoints)
await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw))

# 5. Stop offboard, then land/disarm
await drone.offboard.stop()
await drone.action.land()
```

`PositionNedYaw` arguments:

| Field | Units | Notes |
|---|---|---|
| `north_m` | meters | NED — positive = north |
| `east_m`  | meters | NED — positive = east |
| `down_m`  | meters | NED — **negative = up**, so altitude 5 m → `down = -5.0` |
| `yaw_deg` | degrees | 0 = north, 90 = east |

Other offboard setpoint types: `VelocityNedYaw`, `VelocityBodyYawspeed`, `Attitude`, `AttitudeRate`, `ActuatorControl`.

### Telemetry streams

All async generators — iterate with `async for` or take one with `__anext__()`:

```python
async for pos in drone.telemetry.position_velocity_ned():
    # pos.position.{north_m, east_m, down_m}
    # pos.velocity.{north_m_s, east_m_s, down_m_s}
    break

async for att in drone.telemetry.attitude_euler():
    # att.roll_deg, att.pitch_deg, att.yaw_deg
    break

async for gps in drone.telemetry.position():
    # gps.{latitude_deg, longitude_deg, absolute_altitude_m, relative_altitude_m}
    break

async for batt in drone.telemetry.battery():
    # batt.{voltage_v, remaining_percent, current_battery_a}
    break

async for in_air in drone.telemetry.in_air():
    # bool — useful for waiting for landing
    if not in_air: break
```

The pattern for "wait until X" is in `fly_demo.py` `wait_for_position()`:

```python
async def wait_for_position(drone, target_n, target_e, target_d, tolerance=1.0, timeout=30):
    for _ in range(int(timeout * 10)):
        async for pos in drone.telemetry.position_velocity_ned():
            n, e, d = pos.position.north_m, pos.position.east_m, pos.position.down_m
            dist = ((n - target_n)**2 + (e - target_e)**2 + (d - target_d)**2)**0.5
            if dist < tolerance:
                return
            break
        await asyncio.sleep(0.1)
```

### Mission API (alternative to offboard)

For pre-planned flights, MAVSDK also has a `mission` plugin:

```python
from mavsdk.mission import MissionItem, MissionPlan

mission_items = [
    MissionItem(latitude_deg, longitude_deg, altitude_m,
                speed_m_s=5.0, is_fly_through=True,
                gimbal_pitch_deg=float('nan'),
                gimbal_yaw_deg=float('nan'),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=float('nan'),
                camera_photo_interval_s=float('nan'),
                acceptance_radius_m=float('nan'),
                yaw_deg=float('nan'),
                camera_photo_distance_m=float('nan'),
                vehicle_action=MissionItem.VehicleAction.NONE),
    # ...
]
await drone.mission.upload_mission(MissionPlan(mission_items))
await drone.mission.start_mission()
```

Use missions for reproducible test flights you want to log/replay.

---

## PX4 Configuration

### Airframes used

| Airframe ID | `PX4_SIM_MODEL` | Used by | Description |
|---|---|---|---|
| `10040` | `sihsim_quadx` | `px4_visualizer.py` setup | PX4 SIH (built-in physics, generic quadrotor) |
| `10050` | `none_go_aero` | `px4_bridge.py` setup | HIL mode, custom Go Aero airframe (defined in `airframes/10050_none_go_aero`) |

To switch modes, change the env vars in the PX4 startup command:

```bash
# Visualizer mode (default)
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d

# HIL mode
PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d
```

### Custom airframe params (Go Aero, `airframes/10050_none_go_aero`)

Key parameters set in the airframe init script:

```bash
param set-default CA_AIRFRAME 0           # Quadrotor X
param set-default CA_ROTOR_COUNT 4

# Per-motor X/Y position (m, body frame) and torque coefficient
param set-default CA_ROTOR0_PX  0.515     # Front-Right (CW)
param set-default CA_ROTOR0_PY  0.468
param set-default CA_ROTOR0_KM  0.05
param set-default CA_ROTOR1_PX -0.515     # Rear-Right (CCW)
param set-default CA_ROTOR1_PY  0.468
param set-default CA_ROTOR1_KM -0.05
param set-default CA_ROTOR2_PX -0.515     # Rear-Left (CW)
param set-default CA_ROTOR2_PY -0.468
param set-default CA_ROTOR2_KM  0.05
param set-default CA_ROTOR3_PX  0.515     # Front-Left (CCW)
param set-default CA_ROTOR3_PY -0.468
param set-default CA_ROTOR3_KM -0.05

param set-default PWM_MAIN_FUNC1 101      # motor 1 → output 1
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

param set-default SENS_EN_GPSSIM 1        # use simulated GPS
param set-default SENS_EN_BAROSIM 1       # use simulated barometer
param set-default SENS_EN_MAGSIM 1        # use simulated magnetometer
```

To change motor positions/spin direction, edit `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero` and rebuild PX4 (`make px4_sitl`).

### Useful px4 shell commands

Connect to the running PX4 process: it accepts commands directly in its stdout terminal (the one started with `bin/px4 -d`).

```
commander arm
commander disarm
commander takeoff
commander land
commander mode auto:loiter
param set MPC_XY_VEL_MAX 8.0
param show MPC_*
listener vehicle_local_position
sensors status
ekf2 status
```

`listener` is your friend for debugging — it dumps the current value of any uORB topic.

---

## Common PX4 Patterns

### Wait for "Ready for takeoff" before sending commands

PX4 prints `Ready for takeoff!` to stdout once EKF has converged and pre-arm checks pass. Wait for this before any arm command. From a script you can poll `commander check`:

```bash
commander check    # prints PASS / FAIL with reason
```

### Recover from `COMMAND_DENIED` on arm

PX4 may be stuck after a previous flight. Hard-restart:

```bash
pkill -9 -f "bin/px4"
rm -f /tmp/px4_lock_*
cd ~/PX4-Autopilot/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d
```

### Get the heartbeat in a different format

The visualizer parses the bitfield directly. MAVSDK exposes the same info as `drone.telemetry.armed()`:

```python
async for armed in drone.telemetry.armed():
    print(f'armed: {armed}')
```

### Send custom MAVLink commands not in MAVSDK

Use `pymavlink` alongside MAVSDK — both can run in the same script:

```python
mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
mav.wait_heartbeat()
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,                        # confirmation
    1, 4, 0, 0, 0, 0, 0,      # params 1–7
)
```

---

## Reference

- **MAVLink message dictionary:** https://mavlink.io/en/messages/common.html
- **PX4 MAVLink subset (custom messages):** https://mavlink.io/en/messages/development.html
- **MAVSDK Python docs:** https://mavsdk.mavlink.io/main/en/python/
- **PX4 SITL docs:** https://docs.px4.io/main/en/simulation/
- **PX4 SIH docs:** https://docs.px4.io/main/en/sim_sih/
- **PX4 HIL docs:** https://docs.px4.io/main/en/simulation/hitl.html
- **PX4 airframe reference:** https://docs.px4.io/main/en/airframes/airframe_reference.html
