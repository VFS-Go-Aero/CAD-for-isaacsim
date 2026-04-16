#!/usr/bin/env python3
"""
PX4 SITL <-> Isaac Sim MAVLink bridge (Hardware-In-The-Loop).

Isaac Sim owns the physics. Each sim step:
  1. Steps PhysX by exactly 1/SENSOR_RATE seconds.
  2. Reads IMU + chassis state from PhysX.
  3. Sends HIL_SENSOR + HIL_GPS + HEARTBEAT to PX4.
  4. Reads the latest HIL_ACTUATOR_CONTROLS from PX4.
  5. Applies per-motor thrust forces and a net yaw torque on the chassis
     rigid body, and drives the visual prop joints at a speed proportional
     to the motor command.

PX4's simulator_mavlink module connects *in* to our socket (TCP by default,
matching PX4's `simulator_mavlink start -c <port>` in px4-rc.mavlinksim).
Both directions flow over that one connection. Default port is 4560 to
match PX4's SITL default. Pass --protocol udp if you've patched the PX4
rc file to launch the simulator link with -u instead.

Usage (on EC2):
  1. Rebuild the scene (includes the chassis_IMU prim):
       python ~/go_aero/build_scene.py --headless

  2. Start PX4 in external-sim mode:
       cd ~/PX4-Autopilot/build/px4_sitl_default
       PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d

  3. Start the bridge:
       python ~/go_aero/px4_bridge.py --device cuda
"""
import os
import sys
import math
import time
import struct
import threading

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

# ============================================================
# CLI args + app launch (must happen before other omni imports)
# ============================================================
from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--px4-port', type=int, default=4560,
                    help='Port the PX4 simulator module connects to (default 4560)')
parser.add_argument('--protocol', choices=('tcp', 'udp'), default='tcp',
                    help='Transport for the PX4 simulator link. PX4 defaults to '
                         'TCP (client) with simulator_mavlink -c / -h, so we run '
                         'as a TCP server. Use --protocol udp if you have patched '
                         'px4-rc.mavlinksim to use -u.')
parser.add_argument('--bind-host', type=str, default='0.0.0.0',
                    help='Interface to bind to (default 0.0.0.0, all)')
parser.add_argument('--render-hz', type=float, default=30.0,
                    help='Viewport render rate (physics still runs at 250 Hz)')
parser.add_argument('--no-noise', action='store_true',
                    help='Disable Gaussian sensor noise (useful for debugging)')
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, Gf, UsdPhysics, Sdf
from pymavlink import mavutil

# Shared frame helpers live alongside this file on both the repo and the
# ~/go_aero/ runtime dir; the bridge does its frame conversions with numpy
# rather than Gf quaternions, but we still load _frames.py to guarantee
# it's deployable next to the bridge (the visualizer imports symbols from
# it at runtime).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _frames  # noqa: E402,F401

app = omni.kit.app.get_app()

# ============================================================
# Constants
# ============================================================
# Physics timing
SENSOR_RATE = 250       # Hz — HIL_SENSOR + physics step rate
GPS_RATE = 10           # Hz
HEARTBEAT_RATE = 1      # Hz
SIM_DT = 1.0 / SENSOR_RATE

# Home location (Boston, for the Mass Robotics demo)
HOME_LAT = 42.3601
HOME_LON = -71.0589
HOME_ALT = 10.0  # m above MSL

# Vehicle physical parameters (must match build_scene.py)
DRONE_MASS = 5.0        # kg
GRAVITY = 9.81          # m/s^2

# Thrust model: quadratic, clamped 0..1 motor command.
# 2 g of collective thrust at 100 % throttle (typical eVTOL margin).
MAX_THRUST_PER_MOTOR = 2.0 * DRONE_MASS * GRAVITY / 4.0  # ~24.5 N

# Reaction torque coefficient (moment per unit thrust). 0.016 m is typical
# for 10-15" props; adjust against real vehicle data later.
K_TAU = 0.016  # m

# Sensor noise (standard deviations). Disable with --no-noise.
ACC_NOISE_STD = 0.05    # m/s^2
GYRO_NOISE_STD = 0.001  # rad/s
MAG_NOISE_STD = 0.002   # Gauss
BARO_NOISE_STD = 0.1    # Pa
GPS_POS_NOISE_STD = 0.3  # m
GPS_VEL_NOISE_STD = 0.05  # m/s

# Boston magnetic field (NED world, Gauss, from NOAA WMM)
B_WORLD_NED = np.array([0.189, 0.028, 0.463])

# MAVLink identity. SimulatorMavlink.cpp uses SIM_COMP_ID = 51 on the PX4
# side; we send as a non-autopilot component so PX4 routes HIL_* messages
# to its simulator handler rather than treating them as own-telemetry.
SOURCE_SYSTEM = 1
SOURCE_COMPONENT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1  # 1

# Propeller layout (mesh-body-frame positions from build_scene.py).
# Spin direction convention: +1 = CCW viewed from above (produces +yaw
# moment around body-up), -1 = CW. Matches CA_ROTORn_KM signs in the
# Go Aero airframe file.
PROP_LAYOUT = {
    'prop_fr': {'pos_body': np.array([0.468, 0.048, -0.515]), 'spin_dir': -1},
    'prop_fl': {'pos_body': np.array([-0.468, 0.048, -0.515]), 'spin_dir': 1},
    'prop_rr': {'pos_body': np.array([0.468, 0.048, 0.515]), 'spin_dir': 1},
    'prop_rl': {'pos_body': np.array([-0.468, 0.048, 0.515]), 'spin_dir': -1},
}
# Fixed iteration order that matches the PX4 airframe's CA_ROTORn_* params:
#   rotor 0 = FR, rotor 1 = RR, rotor 2 = RL, rotor 3 = FL
PROP_ORDER = ['prop_fr', 'prop_rr', 'prop_rl', 'prop_fl']


# ============================================================
# Helpers
# ============================================================
def quat_rotate(q_wxyz, v):
    """Rotate a 3-vector by a (w,x,y,z) quaternion."""
    w, x, y, z = q_wxyz
    # Hamilton product q * (0, v) * q_conj, expanded.
    vx, vy, vz = v
    # t = 2 * (q.xyz cross v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v + w*t + q.xyz cross t
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return np.array([rx, ry, rz])


def quat_conj(q_wxyz):
    return np.array([q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def mesh_body_to_frd(v):
    """Map a vector from mesh-body axes (+X=right, +Y=up, +Z=back) to
    PX4 FRD body axes (+X=fwd, +Y=right, +Z=down)."""
    return np.array([-v[2], v[0], -v[1]])


def enu_to_ned(v):
    """ENU (E, N, U) -> NED (N, E, D)."""
    return np.array([v[1], v[0], -v[2]])


def pressure_from_altitude(alt_m):
    """ISA barometric formula, returns hPa."""
    return 1013.25 * (1.0 - 2.25577e-5 * alt_m) ** 5.2559


# ============================================================
# Scene load
# ============================================================
print('[INFO] Opening vehicle scene...', flush=True)
omni.usd.get_context().open_stage(os.path.expanduser('~/go_aero/vehicle_scene.usd'))

for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
cam = ViewportCameraState('/OmniverseKit_Persp')
cam.set_position_world(Gf.Vec3d(8.0, -8.0, 5.0), True)
cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)

# ============================================================
# Physics handles
# ============================================================
# Rigid body wrapper — isaacsim.core.prims.SingleRigidPrim is the low-level
# wrapper that exposes get_world_pose / get_linear_velocity /
# apply_forces_and_torques_at_pos against an already-authored USD rigid
# body. We use it instead of an IsaacLab RigidObject because the scene is
# pre-built (no spawn step) and this is a one-body system.
from isaacsim.core.prims import SingleRigidPrim
chassis = SingleRigidPrim('/World/Vehicle/chassis', name='chassis')

# IMU sensor attached to chassis — reads body-frame lin_acc (with gravity)
# and ang_vel at the physics step rate. Outputs are in the prim's local
# frame, which is the mesh-body frame (Y-up); we convert to FRD at send.
from isaacsim.sensors.physics import IMUSensor
imu = IMUSensor(
    prim_path='/World/Vehicle/chassis/chassis_IMU',
    name='chassis_imu',
    frequency=SENSOR_RATE,
    translation=np.zeros(3),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # (w,x,y,z) identity
)

# Prop joint drives — visual spin only (decoupled from the force model).
prop_drives = {}
for name in PROP_LAYOUT:
    joint = stage.GetPrimAtPath(f'/World/Vehicle/joints/{name}_joint')
    if joint:
        prop_drives[name] = UsdPhysics.DriveAPI.Get(joint, 'angular')

# ============================================================
# MAVLink — single bidirectional socket
# ============================================================
# PX4's simulator_mavlink defaults to TCP (client), connecting out to the
# simulator on port 4560 — see PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
# px4-rc.mavlinksim ("simulator_mavlink start -c $simulator_tcp_port").
# We therefore default to 'tcpin' (TCP server) so PX4 works out of the box.
# 'udp' is supported for the case where the user has patched the rc file to
# launch simulator_mavlink with -u instead; pymavlink's udpin auto-replies
# to the last sender, so both directions still flow on one socket.
if args.protocol == 'tcp':
    mav_url = f'tcpin:{args.bind_host}:{args.px4_port}'
else:
    mav_url = f'udpin:{args.bind_host}:{args.px4_port}'
print(f'[INFO] Binding MAVLink {args.protocol.upper()} on '
      f'{args.bind_host}:{args.px4_port}...', flush=True)
mav = mavutil.mavlink_connection(
    mav_url,
    source_system=SOURCE_SYSTEM,
    source_component=SOURCE_COMPONENT,
)

# Shared state between the MAVLink receiver thread and the physics loop.
motor_commands = [0.0, 0.0, 0.0, 0.0]  # rotor order (FR, RR, RL, FL) per PX4 mixer
armed = False
motor_lock = threading.Lock()
px4_connected = threading.Event()


def mavlink_receiver():
    """Background thread: drain PX4 messages, update motor + arm state."""
    global armed
    while simulation_app.is_running():
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
        except (OSError, struct.error) as e:
            print(f'[WARN] MAVLink recv error: {e}', flush=True)
            continue
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == 'HIL_ACTUATOR_CONTROLS':
            # msg.controls is a 16-element list; rotors 0..3 are the quad motors.
            # PX4 mixer emits [0, 1] throttle per motor.
            with motor_lock:
                motor_commands[0] = max(0.0, min(1.0, float(msg.controls[0])))
                motor_commands[1] = max(0.0, min(1.0, float(msg.controls[1])))
                motor_commands[2] = max(0.0, min(1.0, float(msg.controls[2])))
                motor_commands[3] = max(0.0, min(1.0, float(msg.controls[3])))
            px4_connected.set()

        elif mtype == 'HEARTBEAT':
            # Parse armed state from the standard MAV_MODE_FLAG_SAFETY_ARMED
            # bit on the HEARTBEAT (not from the deprecated .mode field on
            # HIL_ACTUATOR_CONTROLS, which is always zero).
            with motor_lock:
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            px4_connected.set()


recv_thread = threading.Thread(target=mavlink_receiver, daemon=True)
recv_thread.start()
print('[INFO] MAVLink receiver started', flush=True)


# ============================================================
# Start physics, then wait for PX4 to connect
# ============================================================
# Use the timeline interface (matches the rest of the codebase). The scene
# already defines its PhysicsScene and dt — we advance via app.update() and
# rely on PhysicsScene's timeStepsPerSecond. For exact 250 Hz stepping you
# can set the sim cfg on the stage; see ARCHITECTURE.md.
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Warm-up: let the physics scene and sensor initialize before we start
# talking to PX4 — otherwise the first IMU frame reads as all-zeros and
# PX4's EKF briefly sees impossible sensor data.
for _ in range(30):
    app.update()

# Initialize the physics wrappers now that the timeline is playing.
chassis.initialize()
imu.initialize()

print(f'[INFO] Waiting for PX4 on {args.protocol.upper()} {args.px4_port}...', flush=True)
print(
    '[INFO]   Start PX4: cd ~/PX4-Autopilot/build/px4_sitl_default && '
    'PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d',
    flush=True,
)

# Block until we see *anything* from PX4 on the socket. The receiver thread
# sets the event on the first HEARTBEAT or HIL_ACTUATOR_CONTROLS message.
while not px4_connected.is_set() and simulation_app.is_running():
    app.update()                       # keep the viewport responsive
    px4_connected.wait(timeout=0.05)   # yields to receiver thread

if not simulation_app.is_running():
    simulation_app.close()
    sys.exit(0)

print('[INFO] PX4 connected. Bridge running.', flush=True)

# ============================================================
# Main loop
# ============================================================
# Sim-time clock. Using the sim timeline's monotonically-increasing time
# instead of wall-clock keeps MAVLink timestamps aligned with physics even
# if the viewport lags momentarily.
last_gps_tick = 0.0
last_hb_tick = 0.0
step = 0

# render_period_steps is reserved for future use when we switch the loop
# over to an explicit isaaclab SimulationContext with sim.step(render=...).
# For now Kit's timeline drives both physics and render at the scene's
# PhysicsScene.timeStepsPerSecond.
_render_period_steps = max(1, int(round(SENSOR_RATE / max(args.render_hz, 1.0))))


def noise(std):
    return 0.0 if args.no_noise else float(np.random.normal(0.0, std))


def noise_vec3(std):
    return np.zeros(3) if args.no_noise else np.random.normal(0.0, std, size=3)


def send_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def send_hil_sensor(t_us, acc_frd, gyro_frd, mag_frd, pressure_hpa, alt_m):
    mav.mav.hil_sensor_send(
        int(t_us),
        float(acc_frd[0]), float(acc_frd[1]), float(acc_frd[2]),
        float(gyro_frd[0]), float(gyro_frd[1]), float(gyro_frd[2]),
        float(mag_frd[0]), float(mag_frd[1]), float(mag_frd[2]),
        float(pressure_hpa),
        0.0,                            # diff_pressure
        float(alt_m),                   # pressure_altitude (m)
        25.0,                           # temperature (C)
        0x1FFF,                         # fields_updated: all 13 fields
        0,                              # sensor id
    )


def send_hil_gps(t_us, pos_enu, vel_enu):
    lat_offset = float(pos_enu[1]) / 111139.0
    lon_offset = float(pos_enu[0]) / (111139.0 * math.cos(math.radians(HOME_LAT)))
    alt_m = float(pos_enu[2]) + HOME_ALT

    # Horizontal ground speed magnitude (cm/s) and course-over-ground (cdeg).
    vn_m = float(vel_enu[1])
    ve_m = float(vel_enu[0])
    vd_m = -float(vel_enu[2])
    vel_m = math.hypot(vn_m, ve_m)
    cog_deg = (math.degrees(math.atan2(ve_m, vn_m)) + 360.0) % 360.0

    mav.mav.hil_gps_send(
        int(t_us),
        3,                               # fix_type: 3D
        int((HOME_LAT + lat_offset) * 1e7),
        int((HOME_LON + lon_offset) * 1e7),
        int(alt_m * 1000),
        int(GPS_POS_NOISE_STD * 100),    # eph (cm)
        int(GPS_POS_NOISE_STD * 100),    # epv (cm)
        int(vel_m * 100),                # vel (cm/s)
        int(vn_m * 100),                 # vn (cm/s)
        int(ve_m * 100),                 # ve (cm/s)
        int(vd_m * 100),                 # vd (cm/s)
        int(cog_deg * 100),              # cog (cdeg)
        14,                              # satellites_visible
        0,                               # id
        0,                               # yaw (cdeg, 0 = not available)
    )


try:
    while simulation_app.is_running():
        # Kit's app.update() is the single physics+render tick. The scene's
        # PhysicsScene.timeStepsPerSecond drives the actual physics dt.
        app.update()
        step += 1

        sim_time = timeline.get_current_time()
        t_us = int(sim_time * 1e6)

        # --------------------------------------------------
        # Read physics state
        # --------------------------------------------------
        pos_w, quat_w = chassis.get_world_pose()  # ENU world, quat (w,x,y,z)
        pos_w = np.asarray(pos_w, dtype=np.float64)
        quat_w = np.asarray(quat_w, dtype=np.float64)
        lin_vel_w = np.asarray(chassis.get_linear_velocity(), dtype=np.float64)

        imu_frame = imu.get_current_frame()
        # lin_acc includes gravity (specific force), already in body/mesh frame.
        lin_acc_mesh = np.asarray(imu_frame['lin_acc'], dtype=np.float64)
        ang_vel_mesh = np.asarray(imu_frame['ang_vel'], dtype=np.float64)

        # --------------------------------------------------
        # Derive PX4-frame sensors
        # --------------------------------------------------
        acc_frd = mesh_body_to_frd(lin_acc_mesh) + noise_vec3(ACC_NOISE_STD)
        gyro_frd = mesh_body_to_frd(ang_vel_mesh) + noise_vec3(GYRO_NOISE_STD)

        # Magnetometer: rotate the NED world field into the body (mesh)
        # frame via the body's world quaternion, then convert to FRD.
        # q_world_chassis takes a body vector and produces a world vector,
        # so we rotate by its conjugate to go world -> body.
        b_world_enu = np.array([B_WORLD_NED[1], B_WORLD_NED[0], -B_WORLD_NED[2]])
        mag_mesh = quat_rotate(quat_conj(quat_w), b_world_enu)
        mag_frd = mesh_body_to_frd(mag_mesh) + noise_vec3(MAG_NOISE_STD)

        # Barometer from altitude above MSL.
        alt_msl = float(pos_w[2]) + HOME_ALT
        pressure_hpa = pressure_from_altitude(alt_msl) + noise(BARO_NOISE_STD) / 100.0

        # --------------------------------------------------
        # Send sensor packets
        # --------------------------------------------------
        send_hil_sensor(t_us, acc_frd, gyro_frd, mag_frd, pressure_hpa, alt_msl)

        if sim_time - last_gps_tick >= 1.0 / GPS_RATE:
            last_gps_tick = sim_time
            send_hil_gps(t_us, pos_w, lin_vel_w)

        if sim_time - last_hb_tick >= 1.0 / HEARTBEAT_RATE:
            last_hb_tick = sim_time
            send_heartbeat()

        # --------------------------------------------------
        # Apply motor forces / torques to the chassis
        # --------------------------------------------------
        with motor_lock:
            cmds = list(motor_commands)

        # Compute net force and moment in mesh-body frame.
        F_body = np.zeros(3)
        M_body = np.zeros(3)
        for i, name in enumerate(PROP_ORDER):
            T_i = MAX_THRUST_PER_MOTOR * (cmds[i] ** 2)
            r_body = PROP_LAYOUT[name]['pos_body']
            F_i = np.array([0.0, T_i, 0.0])  # mesh +Y = body up
            F_body += F_i
            M_body += np.cross(r_body, F_i)
            # Reaction yaw torque acts opposite to prop spin direction.
            # CCW prop (spin_dir=+1) creates a +yaw (body-down, FRD +Z)
            # reaction moment, which in mesh-body is -Y.
            M_body[1] += -K_TAU * T_i * PROP_LAYOUT[name]['spin_dir']

            # Visual prop spin (decoupled from force model).
            omega = cmds[i] * 1500.0 * PROP_LAYOUT[name]['spin_dir']  # deg/s
            drive = prop_drives.get(name)
            if drive:
                drive.GetTargetVelocityAttr().Set(float(omega))

        # Convert body-frame force/torque to world frame for apply call.
        F_world = quat_rotate(quat_w, F_body)
        M_world = quat_rotate(quat_w, M_body)

        chassis.apply_forces_and_torques_at_pos(
            forces=np.asarray([F_world], dtype=np.float32),
            torques=np.asarray([M_world], dtype=np.float32),
            positions=None,           # apply at COM -> pure force + moment
            is_global=True,
        )

        # --------------------------------------------------
        # Status log
        # --------------------------------------------------
        if step % (SENSOR_RATE * 2) == 0:
            avg = sum(cmds) / 4.0
            with motor_lock:
                is_armed = armed
            print(
                f'[INFO] t={sim_time:6.2f}s  z={pos_w[2]:+.2f}m  '
                f'armed={is_armed}  motors=[{cmds[0]:.2f},{cmds[1]:.2f},'
                f'{cmds[2]:.2f},{cmds[3]:.2f}] avg={avg:.2f}',
                flush=True,
            )

except KeyboardInterrupt:
    print('[INFO] SIGINT received, stopping...', flush=True)

timeline.stop()
try:
    mav.close()
except Exception:
    pass
simulation_app.close()
print('[INFO] Done.', flush=True)
