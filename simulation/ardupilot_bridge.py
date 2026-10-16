#!/usr/bin/env python3
"""
ArduPilot SITL (SIM_JSON) <-> Isaac Sim PhysX Bridge (Hardware/Software-In-The-Loop).

Isaac Sim owns the physics (Mode 2). Each simulation step:
  1. Drains incoming binary servo PWM packets from ArduPilot SITL (`-f JSON`)
     over UDP (default port 9002).
  2. Maps ArduCopter Quad-X (`FRAME_CLASS=1, FRAME_TYPE=1`) motor channels onto
     the Go Aero `PROP_LAYOUT` and applies per-motor thrust forces and reaction
     yaw torques to `/World/Vehicle/chassis`.
  3. Steps PhysX by `SIM_DT = 1.0 / SENSOR_RATE` and reads IMU + chassis pose/velocity.
  4. Converts state from Isaac Sim ENU / mesh-body (Y-up) into NED / FRD and sends
     one lockstep newline-delimited JSON sensor state packet expected by ArduPilot's
     `SIM_JSON` (`libraries/SITL/SIM_JSON.cpp`).

Motor mapping (`ARDU_PROP_ORDER` vs `PX4_PROP_ORDER`):
  ArduCopter Quad-X (`FRAME_CLASS=1, FRAME_TYPE=1`):
    - Motor 1 (servo1, pwm[0]): Front-Right (`prop_fr`, CCW)
    - Motor 2 (servo2, pwm[1]): Rear-Left   (`prop_rl`, CCW)
    - Motor 3 (servo3, pwm[2]): Front-Left  (`prop_fl`, CW)
    - Motor 4 (servo4, pwm[3]): Rear-Right  (`prop_rr`, CW)
  PX4 Quad-X (`10050_none_go_aero`):
    - Rotor 0 (controls[0]): Front-Right (`prop_fr`, CCW)
    - Rotor 1 (controls[1]): Rear-Right  (`prop_rr`, CW)
    - Rotor 2 (controls[2]): Rear-Left   (`prop_rl`, CCW)
    - Rotor 3 (controls[3]): Front-Left  (`prop_fl`, CW)

Usage (on EC2):
  1. Build the USD scene:
       python ~/go_aero/build_scene.py --headless

  2. Start the Isaac Sim ArduPilot bridge:
       python ~/go_aero/ardupilot_bridge.py --device cuda --port 9002

  3. Start ArduPilot SITL with the JSON physics backend:
       cd ~/ardupilot
       Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --no-rebuild \\
         --add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param \\
         --out=udp:127.0.0.1:14550 \\
         --out=udp:127.0.0.1:14540
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
from typing import Sequence

try:
    import numpy as np
except ImportError:  # pragma: no cover - fallback for bare system python without numpy
    np = None  # type: ignore[assignment]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _compat import (  # noqa: E402
    LIDAR_MOUNT_PATH,
    LIDAR_MOUNT_POS_MESH,
    LIDAR_MOUNT_ROT_X_DEG,
    LIDAR_SENSOR_PATH,
    ZED1_MOUNT_PATH,
    ZED1_MOUNT_POS_MESH,
    ZED1_OPTICAL_FRAME_ID,
    ZED2_MOUNT_PATH,
    ZED2_MOUNT_POS_MESH,
    ZED2_OPTICAL_FRAME_ID,
    add_app_launcher_args,
    enable_required_extensions,
    ensure_xform_pose,
    launch_app,
)


def _vec(vals: Sequence[float]):
    """Return a float64 numpy array if numpy is available, else a list of floats."""
    floats = [float(x) for x in vals]
    if np is not None:
        return np.array(floats, dtype=np.float64)
    return floats


def _cross3(a: Sequence[float], b: Sequence[float]):
    """3D cross product `a x b` independent of numpy."""
    return _vec((
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ))


# ============================================================
# Constants & Motor Layouts (pure Python, importable by pytest)
# ============================================================
SENSOR_RATE = 250       # Hz — physics step target rate
SIM_DT = 1.0 / SENSOR_RATE

# ArduPilot SIM_JSON binary packet magic values (libraries/SITL/SIM_JSON.cpp)
ARDU_MAGIC_16CH = 18458  # 0x481A (16-channel servo packet, 40 bytes)
ARDU_MAGIC_32CH = 29569  # 0x7381 (32-channel servo packet, 72 bytes)

# Home location (Boston, for the Mass Robotics demo)
HOME_LAT = 42.3601
HOME_LON = -71.0589
HOME_ALT = 10.0  # m above MSL

# Vehicle physical parameters (must match build_scene.py)
DRONE_MASS = 5.0        # kg
GRAVITY = 9.81          # m/s^2
MAX_THRUST_PER_MOTOR = 2.0 * DRONE_MASS * GRAVITY / 4.0  # ~24.5 N per motor (2g total)
K_TAU = 0.016           # m — reaction yaw torque coefficient per unit thrust

# Sensor noise standard deviations (disabled with --no-noise)
ACC_NOISE_STD = 0.05    # m/s^2
GYRO_NOISE_STD = 0.001  # rad/s

# Propeller layout in mesh-body frame (+X=right, +Y=up, +Z=back so -Z=forward).
# Both PX4 and ArduCopter Quad-X share the same physical prop spin directions:
#   +1 = CCW viewed from above (prop_fr, prop_rl) -> produces +yaw (nose-right,
#        FRD +Z) reaction torque, which corresponds to -Y moment in mesh-body.
#   -1 = CW viewed from above  (prop_fl, prop_rr) -> produces -yaw (nose-left,
#        FRD -Z) reaction torque, which corresponds to +Y moment in mesh-body.
PROP_LAYOUT = {
    'prop_fr': {'pos_body': _vec((0.468, 0.048, -0.515)), 'spin_dir': 1, 'ardu_spin_dir': 1},
    'prop_fl': {'pos_body': _vec((-0.468, 0.048, -0.515)), 'spin_dir': -1, 'ardu_spin_dir': -1},
    'prop_rr': {'pos_body': _vec((0.468, 0.048, 0.515)), 'spin_dir': -1, 'ardu_spin_dir': -1},
    'prop_rl': {'pos_body': _vec((-0.468, 0.048, 0.515)), 'spin_dir': 1, 'ardu_spin_dir': 1},
}

# ArduCopter default Quad-X motor order (FRAME_CLASS=1, FRAME_TYPE=1):
#   Motor 1 (pwm[0]) -> Front-Right (prop_fr)
#   Motor 2 (pwm[1]) -> Rear-Left   (prop_rl)
#   Motor 3 (pwm[2]) -> Front-Left  (prop_fl)
#   Motor 4 (pwm[3]) -> Rear-Right  (prop_rr)
ARDU_PROP_ORDER = ['prop_fr', 'prop_rl', 'prop_fl', 'prop_rr']

# PX4 Quad-X motor order (10050_none_go_aero):
#   Rotor 0 -> Front-Right (prop_fr)
#   Rotor 1 -> Rear-Right  (prop_rr)
#   Rotor 2 -> Rear-Left   (prop_rl)
#   Rotor 3 -> Front-Left  (prop_fl)
PX4_PROP_ORDER = ['prop_fr', 'prop_rr', 'prop_rl', 'prop_fl']

MOTOR_MAPS = {
    'arducopter_quadx': ARDU_PROP_ORDER,
    'px4_quadx': PX4_PROP_ORDER,
}

_COS45 = math.sqrt(0.5)


class ServoPacket(dict):
    """Parsed ArduPilot SIM_JSON binary servo packet supporting dict and attribute access."""

    __getattr__ = dict.__getitem__


# ============================================================
# Pure-Python Protocol & Coordinate Frame Helpers
# ============================================================
def pwm_to_norm(pwm_us: int | float, pwm_min: float = 1000.0, pwm_max: float = 2000.0) -> float:
    """Normalize a servo PWM value in microseconds [1000, 2000] to [0.0, 1.0]."""
    val = float(pwm_us)
    pmin = float(pwm_min)
    pmax = float(pwm_max)
    if not math.isfinite(val) or not math.isfinite(pmin) or not math.isfinite(pmax) or pmax <= pmin:
        return 0.0
    if val <= pmin:
        return 0.0
    return max(0.0, min(1.0, (val - pmin) / (pmax - pmin)))


def parse_ardupilot_servo_packet(data: bytes) -> ServoPacket:
    """
    Parse a binary servo packet sent by ArduPilot SITL's SIM_JSON backend.

    Packet layout (`<HHI16H` = 40 bytes, `<HHI8H` = 24 bytes, or `<HHI32H` = 72 bytes):
      - uint16 magic:       18458 (0x481A, 16-ch) or 29569 (0x7381, 32-ch)
      - uint16 frame_rate:  SITL loop rate (Hz)
      - uint32 frame_count: monotonically increasing SITL frame counter
      - uint16 pwm[N]:      servo PWM outputs in microseconds (typically 1000..2000)

    Returns a `ServoPacket` (dict subclass) with keys:
      `magic`, `frame_rate`, `frame_count`, `pwm`, `motors_norm`, `armed`.
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError(f'Expected bytes-like object, got {type(data).__name__}')
    n_bytes = len(data)
    if n_bytes < 24 or (n_bytes - 8) % 2 != 0:
        raise ValueError(
            f'Invalid ArduPilot servo packet length {n_bytes}; expected 24, 40, or 72 bytes'
        )

    magic, frame_rate, frame_count = struct.unpack('<HHI', data[:8])
    if magic not in (ARDU_MAGIC_16CH, ARDU_MAGIC_32CH):
        raise ValueError(
            f'Invalid ArduPilot servo packet magic 0x{magic:04x} ({magic}); '
            f'expected 0x481a ({ARDU_MAGIC_16CH}) or 0x7381 ({ARDU_MAGIC_32CH})'
        )

    n_channels = (n_bytes - 8) // 2
    if n_channels not in (8, 16, 32):
        raise ValueError(f'Unsupported servo channel count {n_channels} (packet length {n_bytes})')

    pwms = list(struct.unpack(f'<{n_channels}H', data[8:]))
    motors_norm = [pwm_to_norm(p) for p in pwms[:4]]
    armed = any(p > 1050 for p in pwms[:4])

    return ServoPacket(
        magic=magic,
        frame_rate=frame_rate,
        frame_count=frame_count,
        pwm=pwms,
        motors_norm=motors_norm,
        armed=armed,
    )


def format_ardupilot_json_reply(
    timestamp_s: float,
    gyro_frd: Sequence[float],
    accel_frd: Sequence[float],
    pos_ned: Sequence[float],
    euler_ned: Sequence[float],
    vel_ned: Sequence[float],
) -> bytes:
    """
    Format the physics state reply expected by ArduPilot SITL `SIM_JSON`.

    All vectors must be in NED / FRD frames:
      - `timestamp_s`: simulation time in seconds
      - `gyro_frd`:    body angular velocity [gx, gy, gz] in rad/s (FRD)
      - `accel_frd`:   body specific force [ax, ay, az] in m/s^2 (FRD; [0, 0, -9.81] at rest)
      - `pos_ned`:     world position [north, east, down] in m (NED)
      - `euler_ned`:   Euler attitude [roll, pitch, yaw] in rad (NED)
      - `vel_ned`:     world linear velocity [vn, ve, vd] in m/s (NED)

    Returns UTF-8 encoded bytes framed with leading and trailing newlines:
      `b"\\n{...}\\n"`.
    """
    payload = {
        'timestamp': float(timestamp_s),
        'imu': {
            'gyro': [float(gyro_frd[0]), float(gyro_frd[1]), float(gyro_frd[2])],
            'accel_body': [float(accel_frd[0]), float(accel_frd[1]), float(accel_frd[2])],
        },
        'position': [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])],
        'attitude': [float(euler_ned[0]), float(euler_ned[1]), float(euler_ned[2])],
        'velocity': [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])],
    }
    return ('\n' + json.dumps(payload) + '\n').encode('utf-8')


def quat_rotate(q_wxyz: Sequence[float], v: Sequence[float]):
    """Rotate a 3-vector by a (w, x, y, z) quaternion."""
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    norm = math.hypot(w, x, y, z)
    if not math.isfinite(norm) or norm <= 0.0:
        return _vec((vx, vy, vz))
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return _vec((rx, ry, rz))


def quat_conj(q_wxyz: Sequence[float]):
    """Return the conjugate of a (w, x, y, z) quaternion."""
    return _vec((q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3]))


def quat_mul(a: Sequence[float], b: Sequence[float]):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bw, bx, by, bz = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    return _vec((
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ))


def mesh_body_to_frd(v: Sequence[float]):
    """Map a vector from mesh-body axes (+X=right, +Y=up, +Z=back) to FRD (+X=fwd, +Y=right, +Z=down)."""
    return _vec((-float(v[2]), float(v[0]), -float(v[1])))


def enu_to_ned(v: Sequence[float]):
    """Convert an ENU vector (east, north, up) to NED (north, east, down)."""
    return _vec((float(v[1]), float(v[0]), -float(v[2])))


def enu_quat_to_ned_euler(q_wxyz: Sequence[float]):
    """
    Convert the chassis world quaternion (w, x, y, z) in Isaac Sim ENU (which
    includes the +90 deg X mesh-fix rotation) into NED Tait-Bryan (roll, pitch, yaw)
    Euler angles in radians.
    """
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    norm = math.hypot(w, x, y, z)
    if not math.isfinite(norm) or norm <= 0.0:
        return _vec((0.0, 0.0, 0.0))
    q_unit = (w / norm, x / norm, y / norm, z / norm)

    # Unwind Q_MESH_FIX = (cos45, +sin45, 0, 0) via right-multiplying its inverse
    ew, ex, ey, ez = quat_mul(q_unit, (_COS45, -_COS45, 0.0, 0.0))
    # Rebase ENU -> NED via similarity transform Q_NED_ENU_INV * q_body_enu * Q_NED_ENU
    nw, nx, ny, nz = float(ew), float(ey), float(ex), -float(ez)

    sinp = max(-1.0, min(1.0, 2.0 * (nw * ny - nz * nx)))
    pitch = math.asin(sinp)
    if abs(sinp) >= 1.0 - 1e-7:
        # Gimbal lock at pitch = +/- 90 deg: attribute coupled angle to yaw
        roll = 0.0
        yaw = math.atan2(-2.0 * (nx * ny - nw * nz), 1.0 - 2.0 * (nx * nx + nz * nz))
    else:
        roll = math.atan2(2.0 * (nw * nx + ny * nz), 1.0 - 2.0 * (nx * nx + ny * ny))
        yaw = math.atan2(2.0 * (nw * nz + nx * ny), 1.0 - 2.0 * (ny * ny + nz * nz))
    return _vec((roll, pitch, yaw))


def compute_motor_forces_and_torques(
    cmds: Sequence[float],
    motor_map: str = 'arducopter_quadx',
):
    """
    Compute net thrust force `F_body` and moment `M_body` in the mesh-body frame
    (+X=right, +Y=up, +Z=back) for 4 normalized motor commands `cmds[0..3]`.
    """
    prop_order = MOTOR_MAPS.get(motor_map, ARDU_PROP_ORDER)
    spin_key = 'ardu_spin_dir' if motor_map == 'arducopter_quadx' else 'spin_dir'

    fx, fy, fz = 0.0, 0.0, 0.0
    mx, my, mz = 0.0, 0.0, 0.0
    for i, name in enumerate(prop_order):
        raw_c = float(cmds[i])
        c = max(0.0, min(1.0, raw_c)) if math.isfinite(raw_c) else 0.0
        T_i = MAX_THRUST_PER_MOTOR * (c ** 2)
        r_body = PROP_LAYOUT[name]['pos_body']
        F_i = (0.0, T_i, 0.0)  # mesh +Y = body up
        fx += F_i[0]
        fy += F_i[1]
        fz += F_i[2]
        tau_i = _cross3(r_body, F_i)
        mx += float(tau_i[0])
        my += float(tau_i[1])
        mz += float(tau_i[2])
        # CCW prop (spin=+1) creates a +yaw (FRD +Z, nose-right) reaction moment,
        # which in mesh-body (+Y=up) is -Y.
        spin_dir = PROP_LAYOUT[name][spin_key]
        my += -K_TAU * T_i * spin_dir

    return _vec((fx, fy, fz)), _vec((mx, my, mz))


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the CLI argument parser for `ardupilot_bridge.py`."""
    parser = argparse.ArgumentParser(
        description='ArduPilot SITL (SIM_JSON) <-> Isaac Sim PhysX bridge'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=9002,
        help='UDP port for ArduPilot SITL SIM_JSON servo packets (default: 9002)',
    )
    parser.add_argument(
        '--bind-host',
        type=str,
        default='0.0.0.0',
        help='Interface to bind UDP socket to (default: 0.0.0.0)',
    )
    parser.add_argument(
        '--motor-map',
        choices=('arducopter_quadx', 'px4_quadx'),
        default='arducopter_quadx',
        help='Motor channel ordering (default: arducopter_quadx = FR, RL, FL, RR)',
    )
    parser.add_argument(
        '--render-hz',
        type=float,
        default=30.0,
        help='Viewport render rate (physics runs at 250 Hz)',
    )
    parser.add_argument(
        '--no-noise',
        action='store_true',
        help='Disable Gaussian sensor noise (useful for debugging)',
    )
    parser.add_argument(
        '--no-wait',
        action='store_true',
        help='Do not block waiting for the first ArduPilot servo packet before starting loop',
    )
    parser.add_argument(
        '--lidar',
        action='store_true',
        help='Attach a downward-facing RTX LiDAR under /World/Vehicle/chassis/lidar_mount',
    )
    parser.add_argument(
        '--zed',
        action='store_true',
        help='Attach simulated ZED stereo/depth cameras on zed1_mount and zed2_mount',
    )
    parser.add_argument(
        '--ros2',
        action='store_true',
        help='Enable isaacsim.ros2.bridge and publish /clock, /tf, /tf_static, and sensor point clouds',
    )
    return parser


# ============================================================
# Isaac Sim Runtime Entry Point
# ============================================================
def main() -> None:
    os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
    os.environ.setdefault('ROS_DISTRO', 'jazzy')

    parser = build_arg_parser()
    add_app_launcher_args(parser)
    args = parser.parse_args()

    # RTX LiDAR, ZED depth cameras, and ROS 2 camera/LiDAR publishers require the
    # rendering pipeline even when running with --headless.
    if args.lidar or args.zed or args.ros2:
        args.enable_cameras = True

    app_launcher = launch_app(args)
    simulation_app = app_launcher.app

    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    import _frames  # noqa: F401

    app = omni.kit.app.get_app()

    # --------------------------------------------------------
    # Load USD Scene
    # --------------------------------------------------------
    scene_path = os.path.expanduser('~/go_aero/vehicle_scene.usd')
    print(f'[INFO] Opening vehicle scene: {scene_path}', flush=True)
    omni.usd.get_context().open_stage(scene_path)

    for i in range(300):
        app.update()
        loading = omni.usd.get_context().get_stage_loading_status()
        if loading[2] == 0 and i > 30:
            break

    stage = omni.usd.get_context().get_stage()

    # Guard ViewportCameraState when running with --headless
    if not getattr(args, 'headless', False):
        try:
            from omni.kit.viewport.utility.camera_state import ViewportCameraState

            cam = ViewportCameraState('/OmniverseKit_Persp')
            cam.set_position_world(Gf.Vec3d(8.0, -8.0, 5.0), True)
            cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
        except Exception as exc:
            print(f'[WARN] Could not set ViewportCameraState: {exc}', flush=True)

    # --------------------------------------------------------
    # Physics Handles
    # --------------------------------------------------------
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.sensors.physics import IMUSensor

    chassis = SingleRigidPrim('/World/Vehicle/chassis', name='chassis')
    imu = IMUSensor(
        prim_path='/World/Vehicle/chassis/chassis_IMU',
        name='chassis_imu',
        frequency=SENSOR_RATE,
        translation=np.zeros(3),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    prop_drives = {}
    for name in PROP_LAYOUT:
        joint = stage.GetPrimAtPath(f'/World/Vehicle/joints/{name}_joint')
        if joint:
            prop_drives[name] = UsdPhysics.DriveAPI.Get(joint, 'angular')

    prop_order = MOTOR_MAPS[args.motor_map]
    spin_key = 'ardu_spin_dir' if args.motor_map == 'arducopter_quadx' else 'spin_dir'
    print(f'[INFO] Motor map: {args.motor_map} -> {prop_order}', flush=True)

    # --------------------------------------------------------
    # Start Timeline & Initialize Physics Wrappers
    # --------------------------------------------------------
    try:
        from pxr import PhysxSchema
        physics_prim = stage.GetPrimAtPath('/World/PhysicsScene')
        if physics_prim.IsValid():
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_prim)
            physx_scene.CreateTimeStepsPerSecondAttr(SENSOR_RATE)
        stage.SetTimeCodesPerSecond(SENSOR_RATE)
    except Exception as exc:
        print(f'[WARN] Could not configure 250 Hz PhysX scene rate: {exc}', flush=True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(30):
        app.update()

    chassis.initialize()
    imu.initialize()

    # Record initial spawn position so ArduPilot NED origin (0, 0, 0) corresponds
    # to the vehicle sitting on the ground / spawn point.
    pos_w, quat_w = chassis.get_world_pose()
    pos_w = np.asarray(pos_w, dtype=np.float64)
    quat_w = np.asarray(quat_w, dtype=np.float64)
    spawn_z = float(pos_w[2])

    # --------------------------------------------------------
    # Optional RTX LiDAR sensor (--lidar flag)
    # --------------------------------------------------------
    lidar_annotator = None
    lidar_hydra_texture = None
    LIDAR_READ_INTERVAL = 25  # ~10 Hz at 250 Hz physics

    if args.lidar:
        import omni.kit.commands
        import omni.replicator.core as rep

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        enable_required_extensions(ext_manager, ['isaacsim.sensors.rtx'])
        for _ in range(10):
            app.update()

        lidar_mount_path = LIDAR_MOUNT_PATH
        if not stage.GetPrimAtPath(lidar_mount_path).IsValid():
            if stage.GetPrimAtPath('/World/Vehicle/prop_fr/lidar_mount').IsValid():
                print(
                    '[WARN] Old USD has lidar_mount under prop_fr (which spins with the prop!). '
                    'Rebuild scene with `python build_scene.py --headless`. '
                    f'Creating {lidar_mount_path} dynamically...',
                    flush=True,
                )
            else:
                print(
                    f'[WARN] {lidar_mount_path} not found in USD; '
                    f'creating dynamically at {LIDAR_MOUNT_POS_MESH}...',
                    flush=True,
                )
        ensure_xform_pose(
            stage,
            lidar_mount_path,
            LIDAR_MOUNT_POS_MESH,
            rotate_x_deg=LIDAR_MOUNT_ROT_X_DEG,
            Gf_module=Gf,
            UsdGeom_module=UsdGeom,
        )

        _, _ = omni.kit.commands.execute(
            'IsaacSensorCreateRtxLidar',
            path='/lidar_sensor',
            parent=lidar_mount_path,
            config='Example_Rotary',
            translation=(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        lidar_hydra_texture = rep.create.render_product(
            LIDAR_SENSOR_PATH,
            resolution=(1, 1),
        )
        lidar_annotator = rep.AnnotatorRegistry.get_annotator(
            'IsaacCreateRTXLidarScanBuffer'
        )
        lidar_annotator.attach([lidar_hydra_texture])
        for _ in range(10):
            app.update()
        print(f'[INFO] RTX LiDAR attached at {LIDAR_SENSOR_PATH}', flush=True)

    # --------------------------------------------------------
    # Optional ZED stereo/depth cameras (--zed flag)
    # --------------------------------------------------------
    zed_render_products = {}

    if args.zed:
        import omni.replicator.core as rep

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        zed_enabled_list = enable_required_extensions(ext_manager, ['sl.sensor.camera'])
        zed_ext_enabled = bool(zed_enabled_list)

        zed1_mount_path = ZED1_MOUNT_PATH
        ensure_xform_pose(
            stage,
            zed1_mount_path,
            ZED1_MOUNT_POS_MESH,
            rotate_x_deg=None,
            Gf_module=Gf,
            UsdGeom_module=UsdGeom,
        )

        zed2_mount_path = ZED2_MOUNT_PATH
        ensure_xform_pose(
            stage,
            zed2_mount_path,
            ZED2_MOUNT_POS_MESH,
            rotate_x_deg=None,
            Gf_module=Gf,
            UsdGeom_module=UsdGeom,
        )

        for zed_name, mount_path in [('zed1', zed1_mount_path), ('zed2', zed2_mount_path)]:
            cam_path = f'{mount_path}/{zed_name}_camera'
            cam_prim = UsdGeom.Camera.Define(stage, cam_path)
            cam_prim.CreateFocalLengthAttr(18.0)
            cam_prim.CreateHorizontalApertureAttr(36.0)
            cam_prim.CreateVerticalApertureAttr(20.25)
            cam_prim.CreateClippingRangeAttr(Gf.Vec2f(0.2, 25.0))
            rp = rep.create.render_product(cam_path, resolution=(640, 360))
            zed_render_products[zed_name] = rp

        for _ in range(10):
            app.update()

        print(
            f'[INFO] ZED cameras attached at {zed1_mount_path}/zed1_camera and '
            f'{zed2_mount_path}/zed2_camera (sl.sensor.camera={zed_ext_enabled})',
            flush=True,
        )

    # --------------------------------------------------------
    # Optional ROS 2 bridge (--ros2 flag)
    # --------------------------------------------------------
    if args.ros2:
        import omni.replicator.core as rep

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        enable_required_extensions(
            ext_manager,
            ['isaacsim.core.nodes', 'isaacsim.ros2.bridge'],
        )
        for _ in range(10):
            app.update()

        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            create_nodes = [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('PublishTF', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
                ('PublishStaticTF', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
            ]
            connect_edges = [
                ('OnPlaybackTick.outputs:tick', 'PublishClock.inputs:execIn'),
                ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
                ('OnPlaybackTick.outputs:tick', 'PublishTF.inputs:execIn'),
                ('ReadSimTime.outputs:simulationTime', 'PublishTF.inputs:timeStamp'),
                ('OnPlaybackTick.outputs:tick', 'PublishStaticTF.inputs:execIn'),
                ('ReadSimTime.outputs:simulationTime', 'PublishStaticTF.inputs:timeStamp'),
            ]
            set_values = [
                ('PublishClock.inputs:topicName', '/clock'),
                ('PublishTF.inputs:topicName', '/tf'),
                ('PublishStaticTF.inputs:topicName', '/tf_static'),
                ('PublishStaticTF.inputs:staticPublisher', True),
            ]

            if args.zed and zed_render_products:
                zed_topics = [
                    ('zed1', '/zed1/zed_node/point_cloud/cloud_registered', ZED1_OPTICAL_FRAME_ID),
                    ('zed2', '/zed2/zed_node/point_cloud/cloud_registered', ZED2_OPTICAL_FRAME_ID),
                ]
                for zed_name, topic_name, frame_id in zed_topics:
                    node_name = f'PublishPCL_{zed_name}'
                    rp_path = zed_render_products[zed_name].path
                    create_nodes.append((node_name, 'isaacsim.ros2.bridge.ROS2CameraHelper'))
                    connect_edges.append(('OnPlaybackTick.outputs:tick', f'{node_name}.inputs:execIn'))
                    set_values.extend([
                        (f'{node_name}.inputs:renderProductPath', rp_path),
                        (f'{node_name}.inputs:topicName', topic_name),
                        (f'{node_name}.inputs:frameId', frame_id),
                        (f'{node_name}.inputs:type', 'depth_pcl'),
                    ])

            og.Controller.edit(
                {'graph_path': '/World/ROS2BridgeGraph', 'evaluator_name': 'execution'},
                {
                    keys.CREATE_NODES: create_nodes,
                    keys.CONNECT: connect_edges,
                    keys.SET_VALUES: set_values,
                },
            )

            tf_targets = [
                p for p in [
                    '/World/Vehicle/chassis',
                    ZED1_MOUNT_PATH,
                    ZED2_MOUNT_PATH,
                    LIDAR_MOUNT_PATH,
                ]
                if stage.GetPrimAtPath(p).IsValid()
            ]
            if tf_targets:
                try:
                    og.Controller.attribute('/World/ROS2BridgeGraph/PublishTF.inputs:targetPrims').set(
                        [Sdf.Path(p) for p in tf_targets]
                    )
                    static_children = [
                        Sdf.Path(p) for p in tf_targets if p != '/World/Vehicle/chassis'
                    ]
                    if stage.GetPrimAtPath('/World/Vehicle/chassis').IsValid() and static_children:
                        og.Controller.attribute(
                            '/World/ROS2BridgeGraph/PublishStaticTF.inputs:parentPrim'
                        ).set([Sdf.Path('/World/Vehicle/chassis')])
                        og.Controller.attribute(
                            '/World/ROS2BridgeGraph/PublishStaticTF.inputs:targetPrims'
                        ).set(static_children)
                    else:
                        og.Controller.attribute(
                            '/World/ROS2BridgeGraph/PublishStaticTF.inputs:targetPrims'
                        ).set([Sdf.Path(p) for p in tf_targets])
                except Exception:
                    pass

            print('[INFO] ROS 2 bridge active (/clock, /tf, /tf_static)', flush=True)
            if args.zed:
                print(
                    '[INFO] ROS 2 publishing ZED point clouds on '
                    '/zed1/zed_node/point_cloud/cloud_registered and '
                    '/zed2/zed_node/point_cloud/cloud_registered',
                    flush=True,
                )
        except Exception as e:
            print(f'[WARN] ROS 2 OmniGraph setup warning: {e}', flush=True)

        if args.lidar and lidar_hydra_texture is not None:
            try:
                lidar_ros2_writer = rep.writers.get('RtxLidarROS2PublishPointCloud')
                lidar_ros2_writer.initialize(topicName='/lidar/points', frameId='lidar_sensor')
                lidar_ros2_writer.attach([lidar_hydra_texture])
                print('[INFO] ROS 2 publishing RTX LiDAR point cloud on /lidar/points', flush=True)
            except Exception as e:
                print(f'[WARN] Could not attach RtxLidarROS2PublishPointCloud: {e}', flush=True)

    # --------------------------------------------------------
    # UDP Socket for ArduPilot SITL SIM_JSON (port 9002)
    # --------------------------------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_host, args.port))
    print(f'[INFO] Bound ArduPilot SIM_JSON UDP socket on {args.bind_host}:{args.port}', flush=True)

    motor_commands = [0.0, 0.0, 0.0, 0.0]
    is_armed = False
    ardu_connected = False
    peer_addr = None
    pending_pkt: ServoPacket | None = None

    print(f'[INFO] Waiting for ArduPilot SITL (-f JSON) on UDP {args.port}...', flush=True)
    print(
        '[INFO]   Start ArduPilot: cd ~/ardupilot && '
        'Tools/autotest/sim_vehicle.py -v ArduCopter -f JSON --no-rebuild '
        '--add-param-file=$HOME/go_aero/params/go_aero_ardupilot.param '
        '--out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14540',
        flush=True,
    )

    if not args.no_wait:
        sock.settimeout(0.05)
        while not ardu_connected and simulation_app.is_running():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                if not getattr(args, 'headless', False):
                    app.update()
                continue
            except OSError:
                break

            try:
                pending_pkt = parse_ardupilot_servo_packet(data)
                peer_addr = addr
                ardu_connected = True
            except ValueError as exc:
                print(f'[WARN] Ignoring malformed packet from {addr}: {exc}', flush=True)

        if not simulation_app.is_running():
            sock.close()
            simulation_app.close()
            sys.exit(0)

        print(f'[INFO] ArduPilot SITL connected from {peer_addr}. Bridge running.', flush=True)

    # Re-sample resting pose after any pre-connection settling ticks so NED down=0
    # matches the actual resting height when lockstep starts.
    pos_w, quat_w = chassis.get_world_pose()
    pos_w = np.asarray(pos_w, dtype=np.float64)
    quat_w = np.asarray(quat_w, dtype=np.float64)
    spawn_z = float(pos_w[2])

    def noise_vec3(std: float) -> np.ndarray:
        return np.zeros(3) if args.no_noise else np.random.normal(0.0, std, size=3)

    # Re-zero simulation time at the moment lockstep begins and advance
    # monotonically by SIM_DT = 1.0 / SENSOR_RATE on each PhysX step.
    step = 0
    sim_time = 0.0
    _render_period_steps = max(1, int(round(SENSOR_RATE / max(args.render_hz, 1.0))))

    try:
        while simulation_app.is_running():
            # --------------------------------------------------
            # 1. Drain all currently waiting UDP packets (non-blocking)
            # --------------------------------------------------
            latest_pkt: ServoPacket | None = pending_pkt
            pending_pkt = None

            sock.setblocking(False)
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
                try:
                    latest_pkt = parse_ardupilot_servo_packet(data)
                    peer_addr = addr
                    ardu_connected = True
                except ValueError as exc:
                    print(f'[WARN] Ignoring malformed packet from {addr}: {exc}', flush=True)

            # If no packet arrived yet and ArduPilot is connected, wait up to a
            # short lockstep timeout (20 ms) for ArduPilot's next servo packet.
            if latest_pkt is None and ardu_connected:
                sock.settimeout(0.020)
                try:
                    data, addr = sock.recvfrom(1024)
                    try:
                        latest_pkt = parse_ardupilot_servo_packet(data)
                        peer_addr = addr
                    except ValueError as exc:
                        print(f'[WARN] Ignoring malformed packet from {addr}: {exc}', flush=True)
                except (socket.timeout, BlockingIOError, OSError):
                    pass

                # Drain any additional packets that arrived right behind it
                sock.setblocking(False)
                while True:
                    try:
                        data, addr = sock.recvfrom(1024)
                    except (BlockingIOError, InterruptedError, OSError):
                        break
                    try:
                        latest_pkt = parse_ardupilot_servo_packet(data)
                        peer_addr = addr
                    except ValueError:
                        pass

            if latest_pkt is not None:
                for i in range(4):
                    motor_commands[i] = latest_pkt.motors_norm[i]
                is_armed = bool(latest_pkt.armed)

            cmds = list(motor_commands)

            # --------------------------------------------------
            # 2. Apply motor forces / torques to PhysX before stepping
            # --------------------------------------------------
            F_body, M_body = compute_motor_forces_and_torques(cmds, motor_map=args.motor_map)

            for i, name in enumerate(prop_order):
                omega = cmds[i] * 1500.0 * PROP_LAYOUT[name][spin_key]
                drive = prop_drives.get(name)
                if drive:
                    drive.GetTargetVelocityAttr().Set(float(omega))

            F_world = quat_rotate(quat_w, F_body)
            M_world = quat_rotate(quat_w, M_body)

            chassis.apply_forces_and_torques_at_pos(
                forces=np.asarray([F_world], dtype=np.float32),
                torques=np.asarray([M_world], dtype=np.float32),
                positions=None,
                is_global=True,
            )

            # --------------------------------------------------
            # 3. Step PhysX by SIM_DT and advance sim_time
            # --------------------------------------------------
            simulation_app.update()
            step += 1
            sim_time = step * SIM_DT

            # --------------------------------------------------
            # 4. Read updated IMU & pose/velocity state from PhysX
            # --------------------------------------------------
            pos_w, quat_w = chassis.get_world_pose()  # ENU world, quat (w, x, y, z)
            pos_w = np.asarray(pos_w, dtype=np.float64)
            quat_w = np.asarray(quat_w, dtype=np.float64)
            lin_vel_w = np.asarray(chassis.get_linear_velocity(), dtype=np.float64)

            imu_frame = imu.get_current_frame()
            lin_acc_mesh = np.asarray(imu_frame['lin_acc'], dtype=np.float64)
            ang_vel_mesh = np.asarray(imu_frame['ang_vel'], dtype=np.float64)

            accel_frd = mesh_body_to_frd(lin_acc_mesh) + noise_vec3(ACC_NOISE_STD)
            gyro_frd = mesh_body_to_frd(ang_vel_mesh) + noise_vec3(GYRO_NOISE_STD)

            # Relative to spawn height so NED down=0 when resting on ground
            pos_rel_enu = np.array([pos_w[0], pos_w[1], pos_w[2] - spawn_z], dtype=np.float64)
            pos_ned = enu_to_ned(pos_rel_enu)
            vel_ned = enu_to_ned(lin_vel_w)
            euler_ned = enu_quat_to_ned_euler(quat_w)

            # --------------------------------------------------
            # 5. Send one lockstep JSON state reply to ArduPilot
            # --------------------------------------------------
            if latest_pkt is not None and peer_addr is not None:
                reply = format_ardupilot_json_reply(
                    timestamp_s=sim_time,
                    gyro_frd=gyro_frd,
                    accel_frd=accel_frd,
                    pos_ned=pos_ned,
                    euler_ned=euler_ned,
                    vel_ned=vel_ned,
                )
                try:
                    sock.sendto(reply, peer_addr)
                except OSError as exc:
                    print(f'[WARN] UDP sendto error: {exc}', flush=True)

            # --------------------------------------------------
            # Optional LiDAR & status log
            # --------------------------------------------------
            if lidar_annotator is not None and step % LIDAR_READ_INTERVAL == 0:
                try:
                    buf = lidar_annotator.get_data()
                    if buf is not None and 'data' in buf:
                        pts = np.array(buf['data']).reshape(-1, 3)
                        if pts.shape[0] > 0 and step % (SENSOR_RATE * 2) == 0:
                            min_dist = float(np.min(np.linalg.norm(pts, axis=1)))
                            print(
                                f'[LIDAR] points={pts.shape[0]}  min_dist={min_dist:.2f}m',
                                flush=True,
                            )
                except Exception as exc:
                    if step % (SENSOR_RATE * 2) == 0:
                        print(f'[LIDAR] read error: {exc}', flush=True)

            if step % (SENSOR_RATE * 2) == 0:
                avg = sum(cmds) / 4.0
                print(
                    f'[INFO] t={sim_time:6.2f}s  z={pos_w[2]:+.2f}m  '
                    f'rpy_deg=({math.degrees(euler_ned[0]):+.1f},'
                    f'{math.degrees(euler_ned[1]):+.1f},'
                    f'{math.degrees(euler_ned[2]):+.1f})  '
                    f'armed={is_armed}  motors=[{cmds[0]:.2f},{cmds[1]:.2f},'
                    f'{cmds[2]:.2f},{cmds[3]:.2f}] avg={avg:.2f}',
                    flush=True,
                )

    except KeyboardInterrupt:
        print('[INFO] SIGINT received, stopping...', flush=True)

    timeline.stop()
    try:
        sock.close()
    except Exception:
        pass
    simulation_app.close()
    print('[INFO] Done.', flush=True)


if __name__ == '__main__':
    main()
