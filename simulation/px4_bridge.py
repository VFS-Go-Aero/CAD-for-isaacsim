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
  1. Rebuild the scene (includes the chassis_IMU, lidar_mount, zed1_mount, zed2_mount prims):
       python ~/go_aero/build_scene.py --headless

  2. Start the bridge (must start before PX4 so the TCP server is listening):
       python ~/go_aero/px4_bridge.py --device cuda
       # Optional sensors + ROS 2 bridge:
       python ~/go_aero/px4_bridge.py --device cuda --lidar --zed --ros2

  3. Start PX4 in external-sim mode:
       cd ~/PX4-Autopilot/build/px4_sitl_default
       PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d
"""
from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import threading
from typing import Sequence

try:
    import numpy as np
except ImportError:  # pragma: no cover - fallback for bare python without numpy
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
# Constants (pure Python / numpy, importable by unit tests)
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
B_WORLD_NED = _vec((0.189, 0.028, 0.463))

# MAVLink identity. We send on system 1 with MAV_COMP_ID_AUTOPILOT1 (1),
# matching the standard MAVLink HIL endpoint identity expected by PX4's
# SimulatorMavlink handler.
SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 1  # mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

# Propeller layout (mesh-body-frame positions from build_scene.py).
# Spin direction convention:
#   +1 = CCW viewed from above (prop_fr, prop_rl)
#   -1 = CW viewed from above  (prop_fl, prop_rr)
# With my = -K_TAU * T_i * spin_dir in mesh +Y (Up), mesh_body_to_frd maps
# tau_z_FRD = -my = +K_TAU * T_i * spin_dir, matching PX4's 10050_none_go_aero
# control allocator signs:
#   CA_ROTOR0_KM = +0.05 (prop_fr, spin_dir = +1)
#   CA_ROTOR1_KM = -0.05 (prop_rr, spin_dir = -1)
#   CA_ROTOR2_KM = +0.05 (prop_rl, spin_dir = +1)
#   CA_ROTOR3_KM = -0.05 (prop_fl, spin_dir = -1)
PROP_LAYOUT = {
    'prop_fr': {'pos_body': _vec((0.468, 0.048, -0.515)), 'spin_dir': 1},
    'prop_fl': {'pos_body': _vec((-0.468, 0.048, -0.515)), 'spin_dir': -1},
    'prop_rr': {'pos_body': _vec((0.468, 0.048, 0.515)), 'spin_dir': -1},
    'prop_rl': {'pos_body': _vec((-0.468, 0.048, 0.515)), 'spin_dir': 1},
}
# Fixed iteration order that matches the PX4 airframe's CA_ROTORn_* params:
#   rotor 0 = FR, rotor 1 = RR, rotor 2 = RL, rotor 3 = FL
PROP_ORDER = ['prop_fr', 'prop_rr', 'prop_rl', 'prop_fl']


# ============================================================
# Pure-Python Helpers (importable without Isaac Sim)
# ============================================================
def quat_rotate(q_wxyz: Sequence[float], v: Sequence[float]):
    """Rotate a 3-vector by a (w,x,y,z) quaternion."""
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    norm = math.hypot(w, x, y, z)
    if not math.isfinite(norm) or norm <= 0.0:
        return _vec((vx, vy, vz))
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    # Hamilton product q * (0, v) * q_conj, expanded.
    # t = 2 * (q.xyz cross v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v + w*t + q.xyz cross t
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return _vec((rx, ry, rz))


def quat_conj(q_wxyz: Sequence[float]):
    """Return the conjugate of a (w,x,y,z) quaternion."""
    return _vec((q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3]))


def quat_mul(a: Sequence[float], b: Sequence[float]):
    """Hamilton product of two (w,x,y,z) quaternions."""
    aw, ax, ay, az = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bw, bx, by, bz = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    return _vec((
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ))


def mesh_body_to_frd(v: Sequence[float]):
    """Map a vector from mesh-body axes (+X=right, +Y=up, +Z=back) to
    PX4 FRD body axes (+X=fwd, +Y=right, +Z=down)."""
    return _vec((-float(v[2]), float(v[0]), -float(v[1])))


def enu_to_ned(v: Sequence[float]):
    """ENU (E, N, U) -> NED (N, E, D)."""
    return _vec((float(v[1]), float(v[0]), -float(v[2])))


def compute_baro_pressure_mbar(alt_m: float) -> float:
    """ISA barometric formula, returns hPa (mbar)."""
    alt = float(alt_m)
    if math.isnan(alt):
        return 1013.25
    base = max(0.0, 1.0 - 2.25577e-5 * alt)
    try:
        return float(1013.25 * (base ** 5.2559))
    except OverflowError:
        return float('inf')


pressure_from_altitude = compute_baro_pressure_mbar


def compute_motor_forces_and_torques(cmds: Sequence[float]):
    """Compute net thrust force `F_body` and moment `M_body` in the mesh-body
    frame (+X=right, +Y=up, +Z=back) for 4 normalized motor commands `cmds[0..3]`
    ordered by `PROP_ORDER` (FR, RR, RL, FL).
    """
    fx, fy, fz = 0.0, 0.0, 0.0
    mx, my, mz = 0.0, 0.0, 0.0
    for i, name in enumerate(PROP_ORDER):
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
        # Reaction yaw torque: my = -K_TAU * T_i * spin_dir in mesh +Y (Up)
        # maps via mesh_body_to_frd to tau_z_FRD = -my = +K_TAU * T_i * spin_dir.
        my += -K_TAU * T_i * PROP_LAYOUT[name]['spin_dir']
    return _vec((fx, fy, fz)), _vec((mx, my, mz))


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the CLI argument parser for `px4_bridge.py`."""
    parser = argparse.ArgumentParser(
        description='PX4 SITL <-> Isaac Sim MAVLink bridge (Hardware-In-The-Loop)'
    )
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
    parser.add_argument('--lidar', action='store_true',
                        help='Attach a downward-facing RTX LiDAR under chassis (front-right arm)')
    parser.add_argument('--zed', action='store_true',
                        help='Attach simulated ZED stereo/depth cameras on zed1_mount and zed2_mount')
    parser.add_argument('--ros2', action='store_true',
                        help='Enable isaacsim.ros2.bridge and publish /clock, /tf, /tf_static, and sensor point clouds')
    return parser


def main() -> None:
    os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
    os.environ.setdefault('ROS_DISTRO', 'jazzy')

    # ============================================================
    # CLI args + app launch (must happen before other omni imports)
    # ============================================================
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
    from pymavlink import mavutil

    import _frames  # noqa: F401

    app = omni.kit.app.get_app()

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

    # Camera (not available in --headless mode)
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        cam = ViewportCameraState('/OmniverseKit_Persp')
        cam.set_position_world(Gf.Vec3d(8.0, -8.0, 5.0), True)
        cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
    except Exception:
        pass  # headless — no viewport

    # ============================================================
    # Physics handles
    # ============================================================
    from isaacsim.core.prims import SingleRigidPrim
    chassis = SingleRigidPrim('/World/Vehicle/chassis', name='chassis')

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
    if args.protocol == 'tcp':
        mav_url = f'tcpin:{args.bind_host}:{args.px4_port}'
    else:
        mav_url = f'udpin:{args.bind_host}:{args.px4_port}'
    print(f'[INFO] Binding MAVLink {args.protocol.upper()} on '
          f'{args.bind_host}:{args.px4_port}...', flush=True)
    mav = mavutil.mavlink_connection(
        mav_url,
        source_system=SOURCE_SYSTEM,
        source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
    )

    # Shared state between the MAVLink receiver thread and the physics loop.
    motor_commands = [0.0, 0.0, 0.0, 0.0]  # rotor order (FR, RR, RL, FL) per PX4 mixer
    armed = False
    motor_lock = threading.Lock()
    px4_connected = threading.Event()

    def mavlink_receiver():
        """Background thread: drain PX4 messages, update motor + arm state."""
        nonlocal armed
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
                    for idx in range(4):
                        raw_c = float(msg.controls[idx])
                        motor_commands[idx] = (
                            max(0.0, min(1.0, raw_c)) if math.isfinite(raw_c) else 0.0
                        )
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

    # Warm-up: let the physics scene and sensor initialize before we start
    # talking to PX4 — otherwise the first IMU frame reads as all-zeros and
    # PX4's EKF briefly sees impossible sensor data.
    for _ in range(30):
        app.update()

    # Initialize the physics wrappers now that the timeline is playing.
    chassis.initialize()
    imu.initialize()

    # ============================================================
    # Optional RTX LiDAR sensor (--lidar flag)
    # ============================================================
    lidar_annotator = None
    lidar_hydra_texture = None
    LIDAR_READ_INTERVAL = 25  # read every 25 physics steps (~10 Hz at 250 Hz)

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

        result, lidar_prim = omni.kit.commands.execute(
            'IsaacSensorCreateRtxLidar',
            path='/lidar_sensor',
            parent=lidar_mount_path,
            config='Example_Rotary',
            translation=(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )

        # Hydra texture for the LiDAR render product
        lidar_hydra_texture = rep.create.render_product(
            LIDAR_SENSOR_PATH,
            resolution=(1, 1),
        )

        # Annotator that delivers point cloud data on the CPU
        lidar_annotator = rep.AnnotatorRegistry.get_annotator(
            'IsaacCreateRTXLidarScanBuffer'
        )
        lidar_annotator.attach([lidar_hydra_texture])

        # Let the LiDAR render pipeline warm up
        for _ in range(10):
            app.update()

        print(f'[INFO] RTX LiDAR attached at {LIDAR_SENSOR_PATH}', flush=True)

    # ============================================================
    # Optional ZED stereo/depth cameras (--zed flag)
    # ============================================================
    zed_render_products = {}

    if args.zed:
        import omni.replicator.core as rep

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        zed_enabled_list = enable_required_extensions(ext_manager, ['sl.sensor.camera'])
        zed_ext_enabled = bool(zed_enabled_list)

        # Ensure /World/Vehicle/chassis/zed1_mount and
        # /World/Vehicle/chassis/zed2_mount exist as horizontal forward-facing
        # mounts at (0.0, -0.127, -0.1524) in mesh-body coordinates.
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

    # ============================================================
    # Optional ROS 2 bridge (--ros2 flag)
    # ============================================================
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
    last_gps_tick = 0.0
    last_hb_tick = 0.0
    step = 0
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
        east_m = float(pos_enu[0]) + noise(GPS_POS_NOISE_STD)
        north_m = float(pos_enu[1]) + noise(GPS_POS_NOISE_STD)
        up_m = float(pos_enu[2]) + noise(GPS_POS_NOISE_STD)
        lat_offset = north_m / 111139.0
        lon_offset = east_m / (111139.0 * math.cos(math.radians(HOME_LAT)))
        alt_m = up_m + HOME_ALT

        # Horizontal ground speed magnitude (cm/s) and course-over-ground (cdeg).
        vn_m = float(vel_enu[1]) + noise(GPS_VEL_NOISE_STD)
        ve_m = float(vel_enu[0]) + noise(GPS_VEL_NOISE_STD)
        vd_m = -(float(vel_enu[2]) + noise(GPS_VEL_NOISE_STD))
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
            b_world_enu = np.array([B_WORLD_NED[1], B_WORLD_NED[0], -B_WORLD_NED[2]])
            mag_mesh = quat_rotate(quat_conj(quat_w), b_world_enu)
            mag_frd = mesh_body_to_frd(mag_mesh) + noise_vec3(MAG_NOISE_STD)

            # Barometer from altitude above MSL.
            alt_msl = float(pos_w[2]) + HOME_ALT
            pressure_hpa = compute_baro_pressure_mbar(alt_msl) + noise(BARO_NOISE_STD) / 100.0

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

            F_body, M_body = compute_motor_forces_and_torques(cmds)

            for i, name in enumerate(PROP_ORDER):
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
            # LiDAR point cloud read (optional)
            # --------------------------------------------------
            if lidar_annotator is not None and step % LIDAR_READ_INTERVAL == 0:
                try:
                    buf = lidar_annotator.get_data()
                    if buf is not None and 'data' in buf:
                        pts = np.array(buf['data']).reshape(-1, 3)
                        if pts.shape[0] > 0:
                            dists = np.linalg.norm(pts, axis=1)
                            min_dist = float(np.min(dists))
                            if step % (SENSOR_RATE * 2) == 0:
                                print(
                                    f'[LIDAR] points={pts.shape[0]}  '
                                    f'min_dist={min_dist:.2f}m',
                                    flush=True,
                                )
                except Exception as e:
                    if step % (SENSOR_RATE * 2) == 0:
                        print(f'[LIDAR] read error: {e}', flush=True)

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


if __name__ == '__main__':
    main()
