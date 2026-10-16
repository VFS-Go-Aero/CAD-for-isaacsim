#!/usr/bin/env python3
"""
PX4 SIH / ArduPilot SITL Visualizer — reads vehicle state via MAVLink and
updates the Go Aero model position/orientation in Isaac Sim.

PX4 SIH (or ArduPilot SITL) handles physics internally. We mirror the pose,
and optionally publish /clock, /tf, and simulated ZED / LiDAR sensor streams
over ROS 2 (--ros2) for SIL autonomy testing.

Usage:
  1. Start PX4 SIH or ArduPilot SITL
  2. python px4_visualizer.py --device cuda
     # Or with ROS 2 /clock + sensor topics for SIL testing:
     python px4_visualizer.py --device cuda --ros2
  3. In another terminal: connect to PX4 and command takeoff:
       cd ~/PX4-Autopilot/build/px4_sitl_default
       bin/px4 -s "commander arm; commander takeoff"
     Or use QGroundControl / MAVSDK / ardu_poshold.py
"""
import argparse
import math
import os
import sys
import threading
import time

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

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

parser = argparse.ArgumentParser()
parser.add_argument(
    '--mavlink-port', '--port',
    dest='mavlink_port',
    type=int,
    default=14550,
    help='PX4/ArduPilot MAVLink UDP port (default 14550)',
)
parser.add_argument('--ros2', action='store_true',
                    help='Enable isaacsim.ros2.bridge and publish /clock, /tf, and sensor topics for SIL testing')
parser.add_argument('--zed', action='store_true',
                    help='Attach simulated ZED stereo/depth cameras on zed1_mount and zed2_mount')
parser.add_argument('--lidar', action='store_true',
                    help='Attach a downward-facing RTX LiDAR under chassis (front-right arm)')
add_app_launcher_args(parser)
args = parser.parse_args()
if args.ros2 or args.zed or args.lidar:
    args.enable_cameras = True
app_launcher = launch_app(args)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from _frames import Q_MESH_FIX, attitude_to_world_quat  # noqa: E402

app = omni.kit.app.get_app()

# Propeller motor index order per autopilot type:
#   PX4 Quad-X (10050_none_go_aero): M1=FR, M2=RR, M3=RL, M4=FL
#   ArduCopter Quad-X (FRAME_CLASS=1, FRAME_TYPE=1): M1=FR, M2=RL, M3=FL, M4=RR
PX4_PROP_ORDER = ['prop_fr', 'prop_rr', 'prop_rl', 'prop_fl']
ARDU_PROP_ORDER = ['prop_fr', 'prop_rl', 'prop_fl', 'prop_rr']

# ============================================================
# Load scene
# ============================================================
print('[INFO] Opening vehicle scene...')
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
    cam = ViewportCameraState("/OmniverseKit_Persp")
    cam.set_position_world(Gf.Vec3d(12.0, -12.0, 8.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 3.0), True)
except Exception:
    pass  # headless — no viewport

# Get vehicle prim for updating pose
vehicle_prim = stage.GetPrimAtPath('/World/Vehicle')

# Disable dynamic PhysX simulation on the vehicle hierarchy so kinematic pose
# updates on /World/Vehicle and visual prop rotations on /World/Vehicle/prop_*
# are not overwritten by gravity or revolute joint drives during timeline.play().
for prim in stage.Traverse():
    p_path = str(prim.GetPath())
    if p_path == '/World/Vehicle' or p_path.startswith('/World/Vehicle/'):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)

# ============================================================
# Reset vehicle xform to use translate + orient (quaternion)
# so we can apply PX4/ArduPilot attitude (roll/pitch/yaw) per frame.
# The original scene baked in rotateX(90) for the Y-up -> Z-up
# mesh fix; we now fold that into the orient quaternion below.
# ============================================================
vehicle_xformable = UsdGeom.Xformable(vehicle_prim)
vehicle_xformable.ClearXformOpOrder()
v_translate_op = vehicle_xformable.AddTranslateOp()
v_orient_op = vehicle_xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

# Seed initial pose: identity attitude (level, facing north) at ground level.
v_translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.05))
v_orient_op.Set(Q_MESH_FIX)

# Get prop xforms for spinning
prop_prims = {}
for name in ['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']:
    prim = stage.GetPrimAtPath(f'/World/Vehicle/{name}')
    if prim:
        prop_prims[name] = prim

# ============================================================
# Optional sensors & ROS 2 bridge (--ros2 / --zed / --lidar)
# ============================================================
lidar_hydra_texture = None
if args.lidar:
    import omni.kit.commands
    import omni.replicator.core as rep

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    enable_required_extensions(ext_manager, ['isaacsim.sensors.rtx'])
    for _ in range(10):
        app.update()

    lidar_mount_path = LIDAR_MOUNT_PATH
    if not stage.GetPrimAtPath(lidar_mount_path).IsValid():
        ensure_xform_pose(
            stage,
            lidar_mount_path,
            LIDAR_MOUNT_POS_MESH,
            rotate_x_deg=LIDAR_MOUNT_ROT_X_DEG,
        )

    omni.kit.commands.execute(
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
    for _ in range(10):
        app.update()
    print(f'[INFO] RTX LiDAR attached at {LIDAR_SENSOR_PATH}', flush=True)

zed_render_products = {}
if args.zed or args.ros2:
    import omni.replicator.core as rep

    zed1_mount_path = ZED1_MOUNT_PATH
    if not stage.GetPrimAtPath(zed1_mount_path).IsValid():
        ensure_xform_pose(stage, zed1_mount_path, ZED1_MOUNT_POS_MESH)

    zed2_mount_path = ZED2_MOUNT_PATH
    if not stage.GetPrimAtPath(zed2_mount_path).IsValid():
        ensure_xform_pose(stage, zed2_mount_path, ZED2_MOUNT_POS_MESH)

    for zed_name, mount_path in [('zed1', zed1_mount_path), ('zed2', zed2_mount_path)]:
        cam_path = f'{mount_path}/{zed_name}_camera'
        cam_prim = UsdGeom.Camera.Define(stage, cam_path)
        cam_prim.CreateFocalLengthAttr(18.0)
        cam_prim.CreateHorizontalApertureAttr(36.0)
        cam_prim.CreateVerticalApertureAttr(20.25)
        cam_prim.CreateClippingRangeAttr(Gf.Vec2f(0.2, 25.0))
        zed_render_products[zed_name] = rep.create.render_product(cam_path, resolution=(640, 360))

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

        for zed_name, topic_name, frame_id in [
            ('zed1', '/zed1/zed_node/point_cloud/cloud_registered', ZED1_OPTICAL_FRAME_ID),
            ('zed2', '/zed2/zed_node/point_cloud/cloud_registered', ZED2_OPTICAL_FRAME_ID),
        ]:
            if zed_name in zed_render_products:
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
        static_tf_targets = [
            p for p in [
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
            except Exception:
                pass
        if static_tf_targets:
            try:
                og.Controller.attribute('/World/ROS2BridgeGraph/PublishStaticTF.inputs:parentPrim').set(
                    [Sdf.Path('/World/Vehicle/chassis')]
                )
                og.Controller.attribute('/World/ROS2BridgeGraph/PublishStaticTF.inputs:targetPrims').set(
                    [Sdf.Path(p) for p in static_tf_targets]
                )
            except Exception:
                pass

        print(
            '[INFO] ROS 2 bridge active (/clock, /tf, /tf_static, ZED point clouds)',
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

# ============================================================
# MAVLink connection
# ============================================================
print(f'[INFO] Connecting to autopilot on UDP port {args.mavlink_port}...')
mav = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.mavlink_port}')

# Vehicle state from PX4 / ArduPilot
vehicle_state = {
    'x': 0.0, 'y': 0.0, 'z': 0.05,  # ENU position (m)
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'motors': [0.0, 0.0, 0.0, 0.0],
    'prop_order': PX4_PROP_ORDER,
    'armed': False,
    'updated': False,
}
state_lock = threading.Lock()


def mavlink_receiver():
    """Background thread: read PX4/ArduPilot MAVLink messages."""
    while simulation_app.is_running():
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                with state_lock:
                    # NED -> Isaac Sim ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned
                    vehicle_state['x'] = msg.y   # east
                    vehicle_state['y'] = msg.x   # north
                    vehicle_state['z'] = -msg.z  # up
                    vehicle_state['updated'] = True

            elif msg_type == 'ATTITUDE':
                with state_lock:
                    vehicle_state['roll'] = msg.roll
                    vehicle_state['pitch'] = msg.pitch
                    vehicle_state['yaw'] = msg.yaw

            elif msg_type == 'SERVO_OUTPUT_RAW':
                with state_lock:
                    # Normalize servo outputs (typically 1000-2000us) and clamp to [0.0, 1.0]
                    vehicle_state['motors'] = [
                        max(0.0, min(1.0, (msg.servo1_raw - 1000.0) / 1000.0)),
                        max(0.0, min(1.0, (msg.servo2_raw - 1000.0) / 1000.0)),
                        max(0.0, min(1.0, (msg.servo3_raw - 1000.0) / 1000.0)),
                        max(0.0, min(1.0, (msg.servo4_raw - 1000.0) / 1000.0)),
                    ]

            elif msg_type == 'HEARTBEAT':
                if getattr(msg, 'type', None) == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                with state_lock:
                    vehicle_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if getattr(msg, 'autopilot', None) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                        vehicle_state['prop_order'] = ARDU_PROP_ORDER
                    elif getattr(msg, 'autopilot', None) == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                        vehicle_state['prop_order'] = PX4_PROP_ORDER

        except Exception:
            time.sleep(0.1)


# Wait for first heartbeat before starting receiver thread
print('[INFO] Waiting for autopilot heartbeat...')
hb = mav.wait_heartbeat(timeout=30)
if hb is None:
    print(f'[ERROR] No heartbeat received within 30s on UDP port {args.mavlink_port}.', flush=True)
    simulation_app.close()
    sys.exit(1)

if getattr(hb, 'autopilot', None) == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    vehicle_state['prop_order'] = ARDU_PROP_ORDER
    autopilot_name = 'ArduPilot'
else:
    vehicle_state['prop_order'] = PX4_PROP_ORDER
    autopilot_name = 'PX4'

print(
    f'[INFO] {autopilot_name} connected! System {mav.target_system}, '
    f'component {mav.target_component}, prop_order={vehicle_state["prop_order"]}',
    flush=True,
)

recv_thread = threading.Thread(target=mavlink_receiver, daemon=True)
recv_thread.start()
print('[INFO] MAVLink receiver started')

# Request data streams
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1  # 50 Hz all streams
)

# ============================================================
# Start sim (no physics needed - we just update poses)
# ============================================================
print('[INFO] Starting visualization...')
timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(60):
    app.update()

print('[INFO] Visualizer running!')
print('[INFO] To fly:')
print('[INFO]   Terminal into PX4: ~/PX4-Autopilot/build/px4_sitl_default/bin/px4-shell')
print('[INFO]   Or use mavlink shell: mavlink_shell.py')
print('[INFO]   commander arm')
print('[INFO]   commander takeoff')
print('[INFO]   commander land', flush=True)

# Prop spin state
prop_angles = {'prop_fr': 0.0, 'prop_fl': 0.0, 'prop_rr': 0.0, 'prop_rl': 0.0}
# Spin directions: FR=CCW, FL=CW, RR=CW, RL=CCW
prop_dirs = {'prop_fr': 1.0, 'prop_fl': -1.0, 'prop_rr': -1.0, 'prop_rl': 1.0}

step = 0
try:
    while simulation_app.is_running():
        app.update()
        step += 1

        with state_lock:
            x = vehicle_state['x']
            y = vehicle_state['y']
            z = vehicle_state['z']
            roll = vehicle_state['roll']
            pitch = vehicle_state['pitch']
            yaw = vehicle_state['yaw']
            motors = vehicle_state['motors'][:]
            prop_order = list(vehicle_state['prop_order'])
            is_armed = vehicle_state['armed']

        # Update vehicle position (without artificial altitude offset)
        v_translate_op.Set(Gf.Vec3d(x, y, max(0.05, z)))

        # Update vehicle orientation: NED body attitude -> ENU world
        # quaternion (with mesh fix folded in). See attitude_to_world_quat above.
        v_orient_op.Set(attitude_to_world_quat(roll, pitch, yaw))

        # Spin props based on motor output using autopilot-specific motor order
        for i, name in enumerate(prop_order):
            if name in prop_prims and i < len(motors):
                # Spin speed proportional to motor output
                spin_speed = motors[i] * 30.0 * prop_dirs[name]  # degrees per frame
                prop_angles[name] += spin_speed

                # Apply rotation around Y axis (prop spin axis in mesh frame)
                prop_prim = prop_prims[name]
                rotate_attr = prop_prim.GetAttribute('xformOp:rotateY')
                if not rotate_attr:
                    xformable = UsdGeom.Xformable(prop_prim)
                    ops = xformable.GetOrderedXformOps()
                    op_names = [str(op.GetOpName()) for op in ops]
                    if 'xformOp:rotateY' not in op_names:
                        xformable.AddRotateYOp().Set(prop_angles[name])
                    rotate_attr = prop_prim.GetAttribute('xformOp:rotateY')
                if rotate_attr:
                    rotate_attr.Set(float(prop_angles[name] % 360.0))

        # Status log
        if step % 300 == 0:
            status = "ARMED" if is_armed else "DISARMED"
            avg_motor = sum(motors) / 4.0
            print(f'[INFO] step {step}, pos=({x:.1f},{y:.1f},{z:.1f}), {status}, throttle={avg_motor:.0%}', flush=True)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
print('[INFO] Done.')
