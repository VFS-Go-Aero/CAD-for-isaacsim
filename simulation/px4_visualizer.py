#!/usr/bin/env python3
"""
MAVLink pose visualizer. Reads LOCAL_POSITION_NED, ATTITUDE, and
SERVO_OUTPUT_RAW and moves the Go Aero model in Isaac Sim.

The autopilot owns the flight dynamics (PX4 SIH, or ArduCopter's built-in
SITL model). This process only mirrors that pose. Rigid bodies in the USD
are disabled so PhysX cannot drop the vehicle through the ground plane.

Usage with ArduCopter (acceptance Mode 1):
  1. sim_vehicle.py -v ArduCopter, with MAVLink outs on UDP 14550 and 14540
  2. python px4_visualizer.py --device cuda --mavlink-port 14550
  3. python ardu_poshold.py --port 14540

Usage with PX4 SIH:
  1. Start PX4 SIH
  2. python px4_visualizer.py --device cuda
  3. fly_demo.py, QGroundControl, or MAVSDK on UDP 14540
"""
import os
import time
import math
import threading

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--mavlink-port', type=int, default=14550, help='PX4 MAVLink UDP port')
parser.add_argument('--load-only', action='store_true',
                    help='Load the scene and exit. Skips the MAVLink connection.')
parser.add_argument('--terrain', action='store_true',
                    help='Show the LiDAR heightfield. Off by default so the vehicle rests on the ground plane.')
# Isaac Lab 3 no longer registers --headless (headless unless --viz kit)
# but still honors a 'headless' launcher arg. Register it before AppLauncher.
parser.add_argument('--headless', action='store_true', help='Run without a viewport')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics
from pymavlink import mavutil

app = omni.kit.app.get_app()

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

if args.load_only:
    vehicle = stage.GetPrimAtPath('/World/Vehicle')
    lidar = stage.GetPrimAtPath('/World/Vehicle/chassis/lidar_mount')
    if not vehicle.IsValid() or not lidar.IsValid():
        print('[CHECK] scene load failed: vehicle or lidar_mount prim missing', flush=True)
        simulation_app.close()
        raise SystemExit(1)
    print('[CHECK] scene loaded', flush=True)
    simulation_app.close()
    raise SystemExit(0)

# Camera. omni.kit.viewport is only loaded when Kit has a display.
if not getattr(args, 'headless', False):
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    cam = ViewportCameraState("/OmniverseKit_Persp")
    cam.set_position_world(Gf.Vec3d(12.0, -12.0, 8.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 3.0), True)

# Get vehicle prim for updating pose
vehicle_prim = stage.GetPrimAtPath('/World/Vehicle')

# ============================================================
# Reset vehicle xform to use translate + orient (quaternion)
# so we can apply PX4 attitude (roll/pitch/yaw) per frame.
# The original scene baked in rotateX(90) for the Y-up -> Z-up
# mesh fix; we now fold that into the orient quaternion below.
# ============================================================
vehicle_xformable = UsdGeom.Xformable(vehicle_prim)
vehicle_xformable.ClearXformOpOrder()
v_translate_op = vehicle_xformable.AddTranslateOp()
v_orient_op = vehicle_xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

# Attitude conversion helpers shared with px4_bridge.py.
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _frames import Q_MESH_FIX, attitude_to_world_quat  # noqa: E402


# Seed initial pose: identity attitude (level, facing north) at spawn height.
v_translate_op.Set(Gf.Vec3d(0.0, 0.0, 1.5))
v_orient_op.Set(Q_MESH_FIX)


# Get prop xforms for spinning
prop_prims = {}
for name in ['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']:
    prim = stage.GetPrimAtPath(f'/World/Vehicle/{name}')
    if prim:
        prop_prims[name] = prim


def _freeze_physics(stage):
    """Stop PhysX from integrating the vehicle. MAVLink owns the pose."""
    vehicle = stage.GetPrimAtPath('/World/Vehicle')
    if vehicle.IsValid() and vehicle.HasAPI(UsdPhysics.ArticulationRootAPI):
        vehicle.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    for path in (
        '/World/Vehicle/chassis',
        '/World/Vehicle/prop_fr',
        '/World/Vehicle/prop_fl',
        '/World/Vehicle/prop_rr',
        '/World/Vehicle/prop_rl',
    ):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        body = UsdPhysics.RigidBodyAPI(prim)
        body.CreateRigidBodyEnabledAttr(False)
        body.CreateKinematicEnabledAttr(True)


def _terrain_height(east_m, north_m, amplitude=0.5):
    """Match the heightfield authored by build_scene.add_terrain."""
    return (
        amplitude * math.sin(east_m * 0.8) * math.cos(north_m * 0.6)
        + (amplitude * 0.5) * math.sin(east_m * 1.5 + 1.0)
        + (amplitude * 0.3) * math.cos(north_m * 2.0 + 0.5)
    )


def _deactivate(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        prim.SetActive(False)
        print(f'[INFO] Deactivated {path}', flush=True)


# frame.stl is Y-up. After the +90 deg mesh fix, mesh Y is world Z, and the
# landing gear reaches mesh Y=-0.524. The collision box only extends to
# -0.275, so a clearance based on the box leaves the feet in the ground plane.
# 0.53 m puts the lowest vertex about 6 mm above the surface at zero altitude.
BELLY_CLEARANCE_M = 0.53

def _chassis_local_translate(stage):
    """Return the chassis translate op value, or None when the op does not exist."""
    prim = stage.GetPrimAtPath('/World/Vehicle/chassis')
    if not prim.IsValid():
        return None
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            value = op.Get()
            if value is not None:
                return (float(value[0]), float(value[1]), float(value[2]))
    return None


def _pin_chassis_local(stage):
    """Keep the body on the vehicle origin. PhysX must not leave a residual."""
    prim = stage.GetPrimAtPath('/World/Vehicle/chassis')
    if not prim.IsValid():
        return
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        body = UsdPhysics.RigidBodyAPI(prim)
        body.CreateRigidBodyEnabledAttr(False)
        body.CreateKinematicEnabledAttr(True)
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
            continue
        value = op.Get()
        if value is None:
            continue
        if abs(float(value[0])) + abs(float(value[1])) + abs(float(value[2])) < 1e-6:
            continue
        op.Set(type(value)(0, 0, 0))


_freeze_physics(stage)
_pin_chassis_local(stage)
physics_scene = stage.GetPrimAtPath('/World/PhysicsScene')
if physics_scene.IsValid():
    physics_scene.SetActive(False)
if not args.terrain:
    _deactivate(stage, '/World/Terrain')
timeline = omni.timeline.get_timeline_interface()
timeline.stop()
print('[INFO] Physics frozen. Visualizer mirrors MAVLink pose only.', flush=True)

# ============================================================
# MAVLink connection
# ============================================================
print(f'[INFO] Connecting to PX4 on UDP port {args.mavlink_port}...')
mav = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.mavlink_port}')

# Vehicle state from PX4
vehicle_state = {
    'x': 0.0, 'y': 0.0, 'z': 1.5,  # NED position (will convert to ENU)
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'motors': [0.0, 0.0, 0.0, 0.0],
    'armed': False,
    'updated': False,
}
state_lock = threading.Lock()


def mavlink_receiver():
    """Background thread: read PX4 MAVLink messages."""
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                with state_lock:
                    # PX4 NED -> Isaac Sim ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned
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
                    # Normalize servo outputs (typically 1000-2000us) to 0-1
                    vehicle_state['motors'] = [
                        max(0, (msg.servo1_raw - 1000) / 1000.0),
                        max(0, (msg.servo2_raw - 1000) / 1000.0),
                        max(0, (msg.servo3_raw - 1000) / 1000.0),
                        max(0, (msg.servo4_raw - 1000) / 1000.0),
                    ]

            elif msg_type == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                if msg.get_srcSystem() != mav.target_system:
                    continue
                with state_lock:
                    vehicle_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

        except Exception as e:
            time.sleep(0.1)


def _wait_for_autopilot(connection, timeout=30.0):
    """Latch onto the vehicle. MAVProxy heartbeats are MAV_TYPE_GCS."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = deadline - time.time()
        hb = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=min(1.0, remaining))
        if hb is None:
            continue
        if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
            print(
                f'[INFO] Ignoring GCS heartbeat from system {hb.get_srcSystem()}',
                flush=True,
            )
            continue
        connection.target_system = hb.get_srcSystem()
        connection.target_component = hb.get_srcComponent()
        return hb
    return None


print('[INFO] Waiting for autopilot heartbeat (ignoring GCS)...')
heartbeat = _wait_for_autopilot(mav, timeout=30)
if heartbeat is None:
    print('[ERROR] No vehicle heartbeat within 30s.', flush=True)
    simulation_app.close()
    raise SystemExit(1)
print(
    f'[INFO] Autopilot connected. System {mav.target_system}, '
    f'component {mav.target_component}, type {heartbeat.type}',
    flush=True,
)

# PX4 still honors REQUEST_DATA_STREAM. ArduCopter ignores it and uses
# SET_MESSAGE_INTERVAL, so send both, and only to the vehicle.
for stream in (
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
):
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component, stream, 50, 1,
    )
for msg_id, hz in (
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50),
    (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20),
):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(msg_id), float(1_000_000 // hz),
        0, 0, 0, 0, 0,
    )

recv_thread = threading.Thread(target=mavlink_receiver, daemon=True)
recv_thread.start()
print('[INFO] Visualizer running!')
print('[INFO] ArduCopter: python ardu_poshold.py --port 14540')
print('[INFO] PX4: commander arm / takeoff / land, or python fly_demo.py', flush=True)

# Prop spin state
prop_angles = {'prop_fr': 0.0, 'prop_fl': 0.0, 'prop_rr': 0.0, 'prop_rl': 0.0}
# Spin directions: FR=CW, FL=CCW, RR=CCW, RL=CW
prop_dirs = {'prop_fr': 1.0, 'prop_fl': -1.0, 'prop_rr': -1.0, 'prop_rl': 1.0}

step = 0
try:
    while simulation_app.is_running():
        # A viewport Play click would step PhysX on the child rigid bodies.
        if timeline.is_playing():
            timeline.stop()
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
            is_armed = vehicle_state['armed']

        # SITL altitude is height above a flat home. The landing gear extends
        # BELLY_CLEARANCE_M below the vehicle origin. With --terrain, lift
        # by the heightfield so the feet stay above the hills.
        surface = _terrain_height(x, y) if args.terrain else 0.0
        v_translate_op.Set(Gf.Vec3d(x, y, surface + BELLY_CLEARANCE_M + max(0.0, z)))

        # Update vehicle orientation: PX4 NED body attitude -> ENU world
        # quaternion (with mesh fix folded in). See attitude_to_world_quat above.
        v_orient_op.Set(attitude_to_world_quat(roll, pitch, yaw))
        chassis_local = _chassis_local_translate(stage)
        _pin_chassis_local(stage)

        # Spin props based on motor output
        for i, name in enumerate(['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']):
            if name in prop_prims and i < len(motors):
                # Spin speed proportional to motor output
                spin_speed = motors[i] * 30.0 * prop_dirs[name]  # degrees per frame
                prop_angles[name] += spin_speed

                # Apply rotation around Y axis (prop spin axis in mesh frame)
                prop_prim = prop_prims[name]
                # Check if rotateY op exists, add if not
                rotate_attr = prop_prim.GetAttribute('xformOp:rotateY')
                if not rotate_attr:
                    xformable = UsdGeom.Xformable(prop_prim)
                    # Get existing ops
                    ops = xformable.GetOrderedXformOps()
                    op_names = [str(op.GetOpName()) for op in ops]
                    if 'xformOp:rotateY' not in op_names:
                        xformable.AddRotateYOp().Set(prop_angles[name])
                    rotate_attr = prop_prim.GetAttribute('xformOp:rotateY')
                if rotate_attr:
                    rotate_attr.Set(float(prop_angles[name] % 360.0))

        # Status log. chassis_local stays (0,0,0) when PhysX is not pulling the body.
        if step % 300 == 0:
            status = "ARMED" if is_armed else "DISARMED"
            avg_motor = sum(motors) / 4.0
            chassis_prim = stage.GetPrimAtPath('/World/Vehicle/chassis')
            if chassis_local is None:
                local_t = '(0.000,0.000,0.000)'
            else:
                local_t = f'({chassis_local[0]:.3f},{chassis_local[1]:.3f},{chassis_local[2]:.3f})'
            world_z = float('nan')
            if chassis_prim.IsValid():
                world_z = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
            print(
                f'[INFO] step {step}, pos=({x:.1f},{y:.1f},{z:.1f}), {status}, '
                f'throttle={avg_motor:.0%}, chassis_local={local_t}, chassis_world_z={world_z:.2f}',
                flush=True,
            )

        # Static camera — no follow (set once at start)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

simulation_app.close()
print('[INFO] Done.')
