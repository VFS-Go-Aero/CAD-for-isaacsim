#!/usr/bin/env python3
"""
PX4 SIH Visualizer — reads vehicle state from PX4 via MAVLink and
updates the Go Aero model position/orientation in Isaac Sim.

PX4 SIH handles all physics internally. We just mirror the pose.

Usage:
  1. Start PX4 SIH (already running if you followed setup)
  2. python px4_visualizer.py --device cuda
  3. In another terminal: connect to PX4 and command takeoff:
       cd ~/PX4-Autopilot/build/px4_sitl_default
       bin/px4 -s "commander arm; commander takeoff"
     Or use QGroundControl / MAVSDK
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

# Camera
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

# Pre-computed constants for the attitude conversion.
#
# Coordinate frames:
#   PX4 NED body:  +X = nose, +Y = right wing, +Z = down
#   PX4 NED world: +X = north, +Y = east,      +Z = down
#   Isaac Sim ENU world: +X = east, +Y = north, +Z = up
#   Mesh local (Y-up): +X = right, +Y = up, +Z = back (so -Z = forward)
#
# Q_MESH_FIX:  +90 deg around X. Maps Y-up mesh to Z-up world. After this,
#              mesh-forward (-Z) lands at world +Y (north).  Identity yaw
#              from PX4 thus already faces north, no extra offset needed.
# Q_NED_ENU:   180 deg around (1,1,0)/sqrt(2). Maps NED basis to ENU basis
#              (swaps X/Y and flips Z). Used to rebase the body quaternion
#              from NED-world reference to ENU-world reference via similarity.
COS45 = math.sqrt(2) / 2
Q_MESH_FIX = Gf.Quatd(COS45, Gf.Vec3d(COS45, 0.0, 0.0))
Q_NED_ENU = Gf.Quatd(0.0, Gf.Vec3d(COS45, COS45, 0.0))
Q_NED_ENU_INV = Q_NED_ENU.GetInverse()


def euler_ned_to_quat(roll, pitch, yaw):
    """PX4 NED body Tait-Bryan (Z-Y-X) euler -> rotation quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Gf.Quatd(
        cr * cp * cy + sr * sp * sy,
        Gf.Vec3d(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ),
    )


def attitude_to_world_quat(roll, pitch, yaw):
    """
    Combine PX4 NED body attitude with the NED->ENU rebase and the mesh-fix
    rotation so the result can be set directly on xformOp:orient and produces
    the correct visual orientation in Isaac Sim.
    """
    q_body_ned = euler_ned_to_quat(roll, pitch, yaw)
    q_body_enu = Q_NED_ENU * q_body_ned * Q_NED_ENU_INV
    return q_body_enu * Q_MESH_FIX


# Seed initial pose: identity attitude (level, facing north) at spawn height.
v_translate_op.Set(Gf.Vec3d(0.0, 0.0, 1.5))
v_orient_op.Set(Q_MESH_FIX)


# Get prop xforms for spinning
prop_prims = {}
for name in ['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']:
    prim = stage.GetPrimAtPath(f'/World/Vehicle/{name}')
    if prim:
        prop_prims[name] = prim

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
                with state_lock:
                    vehicle_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

        except Exception as e:
            time.sleep(0.1)


recv_thread = threading.Thread(target=mavlink_receiver, daemon=True)
recv_thread.start()
print('[INFO] MAVLink receiver started, waiting for PX4 data...')

# Wait for first message
print('[INFO] Waiting for PX4 heartbeat...')
mav.wait_heartbeat(timeout=30)
print(f'[INFO] PX4 connected! System {mav.target_system}, component {mav.target_component}')

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
# Spin directions: FR=CW, FL=CCW, RR=CCW, RL=CW
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
            is_armed = vehicle_state['armed']

        # Update vehicle position (offset so feet sit on ground)
        v_translate_op.Set(Gf.Vec3d(x, y, max(0.5, z + 0.5)))

        # Update vehicle orientation: PX4 NED body attitude -> ENU world
        # quaternion (with mesh fix folded in). See attitude_to_world_quat above.
        v_orient_op.Set(attitude_to_world_quat(roll, pitch, yaw))

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

        # Status log
        if step % 300 == 0:
            status = "ARMED" if is_armed else "DISARMED"
            avg_motor = sum(motors) / 4.0
            print(f'[INFO] step {step}, pos=({x:.1f},{y:.1f},{z:.1f}), {status}, throttle={avg_motor:.0%}', flush=True)

        # Static camera — no follow (set once at start)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
print('[INFO] Done.')
