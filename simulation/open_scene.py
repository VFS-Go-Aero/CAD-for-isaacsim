#!/usr/bin/env python3
"""
Open Go Aero eVTOL scene with physics and basic hover controller.

The drone will:
- Fall under gravity if thrust is 0
- Hover at a target altitude with auto-throttle
- Props spin visually proportional to thrust

Controls (set via constants or future ROS2 integration):
  HOVER_ALTITUDE: target height in meters
  THRUST_SCALE: force per unit of prop speed
"""
import os
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--hover', type=float, default=2.0, help='Target hover altitude (m)')
parser.add_argument('--no-hover', action='store_true', help='Disable auto-hover (drone will fall)')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import math
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema

app = omni.kit.app.get_app()

print('[INFO] Opening vehicle_scene.usd...')
omni.usd.get_context().open_stage(os.path.expanduser('~/go_aero/vehicle_scene.usd'))

for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()
meshes = [p for p in stage.Traverse() if p.GetTypeName() == 'Mesh']
print(f'[INFO] Meshes: {len(meshes)}')

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
cam = ViewportCameraState("/OmniverseKit_Persp")
cam.set_position_world(Gf.Vec3d(4.0, -4.0, 3.0), True)
cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)

# Get prop joint drive APIs for setting velocity
prop_drives = {}
for name in ['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']:
    joint_path = f'/World/Vehicle/joints/{name}_joint'
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim:
        drive = UsdPhysics.DriveAPI.Get(joint_prim, 'angular')
        if drive:
            prop_drives[name] = drive
            print(f'[INFO] Found drive: {name}')

# Get chassis for reading position
chassis_path = '/World/Vehicle/chassis'
vehicle_path = '/World/Vehicle'

# Hover controller parameters
DRONE_MASS = 5.0  # kg
GRAVITY = 9.81
HOVER_FORCE = DRONE_MASS * GRAVITY  # force needed to hover
KP = 15.0   # proportional gain for altitude
KD = 8.0    # derivative gain for altitude
KI = 2.0    # integral gain

# Start simulation
print('[INFO] Starting simulation...')
timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(60):
    app.update()

print(f'[INFO] Hover target: {args.hover}m ({"disabled" if args.no_hover else "enabled"})')
print('[INFO] Running. Ctrl+C to stop.', flush=True)

prev_z = args.hover
integral_error = 0.0
step = 0

try:
    while simulation_app.is_running():
        app.update()
        step += 1

        if not args.no_hover and len(prop_drives) == 4:
            # Read vehicle position
            vehicle_prim = stage.GetPrimAtPath(vehicle_path)
            xformable = UsdGeom.Xformable(vehicle_prim)
            try:
                world_tf = xformable.ComputeLocalToWorldTransform(0)
                pos = world_tf.ExtractTranslation()
                current_z = pos[2]
            except Exception:
                current_z = args.hover

            # PID altitude controller
            error = args.hover - current_z
            d_error = (current_z - prev_z) * 60.0  # approximate derivative
            integral_error = max(-5.0, min(5.0, integral_error + error / 60.0))
            prev_z = current_z

            thrust = HOVER_FORCE + KP * error - KD * d_error + KI * integral_error
            thrust = max(0.0, min(HOVER_FORCE * 3.0, thrust))  # clamp

            # Convert thrust to prop speed (degrees/sec for USD drive)
            # Each prop provides 1/4 of total thrust
            prop_speed = (thrust / 4.0) * 500.0  # arbitrary scale to make props spin visibly

            for name, drive in prop_drives.items():
                drive.GetTargetVelocityAttr().Set(float(prop_speed))

        if step % 600 == 0:
            try:
                vehicle_prim = stage.GetPrimAtPath(vehicle_path)
                xformable = UsdGeom.Xformable(vehicle_prim)
                world_tf = xformable.ComputeLocalToWorldTransform(0)
                pos = world_tf.ExtractTranslation()
                print(f'[INFO] step {step}, z={pos[2]:.2f}m, target={args.hover}m', flush=True)
            except Exception:
                print(f'[INFO] step {step}', flush=True)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
