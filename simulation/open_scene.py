#!/usr/bin/env python3
"""
Open Go Aero eVTOL scene with physics and basic hover controller.

The drone will:
- Fall under gravity if thrust is 0 (--no-hover)
- Hover at a target altitude by applying an upward force on the chassis
- Spin the props visually in proportion to that force

Controls:
  --hover: target height in meters
"""
import os
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--hover', type=float, default=2.0, help='Target hover altitude (m)')
parser.add_argument('--no-hover', action='store_true', help='Disable auto-hover (drone will fall)')
parser.add_argument('--steps', type=int, default=0,
                    help='Stop after this many simulation steps (0 runs until Ctrl-C)')
# Isaac Lab 3 no longer registers --headless (headless unless --viz kit)
# but still honors a 'headless' launcher arg. Register it before AppLauncher.
parser.add_argument('--headless', action='store_true', help='Run without a viewport')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import math
import numpy as np
from pxr import Gf, UsdPhysics

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

# Camera. omni.kit.viewport is only loaded when Kit has a display.
if not getattr(args, 'headless', False):
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

# Hover controller parameters
DRONE_MASS = 5.0  # kg
GRAVITY = 9.81
HOVER_FORCE = DRONE_MASS * GRAVITY  # force needed to hover
KP = 15.0   # proportional gain for altitude
KD = 8.0    # derivative gain for altitude
KI = 2.0    # integral gain

# Start simulation. Joint drives only spin the props; lift is a force on the chassis.
print('[INFO] Starting simulation...')
ext_manager = app.get_extension_manager()
if not ext_manager.set_extension_enabled_immediate('isaacsim.core.experimental.prims', True):
    raise RuntimeError('Could not enable extension isaacsim.core.experimental.prims')
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager

timeline = omni.timeline.get_timeline_interface()
timeline.play()
SimulationManager.initialize_physics()
chassis = RigidPrim(chassis_path)
for _ in range(5):
    app.update()

prev_z = args.hover
integral_error = 0.0


def apply_hover():
    """PID altitude hold. Returns chassis world Z."""
    global prev_z, integral_error
    pos_w, _ = chassis.get_world_poses()
    current_z = float(pos_w.numpy()[0][2])
    if args.no_hover:
        return current_z

    error = args.hover - current_z
    d_error = (current_z - prev_z) * 60.0  # approximate derivative
    integral_error = max(-5.0, min(5.0, integral_error + error / 60.0))
    prev_z = current_z

    thrust = HOVER_FORCE + KP * error - KD * d_error + KI * integral_error
    thrust = max(0.0, min(HOVER_FORCE * 3.0, thrust))
    chassis.apply_forces_and_torques_at_pos(
        forces=np.asarray([[0.0, 0.0, thrust]], dtype=np.float32),
        torques=np.zeros((1, 3), dtype=np.float32),
        positions=None,
        local_frame=False,
    )

    prop_speed = (thrust / 4.0) * 500.0
    for drive in prop_drives.values():
        drive.GetTargetVelocityAttr().Set(float(prop_speed))
    return current_z


for _ in range(60):
    apply_hover()
    app.update()

print(f'[INFO] Hover target: {args.hover}m ({"disabled" if args.no_hover else "enabled"})')
print('[INFO] Running. Ctrl+C to stop.', flush=True)

step = 0

try:
    while simulation_app.is_running():
        current_z = apply_hover()
        app.update()
        step += 1

        if step % 600 == 0:
            print(f'[INFO] step {step}, z={current_z:.2f}m, target={args.hover}m', flush=True)

        if args.steps and step >= args.steps:
            print(f'[CHECK] completed steps={step}', flush=True)
            break

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
