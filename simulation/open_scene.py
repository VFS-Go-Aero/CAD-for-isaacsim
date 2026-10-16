#!/usr/bin/env python3
"""
Open Go Aero eVTOL scene with physics and basic hover controller.

The drone will:
- Fall under gravity if thrust is 0 (--no-hover)
- Hover at a target altitude with PID auto-throttle applied to /World/Vehicle/chassis
- Spin all 4 propellers in counter-rotating directions proportional to thrust

Controls (set via CLI flags or constants):
  --hover: target height in meters (default: 2.0)
  --no-hover: disable auto-hover
"""
import argparse
import os
import sys

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _compat import (  # noqa: E402
    CHASSIS_PATH,
    VEHICLE_PATH,
    add_app_launcher_args,
    launch_app,
)

parser = argparse.ArgumentParser(description='Open Go Aero eVTOL USD scene with hover controller')
parser.add_argument('--hover', type=float, default=2.0, help='Target hover altitude (m)')
parser.add_argument('--no-hover', action='store_true', help='Disable auto-hover (drone will fall)')
add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = launch_app(args)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

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

# Camera (not available in --headless mode)
try:
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    cam = ViewportCameraState('/OmniverseKit_Persp')
    cam.set_position_world(Gf.Vec3d(4.0, -4.0, 3.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
except Exception:
    pass  # headless — no viewport

# Get prop joint drive APIs for setting velocity
PROP_DIRS = {'prop_fr': 1.0, 'prop_rl': 1.0, 'prop_fl': -1.0, 'prop_rr': -1.0}
prop_drives = {}
for name in PROP_DIRS:
    joint_path = f'{VEHICLE_PATH}/joints/{name}_joint'
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim:
        drive = UsdPhysics.DriveAPI.Get(joint_prim, 'angular')
        if drive:
            prop_drives[name] = drive
            print(f'[INFO] Found drive: {name}')

# Hover controller parameters
DRONE_MASS = 5.0  # kg
GRAVITY = 9.81
HOVER_FORCE = DRONE_MASS * GRAVITY  # force needed to hover
KP = 15.0   # proportional gain for altitude
KD = 8.0    # derivative gain for altitude
KI = 2.0    # integral gain

# Prepare PhysxForceAPI fallback on /World/Vehicle/chassis before timeline play
chassis_prim = stage.GetPrimAtPath(CHASSIS_PATH)
force_api = None
if chassis_prim:
    try:
        force_api = PhysxSchema.PhysxForceAPI.Apply(chassis_prim)
        force_api.CreateWorldFrameEnabledAttr().Set(True)
        force_api.CreateModeAttr().Set('force')
        force_api.CreateForceAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    except Exception:
        force_api = None

# Start simulation
print('[INFO] Starting simulation...')
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()

# Initialize SingleRigidPrim handle for applying vertical thrust to /World/Vehicle/chassis
chassis_rigid = None
for _mod_name in ('isaacsim.core.prims', 'omni.isaac.core.prims'):
    try:
        _prims_mod = __import__(_mod_name, fromlist=['SingleRigidPrim'])
        chassis_rigid = _prims_mod.SingleRigidPrim(CHASSIS_PATH, name='chassis')
        chassis_rigid.initialize()
        break
    except Exception:
        chassis_rigid = None

prev_z = None
integral_error = 0.0


def _read_chassis_z() -> float:
    """Read current world Z altitude of the dynamic rigid body `/World/Vehicle/chassis`."""
    if chassis_rigid is not None:
        try:
            pos_w, _ = chassis_rigid.get_world_pose()
            return float(np.asarray(pos_w, dtype=np.float64).reshape(-1)[2])
        except Exception:
            pass
    prim = stage.GetPrimAtPath(CHASSIS_PATH)
    if prim:
        try:
            world_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return float(world_tf.ExtractTranslation()[2])
        except Exception:
            pass
    return float(args.hover)


def _apply_hover_step() -> float:
    """Compute PID vertical thrust on `/World/Vehicle/chassis` and spin counter-rotating props."""
    global prev_z, integral_error
    current_z = _read_chassis_z()
    if args.no_hover:
        return current_z

    if prev_z is None:
        prev_z = current_z
    error = args.hover - current_z
    d_error = (current_z - prev_z) * 60.0
    integral_error = max(-5.0, min(5.0, integral_error + error / 60.0))
    prev_z = current_z

    thrust = HOVER_FORCE + KP * error - KD * d_error + KI * integral_error
    thrust = max(0.0, min(HOVER_FORCE * 3.0, thrust))

    # Apply vertical thrust force [0.0, 0.0, thrust] to /World/Vehicle/chassis
    applied = False
    if chassis_rigid is not None:
        for _f_arr in (
            np.asarray([[0.0, 0.0, float(thrust)]], dtype=np.float32),
            np.array([0.0, 0.0, float(thrust)], dtype=np.float32),
        ):
            try:
                chassis_rigid.apply_forces_and_torques_at_pos(
                    forces=_f_arr,
                    positions=None,
                    is_global=True,
                )
                applied = True
                break
            except Exception:
                continue
    if not applied and force_api is not None:
        try:
            force_api.GetForceAttr().Set(Gf.Vec3f(0.0, 0.0, float(thrust)))
        except Exception:
            pass

    # Spin the 4 propellers with counter-rotating directions so reaction torques cancel
    prop_speed = (thrust / 4.0) * 500.0
    for name, drive in prop_drives.items():
        direction = PROP_DIRS.get(name, 1.0)
        drive.GetTargetVelocityAttr().Set(float(direction * prop_speed))

    return current_z


# Warm up physics while actively holding hover thrust on /World/Vehicle/chassis
for _ in range(60):
    _apply_hover_step()
    app.update()

print(f'[INFO] Hover target: {args.hover}m ({"disabled" if args.no_hover else "enabled"})')
print('[INFO] Running. Ctrl+C to stop.', flush=True)

step = 0

try:
    while simulation_app.is_running():
        current_z = _apply_hover_step()
        app.update()
        step += 1

        if step % 600 == 0:
            print(f'[INFO] step {step}, z={current_z:.2f}m, target={args.hover}m', flush=True)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
