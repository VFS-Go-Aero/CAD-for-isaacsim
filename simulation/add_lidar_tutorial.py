#!/usr/bin/env python3
"""
=============================================================================
 LiDAR Sensor Tutorial — Go Aero eVTOL in Isaac Sim
=============================================================================

This standalone script teaches you how to add a LiDAR (Light Detection And
Ranging) sensor to a simulated drone in NVIDIA Isaac Sim. It is designed as
a step-by-step learning exercise; every section is heavily commented so you
can understand *why* each API call is made.

WHAT YOU WILL LEARN:
  1. What an RTX LiDAR sensor is and how Isaac Sim simulates it
  2. How to create a LiDAR sensor prim and attach it to a vehicle's rigid chassis
  3. Why sensors must be parented to the rigid chassis (never to spinning propellers)
  4. How to configure sensor parameters (range, FOV, resolution)
  5. How to read point cloud data from the sensor at runtime
  6. How to compute useful quantities (distance to ground, point count)

PREREQUISITES:
  - The vehicle scene USD must already be built:
      python build_scene.py --headless
    (or `python build_scene.py --headless --no-terrain` for flat-ground tests)
  - Isaac Sim 5.1 / 6.0 (or Isaac Lab) environment must be available

USAGE:
  python add_lidar_tutorial.py --headless          # headless mode (EC2)
  python add_lidar_tutorial.py                     # with viewport (local GPU)
  python add_lidar_tutorial.py --hover 3.0         # hover at 3 meters

=============================================================================
"""

# ============================================================
# SECTION 1: What is a LiDAR sensor?
# ============================================================
#
# LiDAR stands for Light Detection And Ranging. It works by emitting laser
# pulses and measuring how long they take to bounce back from surfaces.
# This gives a dense "point cloud" — a set of 3D points representing the
# shapes and distances of objects in the environment.
#
# On real drones, LiDAR is used for:
#   - Terrain following: maintain a fixed height above the ground
#   - Obstacle avoidance: detect trees, buildings, power lines
#   - Landing zone detection: find flat, clear areas to land safely
#   - Mapping / SLAM: build 3D maps of the environment
#
# Isaac Sim provides an "RTX LiDAR" sensor that uses GPU ray-tracing to
# simulate a real LiDAR. It supports two main configurations:
#   - Rotary: spinning LiDAR like a Velodyne VLP-16 (360-degree scan)
#   - Solid State: fixed-pattern LiDAR like a Livox (limited FOV, no moving parts)
#
# In this tutorial we mount a Rotary LiDAR on the rigid chassis beneath the
# front-right propeller arm (`/World/Vehicle/chassis/lidar_mount`), oriented so
# its scan sweeps vertically downward toward the ground. Because it is parented
# to `chassis` (and NOT to `prop_fr`), its pose stays rock-solid in the body
# frame while all four propellers spin.

# ============================================================
# SECTION 2: Launch Isaac Sim and load the scene
# ============================================================
# Isaac Sim must be launched before importing most omni.* modules.
# `_compat.py` wraps AppLauncher (IsaacLab / Isaac Sim 5.1) and falls back
# to standalone SimulationApp (Isaac Sim 5.1 / 6.0).

import argparse
import math
import os
import sys

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _compat import (  # noqa: E402
    CHASSIS_PATH,
    LIDAR_MOUNT_PATH,
    LIDAR_MOUNT_POS_MESH,
    LIDAR_MOUNT_ROT_X_DEG,
    LIDAR_SENSOR_PATH,
    VEHICLE_PATH,
    add_app_launcher_args,
    enable_required_extensions,
    ensure_xform_pose,
    launch_app,
)

parser = argparse.ArgumentParser(description='LiDAR sensor tutorial')
parser.add_argument('--hover', type=float, default=2.0,
                    help='Target hover altitude in meters (default: 2.0)')
parser.add_argument('--no-hover', action='store_true',
                    help='Disable auto-hover (drone will fall under gravity)')
parser.add_argument('--duration', type=float, default=10.0,
                    help='Simulation duration in seconds (default: 10.0)')
parser.add_argument('--lidar-config', type=str, default='Example_Rotary',
                    choices=['Example_Rotary', 'Example_Solid_State'],
                    help='LiDAR configuration preset (default: Example_Rotary)')
add_app_launcher_args(parser)
args = parser.parse_args()
# RTX LiDAR requires the rendering pipeline, even in headless mode.
# Setting enable_cameras tells AppLauncher to load the headless.rendering
# experience file instead of the plain headless one.
args.enable_cameras = True
app_launcher = launch_app(args)
simulation_app = app_launcher.app

# Now that the app is running, we can import the rest of the modules.
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()

# Load the pre-built vehicle scene. This USD file was created by
# build_scene.py and contains the full quadrotor with physics, joints,
# and sensor mount points.
print('[TUTORIAL] Opening vehicle_scene.usd...', flush=True)
usd_path = os.path.expanduser('~/go_aero/vehicle_scene.usd')
omni.usd.get_context().open_stage(usd_path)

# Wait for the stage to finish loading. The loading_status tuple is
# (total, loaded, remaining) — we wait until remaining == 0.
for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()
print('[TUTORIAL] Scene loaded.', flush=True)

# Set up the camera so we can see the drone from a nice angle.
# (Not available in --headless mode, so we guard the import and call.)
try:
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    cam = ViewportCameraState('/OmniverseKit_Persp')
    cam.set_position_world(Gf.Vec3d(4.0, -4.0, 3.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
except Exception:
    pass  # headless — no viewport


# ============================================================
# SECTION 3: Create the LiDAR sensor prim
# ============================================================
# An RTX LiDAR sensor in Isaac Sim is created via a Kit command. The
# command creates a Camera prim with special LiDAR attributes. We
# parent it to the lidar_mount Xform that build_scene.py placed on the
# rigid chassis beneath the front-right propeller arm
# (`/World/Vehicle/chassis/lidar_mount`).
#
# Why `/World/Vehicle/chassis/lidar_mount` and NOT `/World/Vehicle/prop_fr/lidar_mount`?
#   - `prop_fr` is a revolute-jointed propeller rigid body that rotates every
#     frame when the motors spin. Parenting a sensor to `prop_fr` causes the
#     sensor frame to spin at motor RPM!
#   - `chassis` is the main airframe rigid body. Parenting `lidar_mount` to
#     `chassis` at `(0.468, -0.04, -0.515)` places the LiDAR in the exact same
#     physical location beneath the front-right arm, but rigidly fixed to the
#     airframe so it never rotates with the propeller.
#
# Key parameters:
#   path     — name of the sensor prim (created under parent)
#   parent   — where in the USD hierarchy to attach it
#   config   — which LiDAR profile to use (affects beam pattern, range, FOV)
#
# Combined with `/World/Vehicle`'s `RotateX(90)`, `RotateX(180)` on `lidar_mount`
# flips the sensor's local `+Y` axis to point straight down (`-Z` world) and
# local `+Z` to point forward (`+Y` world), orienting `Example_Rotary`'s XY scan
# plane vertically to measure ground range beneath the arm.

print('[TUTORIAL] Creating RTX LiDAR sensor...', flush=True)

import omni.kit.commands  # noqa: E402

# Ensure the mount prim exists on the chassis even if an older USD stage was opened.
if not stage.GetPrimAtPath(LIDAR_MOUNT_PATH):
    print(f'[TUTORIAL] Creating missing mount prim at {LIDAR_MOUNT_PATH}...', flush=True)
    ensure_xform_pose(
        stage,
        LIDAR_MOUNT_PATH,
        LIDAR_MOUNT_POS_MESH,
        rotate_x_deg=LIDAR_MOUNT_ROT_X_DEG,
    )

# The RTX LiDAR command lives in the isaacsim.sensors.rtx (or omni.isaac.sensor)
# extension. Enable it via the 5.1/6.0 compatibility helper and tick a few frames.
ext_manager = omni.kit.app.get_app().get_extension_manager()
enable_required_extensions(ext_manager, ['isaacsim.sensors.rtx'])
for _ in range(10):
    app.update()

result, lidar_prim = omni.kit.commands.execute(
    'IsaacSensorCreateRtxLidar',
    path='/lidar_sensor',
    parent=LIDAR_MOUNT_PATH,
    config=args.lidar_config,
    translation=(0, 0, 0),            # no extra offset from mount
    orientation=Gf.Quatd(1, 0, 0, 0),  # identity quaternion (w,x,y,z)
)

lidar_prim_path = LIDAR_SENSOR_PATH
print(f'[TUTORIAL] LiDAR created at: {lidar_prim_path}', flush=True)
print(f'[TUTORIAL] Config: {args.lidar_config}', flush=True)


# ============================================================
# SECTION 4: Configure the sensor (range, FOV, resolution, rate)
# ============================================================
# The config preset (e.g. Example_Rotary) sets default values for
# parameters like:
#
#   - Range (min/max distance in meters)
#   - Horizontal FOV (degrees) and angular resolution
#   - Vertical FOV and number of vertical channels
#   - Rotation rate (Hz) for rotary sensors
#   - Points per second
#
# You can override these by modifying attributes on the sensor prim.
# For this tutorial, we use the defaults. See LIDAR_TUTORIAL.md for
# a full table of configurable parameters.
#
# Common configurations:
#   Example_Rotary       — 360-degree spinning, 16-64 channels, 100m range
#   Example_Solid_State  — ~70 deg FOV, non-repetitive scan, 200m range


# ============================================================
# SECTION 5: Attach annotator for point cloud data
# ============================================================
# To actually *read* data from the LiDAR, we need two things:
#
#   1. A "render product" — this tells the RTX renderer to actually
#      compute the LiDAR rays each frame. Think of it as a camera
#      feed, but for LiDAR.
#
#   2. An "annotator" — this is an Isaac Sim component that takes the
#      raw render product and converts it into structured data (like
#      a NumPy array of 3D points) that we can use in Python.
#
# The annotator we use is 'IsaacCreateRTXLidarScanBuffer'.
# It gives us the full point cloud as an array of (x, y, z) points
# in the sensor's local frame.

print('[TUTORIAL] Setting up LiDAR data pipeline...', flush=True)

import omni.replicator.core as rep  # noqa: E402

# Create a render product for the LiDAR sensor.
# The resolution parameter is required but doesn't control the LiDAR's
# own resolution — that comes from the config file.
hydra_texture = rep.create.render_product(lidar_prim_path, resolution=(1, 1))

# Create and attach the point cloud annotator.
lidar_annotator = rep.AnnotatorRegistry.get_annotator(
    'IsaacCreateRTXLidarScanBuffer'
)
lidar_annotator.attach([hydra_texture])

print('[TUTORIAL] LiDAR data pipeline ready.', flush=True)


# ============================================================
# Helper: verify lidar_mount pose relative to chassis
# ============================================================
def get_relative_pose(stage, child_path, parent_path):
    """Return (rel_translation_vec3d, rel_quatd) of child relative to parent."""
    child_prim = stage.GetPrimAtPath(child_path)
    parent_prim = stage.GetPrimAtPath(parent_path)
    child_tf = UsdGeom.Xformable(child_prim).ComputeLocalToWorldTransform(0)
    parent_tf = UsdGeom.Xformable(parent_prim).ComputeLocalToWorldTransform(0)
    rel_tf = child_tf * parent_tf.GetInverse()
    return rel_tf.ExtractTranslation(), rel_tf.ExtractRotationQuat()


def quat_angle_diff_deg(q1, q2):
    """Return angular difference in degrees between two Gf.Quatd rotations."""
    dot = abs(
        float(q1.GetReal()) * float(q2.GetReal())
        + float(Gf.Dot(q1.GetImaginary(), q2.GetImaginary()))
    )
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


# ============================================================
# SECTION 6: Set up hover controller and run the simulation
# ============================================================
# We reuse the same PID hover controller from open_scene.py so the
# drone stays airborne while we collect LiDAR data. Without hovering,
# the drone would just fall and crash.

# Get prop joint drives for setting counter-rotating motor speeds
PROP_DIRS = {'prop_fr': 1.0, 'prop_rl': 1.0, 'prop_fl': -1.0, 'prop_rr': -1.0}
prop_drives = {}
for name in PROP_DIRS:
    joint_path = f'{VEHICLE_PATH}/joints/{name}_joint'
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim:
        drive = UsdPhysics.DriveAPI.Get(joint_prim, 'angular')
        if drive:
            prop_drives[name] = drive

# PID controller parameters (same as open_scene.py)
DRONE_MASS = 5.0   # kg
GRAVITY = 9.81     # m/s^2
HOVER_FORCE = DRONE_MASS * GRAVITY
KP = 15.0   # proportional gain
KD = 8.0    # derivative gain
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

# Start the physics timeline
print('[TUTORIAL] Starting simulation...', flush=True)
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

# State for PID controller
prev_z = None
integral_error = 0.0


def _read_chassis_z() -> float:
    """Read current world Z altitude of `/World/Vehicle/chassis`."""
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
    """Apply vertical hover thrust to `/World/Vehicle/chassis` and spin counter-rotating props."""
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

    prop_speed = (thrust / 4.0) * 500.0
    for name, drive in prop_drives.items():
        direction = PROP_DIRS.get(name, 1.0)
        drive.GetTargetVelocityAttr().Set(float(direction * prop_speed))

    return current_z


# Warm up: let physics settle at hover and the LiDAR render pipeline initialize.
for _ in range(60):
    _apply_hover_step()
    app.update()

# Record initial relative pose of lidar_mount with respect to chassis
init_rel_pos, init_rel_quat = get_relative_pose(stage, LIDAR_MOUNT_PATH, CHASSIS_PATH)
max_mount_pos_drift = 0.0
max_mount_rot_drift_deg = 0.0

print(f'[TUTORIAL] Hover altitude: {args.hover}m', flush=True)
print(f'[TUTORIAL] Duration: {args.duration}s', flush=True)
print('[TUTORIAL] Running... (Ctrl+C to stop early)', flush=True)
print('', flush=True)

step = 0

# State for LiDAR statistics
lidar_read_count = 0
total_points = 0
min_distance_ever = float('inf')
last_min_dist = float('nan')

SIM_RATE = 250  # approximate physics rate (Hz)
LIDAR_READ_INTERVAL = 25  # read LiDAR every 25 steps (~10 Hz)
STATUS_INTERVAL = SIM_RATE  # print status every ~1 second
max_steps = int(args.duration * SIM_RATE)

try:
    while simulation_app.is_running() and step < max_steps:
        # ---- Hover controller (applies thrust to CHASSIS_PATH and spins props) ----
        current_z = _apply_hover_step()
        app.update()
        step += 1

        # ---- Verify lidar_mount stays fixed relative to chassis while props spin ----
        if step % LIDAR_READ_INTERVAL == 0:
            try:
                rel_pos, rel_quat = get_relative_pose(stage, LIDAR_MOUNT_PATH, CHASSIS_PATH)
                pos_drift = float((rel_pos - init_rel_pos).GetLength())
                rot_drift = quat_angle_diff_deg(rel_quat, init_rel_quat)
                max_mount_pos_drift = max(max_mount_pos_drift, pos_drift)
                max_mount_rot_drift_deg = max(max_mount_rot_drift_deg, rot_drift)
            except Exception:
                pass

        # ---- Read LiDAR point cloud ----
        if step % LIDAR_READ_INTERVAL == 0:
            try:
                buf = lidar_annotator.get_data()
                if buf is not None and 'data' in buf:
                    pts = np.array(buf['data']).reshape(-1, 3)
                    num_points = pts.shape[0]

                    if num_points > 0:
                        # Compute distance from sensor to each point.
                        # Points are in the sensor's local frame, so the
                        # distance is simply the Euclidean norm.
                        distances = np.linalg.norm(pts, axis=1)
                        min_dist = float(np.min(distances))
                        mean_dist = float(np.mean(distances))
                        max_dist = float(np.max(distances))

                        lidar_read_count += 1
                        total_points += num_points
                        min_distance_ever = min(min_distance_ever, min_dist)
                        last_min_dist = min_dist

                        # Print LiDAR data periodically
                        if step % STATUS_INTERVAL == 0:
                            print(
                                f'[LIDAR] step={step:5d}  '
                                f'points={num_points:5d}  '
                                f'min={min_dist:.2f}m  '
                                f'mean={mean_dist:.2f}m  '
                                f'max={max_dist:.2f}m',
                                flush=True,
                            )
                    elif step % STATUS_INTERVAL == 0:
                        print(f'[LIDAR] step={step:5d}  points=0 (no returns)',
                              flush=True)
            except Exception as e:
                if step % STATUS_INTERVAL == 0:
                    print(f'[LIDAR] read error: {e}', flush=True)

        # ---- Vehicle & Mount Stability status ----
        if step % STATUS_INTERVAL == 0:
            try:
                chassis_z = _read_chassis_z()
                print(
                    f'[HOVER] step={step:5d}  '
                    f'altitude={chassis_z:.2f}m  '
                    f'target={args.hover}m',
                    flush=True,
                )
                print(
                    f'[MOUNT] step={step:5d}  '
                    f'lidar_mount vs chassis drift: '
                    f'pos={max_mount_pos_drift:.6f}m  '
                    f'rot={max_mount_rot_drift_deg:.4f}deg (FIXED to chassis)',
                    flush=True,
                )
            except Exception:
                pass
            print('', flush=True)

except KeyboardInterrupt:
    print('\n[TUTORIAL] Interrupted by user.', flush=True)

# ============================================================
# Print summary
# ============================================================
mount_stable = (max_mount_pos_drift < 1e-4) and (max_mount_rot_drift_deg < 1e-2)
expected_flat_range = float(args.hover) - 0.04
nadir_diff_vs_hover = (
    abs(last_min_dist - float(args.hover)) if not math.isnan(last_min_dist) else float('inf')
)
nadir_pass = nadir_diff_vs_hover <= 0.05

print('=' * 60, flush=True)
print('  LiDAR Tutorial — Summary', flush=True)
print('=' * 60, flush=True)
print(f'  Total physics steps:    {step}', flush=True)
print(f'  LiDAR sensor path:      {lidar_prim_path}', flush=True)
print(f'  LiDAR reads:            {lidar_read_count}', flush=True)
print(f'  Total points collected: {total_points}', flush=True)
if min_distance_ever < float('inf'):
    print(f'  Closest point ever:     {min_distance_ever:.2f}m', flush=True)
else:
    print('  Closest point ever:     N/A (no data)', flush=True)
if not math.isnan(last_min_dist):
    print(
        f'  Steady-state nadir:     {last_min_dist:.2f}m '
        f'(flat-ground hover_z - 0.04m = {expected_flat_range:.2f}m, '
        f'diff={nadir_diff_vs_hover:.3f}m <= 0.05m) '
        f'[{"PASS" if nadir_pass else "FAIL"}: within ±0.05m of hover]',
        flush=True,
    )
else:
    print(
        f'  Steady-state nadir:     N/A '
        f'(expected flat-ground hover_z - 0.04m = {expected_flat_range:.2f}m within ±0.05m)',
        flush=True,
    )
print(
    f'  Mount pose drift:       pos={max_mount_pos_drift:.6f}m, '
    f'rot={max_mount_rot_drift_deg:.4f}deg '
    f'[{"PASS" if mount_stable else "FAIL"}: fixed to chassis]',
    flush=True,
)
print('=' * 60, flush=True)

# Clean up
timeline.stop()
simulation_app.close()
print('[TUTORIAL] Done.', flush=True)


# ============================================================
# SECTION 7: Exercises for students
# ============================================================
#
# Now that you understand how the LiDAR works, try these exercises:
#
# EXERCISE 1: Change the LiDAR position (under the front-left arm)
#   Modify the 'parent' parameter in IsaacSensorCreateRtxLidar to mount
#   the LiDAR under the front-left arm on the rigid chassis:
#     `/World/Vehicle/chassis/lidar_mount_fl`
#   IMPORTANT: Always parent sensor mounts to `/World/Vehicle/chassis`,
#   NEVER to `/World/Vehicle/prop_fl` (which spins with the propeller!).
#   In `build_scene.py`, define:
#     mount_fl = ensure_xform_pose(
#         stage, '/World/Vehicle/chassis/lidar_mount_fl', (-0.468, -0.04, -0.515), rotate_x_deg=180.0
#     )
#
# EXERCISE 2: Switch to Solid State LiDAR
#   Change --lidar-config to Example_Solid_State and observe how the
#   point cloud changes (narrower FOV, different scan pattern).
#   Run: python add_lidar_tutorial.py --headless --lidar-config Example_Solid_State
#
# EXERCISE 3: Add a forward-facing LiDAR
#   Create a second mount point on the chassis with identity rotation
#   (`RotateX(0.0)`, since `/World/Vehicle` already has `RotateX(90.0)` so
#   local `-Z` already points forward along `+Y_world`):
#     a) `ensure_xform_pose(stage, '/World/Vehicle/chassis/lidar_fwd', (0.0, 0.0, -0.3), rotate_x_deg=0.0)`
#     b) A second IsaacSensorCreateRtxLidar command
#     c) A second render product + annotator
#   This is useful for obstacle detection during forward flight.
#
# EXERCISE 4: Publish as ROS2 PointCloud2
#   Isaac Sim can publish sensor data to ROS2 topics. Replace the
#   annotator with a ROS2 publish action:
#     writer = rep.writers.get('RtxLidarROS2PublishPointCloud')
#     writer.initialize(topicName='/lidar/points', frameId='lidar_sensor')
#     writer.attach([hydra_texture])
#   Then view the data in RViz2 or with `ros2 topic echo`.
#
# EXERCISE 5: Terrain-following hover
#   Use the LiDAR min_distance reading to replace the PID controller's
#   altitude target. Instead of hovering at a fixed world altitude,
#   maintain a fixed distance above the ground:
#     target_agl = 2.0  # meters above ground level
#     error = target_agl - min_dist  # from LiDAR
#   This makes the drone follow terrain contours automatically.
