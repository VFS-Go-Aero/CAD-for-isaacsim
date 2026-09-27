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
  2. How to create a LiDAR sensor prim and attach it to a vehicle
  3. How to configure sensor parameters (range, FOV, resolution)
  4. How to read point cloud data from the sensor at runtime
  5. How to compute useful quantities (distance to ground, point count)

PREREQUISITES:
  - The vehicle scene USD must already be built:
      python build_scene.py --headless
  - Isaac Sim / Isaac Lab environment must be available

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
# In this tutorial we mount a Rotary LiDAR under the front-right propeller
# arm, pointing downward toward the ground.

# ============================================================
# SECTION 2: Launch Isaac Sim and load the scene
# ============================================================
# Isaac Sim must be launched before importing most omni.* modules.
# The AppLauncher handles this — it parses CLI args (like --headless)
# and boots the Omniverse runtime.

import os
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'

from isaaclab.app import AppLauncher
import argparse

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
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
# RTX LiDAR requires the rendering pipeline, even in headless mode.
# Setting enable_cameras tells AppLauncher to load the headless.rendering
# experience file instead of the plain headless one.
args.enable_cameras = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Now that the app is running, we can import the rest of the modules.
import numpy as np
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics

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
# (Not available in --headless mode, so we guard the import.)
try:
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    cam = ViewportCameraState('/OmniverseKit_Persp')
    cam.set_position_world(Gf.Vec3d(4.0, -4.0, 3.0), True)
    cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
except (ImportError, ModuleNotFoundError):
    pass  # headless — no viewport


# ============================================================
# SECTION 3: Create the LiDAR sensor prim
# ============================================================
# An RTX LiDAR sensor in Isaac Sim is created via a Kit command. The
# command creates a Camera prim with special LiDAR attributes. We
# parent it to the lidar_mount Xform that build_scene.py placed under
# the front-right prop arm.
#
# Key parameters:
#   path     — name of the sensor prim (created under parent)
#   parent   — where in the USD hierarchy to attach it
#   config   — which LiDAR profile to use (affects beam pattern, range, FOV)
#
# The mount prim already has a RotateX(180) so the sensor's Z-axis
# (default ray direction) points downward toward the ground.

print('[TUTORIAL] Creating RTX LiDAR sensor...', flush=True)

import omni.kit.commands

# The RTX LiDAR command lives in the isaacsim.sensors.rtx extension.
# IsaacLab's headless experience files don't load it by default, so we
# enable it explicitly. The extension manager loads it asynchronously;
# we tick a few frames to let it finish registering its commands.
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate('isaacsim.sensors.rtx', True)
for _ in range(10):
    app.update()

result, lidar_prim = omni.kit.commands.execute(
    'IsaacSensorCreateRtxLidar',
    path='/lidar_sensor',
    parent='/World/Vehicle/prop_fr/lidar_mount',
    config=args.lidar_config,
    translation=(0, 0, 0),            # no extra offset from mount
    orientation=Gf.Quatd(1, 0, 0, 0),  # identity quaternion (w,x,y,z)
)

lidar_prim_path = '/World/Vehicle/prop_fr/lidar_mount/lidar_sensor'
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

import omni.replicator.core as rep

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
# SECTION 6: Set up hover controller and run the simulation
# ============================================================
# We reuse the same PID hover controller from open_scene.py so the
# drone stays airborne while we collect LiDAR data. Without hovering,
# the drone would just fall and crash.

# Get prop joint drives for setting motor speeds
prop_drives = {}
for name in ['prop_fr', 'prop_fl', 'prop_rr', 'prop_rl']:
    joint_path = f'/World/Vehicle/joints/{name}_joint'
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

vehicle_path = '/World/Vehicle'

# Start the physics timeline
print('[TUTORIAL] Starting simulation...', flush=True)
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Warm up: let physics settle and the LiDAR render pipeline initialize.
for _ in range(60):
    app.update()

print(f'[TUTORIAL] Hover altitude: {args.hover}m', flush=True)
print(f'[TUTORIAL] Duration: {args.duration}s', flush=True)
print('[TUTORIAL] Running... (Ctrl+C to stop early)', flush=True)
print('', flush=True)

# State for PID controller
prev_z = args.hover
integral_error = 0.0
step = 0

# State for LiDAR statistics
lidar_read_count = 0
total_points = 0
min_distance_ever = float('inf')

SIM_RATE = 250  # approximate physics rate (Hz)
LIDAR_READ_INTERVAL = 25  # read LiDAR every 25 steps (~10 Hz)
STATUS_INTERVAL = SIM_RATE  # print status every ~1 second
max_steps = int(args.duration * SIM_RATE)

try:
    while simulation_app.is_running() and step < max_steps:
        app.update()
        step += 1

        # ---- Hover controller ----
        if not args.no_hover and len(prop_drives) == 4:
            vehicle_prim = stage.GetPrimAtPath(vehicle_path)
            xformable = UsdGeom.Xformable(vehicle_prim)
            try:
                world_tf = xformable.ComputeLocalToWorldTransform(0)
                pos = world_tf.ExtractTranslation()
                current_z = pos[2]
            except Exception:
                current_z = args.hover

            error = args.hover - current_z
            d_error = (current_z - prev_z) * 60.0
            integral_error = max(-5.0, min(5.0, integral_error + error / 60.0))
            prev_z = current_z

            thrust = HOVER_FORCE + KP * error - KD * d_error + KI * integral_error
            thrust = max(0.0, min(HOVER_FORCE * 3.0, thrust))

            prop_speed = (thrust / 4.0) * 500.0
            for name, drive in prop_drives.items():
                drive.GetTargetVelocityAttr().Set(float(prop_speed))

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

        # ---- Vehicle status ----
        if step % STATUS_INTERVAL == 0:
            try:
                vehicle_prim = stage.GetPrimAtPath(vehicle_path)
                xformable = UsdGeom.Xformable(vehicle_prim)
                world_tf = xformable.ComputeLocalToWorldTransform(0)
                pos = world_tf.ExtractTranslation()
                print(
                    f'[HOVER] step={step:5d}  '
                    f'altitude={pos[2]:.2f}m  '
                    f'target={args.hover}m',
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
print('=' * 60, flush=True)
print('  LiDAR Tutorial — Summary', flush=True)
print('=' * 60, flush=True)
print(f'  Total physics steps:    {step}', flush=True)
print(f'  LiDAR reads:            {lidar_read_count}', flush=True)
print(f'  Total points collected:  {total_points}', flush=True)
if min_distance_ever < float('inf'):
    print(f'  Closest point ever:     {min_distance_ever:.2f}m', flush=True)
else:
    print('  Closest point ever:     N/A (no data)', flush=True)
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
# EXERCISE 1: Change the LiDAR position
#   Modify the 'parent' parameter in IsaacSensorCreateRtxLidar to mount
#   the LiDAR under a different prop arm (e.g., prop_fl, prop_rr, prop_rl).
#   Note: you'll need to add a lidar_mount Xform under that arm first,
#   similar to what build_scene.py does for prop_fr.
#
# EXERCISE 2: Switch to Solid State LiDAR
#   Change --lidar-config to Example_Solid_State and observe how the
#   point cloud changes (narrower FOV, different scan pattern).
#   Run: python add_lidar_tutorial.py --headless --lidar-config Example_Solid_State
#
# EXERCISE 3: Add a forward-facing LiDAR
#   Create a second mount point on the chassis (not rotated 180 degrees)
#   so the LiDAR faces forward. You'll need:
#     a) A new Xform under /World/Vehicle/chassis with appropriate rotation
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
