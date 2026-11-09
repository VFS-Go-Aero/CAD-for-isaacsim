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
# Isaac Sim 6.2 provides an RTX LiDAR through isaacsim.sensors.experimental.rtx.
# It uses GPU ray-tracing. Two presets used here:
#   - Example_Rotary: spinning LiDAR like a Velodyne VLP-16 (360-degree scan)
#   - Example_Solid_State: fixed-pattern LiDAR like a Livox (limited FOV)
#
# In this tutorial we mount a Rotary LiDAR on the chassis, under the
# front-right arm, so the sensor does not spin with a propeller.

# ============================================================
# SECTION 2: Launch Isaac Sim and load the scene
# ============================================================
# Isaac Sim must be launched before importing most omni.* modules.
# The AppLauncher handles this. Isaac Lab 3 no longer registers --headless
# (it runs headless unless --viz kit is passed) but still honors the flag
# when this script registers it before add_app_launcher_args.

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
# Must be registered before AppLauncher.add_app_launcher_args. Isaac Lab 3
# dropped --headless from the launcher, but AppLauncher still reads the flag.
parser.add_argument('--headless', action='store_true',
                    help='Run without a viewport')
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
# Isaac Lab 3 has no viewport unless --viz kit was passed.
if not args.headless:
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        cam = ViewportCameraState('/OmniverseKit_Persp')
        cam.set_position_world(Gf.Vec3d(4.0, -4.0, 3.0), True)
        cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)
    except (ImportError, ModuleNotFoundError):
        pass


# ============================================================
# SECTION 3: Create the LiDAR sensor prim
# ============================================================
# Isaac Sim 6.2 replaced isaacsim.sensors.rtx and the
# IsaacSensorCreateRtxLidar Kit command with
# isaacsim.sensors.experimental.rtx. Lidar.create authors an OmniLidar
# prim from a named config. The parent is the lidar_mount Xform that
# build_scene.py placed under the chassis, not under a propeller.
#
# Key parameters:
#   path          — full prim path of the sensor
#   config        — which LiDAR profile to use (beam pattern, range, FOV)
#   translations  — local offset from the mount, shape (1, 3)
#   orientations  — local quaternion wxyz, shape (1, 4)
#
# The mount prim already has a RotateX(180) so the sensor's Z-axis
# (default ray direction) points downward toward the ground.

print('[TUTORIAL] Creating RTX LiDAR sensor...', flush=True)

ext_manager = omni.kit.app.get_app().get_extension_manager()
if not ext_manager.set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx', True):
    raise RuntimeError('Could not enable extension isaacsim.sensors.experimental.rtx')
from isaacsim.sensors.experimental.rtx import (
    Lidar, LidarSensor, generic_model_output, parse_generic_model_output_data,
)

lidar_prim_path = '/World/Vehicle/chassis/lidar_mount/lidar_sensor'
lidar_sensor = LidarSensor(
    Lidar.create(
        lidar_prim_path,
        config=args.lidar_config,
        translations=[[0.0, 0.0, 0.0]],
        orientations=[[1.0, 0.0, 0.0, 0.0]],
    ),
    annotators=['generic-model-output'],
)
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
# You can override these with Lidar.create arguments such as tick_rate,
# or by setting attributes on the OmniLidar prim after create.
# For this tutorial, we use the preset defaults. See LIDAR_TUTORIAL.md for
# a full table of configurable parameters.
#
# Common configurations:
#   Example_Rotary       — 360-degree spinning, 16-64 channels, 100m range
#   Example_Solid_State  — ~70 deg FOV, non-repetitive scan, 200m range


# ============================================================
# SECTION 5: Read the generic model output
# ============================================================
# LidarSensor was created with the 'generic-model-output' annotator.
# That is the Isaac Sim 6.2 replacement for the replicator annotator
# IsaacCreateRTXLidarScanBuffer. get_data returns a buffer that
# parse_generic_model_output_data turns into azimuth, elevation, and
# range (spherical) or xyz points (cartesian).
#
# The render product is owned by LidarSensor. RTX still has to trace
# the rays, which is why enable_cameras was set before launch and why
# the warm-up below steps the app until the first scan arrives.
#
# The old IsaacCreateRTXLidarScanBuffer annotator kept the last cloud.
# generic-model-output is empty on most steps, and missed beams are
# range 0. Slice to numElements and drop those misses so a minimum is
# a real return, the same quantity the old point list produced.

print('[TUTORIAL] Setting up LiDAR data pipeline...', flush=True)


def _prim_float(prim, name, default):
    attr = prim.GetAttribute(name)
    if not attr.IsValid():
        return default
    value = attr.Get()
    return float(value) if value else default


_lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
LIDAR_NEAR_M = max(0.0, _prim_float(_lidar_prim, 'omni:sensor:Core:nearRangeM', 0.0))
LIDAR_FAR_M = _prim_float(_lidar_prim, 'omni:sensor:Core:farRangeM', 100.0)
if LIDAR_FAR_M <= LIDAR_NEAR_M:
    LIDAR_FAR_M = 100.0


def lidar_ranges():
    """Return valid ranges (m) from the latest scan, or None if it is empty."""
    data, _ = lidar_sensor.get_data('generic-model-output')
    if data is None:
        return None
    gmo = parse_generic_model_output_data(data)
    count = int(gmo.numElements)
    if count <= 0:
        return None
    if gmo.elementsCoordsType == generic_model_output.CoordsType.SPHERICAL:
        ranges = np.asarray(gmo.z, dtype=np.float64)[:count]
    else:
        ranges = np.linalg.norm(
            np.stack(
                [
                    np.asarray(gmo.x, dtype=np.float64)[:count],
                    np.asarray(gmo.y, dtype=np.float64)[:count],
                    np.asarray(gmo.z, dtype=np.float64)[:count],
                ],
                axis=1,
            ),
            axis=1,
        )
    ranges = ranges[
        np.isfinite(ranges)
        & (ranges > 0.0)
        & (ranges >= LIDAR_NEAR_M)
        & (ranges <= LIDAR_FAR_M)
    ]
    if ranges.size == 0:
        return None
    return ranges


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

# Start the physics timeline. Under Isaac Lab 3, playing the timeline
# does not create the PhysX tensor view, so initialize it explicitly.
# This tutorial still steps with app.update(), the same pattern as
# px4_bridge.py before a caller applies its own motor wrench.
print('[TUTORIAL] Starting simulation...', flush=True)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
from isaacsim.core.simulation_manager import SimulationManager
SimulationManager.initialize_physics()

# Warm up until the first LiDAR scan arrives. The render pipeline needs
# several frames before generic-model-output has points.
first_ranges = None
for _ in range(300):
    app.update()
    first_ranges = lidar_ranges()
    if first_ranges is not None:
        break
if first_ranges is None:
    print('[WARN] RTX LiDAR returned no points during warm-up', flush=True)
else:
    print(f'[LIDAR] first scan: points={first_ranges.size}  '
          f'min_dist={float(np.min(first_ranges)):.2f}m', flush=True)

print(f'[TUTORIAL] Hover altitude: {args.hover}m', flush=True)
print(f'[TUTORIAL] Duration: {args.duration}s', flush=True)
print('[TUTORIAL] Running... (Ctrl+C to stop early)', flush=True)
print('', flush=True)

# State for PID controller
prev_z = args.hover
integral_error = 0.0
step = 0

# State for LiDAR statistics. last_ranges is the previous non-empty
# scan, which is what the old scan-buffer annotator kept between ticks.
lidar_read_count = 0
total_points = 0
min_distance_ever = float('inf')
last_ranges = first_ranges

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
                distances = lidar_ranges()
                if distances is not None and distances.size > 0:
                    last_ranges = distances
                    num_points = int(distances.size)
                    min_dist = float(np.min(distances))
                    lidar_read_count += 1
                    total_points += num_points
                    min_distance_ever = min(min_distance_ever, min_dist)

                # Print the latest cloud, including one that arrived
                # between status steps. An empty tick is not a new scan.
                reported = distances if distances is not None and distances.size > 0 else last_ranges
                if step % STATUS_INTERVAL == 0:
                    if reported is not None and reported.size > 0:
                        print(
                            f'[LIDAR] step={step:5d}  '
                            f'points={int(reported.size):5d}  '
                            f'min={float(np.min(reported)):.2f}m  '
                            f'mean={float(np.mean(reported)):.2f}m  '
                            f'max={float(np.max(reported)):.2f}m',
                            flush=True,
                        )
                    else:
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
#   In build_scene.py, change the chassis lidar_mount translate and
#   rebuild the scene. Keep the parent at /World/Vehicle/chassis.
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
#     b) A second Lidar.create / LidarSensor at that path
#     c) A second generic-model-output read
#   This is useful for obstacle detection during forward flight.
#
# EXERCISE 4: Publish as ROS2 PointCloud2
#   px4_bridge.py can enable the ROS 2 bridge separately. A second sensor
#   is another LidarSensor with annotators=['generic-model-output']; a ROS
#   publisher would consume lidar_ranges() rather than a replicator writer.
#
# EXERCISE 5: Terrain-following hover
#   Use the LiDAR min_distance reading to replace the PID controller's
#   altitude target. Instead of hovering at a fixed world altitude,
#   maintain a fixed distance above the ground:
#     target_agl = 2.0  # meters above ground level
#     error = target_agl - min_dist  # from LiDAR
#   This makes the drone follow terrain contours automatically.
