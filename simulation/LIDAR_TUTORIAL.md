# LiDAR Sensor Tutorial — Go Aero eVTOL

Add a downward-facing LiDAR sensor to the Go Aero quadrotor in Isaac Sim,
read point cloud data, and compute distance to the ground.

## What is LiDAR and why do drones need it?

LiDAR (Light Detection And Ranging) emits laser pulses and measures the time
they take to bounce back from surfaces. The result is a **point cloud** — a
set of 3D coordinates representing the geometry of the environment.

Real-world drone applications:

| Use case | How LiDAR helps |
|---|---|
| Terrain following | Maintain constant height above uneven ground |
| Obstacle avoidance | Detect trees, buildings, power lines ahead |
| Landing zone detection | Find flat, clear areas to set down safely |
| 3D mapping / SLAM | Build dense maps for survey or inspection |
| Altitude hold | More accurate than barometer near the ground |

Isaac Sim's **RTX LiDAR** uses GPU ray-tracing to simulate a real LiDAR
sensor at full fidelity — matching real beam patterns, ranges, and noise
characteristics.

## Prerequisites

1. Isaac Sim / Isaac Lab environment is installed and working.
2. The vehicle scene USD has been built:

```bash
python build_scene.py --headless
```

This creates `~/go_aero/vehicle_scene.usd` with the drone model,
physics, and the `lidar_mount` Xform under the front-right prop arm.

3. Verify the mount point exists:

```bash
# In a Python shell with Isaac Sim loaded:
stage.GetPrimAtPath('/World/Vehicle/prop_fr/lidar_mount')
# Should return a valid Xform prim
```

## Quick start

Run the self-contained tutorial script:

```bash
# Headless (EC2 / no display)
python add_lidar_tutorial.py --headless

# With viewport (local GPU)
python add_lidar_tutorial.py

# Custom hover altitude and duration
python add_lidar_tutorial.py --headless --hover 3.0 --duration 20.0
```

Expected output:

```
[TUTORIAL] Opening vehicle_scene.usd...
[TUTORIAL] Scene loaded.
[TUTORIAL] Creating RTX LiDAR sensor...
[TUTORIAL] LiDAR created at: /World/Vehicle/prop_fr/lidar_mount/lidar_sensor
[TUTORIAL] Config: Example_Rotary
[TUTORIAL] Setting up LiDAR data pipeline...
[TUTORIAL] LiDAR data pipeline ready.
[TUTORIAL] Starting simulation...
[TUTORIAL] Hover altitude: 2.0m
[TUTORIAL] Duration: 10.0s
[TUTORIAL] Running...

[LIDAR] step=  250  points= 4812  min=1.48m  mean=1.95m  max=3.41m
[HOVER] step=  250  altitude=1.98m  target=2.0m
...
============================================================
  LiDAR Tutorial — Summary
============================================================
  Total physics steps:    2500
  LiDAR reads:            100
  Total points collected:  481200
  Closest point ever:     1.47m
============================================================
```

## Using LiDAR with the PX4 bridge

The full HIL bridge also supports LiDAR via the `--lidar` flag:

```bash
python px4_bridge.py --device cuda --lidar
```

This attaches the same RTX LiDAR sensor and prints distance data
alongside the normal sensor telemetry. The flag is optional — omitting
it preserves the existing sensor-only workflow.

## Code walkthrough

The tutorial script (`add_lidar_tutorial.py`) is organized into seven
sections. Here's what each one does:

### Section 1 — What is a LiDAR sensor? (comments only)

Background on LiDAR technology, drone applications, and the two
Isaac Sim sensor types (Rotary vs. Solid State).

### Section 2 — Launch Isaac Sim and load the scene

```python
from isaaclab.app import AppLauncher
# ... parse args, launch app ...
omni.usd.get_context().open_stage('~/go_aero/vehicle_scene.usd')
```

Isaac Sim must be launched (via `AppLauncher`) before importing any
`omni.*` modules. The stage loading loop waits until all USD assets
are fully composed.

### Section 3 — Create the LiDAR sensor prim

```python
import omni.kit.commands

result, lidar_prim = omni.kit.commands.execute(
    'IsaacSensorCreateRtxLidar',
    path='/lidar_sensor',
    parent='/World/Vehicle/prop_fr/lidar_mount',
    config='Example_Rotary',
    translation=(0, 0, 0),
    orientation=Gf.Quatd(1, 0, 0, 0),
)
```

The `IsaacSensorCreateRtxLidar` command creates a Camera-type prim with
special LiDAR attributes. It is parented to the mount Xform, which
already has a `RotateX(180)` so the sensor faces downward.

### Section 4 — Configure the sensor

The `config` parameter selects a preset that defines range, FOV,
resolution, and scan pattern. See the **Sensor parameters** table below.

### Section 5 — Attach annotator for point cloud data

```python
import omni.replicator.core as rep

hydra_texture = rep.create.render_product(lidar_prim_path, resolution=(1, 1))

lidar_annotator = rep.AnnotatorRegistry.get_annotator(
    'IsaacCreateRTXLidarScanBuffer'
)
lidar_annotator.attach([hydra_texture])
```

Two components are needed to read LiDAR data:

1. **Render product** — tells the RTX renderer to compute LiDAR rays
   each frame.
2. **Annotator** — converts raw render output into a structured NumPy
   array of `(x, y, z)` points in the sensor's local frame.

### Section 6 — Run simulation and read data

The main loop combines a PID hover controller (same as `open_scene.py`)
with periodic LiDAR reads:

```python
buf = lidar_annotator.get_data()
pts = np.array(buf['data']).reshape(-1, 3)
distances = np.linalg.norm(pts, axis=1)
min_dist = float(np.min(distances))
```

Points are in the sensor's local coordinate frame. The Euclidean norm
of each point gives the distance from the sensor to that surface.

### Section 7 — Exercises (comments only)

Five hands-on exercises to deepen understanding (see below).

## Sensor parameters

These parameters are set by the config preset. You can override them
by modifying attributes on the sensor prim after creation.

| Parameter | Example_Rotary | Example_Solid_State | Description |
|---|---|---|---|
| Max range | 100 m | 200 m | Maximum detection distance |
| Min range | 0.4 m | 0.05 m | Minimum detection distance |
| Horizontal FOV | 360 deg | ~70 deg | Horizontal scan coverage |
| Vertical FOV | ~30 deg | ~5 deg | Vertical scan coverage |
| Channels | 16–64 | N/A | Number of vertical laser beams |
| Rotation rate | 10–20 Hz | N/A | Revolutions per second |
| Points/second | ~300K | ~300K | Total point generation rate |
| Scan pattern | Repetitive | Non-repetitive | How beams sweep the FOV |

## Exercises

### Exercise 1: Change the LiDAR position

Mount the LiDAR under a different prop arm (e.g., `prop_fl`).

1. In `build_scene.py`, add a new mount Xform:
   ```python
   mount = UsdGeom.Xform.Define(stage, '/World/Vehicle/prop_fl/lidar_mount')
   mount.AddTranslateOp().Set(Gf.Vec3d(0, -0.15, 0))
   mount.AddRotateXOp().Set(180.0)
   ```
2. Rebuild the scene: `python build_scene.py --headless`
3. In the tutorial script, change the `parent` parameter:
   ```python
   parent='/World/Vehicle/prop_fl/lidar_mount',
   ```
4. Run and compare the point cloud — is it different?

### Exercise 2: Switch to Solid State LiDAR

```bash
python add_lidar_tutorial.py --headless --lidar-config Example_Solid_State
```

Observe the differences:
- Fewer points per scan (narrower FOV)
- Non-repetitive scan pattern (better coverage over time)
- Different range characteristics

### Exercise 3: Add a forward-facing LiDAR

Create a second LiDAR for obstacle detection:

1. Add a forward-facing mount under the chassis (no 180-degree flip):
   ```python
   fwd_mount = UsdGeom.Xform.Define(stage, '/World/Vehicle/chassis/lidar_fwd')
   fwd_mount.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.3))  # front of chassis
   fwd_mount.AddRotateXOp().Set(90.0)  # tilt forward
   ```
2. Create a second sensor, render product, and annotator.
3. Read both point clouds and compare: one shows the ground, the other
   shows obstacles ahead.

### Exercise 4: Publish LiDAR data as ROS2 PointCloud2

Replace the CPU annotator with a ROS2 publisher:

```python
writer = rep.writers.get('RtxLidarROS2PublishPointCloud')
writer.initialize(topicName='/lidar/points', frameId='lidar_sensor')
writer.attach([hydra_texture])
```

Then visualize in RViz2:
```bash
ros2 topic echo /lidar/points --no-arr
rviz2  # add PointCloud2 display, set topic to /lidar/points
```

### Exercise 5: Terrain-following hover

Use LiDAR altitude instead of world-frame altitude for the hover
controller:

```python
# Replace the PID error calculation:
target_agl = 2.0  # meters Above Ground Level
error = target_agl - min_dist  # min_dist from LiDAR
```

This makes the drone follow terrain contours. Test by adding a ramp or
hill to the scene (a tilted box prim) and watch the drone maintain
constant height above the surface.

## Architecture: how it fits together

```
build_scene.py                    px4_bridge.py / tutorial script
--------------                    --------------------------------
Creates USD prims:                At runtime:
  /World/Vehicle/                   1. Loads vehicle_scene.usd
    chassis/                        2. Creates RTX LiDAR sensor
      chassis_IMU  ←── IMU mount       on the mount prim
    prop_fr/                        3. Attaches render product
      lidar_mount  ←── LiDAR mount     + annotator
    prop_fl/                        4. Physics loop:
    prop_rr/                           - Steps PhysX
    prop_rl/                           - Reads IMU, GPS, baro, mag
                                       - Reads LiDAR point cloud
                                       - Sends to PX4 / prints data
```

The pattern is the same for all sensors: **create a mount prim in the
USD at build time, attach the sensor at runtime.** This separation
keeps the USD file clean and lets you swap sensor configurations
without rebuilding the scene.

## Reference

- [Isaac Sim RTX LiDAR documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html)
- [Isaac Sim sensor overview](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html)
- [Replicator annotators reference](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html)
- [PX4 distance sensor integration](https://docs.px4.io/main/en/sensor/rangefinders.html)
