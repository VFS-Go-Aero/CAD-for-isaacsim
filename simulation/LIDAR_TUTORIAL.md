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

1. Isaac Sim 5.1 or 6.0 / Isaac Lab environment is installed and working.
2. The vehicle scene USD has been built:

```bash
# Default scene with rolling-hills terrain:
python build_scene.py --headless

# Or flat ground only (z = 0.0) for altitude range verification:
python build_scene.py --headless --no-terrain
```

This creates `~/go_aero/vehicle_scene.usd` with the drone model,
physics, and the `lidar_mount` Xform under the rigid `chassis` prim at
`/World/Vehicle/chassis/lidar_mount` (positioned beneath the front-right
arm at `(0.468, -0.04, -0.515)` so it is fixed to the airframe and never
rotates with the spinning `prop_fr` propeller).

3. Verify the mount point exists:

```bash
# In a Python shell with Isaac Sim loaded:
stage.GetPrimAtPath('/World/Vehicle/chassis/lidar_mount')
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
[TUTORIAL] LiDAR created at: /World/Vehicle/chassis/lidar_mount/lidar_sensor
[TUTORIAL] Config: Example_Rotary
[TUTORIAL] Setting up LiDAR data pipeline...
[TUTORIAL] LiDAR data pipeline ready.
[TUTORIAL] Starting simulation...
[TUTORIAL] Hover altitude: 2.0m
[TUTORIAL] Duration: 10.0s
[TUTORIAL] Running...

[LIDAR] step=  250  points= 4812  min=1.96m  mean=2.12m  max=3.41m
[HOVER] step=  250  altitude=2.00m  target=2.0m
[MOUNT] step=  250  lidar_mount vs chassis drift: pos=0.000000m  rot=0.0000deg (FIXED to chassis)
...
============================================================
  LiDAR Tutorial — Summary
============================================================
  Total physics steps:    2500
  LiDAR sensor path:      /World/Vehicle/chassis/lidar_mount/lidar_sensor
  LiDAR reads:            100
  Total points collected: 481200
  Closest point ever:     1.96m
  Steady-state nadir:     1.96m (flat-ground hover_z - 0.04m = 1.96m, diff=0.040m <= 0.05m) [PASS: within ±0.05m of hover]
  Mount pose drift:       pos=0.000000m, rot=0.0000deg [PASS: fixed to chassis]
============================================================
```

> **Why is `lidar_mount` parented to `chassis` instead of `prop_fr`?**
> In the USD articulation, `/World/Vehicle/prop_fr` is a revolute-jointed rigid
> body that rotates at motor RPM on every physics step. Parenting any sensor to
> a propeller prim causes the sensor's coordinate frame to spin with the motor!
> Mounting under `/World/Vehicle/chassis/lidar_mount` at mesh coordinates
> `(0.468, -0.04, -0.515)` places the LiDAR at the exact same physical location
> beneath the front-right arm (`4 cm` below the chassis origin), keeping the
> sensor rigidly fixed to the airframe and ensuring flat-ground downward range
> matches hover altitude within `±5 cm` (`1.96 m` at `2.00 m` hover).

## Using LiDAR with the PX4 bridge

The full HIL bridge also supports LiDAR via the `--lidar` flag:

```bash
python px4_bridge.py --device cuda --lidar
```

This attaches the same RTX LiDAR sensor at `/World/Vehicle/chassis/lidar_mount/lidar_sensor`
and prints distance data alongside the normal sensor telemetry. The flag is
optional — omitting it preserves the existing sensor-only workflow.

## Code walkthrough

The tutorial script (`add_lidar_tutorial.py`) is organized into seven
sections. Here's what each one does:

### Section 1 — What is a LiDAR sensor? (comments only)

Background on LiDAR technology, drone applications, and the two
Isaac Sim sensor types (Rotary vs. Solid State).

### Section 2 — Launch Isaac Sim and load the scene

```python
from _compat import add_app_launcher_args, launch_app
# ... parse args, launch app ...
omni.usd.get_context().open_stage('~/go_aero/vehicle_scene.usd')
```

Isaac Sim must be launched (via `AppLauncher` or `SimulationApp`) before
importing any `omni.*` modules. The stage loading loop waits until all USD
assets are fully composed.

### Section 3 — Create the LiDAR sensor prim

```python
import omni.kit.commands

result, lidar_prim = omni.kit.commands.execute(
    'IsaacSensorCreateRtxLidar',
    path='/lidar_sensor',
    parent='/World/Vehicle/chassis/lidar_mount',
    config='Example_Rotary',
    translation=(0, 0, 0),
    orientation=Gf.Quatd(1, 0, 0, 0),
)
```

The `IsaacSensorCreateRtxLidar` command creates a Camera-type prim with
special LiDAR attributes. It is parented to `/World/Vehicle/chassis/lidar_mount`,
which has `RotateX(180)` so the sensor's local `+Y` axis points straight down
(`-Z` world) and `Example_Rotary`'s XY scan plane sweeps vertically beneath the arm.

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

### Section 6 — Run simulation, verify mount stability, and read data

The main loop combines a PID hover controller on `/World/Vehicle/chassis`
with periodic LiDAR reads and a relative-pose verification check confirming
that `lidar_mount` stays stationary relative to `chassis` while the
propellers spin:

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

Mount the LiDAR under the front-left arm on the rigid `chassis` (never under a
spinning propeller prim like `prop_fl`!).

1. In `build_scene.py`, add a new mount Xform under `/World/Vehicle/chassis`:
   ```python
   mount_fl = ensure_xform_pose(
       stage, '/World/Vehicle/chassis/lidar_mount_fl', (-0.468, -0.04, -0.515), rotate_x_deg=180.0
   )
   ```
2. Rebuild the scene: `python build_scene.py --headless`
3. In the tutorial script, change the `parent` parameter:
   ```python
   parent='/World/Vehicle/chassis/lidar_mount_fl',
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

1. Add a forward-facing mount under the chassis using identity rotation
   (`RotateX(0.0)`, since `/World/Vehicle` already has `RotateX(90.0)` so
   local `-Z` already points forward along `+Y_world`):
   ```python
   fwd_mount = UsdGeom.Xform.Define(stage, '/World/Vehicle/chassis/lidar_fwd')
   fwd_mount.ClearXformOpOrder()
   fwd_mount.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.3))  # front of chassis
   fwd_mount.AddRotateXOp().Set(0.0)  # identity rotation: local -Z points forward along +Y_world
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
      chassis_IMU  ←── IMU mount       on /World/Vehicle/chassis/lidar_mount
      lidar_mount  ←── LiDAR mount  3. Attaches render product
      zed1_mount   ←── ZED #1 mount    + annotator
      zed2_mount   ←── ZED #2 mount 4. Physics loop:
    prop_fr/                           - Steps PhysX (spins prop_fr..rl)
    prop_fl/                           - Reads IMU, GPS, baro, mag
    prop_rr/                           - Reads LiDAR point cloud
    prop_rl/                           - Sends to PX4 / prints data
```

The pattern is the same for all sensors: **create a mount prim on the rigid
`chassis` in the USD at build time, attach the sensor at runtime.** This
separation keeps the USD file clean, prevents sensors from spinning with
propeller joints, and lets you swap sensor configurations without rebuilding
the scene.

## Reference

- [Isaac Sim RTX LiDAR documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html)
- [Isaac Sim sensor overview](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html)
- [Replicator annotators reference](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html)
- [PX4 distance sensor integration](https://docs.px4.io/main/en/sensor/rangefinders.html)
