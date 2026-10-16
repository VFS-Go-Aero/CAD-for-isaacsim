# Stereolabs ZED 2i Dual Camera Tutorial — Go Aero eVTOL

Add two forward-facing Stereolabs ZED 2i stereo depth cameras (`zed1` and `zed2`)
to the Go Aero eVTOL quadrotor in NVIDIA Isaac Sim, generate 3D point clouds,
verify depth accuracy against calibration obstacles at 3 m, 5 m, and 10 m, and
publish `sensor_msgs/msg/PointCloud2` streams to ROS2 Jazzy.

## Why Dual ZED 2i Cameras on an eVTOL?

The Stereolabs ZED 2i is a wide-FOV (`110°` horizontal) stereo RGB-D camera
used on the Go Aero aircraft for forward perception:

| Use case | How ZED 2i stereo depth helps |
|---|---|
| Forward obstacle avoidance | Detects buildings, trees, towers, and wires at `0.3–20 m` |
| Redundant field of view | Dual cameras (`zed1`, `zed2`) provide sensor redundancy and wider coverage |
| Dense 3D point clouds | Produces colored per-pixel 3D points even on non-reflective surfaces |
| Visual-inertial odometry (VIO) | Tracks 6-DoF aircraft pose in GPS-denied environments |

---

## Coordinate Frame & Mount Alignment (`multi_zed.launch.py` ↔ USD)

In the real aircraft's ROS2 launch file (`multi_zed.launch.py`), both cameras
are attached to `base_link` (FLU: `+X = Forward`, `+Y = Left`, `+Z = Up`):

| Transform | ROS2 FLU (`x, y, z`) | STL Mesh Frame (`+X=right, +Y=up, -Z=fwd`) | USD Prim Path |
|---|---|---|---|
| `base_link -> zed1_camera_link` | `(+0.1524, 0.0, -0.127)` m | `(0.0, -0.127, -0.1524)` m | `/World/Vehicle/chassis/zed1_mount` |
| `base_link -> zed2_camera_link` | `(+0.1524, 0.0, -0.127)` m | `(0.0, -0.127, -0.1524)` m | `/World/Vehicle/chassis/zed2_mount` |

Because `/World/Vehicle` applies `RotateX(90)` to map the Y-up STL mesh into
Isaac Sim's Z-up ENU world:
- Mesh `+X` maps to World `+X` (East / Right)
- Mesh `+Y` maps to World `+Z` (Up)
- Mesh `-Z` maps to World `+Y` (North / Forward)

A `UsdGeom.Camera` looks along its local `-Z` axis with `+Y` up. Therefore,
mounting `zed1_camera` and `zed2_camera` with identity rotation under
`/World/Vehicle/chassis/zed1_mount` and `/World/Vehicle/chassis/zed2_mount`
places both cameras **horizontal to the ground plane** and **facing forward**
along world `+Y` (North).

---

## Two Supported Simulation Backends (`5.1` & `6.0` Compatible)

| Backend (`--zed-backend`) | How it works | Isaac Sim 5.1 | Isaac Sim 6.0 | ZED SDK Required? |
|---|---|---|---|---|
| `ros2_depth` *(default)* | Native Isaac Sim `UsdGeom.Camera` + Replicator depth/PointCloud2 + `isaacsim.ros2.bridge` (`ROS2CameraHelper(type='depth_pcl')`) | Supported | Supported | No |
| `zed_stream` | Stereolabs `sl.sensor.camera` OmniGraph streamer over local IPC ports `30000` (`zed1`) & `30002` (`zed2`) to `zed_ros2_wrapper` | Supported | Automatic fallback to `ros2_depth` | Yes (ZED SDK 5.x) |

> **Why default to `ros2_depth`?**
> `ros2_depth` publishes the exact same ROS2 topics
> (`/zed1/zed_node/point_cloud/cloud_registered`,
> `/zed2/zed_node/point_cloud/cloud_registered`, `/tf_static`, `/clock`)
> expected by downstream perception nodes (`obstacle_detection`) and works
> identically on both **Isaac Sim 5.1** and **Isaac Sim 6.0** without requiring
> the proprietary Stereolabs C++ streamer binary.

---

## Prerequisites

1. Build the vehicle scene USD with the textured calibration obstacles:

```bash
python build_scene.py --headless --no-terrain --obstacles
```

This creates `~/go_aero/vehicle_scene.usd` with:
- `/World/Vehicle/chassis/zed1_mount` and `/World/Vehicle/chassis/zed2_mount`
- `/World/Obstacles/obstacle_3m` at `(0.0, 3.0, 1.5)` (`1.0 x 0.4 x 3.0` m)
- `/World/Obstacles/obstacle_5m` at `(0.0, 5.0, 1.5)` (`1.2 x 0.4 x 3.0` m)
- `/World/Obstacles/obstacle_10m` at `(2.0, 10.0, 2.0)` (`1.5 x 0.5 x 4.0` m)

*(Note: Even if you run `add_zed_tutorial.py` on a `vehicle_scene.usd` built
without `--obstacles`, the tutorial script automatically spawns the missing
mounts and obstacles at startup.)*

---

## Quick Start

```bash
# Headless depth & point-cloud verification (default ros2_depth backend)
python add_zed_tutorial.py --headless

# Enable ROS2 PointCloud2 publishing (/zed1/... and /zed2/...)
python add_zed_tutorial.py --headless --ros2

# Use Stereolabs IPC streamer backend (falls back cleanly if not installed)
python add_zed_tutorial.py --headless --zed-backend zed_stream

# Higher resolution (HD720)
python add_zed_tutorial.py --headless --resolution 1280x720
```

Expected output:

```
[ZED-TUTORIAL] Opening vehicle_scene.usd...
[ZED-TUTORIAL] Scene loaded.
[ZED-TUTORIAL] Created zed1 camera at: /World/Vehicle/chassis/zed1_mount/zed1_camera
[ZED-TUTORIAL] Created zed2 camera at: /World/Vehicle/chassis/zed2_mount/zed2_camera
[ZED-TUTORIAL] Attaching render products (640x360) and depth annotators...
[ZED-TUTORIAL] Starting simulation...
[ZED-TUTORIAL] Backend: ros2_depth
[ZED-TUTORIAL] Hover altitude: 1.5m | Duration: 10.0s

[ZED1] step=  250  points=198420  obstacle_3m=3.00m (err 0.1%)  obstacle_5m=5.00m (err 0.1%)  obstacle_10m=10.01m (err 0.1%)
[ZED2] step=  250  points=198420  obstacle_3m=3.00m (err 0.1%)  obstacle_5m=5.00m (err 0.1%)  obstacle_10m=10.01m (err 0.1%)
...
========================================================================
  Stereolabs ZED 2i Dual Camera Tutorial — Verification Summary
========================================================================
  Total physics steps:  2500
  Active ZED backend:   ros2_depth
  ZED 1 frames read:    100 (/World/Vehicle/chassis/zed1_mount/zed1_camera)
  ZED 2 frames read:    100 (/World/Vehicle/chassis/zed2_mount/zed2_camera)
------------------------------------------------------------------------
  Camera | Obstacle     | Nominal | Measured | Rel Error | Tol    | Status
------------------------------------------------------------------------
  zed1   | obstacle_3m  |   3.00m |    3.00m |     0.05% | ± 5.0% | PASS
  zed1   | obstacle_5m  |   5.00m |    5.00m |     0.04% | ± 5.0% | PASS
  zed1   | obstacle_10m |  10.00m |   10.01m |     0.10% | ±10.0% | PASS
  zed2   | obstacle_3m  |   3.00m |    3.00m |     0.05% | ± 5.0% | PASS
  zed2   | obstacle_5m  |   5.00m |    5.00m |     0.04% | ± 5.0% | PASS
  zed2   | obstacle_10m |  10.00m |   10.01m |     0.10% | ±10.0% | PASS
========================================================================
```

---

## Architecture: How It Fits Together

```
build_scene.py                        px4_bridge.py / add_zed_tutorial.py
--------------                        -----------------------------------
Creates USD prims:                    At runtime:
  /World/Vehicle/                       1. Loads vehicle_scene.usd
    chassis/                            2. Spawns UsdGeom.Camera prims:
      chassis_IMU  ←── IMU mount           - zed1_mount/zed1_camera
      lidar_mount  ←── LiDAR mount         - zed2_mount/zed2_camera
      zed1_mount   ←── ZED #1 mount     3. Attaches Replicator render products
      zed2_mount   ←── ZED #2 mount        + ROS2CameraHelper(type='depth_pcl')
  /World/Obstacles/                     4. Publishes ROS2 topics:
    obstacle_3m    ←── 3.0 m ahead         - /zed1/zed_node/point_cloud/cloud_registered
    obstacle_5m    ←── 5.0 m ahead         - /zed2/zed_node/point_cloud/cloud_registered
    obstacle_10m   ←── 10.0 m ahead        - /tf_static, /clock
```

---

## Exercises

### Exercise 1: Separate `zed1` and `zed2` Horizontally
Modify `ZED1_MOUNT_POS_MESH` and `ZED2_MOUNT_POS_MESH` in `_compat.py` (or in
`build_scene.py`) to place `zed1` on the left nose (`X = -0.15`) and `zed2` on
the right nose (`X = +0.15`), rebuild the scene, and compare their fields of
view around nearby obstacles.

### Exercise 2: Visualize Dual Point Clouds in RViz2
Run the tutorial with `--ros2`:
```bash
python add_zed_tutorial.py --headless --ros2 --duration 60
```
In a second terminal with ROS2 Jazzy sourced:
```bash
ros2 topic list | grep zed
rviz2
```
Set Fixed Frame to `zed1_camera_link` and add `PointCloud2` displays for
`/zed1/zed_node/point_cloud/cloud_registered` and
`/zed2/zed_node/point_cloud/cloud_registered`.

### Exercise 3: Connect `obstacle_detection`
While `add_zed_tutorial.py --headless --ros2` (or `px4_bridge.py --zed --ros2`) is
running, start the ROS2 `obstacle_detection` node and verify that it detects
`obstacle_3m` at `3.0 m` within `±0.15 m`.
