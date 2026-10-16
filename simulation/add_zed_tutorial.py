#!/usr/bin/env python3
"""
=============================================================================
 Stereolabs ZED 2i Dual Camera Tutorial — Go Aero eVTOL in Isaac Sim
=============================================================================

This standalone tutorial script teaches you how to simulate two forward-facing
Stereolabs ZED 2i stereo depth cameras (`zed1` and `zed2`) mounted on the
Go Aero eVTOL chassis in NVIDIA Isaac Sim 5.1 and 6.0 (Issue #3).

WHAT YOU WILL LEARN:
  1. How stereo depth / RGB-D cameras work and why eVTOL aircraft use dual ZEDs
  2. How the ZED camera mounts (`zed1_mount`, `zed2_mount`) map between ROS2
     `base_link` (FLU) in `multi_zed.launch.py` and Isaac Sim's USD mesh frame
  3. How to support two simulation backends:
       - `ros2_depth` (default): Native Isaac Sim RGB-D + Replicator depth &
         3D point cloud pipeline. Works out-of-the-box on both Isaac Sim 5.1
         and Isaac Sim 6.0 without proprietary C++ streamer plugins.
       - `zed_stream`: Stereolabs `sl.sensor.camera` OmniGraph streamer over
         local IPC ports 30000 (`zed1`) and 30002 (`zed2`) for hardware-in-loop
         with the ZED SDK and `zed_ros2_wrapper` (`multi_zed.launch.py`).
  4. How to generate dense 3D point clouds and verify median depth accuracy
     against known 3D obstacles at 3 m (±5%), 5 m (±5%), and 10 m (±10%).
  5. How to publish `/zed1/zed_node/point_cloud/cloud_registered`,
     `/zed2/zed_node/point_cloud/cloud_registered`, `/tf_static`, and `/clock`
     to ROS2 Jazzy for RViz2 visualization and obstacle detection.

PREREQUISITES:
  - Build the vehicle scene USD (optionally with `--obstacles` and `--no-terrain`):
      python build_scene.py --headless --no-terrain --obstacles
    (If `/World/Obstacles` or `zed1_mount`/`zed2_mount` are not in the USD yet,
     this script automatically creates them at runtime.)

USAGE:
  python add_zed_tutorial.py --headless
  python add_zed_tutorial.py --headless --ros2
  python add_zed_tutorial.py --headless --zed-backend zed_stream
  python add_zed_tutorial.py --hover 1.5 --duration 10.0

=============================================================================
"""

# ============================================================
# SECTION 1: Why Dual ZED 2i Cameras on the Go Aero eVTOL?
# ============================================================
#
# The Stereolabs ZED 2i is a ruggedized stereo RGB-D camera with a 12 cm
# baseline, 110-degree horizontal field of view, and 0.3 m – 20 m depth range.
#
# On the Go Aero eVTOL, two ZED 2i cameras (`zed1` and `zed2`) are mounted
# beneath the forward nose of the chassis:
#   - In ROS2 (`multi_zed.launch.py`), `base_link -> zed1_camera_link` and
#     `base_link -> zed2_camera_link` have static transform:
#       FLU = (+0.1524 m forward, 0.0 m left, -0.127 m up), roll=0, pitch=0, yaw=0
#   - In the Go Aero STL mesh frame (`+X = right`, `+Y = up`, `-Z = forward`):
#       mesh_pos = (0.0, -0.127, -0.1524)
#   - Because `/World/Vehicle` has `RotateX(90)`, a child Xform of `chassis`
#     with identity rotation has its optical axis (`-Z`) pointing horizontally
#     along world `+Y` (North) and its up axis (`+Y`) along world `+Z` (Up).

import argparse
import math
import os
import sys

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _compat import (  # noqa: E402
    CHASSIS_PATH,
    VEHICLE_PATH,
    ZED1_CAMERA_PATH,
    ZED1_FRAME_ID,
    ZED1_MOUNT_PATH,
    ZED1_MOUNT_POS_MESH,
    ZED1_OPTICAL_FRAME_ID,
    ZED2_CAMERA_PATH,
    ZED2_FRAME_ID,
    ZED2_MOUNT_PATH,
    ZED2_MOUNT_POS_MESH,
    ZED2_OPTICAL_FRAME_ID,
    ZED_CLIP_FAR_M,
    ZED_CLIP_NEAR_M,
    ZED_FOCAL_LENGTH_MM,
    ZED_H_APERTURE_MM,
    ZED_V_APERTURE_MM,
    add_app_launcher_args,
    enable_required_extensions,
    ensure_xform_pose,
    launch_app,
)

try:
    import numpy as np
except ImportError:
    np = None  # type: ignore[assignment]

# ============================================================
# Shared ZED 2i Optical Intrinsics & Obstacle Calibration Specs
# ============================================================
FOCAL_LENGTH_MM = ZED_FOCAL_LENGTH_MM
H_APERTURE_MM = ZED_H_APERTURE_MM
V_APERTURE_MM = ZED_V_APERTURE_MM
CLIP_NEAR_M = ZED_CLIP_NEAR_M
CLIP_FAR_M = ZED_CLIP_FAR_M

OBSTACLE_SPECS = [
    {
        'name': 'obstacle_3m',
        'prim_path': '/World/Obstacles/obstacle_3m',
        'nominal_dist_m': 3.0,
        'center_xyz': (0.0, 3.0, 1.5),
        'size_xyz': (1.0, 0.4, 3.0),
        'color_a': (0.90, 0.35, 0.15),
        'color_b': (0.20, 0.25, 0.30),
        'window_uv': (0.28, 0.72, 0.35, 0.75),
        'tolerance_pct': 5.0,
        # ROI filter in world / vehicle frame (X=east, Y=north)
        'roi_x': (-0.55, 0.55),
        'roi_y': (2.30, 3.60),
    },
    {
        'name': 'obstacle_5m',
        'prim_path': '/World/Obstacles/obstacle_5m',
        'nominal_dist_m': 5.0,
        'center_xyz': (0.0, 5.0, 1.5),
        'size_xyz': (1.2, 0.4, 3.0),
        'color_a': (0.20, 0.75, 0.35),
        'color_b': (0.15, 0.20, 0.25),
        'window_uv': None,
        'tolerance_pct': 5.0,
        'roi_x': (-0.65, 0.65),
        'roi_y': (4.30, 5.60),
    },
    {
        'name': 'obstacle_10m',
        'prim_path': '/World/Obstacles/obstacle_10m',
        'nominal_dist_m': 10.0,
        'center_xyz': (2.0, 10.0, 2.0),
        'size_xyz': (1.5, 0.5, 4.0),
        'color_a': (0.25, 0.55, 0.90),
        'color_b': (0.18, 0.18, 0.24),
        'window_uv': None,
        'tolerance_pct': 10.0,
        'roi_x': (1.10, 2.90),
        'roi_y': (9.00, 11.00),
    },
]


class _UnprojectedROIResult(tuple):
    """3-tuple `(x_cam, y_cam, z_cam)` in local USD camera coordinates
    (`+X` right, `+Y` up, `-Z` forward) whose `float()` conversion and
    `.depth_m` attribute return the positive median optical depth `d = -z_cam`.
    """

    @property
    def depth_m(self) -> float:
        return abs(float(self[2]))

    def __float__(self) -> float:
        return abs(float(self[2]))


def unproject_depth_roi_median(
    depth_map,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    u_min: int,
    u_max: int,
    v_min: int,
    v_max: int,
):
    """Pure-Python / NumPy helper for unit tests that unprojects valid depth
    pixels inside `[u_min..u_max, v_min..v_max]` into local USD camera coordinates
    `(x_cam = (u - cx) * d / fx, y_cam = -(v - cy) * d / fy, z_cam = -d)` and
    returns the median `(x_cam, y_cam, z_cam)` as an `_UnprojectedROIResult`
    (which unpacks as a 3-tuple `(x, y, z)` and converts to `float` as `d = -z_cam`).
    """
    xs = []
    ys = []
    zs = []
    rows = depth_map.tolist() if hasattr(depth_map, 'tolist') else list(depth_map)
    h = len(rows)
    w = len(rows[0]) if h > 0 else 0
    v0 = max(0, int(v_min))
    v1 = min(h, int(v_max) + 1 if int(v_max) < h else h)
    u0 = max(0, int(u_min))
    u1 = min(w, int(u_max) + 1 if int(u_max) < w else w)

    for v in range(v0, v1):
        row = rows[v]
        for u in range(u0, u1):
            d = float(row[u])
            if math.isfinite(d) and CLIP_NEAR_M < d < (CLIP_FAR_M - 0.5):
                xs.append((float(u) - float(cx)) * d / float(fx))
                ys.append(-(float(v) - float(cy)) * d / float(fy))
                zs.append(-d)

    if not zs:
        return _UnprojectedROIResult((float('nan'), float('nan'), float('nan')))

    def _median(vals):
        s = sorted(vals)
        n = len(s)
        mid = n // 2
        return float(s[mid]) if (n % 2 == 1) else 0.5 * (float(s[mid - 1]) + float(s[mid]))

    return _UnprojectedROIResult((_median(xs), _median(ys), _median(zs)))


# Default intrinsics for 640x360 (updated when CLI args are parsed)
res_w, res_h = 640, 360
FX = res_w * (FOCAL_LENGTH_MM / H_APERTURE_MM)
FY = res_h * (FOCAL_LENGTH_MM / V_APERTURE_MM)
CX = res_w * 0.5
CY = res_h * 0.5

if np is not None:
    _u_grid, _v_grid = np.meshgrid(
        np.arange(res_w, dtype=np.float32),
        np.arange(res_h, dtype=np.float32),
    )
else:
    _u_grid, _v_grid = None, None


def depth_to_world_points(depth_img, cam_tf, fx=None, fy=None, cx=None, cy=None):
    """Convert a `distance_to_image_plane` (H, W) array into `(N, 3)` world ENU
    coordinates `(X_east, Y_north, Z_up)` using the full 4x4 camera-to-world
    transform `cam_tf`.

    In the USD camera local frame (`+X` right, `+Y` up, `-Z` optical forward):
      - `x_cam = (u - CX) * d / FX`
      - `y_cam = -(v - CY) * d / FY`
      - `z_cam = -d`
    Each local camera point `[x_cam, y_cam, z_cam, 1.0]` is transformed into
    world coordinates by multiplying with the 4x4 USD local-to-world matrix
    `cam_tf` (`pts_world = pts_cam @ R + t`).
    """
    if isinstance(depth_img, dict):
        depth_img = depth_img.get('data')
    if hasattr(depth_img, 'numpy'):
        depth_img = depth_img.numpy()
    if depth_img is None or np is None:
        return np.zeros((0, 3), dtype=np.float32) if np is not None else []
    depth = np.asarray(depth_img, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    elif depth.ndim == 3 and depth.shape[0] == 1:
        depth = depth[0]
    if depth.ndim != 2:
        return np.zeros((0, 3), dtype=np.float32)

    valid = np.isfinite(depth) & (depth > CLIP_NEAR_M) & (depth < CLIP_FAR_M - 0.5)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    h_img, w_img = depth.shape
    fx_val = float(fx if fx is not None else FX)
    fy_val = float(fy if fy is not None else FY)
    cx_val = float(cx if cx is not None else CX)
    cy_val = float(cy if cy is not None else CY)

    if _u_grid is not None and _u_grid.shape == (h_img, w_img):
        u = _u_grid[valid]
        v = _v_grid[valid]
    else:
        ug, vg = np.meshgrid(
            np.arange(w_img, dtype=np.float32),
            np.arange(h_img, dtype=np.float32),
        )
        u = ug[valid]
        v = vg[valid]

    d = depth[valid]
    x_cam = (u - cx_val) * d / fx_val
    y_cam = -(v - cy_val) * d / fy_val
    z_cam = -d
    pts_cam = np.stack([x_cam, y_cam, z_cam], axis=-1)

    # Convert cam_tf (4x4 Gf.Matrix4d / array, or 3-element translation fallback)
    tf_arr = np.asarray(cam_tf, dtype=np.float32)
    if tf_arr.shape == (4, 4):
        rot = tf_arr[:3, :3]
        trans = tf_arr[3, :3]
        return pts_cam @ rot + trans

    # Fallback if a 3-element world position was passed: apply level North-facing
    # USD camera basis (+X_cam -> +X_world, +Y_cam -> +Z_world, -Z_cam -> +Y_world)
    trans = tf_arr.reshape(-1)[:3]
    rot_level = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=np.float32,
    )
    return pts_cam @ rot_level + trans


def evaluate_obstacle_depths(pts_world, cam_world_pos=None):
    """Measure median obstacle distance for the 3m, 5m, and 10m obstacles.

    For each obstacle, filters 3D points falling inside the obstacle's known
    `(X, Y, Z)` bounding region (excluding ground returns with `Z < 0.25 m`),
    computes the median front-surface distance and equivalent obstacle-center
    distance along world `+Y`, and checks relative error against tolerance.
    """
    results = {}
    if pts_world is None or pts_world.shape[0] == 0:
        return results

    # Exclude ground plane points
    non_ground = pts_world[pts_world[:, 2] > 0.25]
    if non_ground.shape[0] == 0:
        return results

    for spec in OBSTACLE_SPECS:
        xmin, xmax = spec['roi_x']
        ymin, ymax = spec['roi_y']
        mask = (
            (non_ground[:, 0] >= xmin)
            & (non_ground[:, 0] <= xmax)
            & (non_ground[:, 1] >= ymin)
            & (non_ground[:, 1] <= ymax)
        )
        cluster = non_ground[mask]
        if cluster.shape[0] == 0:
            continue

        # Front face of the obstacle is at center_y - size_y / 2.
        # Adding half-thickness size_y / 2 converts front-surface Y to center Y
        # so it can be compared directly against the nominal 3.0m / 5.0m / 10.0m
        # obstacle distance.
        half_thickness = spec['size_xyz'][1] * 0.5
        measured_center_y = float(np.median(cluster[:, 1]) + half_thickness)
        gt_dist = float(spec['nominal_dist_m'])
        rel_err_pct = abs(measured_center_y - gt_dist) / gt_dist * 100.0
        passed = rel_err_pct <= spec['tolerance_pct']

        results[spec['name']] = {
            'nominal_m': gt_dist,
            'measured_m': measured_center_y,
            'rel_err_pct': rel_err_pct,
            'tolerance_pct': spec['tolerance_pct'],
            'num_points': int(cluster.shape[0]),
            'passed': passed,
        }
    return results


def _setup_zed_stream_action_graph():
    """Create OmniGraph Action Graph for Stereolabs `sl.sensor.camera` local IPC
    streaming on ports 30000 (`zed1`) and 30002 (`zed2`).
    """
    import omni.graph.core as og
    from pxr import Sdf

    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': '/World/ZedStreamActionGraph', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('Zed1Streamer', 'sl.sensor.camera.OgnZEDSimCameraNode'),
                ('Zed2Streamer', 'sl.sensor.camera.OgnZEDSimCameraNode'),
            ],
            keys.CONNECT: [
                ('OnPlaybackTick.outputs:tick', 'Zed1Streamer.inputs:execIn'),
                ('OnPlaybackTick.outputs:tick', 'Zed2Streamer.inputs:execIn'),
            ],
            keys.SET_VALUES: [
                ('Zed1Streamer.inputs:streamingPort', 30000),
                ('Zed1Streamer.inputs:ipc', True),
                ('Zed2Streamer.inputs:streamingPort', 30002),
                ('Zed2Streamer.inputs:ipc', True),
            ],
        },
    )
    for attr_path, cam_path in [
        ('/World/ZedStreamActionGraph/Zed1Streamer.inputs:cameraPrim', ZED1_CAMERA_PATH),
        ('/World/ZedStreamActionGraph/Zed2Streamer.inputs:cameraPrim', ZED2_CAMERA_PATH),
    ]:
        try:
            og.Controller.attribute(attr_path).set([Sdf.Path(cam_path)])
        except Exception:
            try:
                og.Controller.attribute(attr_path).set(cam_path)
            except Exception:
                pass


def _setup_ros2_zed_graph(rp_zed1, rp_zed2, publish_camera_pcl: bool = True):
    """Create OmniGraph ROS2 bridge pipeline using `ROS2PublishClock`,
    `ROS2PublishTransformTree`, and `ROS2CameraHelper(type='depth_pcl')`
    with optical frame IDs `ZED1_OPTICAL_FRAME_ID` (`zed1_camera_optical_frame`)
    and `ZED2_OPTICAL_FRAME_ID` (`zed2_camera_optical_frame`).
    """
    import omni.graph.core as og
    from pxr import Sdf

    rp1_path = rp_zed1.path if hasattr(rp_zed1, 'path') else str(rp_zed1)
    rp2_path = rp_zed2.path if hasattr(rp_zed2, 'path') else str(rp_zed2)
    keys = og.Controller.Keys
    create_nodes = [
        ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
        ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
        ('PublishTF', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
    ]
    connect_edges = [
        ('OnPlaybackTick.outputs:tick', 'PublishClock.inputs:execIn'),
        ('OnPlaybackTick.outputs:tick', 'PublishTF.inputs:execIn'),
        ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
        ('ReadSimTime.outputs:simulationTime', 'PublishTF.inputs:timeStamp'),
    ]
    set_values = [
        ('PublishClock.inputs:topicName', '/clock'),
        ('PublishTF.inputs:topicName', '/tf_static'),
        ('PublishTF.inputs:staticPublisher', True),
    ]
    if publish_camera_pcl:
        create_nodes.extend([
            ('Zed1CameraHelper', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ('Zed2CameraHelper', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
        ])
        connect_edges.extend([
            ('OnPlaybackTick.outputs:tick', 'Zed1CameraHelper.inputs:execIn'),
            ('OnPlaybackTick.outputs:tick', 'Zed2CameraHelper.inputs:execIn'),
        ])
        set_values.extend([
            ('Zed1CameraHelper.inputs:renderProductPath', rp1_path),
            ('Zed1CameraHelper.inputs:frameId', ZED1_OPTICAL_FRAME_ID),
            ('Zed1CameraHelper.inputs:topicName', '/zed1/zed_node/point_cloud/cloud_registered'),
            ('Zed1CameraHelper.inputs:type', 'depth_pcl'),
            ('Zed2CameraHelper.inputs:renderProductPath', rp2_path),
            ('Zed2CameraHelper.inputs:frameId', ZED2_OPTICAL_FRAME_ID),
            ('Zed2CameraHelper.inputs:topicName', '/zed2/zed_node/point_cloud/cloud_registered'),
            ('Zed2CameraHelper.inputs:type', 'depth_pcl'),
        ])

    og.Controller.edit(
        {'graph_path': '/World/ZedRos2ActionGraph', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CONNECT: connect_edges,
            keys.SET_VALUES: set_values,
        },
    )
    for attr_path, paths in [
        ('/World/ZedRos2ActionGraph/PublishTF.inputs:parentPrim', [CHASSIS_PATH]),
        ('/World/ZedRos2ActionGraph/PublishTF.inputs:targetPrims', [ZED1_MOUNT_PATH, ZED2_MOUNT_PATH]),
    ]:
        try:
            og.Controller.attribute(attr_path).set([Sdf.Path(p) for p in paths])
        except Exception:
            try:
                og.Controller.attribute(attr_path).set(paths[0] if len(paths) == 1 else paths)
            except Exception:
                pass


if __name__ == '__main__':
    # ============================================================
    # SECTION 2: Parse CLI arguments and launch Isaac Sim
    # ============================================================
    parser = argparse.ArgumentParser(description='Stereolabs ZED 2i dual camera tutorial')
    parser.add_argument('--hover', type=float, default=1.5,
                        help='Target hover altitude in meters (default: 1.5)')
    parser.add_argument('--no-hover', action='store_true',
                        help='Disable auto-hover (drone will fall under gravity)')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Simulation duration in seconds (default: 10.0)')
    parser.add_argument('--zed-backend', type=str, default='ros2_depth',
                        choices=['ros2_depth', 'zed_stream'],
                        help='Simulation backend: ros2_depth (native Isaac Sim 5.1/6.0 RGB-D + '
                             'PointCloud2) or zed_stream (Stereolabs sl.sensor.camera IPC streamer)')
    parser.add_argument('--ros2', action='store_true',
                        help='Publish /zed1 & /zed2 PointCloud2, /tf_static, and /clock via ROS2 bridge')
    parser.add_argument('--resolution', type=str, default='640x360',
                        help='Camera resolution WxH (default: 640x360; use 1280x720 for HD720)')
    add_app_launcher_args(parser)
    args = parser.parse_args()

    # Camera rendering is required even in headless mode
    args.enable_cameras = True
    app_launcher = launch_app(args)
    simulation_app = app_launcher.app

    import omni.kit.app  # noqa: E402
    import omni.replicator.core as rep  # noqa: E402
    import omni.timeline  # noqa: E402
    import omni.usd  # noqa: E402
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt  # noqa: E402

    app = omni.kit.app.get_app()

    # Parse resolution string (e.g. "640x360" -> (640, 360))
    try:
        res_w, res_h = [int(x) for x in args.resolution.lower().split('x')]
    except ValueError:
        res_w, res_h = 640, 360

    FX = res_w * (FOCAL_LENGTH_MM / H_APERTURE_MM)
    FY = res_h * (FOCAL_LENGTH_MM / V_APERTURE_MM)
    CX = res_w * 0.5
    CY = res_h * 0.5
    _u_grid, _v_grid = np.meshgrid(
        np.arange(res_w, dtype=np.float32),
        np.arange(res_h, dtype=np.float32),
    )

    # Open the pre-built vehicle scene USD
    print('[ZED-TUTORIAL] Opening vehicle_scene.usd...', flush=True)
    usd_path = os.path.expanduser('~/go_aero/vehicle_scene.usd')
    omni.usd.get_context().open_stage(usd_path)

    for i in range(300):
        app.update()
        loading = omni.usd.get_context().get_stage_loading_status()
        if loading[2] == 0 and i > 30:
            break

    stage = omni.usd.get_context().get_stage()
    print('[ZED-TUTORIAL] Scene loaded.', flush=True)

    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        cam = ViewportCameraState('/OmniverseKit_Persp')
        cam.set_position_world(Gf.Vec3d(4.5, -3.5, 3.0), True)
        cam.set_target_world(Gf.Vec3d(0.5, 4.0, 1.5), True)
    except Exception:
        pass  # headless

    # ============================================================
    # SECTION 3: Ensure ZED mounts and textured 3D obstacles exist
    # ============================================================
    def _create_checkerboard_obstacle(stage, spec):
        cx, cy, cz = spec['center_xyz']
        sx, sy, sz = spec['size_xyz']
        hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
        window_uv = spec['window_uv']
        color_a = spec['color_a']
        color_b = spec['color_b']

        ensure_xform_pose(stage, spec['prim_path'], (cx, cy, cz))
        m = UsdGeom.Mesh.Define(stage, f"{spec['prim_path']}/mesh")

        verts = []
        faces = []
        face_colors = []

        def add_quad_grid(p00, p10, p11, p01, nu, nv, allow_window=False):
            for iu in range(nu):
                u0, u1 = iu / float(nu), (iu + 1) / float(nu)
                uc = 0.5 * (u0 + u1)
                for iv in range(nv):
                    v0, v1 = iv / float(nv), (iv + 1) / float(nv)
                    vc = 0.5 * (v0 + v1)
                    if allow_window and window_uv is not None:
                        umin, umax, vmin, vmax = window_uv
                        if umin <= uc <= umax and vmin <= vc <= vmax:
                            continue

                    def bilerp(u, v):
                        return (
                            (1 - u) * (1 - v) * p00
                            + u * (1 - v) * p10
                            + u * v * p11
                            + (1 - u) * v * p01
                        )

                    base = len(verts)
                    for pt in (bilerp(u0, v0), bilerp(u1, v0), bilerp(u1, v1), bilerp(u0, v1)):
                        verts.append(Gf.Vec3f(float(pt[0]), float(pt[1]), float(pt[2])))
                    faces.extend([base, base + 1, base + 2, base, base + 2, base + 3])
                    col = color_a if ((iu + iv) % 2 == 0) else color_b
                    c_vec = Gf.Vec3f(float(col[0]), float(col[1]), float(col[2]))
                    face_colors.extend([c_vec, c_vec])

        c000 = np.array([-hx, -hy, -hz])
        c100 = np.array([hx, -hy, -hz])
        c110 = np.array([hx, hy, -hz])
        c010 = np.array([-hx, hy, -hz])
        c001 = np.array([-hx, -hy, hz])
        c101 = np.array([hx, -hy, hz])
        c111 = np.array([hx, hy, hz])
        c011 = np.array([-hx, hy, hz])

        add_quad_grid(c000, c100, c101, c001, 12, 18, allow_window=True)
        add_quad_grid(c110, c010, c011, c111, 12, 18, allow_window=True)
        add_quad_grid(c010, c000, c001, c011, 6, 18)
        add_quad_grid(c100, c110, c111, c101, 6, 18)
        add_quad_grid(c001, c101, c111, c011, 12, 6)
        add_quad_grid(c010, c110, c100, c000, 12, 6)

        m.GetPointsAttr().Set(Vt.Vec3fArray(verts))
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces))
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * (len(faces) // 3)))
        m.GetSubdivisionSchemeAttr().Set('none')
        m.GetDoubleSidedAttr().Set(True)
        m.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)]))

        primvars_api = UsdGeom.PrimvarsAPI(m)
        c_pv = primvars_api.CreatePrimvar(
            'displayColor', Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform
        )
        c_pv.Set(Vt.Vec3fArray(face_colors))
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())

    def ensure_mounts_and_obstacles(stage):
        """Ensure zed1_mount, zed2_mount, and the 3 calibration obstacles exist."""
        for mount_path, mount_pos in [
            (ZED1_MOUNT_PATH, ZED1_MOUNT_POS_MESH),
            (ZED2_MOUNT_PATH, ZED2_MOUNT_POS_MESH),
        ]:
            if not stage.GetPrimAtPath(mount_path):
                print(f'[ZED-TUTORIAL] Creating missing mount: {mount_path}', flush=True)
                ensure_xform_pose(stage, mount_path, mount_pos)

        if not stage.GetPrimAtPath('/World/Obstacles'):
            UsdGeom.Xform.Define(stage, '/World/Obstacles')

        for spec in OBSTACLE_SPECS:
            if not stage.GetPrimAtPath(spec['prim_path']):
                _create_checkerboard_obstacle(stage, spec)
                print(
                    f"[ZED-TUTORIAL] Created obstacle {spec['name']} at "
                    f"{spec['center_xyz']} (size {spec['size_xyz']}m)",
                    flush=True,
                )

    ensure_mounts_and_obstacles(stage)

    # ============================================================
    # SECTION 4: Create ZED 1 and ZED 2 Camera Prims
    # ============================================================
    def create_zed_camera_prim(stage, cam_path):
        """Create a UsdGeom.Camera with ZED 2i intrinsics."""
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.GetFocalLengthAttr().Set(FOCAL_LENGTH_MM)
        cam.GetHorizontalApertureAttr().Set(H_APERTURE_MM)
        cam.GetVerticalApertureAttr().Set(V_APERTURE_MM)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(CLIP_NEAR_M, CLIP_FAR_M))
        return cam

    create_zed_camera_prim(stage, ZED1_CAMERA_PATH)
    create_zed_camera_prim(stage, ZED2_CAMERA_PATH)
    print(f'[ZED-TUTORIAL] Created zed1 camera at: {ZED1_CAMERA_PATH}', flush=True)
    print(f'[ZED-TUTORIAL] Created zed2 camera at: {ZED2_CAMERA_PATH}', flush=True)

    # ============================================================
    # SECTION 5: Configure the ZED Backend (`ros2_depth` vs `zed_stream`)
    # ============================================================
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    active_backend = args.zed_backend

    if active_backend == 'zed_stream':
        print('[ZED-TUTORIAL] Attempting to enable Stereolabs sl.sensor.camera extension...',
              flush=True)
        enabled_sl = enable_required_extensions(ext_manager, ['sl.sensor.camera'])
        for _ in range(10):
            app.update()
        if not enabled_sl:
            print(
                '[ZED-TUTORIAL] NOTE: Stereolabs sl.sensor.camera extension is not installed '
                'in this Isaac Sim environment (Stereolabs binary plugin targets 5.1; '
                'Isaac Sim 6.0 uses native ros2_depth). Falling back to ros2_depth backend.',
                flush=True,
            )
            active_backend = 'ros2_depth'
        else:
            try:
                _setup_zed_stream_action_graph()
                print(
                    '[ZED-TUTORIAL] sl.sensor.camera Action Graph streaming on IPC ports '
                    '30000 (zed1) and 30002 (zed2).',
                    flush=True,
                )
            except Exception as exc:
                print(
                    f'[ZED-TUTORIAL] Could not build sl.sensor.camera Action Graph ({exc}); '
                    'falling back to ros2_depth.',
                    flush=True,
                )
                active_backend = 'ros2_depth'

    # Create Replicator render products and depth annotators for both cameras so
    # we can verify 3D point cloud accuracy in Python regardless of backend.
    print(
        f'[ZED-TUTORIAL] Attaching render products ({res_w}x{res_h}) and depth annotators...',
        flush=True,
    )
    rp_zed1 = rep.create.render_product(ZED1_CAMERA_PATH, resolution=(res_w, res_h))
    rp_zed2 = rep.create.render_product(ZED2_CAMERA_PATH, resolution=(res_w, res_h))

    depth_annot_zed1 = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
    depth_annot_zed2 = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
    depth_annot_zed1.attach([rp_zed1])
    depth_annot_zed2.attach([rp_zed2])

    # Optional ROS2 PointCloud2 (`ROS2CameraHelper(type='depth_pcl')`), `/tf_static`, and `/clock`
    if args.ros2:
        print('[ZED-TUTORIAL] Enabling ROS2 Bridge for PointCloud2 / TF / Clock publishing...',
              flush=True)
        enable_required_extensions(ext_manager, ['isaacsim.ros2.bridge', 'isaacsim.core.nodes'])
        for _ in range(10):
            app.update()

        publish_pcl = (active_backend != 'zed_stream')
        try:
            _setup_ros2_zed_graph(rp_zed1, rp_zed2, publish_camera_pcl=publish_pcl)
            print(
                f'[ZED-TUTORIAL] Configured OmniGraph ROS2CameraHelper(type="depth_pcl"), '
                f'ROS2PublishTransformTree (/tf_static), and ROS2PublishClock (/clock) '
                f'for frames {ZED1_OPTICAL_FRAME_ID} and {ZED2_OPTICAL_FRAME_ID} '
                f'(mounts {ZED1_FRAME_ID}, {ZED2_FRAME_ID}).',
                flush=True,
            )
        except Exception as exc:
            print(
                f'[ZED-TUTORIAL] OmniGraph ROS2 setup fallback to Replicator writers ({exc})',
                flush=True,
            )
            if publish_pcl:
                for cam_id, rp, topic_name, frame_id in [
                    ('zed1', rp_zed1, '/zed1/zed_node/point_cloud/cloud_registered', ZED1_OPTICAL_FRAME_ID),
                    ('zed2', rp_zed2, '/zed2/zed_node/point_cloud/cloud_registered', ZED2_OPTICAL_FRAME_ID),
                ]:
                    try:
                        writer = rep.writers.get('ROS2PublishPointCloud')
                        writer.initialize(topicName=topic_name, frameId=frame_id)
                        writer.attach([rp])
                    except Exception:
                        pass

    # ============================================================
    # SECTION 6: Hover controller and main simulation loop
    # ============================================================
    PROP_DIRS = {'prop_fr': 1.0, 'prop_rl': 1.0, 'prop_fl': -1.0, 'prop_rr': -1.0}
    prop_drives = {}
    for name in PROP_DIRS:
        joint_prim = stage.GetPrimAtPath(f'{VEHICLE_PATH}/joints/{name}_joint')
        if joint_prim:
            drive = UsdPhysics.DriveAPI.Get(joint_prim, 'angular')
            if drive:
                prop_drives[name] = drive

    DRONE_MASS = 5.0
    GRAVITY = 9.81
    HOVER_FORCE = DRONE_MASS * GRAVITY
    KP = 15.0
    KD = 8.0
    KI = 2.0

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

    print('[ZED-TUTORIAL] Starting simulation...', flush=True)
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

    # Warm up physics while actively holding hover thrust on /World/Vehicle/chassis
    for _ in range(60):
        _apply_hover_step()
        app.update()

    print(f'[ZED-TUTORIAL] Backend: {active_backend}', flush=True)
    print(f'[ZED-TUTORIAL] Hover altitude: {args.hover}m | Duration: {args.duration}s', flush=True)
    print('[ZED-TUTORIAL] Running... (Ctrl+C to stop early)\n', flush=True)

    step = 0

    SIM_RATE = 60
    ZED_READ_INTERVAL = 6   # ~10 Hz at 60 Hz step rate
    STATUS_INTERVAL = SIM_RATE
    max_steps = int(args.duration * SIM_RATE)

    zed_reads = {'zed1': 0, 'zed2': 0}
    latest_eval = {'zed1': {}, 'zed2': {}}

    try:
        while simulation_app.is_running() and step < max_steps:
            # ---- PID Hover Controller on CHASSIS_PATH ----
            _apply_hover_step()
            app.update()
            step += 1

            # ---- Read ZED 1 & ZED 2 Depth + Point Clouds ----
            if step % ZED_READ_INTERVAL == 0:
                for cam_label, cam_path, annot in [
                    ('zed1', ZED1_CAMERA_PATH, depth_annot_zed1),
                    ('zed2', ZED2_CAMERA_PATH, depth_annot_zed2),
                ]:
                    try:
                        cam_prim = stage.GetPrimAtPath(cam_path)
                        cam_tf = UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(0)
                        cam_pos = cam_tf.ExtractTranslation()

                        depth_data = annot.get_data()
                        pts_w = depth_to_world_points(depth_data, cam_tf)
                        if pts_w.shape[0] > 0:
                            zed_reads[cam_label] += 1
                            eval_res = evaluate_obstacle_depths(pts_w, cam_pos)
                            if eval_res:
                                latest_eval[cam_label].update(eval_res)

                            if step % STATUS_INTERVAL == 0:
                                summary_parts = [
                                    f"{k}={v['measured_m']:.2f}m (err {v['rel_err_pct']:.1f}%)"
                                    for k, v in eval_res.items()
                                ]
                                print(
                                    f"[{cam_label.upper()}] step={step:5d}  "
                                    f"points={pts_w.shape[0]:6d}  "
                                    + ('  '.join(summary_parts) if summary_parts else 'scanning...'),
                                    flush=True,
                                )
                    except Exception as exc:
                        if step % STATUS_INTERVAL == 0:
                            print(f'[{cam_label.upper()}] read error: {exc}', flush=True)

            if step % STATUS_INTERVAL == 0:
                print('', flush=True)

    except KeyboardInterrupt:
        print('\n[ZED-TUTORIAL] Interrupted by user.', flush=True)

    # ============================================================
    # SECTION 7: Final Depth Accuracy Summary Table
    # ============================================================
    print('=' * 72, flush=True)
    print('  Stereolabs ZED 2i Dual Camera Tutorial — Verification Summary', flush=True)
    print('=' * 72, flush=True)
    print(f'  Total physics steps:  {step}', flush=True)
    print(f'  Active ZED backend:   {active_backend}', flush=True)
    print(f'  ZED 1 frames read:    {zed_reads["zed1"]} ({ZED1_CAMERA_PATH})', flush=True)
    print(f'  ZED 2 frames read:    {zed_reads["zed2"]} ({ZED2_CAMERA_PATH})', flush=True)
    print('-' * 72, flush=True)
    print('  Camera | Obstacle     | Nominal | Measured | Rel Error | Tol    | Status', flush=True)
    print('-' * 72, flush=True)

    for cam_label in ('zed1', 'zed2'):
        for spec in OBSTACLE_SPECS:
            obs_name = spec['name']
            entry = latest_eval[cam_label].get(obs_name)
            if entry is not None:
                status_str = 'PASS' if entry['passed'] else 'FAIL'
                print(
                    f"  {cam_label:6s} | {obs_name:12s} | "
                    f"{entry['nominal_m']:6.2f}m | {entry['measured_m']:7.2f}m | "
                    f"{entry['rel_err_pct']:8.2f}% | ±{entry['tolerance_pct']:4.1f}% | {status_str}",
                    flush=True,
                )
            else:
                print(
                    f"  {cam_label:6s} | {obs_name:12s} | "
                    f"{spec['nominal_dist_m']:6.2f}m |     N/A |       N/A | "
                    f"±{spec['tolerance_pct']:4.1f}% | NO DATA",
                    flush=True,
                )

    print('=' * 72, flush=True)

    timeline.stop()
    simulation_app.close()
    print('[ZED-TUTORIAL] Done.', flush=True)


# ============================================================
# SECTION 8: Exercises for students
# ============================================================
#
# EXERCISE 1: Compare Stereo Baseline vs. Dual-ZED Redundancy
#   In `build_scene.py`, `zed1_mount` and `zed2_mount` are co-located at
#   `(0.0, -0.127, -0.1524)` to match `multi_zed.launch.py`. Try offsetting
#   `zed2_mount` laterally by `+0.15` m (`X = +0.15`) and `zed1_mount` by
#   `-0.15` m (`X = -0.15`) on the chassis, rebuild the scene, and compare
#   which parts of `obstacle_5m` each camera can see around `obstacle_3m`.
#
# EXERCISE 2: Stream to RViz2 and `obstacle_detection`
#   Run with `--ros2`:
#     python add_zed_tutorial.py --headless --ros2 --duration 30
#   (or `python px4_bridge.py --zed --ros2`)
#   In another terminal (with ROS2 Jazzy sourced), echo or visualize:
#     ros2 topic list
#     ros2 topic hz /zed1/zed_node/point_cloud/cloud_registered
#
# EXERCISE 3: Switch between `ros2_depth` and `zed_stream`
#   On an Isaac Sim 5.1 system with the Stereolabs `sl.sensor.camera` plugin
#   installed and ZED SDK 5.x, run:
#     python add_zed_tutorial.py --zed-backend zed_stream
#   Then launch `ros2 launch multi_zed.launch.py` in a second terminal to
#   connect two real `zed_ros2_wrapper` nodes over local IPC ports 30000/30002.
