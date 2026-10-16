#!/usr/bin/env python3
"""
Isaac Sim 5.1 (Kit 107.3 / IsaacLab 2.3.x) & Isaac Sim 6.0 (Kit 110)
compatibility layer and shared scene mount path constants (Issue #6).

This module intentionally avoids top-level `omni.*` and `pxr` imports so it can
be imported before `AppLauncher` or `SimulationApp` boots the Omniverse Kit
runtime.
"""

from __future__ import annotations

import argparse
import os
from typing import Any, Iterable, List, Optional, Sequence, Tuple, Union

# ============================================================
# Shared USD stage prim paths & mount coordinates
# ============================================================
VEHICLE_PATH = '/World/Vehicle'
CHASSIS_PATH = '/World/Vehicle/chassis'
IMU_MOUNT_PATH = '/World/Vehicle/chassis/chassis_IMU'

# Issue #7: LiDAR mount lives under the rigid chassis (NOT under prop_fr,
# which rotates at motor speed). Positioned under the front-right arm in
# the STL mesh frame (+X=right, +Y=up, -Z=forward):
#   X = +0.468 m (front-right arm lateral offset)
#   Y = -0.040 m (4 cm below chassis origin -> world Z = hover_z - 0.04 m,
#                 within ±5 cm of hover altitude on flat ground)
#   Z = -0.515 m (front-right arm longitudinal offset)
LIDAR_MOUNT_PATH = '/World/Vehicle/chassis/lidar_mount'
LIDAR_SENSOR_PATH = '/World/Vehicle/chassis/lidar_mount/lidar_sensor'
LIDAR_MOUNT_POS_MESH: Tuple[float, float, float] = (0.468, -0.04, -0.515)
LIDAR_MOUNT_ROT_X_DEG: float = 180.0

# Issue #3: Stereolabs ZED 2i dual camera mounts under the chassis.
# Matches multi_zed.launch.py static transform from base_link (FLU):
#   FLU (x=+0.1524, y=0.0, z=-0.127) -> STL mesh (+X=right, +Y=up, -Z=fwd):
#   (0.0, -0.127, -0.1524)
ZED1_MOUNT_PATH = '/World/Vehicle/chassis/zed1_mount'
ZED2_MOUNT_PATH = '/World/Vehicle/chassis/zed2_mount'
ZED1_CAMERA_PATH = '/World/Vehicle/chassis/zed1_mount/zed1_camera'
ZED2_CAMERA_PATH = '/World/Vehicle/chassis/zed2_mount/zed2_camera'
ZED1_MOUNT_POS_MESH: Tuple[float, float, float] = (0.0, -0.127, -0.1524)
ZED2_MOUNT_POS_MESH: Tuple[float, float, float] = (0.0, -0.127, -0.1524)

# Shared ZED 2i ROS2 frame IDs and optical parameters
ZED1_FRAME_ID = 'zed1_camera_link'
ZED2_FRAME_ID = 'zed2_camera_link'
ZED1_OPTICAL_FRAME_ID = 'zed1_camera_optical_frame'
ZED2_OPTICAL_FRAME_ID = 'zed2_camera_optical_frame'
ZED_FOCAL_LENGTH_MM = 18.0
ZED_H_APERTURE_MM = 36.0
ZED_V_APERTURE_MM = 20.25
ZED_CLIP_NEAR_M = 0.2
ZED_CLIP_FAR_M = 25.0

# Candidate extension names across Isaac Sim 5.1 (Kit 107.3) and 6.0 (Kit 110)
EXTENSION_CANDIDATES = {
    'isaacsim.sensors.rtx': ('isaacsim.sensors.rtx', 'omni.isaac.sensor'),
    'omni.isaac.sensor': ('isaacsim.sensors.rtx', 'omni.isaac.sensor'),
    'isaacsim.sensors.physics': ('isaacsim.sensors.physics', 'omni.isaac.sensor'),
    'isaacsim.sensors.camera': ('isaacsim.sensors.camera', 'omni.isaac.sensor'),
    'isaacsim.ros2.bridge': ('isaacsim.ros2.bridge', 'omni.isaac.ros2_bridge'),
    'omni.isaac.ros2_bridge': ('isaacsim.ros2.bridge', 'omni.isaac.ros2_bridge'),
    'isaacsim.core.nodes': ('isaacsim.core.nodes', 'omni.isaac.core_nodes'),
    'omni.isaac.core_nodes': ('isaacsim.core.nodes', 'omni.isaac.core_nodes'),
    'sl.sensor.camera': ('sl.sensor.camera', 'sl.sensor.camera.bridge', 'omni.isaac.zed'),
}


def ensure_xform_pose(
    stage: Any,
    prim_path: str,
    translate_xyz: Sequence[float],
    rotate_x_deg: Optional[float] = None,
) -> Any:
    """Define a `UsdGeom.Xform` at `prim_path`, clear any existing xformOpOrder
    to prevent duplicate op errors (`pxr.Tf.ErrorException: An xformOp of type
    translate already exists`), and set its translation and optional X rotation.
    """
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim() if hasattr(xform, 'GetPrim') else None
    xform.ClearXformOpOrder()
    if prim is not None and hasattr(prim, 'RemoveProperty') and hasattr(prim, 'HasAttribute'):
        for prop_name in ('xformOp:translate', 'xformOp:rotateX'):
            try:
                if prim.HasAttribute(prop_name):
                    prim.RemoveProperty(prop_name)
            except Exception:
                pass
    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(translate_xyz[0]), float(translate_xyz[1]), float(translate_xyz[2]))
    )
    if rotate_x_deg is not None:
        xform.AddRotateXOp().Set(float(rotate_x_deg))
    return xform


class _AppHandle:
    """Wrapper around SimulationApp / AppLauncher supporting all access styles:
      - `simulation_app = launch_app(args)` -> `.is_running()`, `.close()`, `.update()`
      - `app_launcher = launch_app(args); simulation_app = app_launcher.app`
      - `app_launcher, simulation_app = launch_app(args)`
    """

    def __init__(self, sim_app: Any, launcher: Any = None) -> None:
        self._sim_app = sim_app
        self.app = sim_app
        self.launcher = launcher if launcher is not None else self

    def is_running(self) -> bool:
        return bool(self._sim_app.is_running())

    def close(self, *args: Any, **kwargs: Any) -> Any:
        return self._sim_app.close(*args, **kwargs)

    def update(self, *args: Any, **kwargs: Any) -> Any:
        return self._sim_app.update(*args, **kwargs)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._sim_app, name)

    def __iter__(self):
        yield self.launcher
        yield self._sim_app


def _add_arg_if_missing(parser: argparse.ArgumentParser, *flags: str, **kwargs: Any) -> None:
    existing = getattr(parser, '_option_string_actions', {})
    if any(flag in existing for flag in flags):
        return
    try:
        parser.add_argument(*flags, **kwargs)
    except argparse.ArgumentError:
        pass


def add_app_launcher_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Register CLI flags via IsaacLab's AppLauncher if available, or add
    standard standalone `isaacsim.SimulationApp` CLI flags as a fallback
    for Isaac Sim 5.1 / 6.0 environments without IsaacLab installed.
    """
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(parser)
        return parser
    except (ImportError, ModuleNotFoundError):
        pass

    _add_arg_if_missing(
        parser,
        '--headless',
        action='store_true',
        default=False,
        help='Force display off at all times.',
    )
    _add_arg_if_missing(
        parser,
        '--device',
        type=str,
        default='cuda:0',
        help='Device to run on (e.g. cuda, cuda:0, cpu).',
    )
    _add_arg_if_missing(
        parser,
        '--enable_cameras',
        action='store_true',
        default=False,
        help='Enable camera and RTX sensor rendering in headless mode.',
    )
    _add_arg_if_missing(
        parser,
        '--livestream',
        type=int,
        default=-1,
        choices=[-1, 0, 1, 2],
        help='Enable livestreaming (-1=off, 0=off, 1=native, 2=WebRTC).',
    )
    _add_arg_if_missing(
        parser,
        '--experience',
        type=str,
        default='',
        help='Config file to load when launching the SimulationApp.',
    )
    return parser


def launch_app(args: Optional[argparse.Namespace] = None) -> _AppHandle:
    """Launch Omniverse Kit via `isaaclab.app.AppLauncher` when available,
    or fall back to `isaacsim.SimulationApp` (Isaac Sim 5.1 / 6.0).
    """
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'yes')

    try:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher(args) if args is not None else AppLauncher()
        return _AppHandle(launcher.app, launcher=launcher)
    except (ImportError, ModuleNotFoundError):
        pass

    headless = bool(getattr(args, 'headless', False)) if args is not None else False
    sim_cfg: dict[str, Any] = {'headless': headless}
    if args is not None:
        experience = getattr(args, 'experience', '')
        if experience:
            sim_cfg['experience'] = experience

        enable_cameras = bool(getattr(args, 'enable_cameras', False))
        if enable_cameras:
            sim_cfg['enable_cameras'] = True

        device = getattr(args, 'device', None)
        if device is not None and str(device):
            sim_cfg['device'] = str(device)
            dev_str = str(device).lower()
            if dev_str.startswith('cuda'):
                parts = dev_str.split(':')
                gpu_id = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
                sim_cfg['physics_gpu'] = gpu_id
                sim_cfg['active_gpu'] = gpu_id
            elif dev_str == 'cpu':
                sim_cfg['physics_gpu'] = -1

        physics_gpu = getattr(args, 'physics_gpu', None)
        if physics_gpu is not None:
            sim_cfg['physics_gpu'] = int(physics_gpu)

        livestream = getattr(args, 'livestream', -1)
        if livestream is not None and int(livestream) >= 0:
            sim_cfg['livestream'] = int(livestream)

    try:
        from isaacsim import SimulationApp
    except (ImportError, ModuleNotFoundError):
        from omni.isaac.kit import SimulationApp  # type: ignore[no-redef]

    sim_app = SimulationApp(sim_cfg)
    return _AppHandle(sim_app, launcher=None)


def enable_Required_extensions(
    ext_manager: Any,
    names: Union[str, Sequence[Union[str, Sequence[str]]], Iterable[Any]],
) -> List[str]:
    """Enable required Omniverse extensions across Isaac Sim 5.1 and 6.0.

    Tries candidate extension names in order (for example,
    `isaacsim.sensors.rtx` -> `omni.isaac.sensor`, and
    `isaacsim.ros2.bridge` -> `omni.isaac.ros2_bridge`) so scripts work
    cleanly across Kit 107.3 (Isaac Sim 5.1) and Kit 110 (Isaac Sim 6.0).
    """
    if ext_manager is None:
        import omni.kit.app
        ext_manager = omni.kit.app.get_app().get_extension_manager()

    if isinstance(names, str):
        groups: List[Tuple[str, ...]] = [
            EXTENSION_CANDIDATES.get(names, (names,))
        ]
    else:
        name_list = list(names)
        groups = []
        seen_candidates = set()
        for item in name_list:
            if isinstance(item, str):
                if item in seen_candidates:
                    continue
                cands = EXTENSION_CANDIDATES.get(item, (item,))
                merged = list(cands)
                for other in name_list:
                    if isinstance(other, str) and other not in merged and other in cands:
                        merged.append(other)
                seen_candidates.update(merged)
                groups.append(tuple(merged))
            elif isinstance(item, (tuple, list)):
                groups.append(tuple(str(x) for x in item))

    enabled: List[str] = []
    for candidates in groups:
        group_enabled = False
        for ext_name in candidates:
            try:
                if hasattr(ext_manager, 'is_extension_enabled') and ext_manager.is_extension_enabled(ext_name):
                    enabled.append(ext_name)
                    group_enabled = True
                    break
                res = ext_manager.set_extension_enabled_immediate(ext_name, True)
                is_on = (
                    bool(res)
                    or (
                        hasattr(ext_manager, 'is_extension_enabled')
                        and bool(ext_manager.is_extension_enabled(ext_name))
                    )
                )
                if is_on:
                    enabled.append(ext_name)
                    group_enabled = True
                    break
            except Exception:
                continue
        if not group_enabled and candidates:
            try:
                ext_manager.set_extension_enabled_immediate(candidates[0], True)
            except Exception:
                pass

    return enabled


# Lowercase alias for convenience
enable_required_extensions = enable_Required_extensions


__all__ = [
    'VEHICLE_PATH',
    'CHASSIS_PATH',
    'IMU_MOUNT_PATH',
    'LIDAR_MOUNT_PATH',
    'LIDAR_SENSOR_PATH',
    'LIDAR_MOUNT_POS_MESH',
    'LIDAR_MOUNT_ROT_X_DEG',
    'ZED1_MOUNT_PATH',
    'ZED2_MOUNT_PATH',
    'ZED1_CAMERA_PATH',
    'ZED2_CAMERA_PATH',
    'ZED1_MOUNT_POS_MESH',
    'ZED2_MOUNT_POS_MESH',
    'ZED1_FRAME_ID',
    'ZED2_FRAME_ID',
    'ZED1_OPTICAL_FRAME_ID',
    'ZED2_OPTICAL_FRAME_ID',
    'ZED_FOCAL_LENGTH_MM',
    'ZED_H_APERTURE_MM',
    'ZED_V_APERTURE_MM',
    'ZED_CLIP_NEAR_M',
    'ZED_CLIP_FAR_M',
    'EXTENSION_CANDIDATES',
    'ensure_xform_pose',
    'add_app_launcher_args',
    'launch_app',
    'enable_Required_extensions',
    'enable_required_extensions',
]
