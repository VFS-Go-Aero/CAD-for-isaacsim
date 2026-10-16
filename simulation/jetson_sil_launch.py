#!/usr/bin/env python3
"""
ROS 2 Software-In-The-Loop (SIL) Launch File for the Go Aero Virtual Jetson.

Replaces the hardware-only portions of `launch_all.launch.py` and
`multi_zed.launch.py` from `VFS-Go-Aero/visual-obstacle-detection`
(`/dev/ttyACM0:115200` serial port and physical GMSL2 ZED X serial numbers
`44659546` / `42203370`) with simulation equivalents while running the
**exact, unmodified** `visual_obstacle_detection` nodes (`point_cloud`,
`obstacle_detection`, `obstacle_to_mavlink`) and identical static camera TFs.

Supported camera input modes:
  1. Direct Isaac Sim ROS 2 PointCloud2 topics (default: both
     `use_synthetic_zed:=false` and `use_zed_sim_stream:=false`):
       Isaac Sim publishes directly on:
         - `/zed1/zed_node/point_cloud/cloud_registered`
         - `/zed2/zed_node/point_cloud/cloud_registered`
  2. Standalone Synthetic ZED Publisher (`use_synthetic_zed:=true`):
       Launches `synthetic_zed_publisher.py` so the full Jetson obstacle
       pipeline can be tested on macOS/Linux without Isaac Sim or a GPU.
       When `use_sim_time:=true`, passes `--publish-clock` so `/clock` is
       published automatically in standalone synthetic mode.
  3. ZED IPC Simulation Streamer (`use_zed_sim_stream:=true`):
       Launches two `zed_wrapper` instances with `sim_mode:=true`,
       `sim_address:=127.0.0.1`, and `sim_port:=30000` / `30002`.

Supported autopilot links (`fcu_url`):
  - ArduPilot SITL (default): `tcp://127.0.0.1:5762` (or `tcp://host.docker.internal:5762`)
  - PX4 SITL:                 `udp://:14540@127.0.0.1:14580`
"""

from __future__ import annotations

import glob
import os
from pathlib import Path
from typing import Any


# =============================================================================
# Pure-Python Launch Configuration Metadata & Plan Builder (Unit-Testable)
# =============================================================================

DEFAULT_LAUNCH_ARGS: dict[str, tuple[str, str]] = {
    "use_sim_time": (
        "true",
        "Use simulation clock (/clock) across all ROS 2 nodes",
    ),
    "fcu_url": (
        "tcp://127.0.0.1:5762",
        "MAVROS FCU connection URL (tcp://127.0.0.1:5762 for ArduPilot SITL, "
        "udp://:14540@127.0.0.1:14580 for PX4 SITL)",
    ),
    "use_synthetic_zed": (
        "false",
        "Launch synthetic_zed_publisher.py to generate test point clouds "
        "without Isaac Sim or a ZED camera",
    ),
    "use_zed_sim_stream": (
        "false",
        "Launch zed_wrapper in sim_mode (ports 30000 and 30002) for Isaac Sim ZED streamer",
    ),
    "latency_ms": (
        "0.0",
        "Artificial stereo-to-MAVLink compute delay in ms (e.g. 80.0 to model Jetson Orin NX)",
    ),
    "enable_px4_2d_bridge": (
        "false",
        "Run latency_relay.py even when latency_ms is 0.0 so PX4 72-sector 2D "
        "OBSTACLE_DISTANCE (/mavros/obstacle/send) is published",
    ),
    "px4_mavlink_udp": (
        "",
        "Optional host:port (e.g. 127.0.0.1:14580) for latency_relay.py to send "
        "MAVLink OBSTACLE_DISTANCE directly to PX4 via pymavlink",
    ),
    "enable_mavros": (
        "true",
        "Launch MAVROS (apm.launch) connected to fcu_url",
    ),
    "verbose": (
        "true",
        "Enable verbose DEBUG logging in visual_obstacle_detection",
    ),
    "obstacle_dist": (
        "4.0",
        "Synthetic obstacle distance in meters ahead of base_link (when use_synthetic_zed:=true)",
    ),
    "sim_address": (
        "127.0.0.1",
        "IP address of the Isaac Sim ZED streamer host (when use_zed_sim_stream:=true)",
    ),
    "exclude_boxes_xml": (
        "",
        "Optional path to excluded_boxes.xml for visual_obstacle_detection point_cloud node",
    ),
}

# Exact static transforms from `VFS-Go-Aero/visual-obstacle-detection`
# (`launch_files/launch/multi_zed.launch.py`).
STATIC_TRANSFORMS: list[dict[str, Any]] = [
    {
        "name": "base_to_zed1_tf",
        "parent_frame": "base_link",
        "child_frame": "zed1_camera_link",
        "xyz": ("0.1524", "0.0", "-0.127"),
        "rpy": ("0.0", "0.0", "0.0"),
        "arguments": [
            "0.1524", "0.0", "-0.127",
            "0.0", "0.0", "0.0",
            "base_link",
            "zed1_camera_link",
        ],
    },
    {
        "name": "base_to_zed2_tf",
        "parent_frame": "base_link",
        "child_frame": "zed2_camera_link",
        "xyz": ("0.1524", "0.0", "-0.127"),
        "rpy": ("0.0", "0.0", "0.0"),
        "arguments": [
            "0.1524", "0.0", "-0.127",
            "0.0", "0.0", "0.0",
            "base_link",
            "zed2_camera_link",
        ],
    },
]

# Optical frame static transforms (`zedN_camera_link -> zedN_camera_optical_frame`:
# `0 0 0 -1.57079632679 0 -1.57079632679`) so both FLU (`zed1_camera_link`)
# and standard ROS optical (`zed1_camera_optical_frame`, +Z forward, +X right,
# +Y down) depth point clouds transform into `base_link` FLU.
OPTICAL_STATIC_TRANSFORMS: list[dict[str, Any]] = [
    {
        "name": "zed1_to_optical_tf",
        "parent_frame": "zed1_camera_link",
        "child_frame": "zed1_camera_optical_frame",
        "xyz": ("0.0", "0.0", "0.0"),
        "rpy": ("-1.57079632679", "0.0", "-1.57079632679"),
        "arguments": [
            "0.0", "0.0", "0.0",
            "-1.57079632679", "0.0", "-1.57079632679",
            "zed1_camera_link",
            "zed1_camera_optical_frame",
        ],
    },
    {
        "name": "zed2_to_optical_tf",
        "parent_frame": "zed2_camera_link",
        "child_frame": "zed2_camera_optical_frame",
        "xyz": ("0.0", "0.0", "0.0"),
        "rpy": ("-1.57079632679", "0.0", "-1.57079632679"),
        "arguments": [
            "0.0", "0.0", "0.0",
            "-1.57079632679", "0.0", "-1.57079632679",
            "zed2_camera_link",
            "zed2_camera_optical_frame",
        ],
    },
]

ZED_SIM_CAMERAS: list[dict[str, str]] = [
    {
        "camera_name": "zed1",
        "camera_model": "zedx",
        "sim_port": "30000",
    },
    {
        "camera_name": "zed2",
        "camera_model": "zedx",
        "sim_port": "30002",
    },
]


def _parse_bool(val: str | bool) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).strip().lower() in ("true", "1", "yes", "on")


def _find_exclude_boxes_xml(configured_path: str = "") -> str:
    """Locate `excluded_boxes.xml` on disk if available."""
    if configured_path and Path(configured_path).is_file():
        return str(Path(configured_path).resolve())

    candidates: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = Path(get_package_share_directory("visual_obstacle_detection"))
        candidates.append(pkg_share / "excluded_boxes.xml")
    except Exception:
        pass

    candidates.extend(
        [
            Path("/ros2_ws/install/visual_obstacle_detection/share/visual_obstacle_detection/excluded_boxes.xml"),
            Path("/ros2_ws/src/visual-obstacle-detection/visual_obstacle_detection/excluded_boxes.xml"),
            Path("/ros2_ws/src/visual-obstacle-detection/excluded_boxes.xml"),
        ]
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def build_sil_launch_plan(overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    """
    Compute the deterministic SIL launch plan for any combination of launch args.

    This pure-Python helper is used both by `generate_launch_description()` at
    runtime and by unit tests (`tests/test_simulation_suite.py`) on machines
    where ROS 2 is not installed.
    """
    cfg = {k: v[0] for k, v in DEFAULT_LAUNCH_ARGS.items()}
    if overrides:
        for k, v in overrides.items():
            cfg[k] = str(v)

    use_sim_time = _parse_bool(cfg["use_sim_time"])
    use_synthetic_zed = _parse_bool(cfg["use_synthetic_zed"])
    use_zed_sim_stream = _parse_bool(cfg["use_zed_sim_stream"])
    enable_px4_2d_bridge = _parse_bool(cfg["enable_px4_2d_bridge"])
    raw_px4_udp = cfg["px4_mavlink_udp"].strip()
    px4_mavlink_udp = (
        "" if raw_px4_udp.lower() in ("none", "false", "off", "null") else raw_px4_udp
    )
    enable_mavros = _parse_bool(cfg["enable_mavros"])
    verbose = _parse_bool(cfg["verbose"])
    try:
        latency_ms = float(cfg["latency_ms"])
    except (TypeError, ValueError):
        latency_ms = 0.0
    try:
        obstacle_dist = float(cfg["obstacle_dist"])
    except (TypeError, ValueError):
        obstacle_dist = 4.0
    use_latency_relay = (latency_ms > 0.0) or enable_px4_2d_bridge or bool(px4_mavlink_udp)

    exclude_boxes_xml = _find_exclude_boxes_xml(cfg.get("exclude_boxes_xml", ""))
    point_cloud_params: dict[str, Any] = {"use_sim_time": use_sim_time}
    if exclude_boxes_xml:
        point_cloud_params["exclude_boxes_xml"] = exclude_boxes_xml

    obstacle_to_mavlink_remappings: list[tuple[str, str]] = []
    if use_latency_relay:
        obstacle_to_mavlink_remappings.append(
            ("/mavros/obstacle_distance_3d/send", "/mavros/obstacle_distance_3d/send_raw")
        )

    vod_nodes = [
        {
            "package": "visual_obstacle_detection",
            "executable": "point_cloud",
            "name": "point_cloud",
            "parameters": [point_cloud_params],
            "remappings": [],
        },
        {
            "package": "visual_obstacle_detection",
            "executable": "obstacle_detection",
            "name": "obstacle_detection",
            "parameters": [{"use_sim_time": use_sim_time, "verbose": verbose}],
            "remappings": [],
        },
        {
            "package": "visual_obstacle_detection",
            "executable": "obstacle_to_mavlink",
            "name": "obstacle_to_mavlink",
            "parameters": [{"use_sim_time": use_sim_time}],
            "remappings": obstacle_to_mavlink_remappings,
        },
    ]

    return {
        "use_sim_time": use_sim_time,
        "fcu_url": cfg["fcu_url"],
        "use_synthetic_zed": use_synthetic_zed,
        "publish_clock": use_synthetic_zed and use_sim_time,
        "use_zed_sim_stream": use_zed_sim_stream,
        "latency_ms": latency_ms,
        "enable_px4_2d_bridge": enable_px4_2d_bridge,
        "px4_mavlink_udp": px4_mavlink_udp,
        "use_latency_relay": use_latency_relay,
        "enable_mavros": enable_mavros,
        "verbose": verbose,
        "obstacle_dist": obstacle_dist,
        "sim_address": cfg["sim_address"],
        "exclude_boxes_xml": exclude_boxes_xml,
        "static_transforms": list(STATIC_TRANSFORMS),
        "optical_transforms": list(OPTICAL_STATIC_TRANSFORMS),
        "vod_nodes": vod_nodes,
        "zed_sim_cameras": list(ZED_SIM_CAMERAS) if use_zed_sim_stream else [],
    }


# =============================================================================
# ROS 2 Launch Description Builder
# =============================================================================

def _find_launch_file(package_name: str, base_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    package_share = get_package_share_directory(package_name)
    candidates = glob.glob(os.path.join(package_share, "launch", f"{base_name}*"))
    if not candidates:
        raise FileNotFoundError(
            f"Launch file for {package_name} not found: {base_name}"
        )
    return candidates[0]


def _source_for_launch_file(path: str):
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    try:
        from launch.launch_description_sources import AnyLaunchDescriptionSource
    except ImportError:
        AnyLaunchDescriptionSource = None
    try:
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
    except ImportError:
        try:
            from launch.launch_description_sources import XMLLaunchDescriptionSource
        except ImportError:
            XMLLaunchDescriptionSource = None

    if path.endswith(".py"):
        return PythonLaunchDescriptionSource(path)
    if AnyLaunchDescriptionSource is not None:
        return AnyLaunchDescriptionSource(path)
    if XMLLaunchDescriptionSource is not None:
        return XMLLaunchDescriptionSource(path)
    raise ImportError(
        "No compatible launch description source available for non-Python launch files."
    )


def _create_actions(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    overrides = {
        name: context.perform_substitution(
            kwargs["launch_configs"][name]
        )
        for name in DEFAULT_LAUNCH_ARGS
        if name in kwargs.get("launch_configs", {})
    }
    plan = build_sil_launch_plan(overrides)
    actions = []

    # 1. Static TF2 transforms:
    #    - base_link -> zed1_camera_link, zed2_camera_link (FLU)
    #    - zed1_camera_link -> zed1_camera_optical_frame, zed2_camera_link -> zed2_camera_optical_frame
    for tf_spec in plan["static_transforms"] + plan.get("optical_transforms", []):
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=tf_spec["name"],
                output="screen",
                arguments=tf_spec["arguments"],
                parameters=[{"use_sim_time": plan["use_sim_time"]}],
            )
        )

    # 2. Optional synthetic ZED point-cloud publisher (No-GPU / No-Isaac-Sim mode)
    sim_scripts_dir = Path(__file__).resolve().parent
    if plan["use_synthetic_zed"]:
        synth_script = str(sim_scripts_dir / "synthetic_zed_publisher.py")
        synth_cmd = [
            "python3",
            synth_script,
            "--obstacle-dist",
            str(plan["obstacle_dist"]),
            "--include-airframe-points",
            "--include-noise",
        ]
        if plan["use_sim_time"]:
            synth_cmd.append("--publish-clock")
        actions.append(
            ExecuteProcess(
                cmd=synth_cmd,
                name="synthetic_zed_publisher",
                output="screen",
            )
        )

    # 3. Optional ZED wrapper in IPC simulation streaming mode
    if plan["use_zed_sim_stream"]:
        try:
            zed_launch = os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            )
            for cam in plan["zed_sim_cameras"]:
                actions.append(
                    IncludeLaunchDescription(
                        launch_description_source=PythonLaunchDescriptionSource(zed_launch),
                        launch_arguments={
                            "camera_name": cam["camera_name"],
                            "camera_model": cam["camera_model"],
                            "sim_mode": "true",
                            "sim_address": plan["sim_address"],
                            "sim_port": cam["sim_port"],
                            "use_sim_time": "true" if plan["use_sim_time"] else "false",
                            "publish_tf": "false",
                            "publish_map_tf": "false",
                            "pub_frame_rate": "15.0",
                            "log_level": "info",
                        }.items(),
                    )
                )
        except Exception as exc:
            warn_msg = (
                f"[jetson_sil_launch] WARNING: zed_wrapper package not installed ({exc}); "
                "skipping ZED IPC simulation streamer launch. Build container with "
                "BASE_IMAGE=stereolabs/zed:5.1-devel-cuda12.6-ubuntu22.04 or use direct PointCloud2 topics."
            )
            print(warn_msg, flush=True)
            actions.append(LogInfo(msg=warn_msg))

    # 4. Unmodified visual_obstacle_detection pipeline nodes
    for node_spec in plan["vod_nodes"]:
        actions.append(
            Node(
                package=node_spec["package"],
                executable=node_spec["executable"],
                name=node_spec["name"],
                output="screen",
                parameters=node_spec["parameters"],
                remappings=node_spec["remappings"],
            )
        )

    # 5. Optional Orin NX compute latency relay + PX4 2D OBSTACLE_DISTANCE bridge
    if plan["use_latency_relay"]:
        relay_script = str(sim_scripts_dir / "latency_relay.py")
        relay_cmd = [
            "python3",
            relay_script,
            "--delay-ms",
            str(plan["latency_ms"]),
            "--input-topic",
            "/mavros/obstacle_distance_3d/send_raw",
            "--output-topic",
            "/mavros/obstacle_distance_3d/send",
        ]
        if plan["px4_mavlink_udp"]:
            relay_cmd.extend(["--px4-mavlink-udp", plan["px4_mavlink_udp"]])
        if plan["use_sim_time"]:
            relay_cmd.extend(["--ros-args", "-p", "use_sim_time:=true"])
        actions.append(
            ExecuteProcess(
                cmd=relay_cmd,
                name="latency_relay",
                output="screen",
            )
        )

    # 6. Optional MAVROS node (apm.launch)
    if plan["enable_mavros"]:
        mavros_launch = _find_launch_file("mavros", "apm.launch")
        actions.append(
            IncludeLaunchDescription(
                launch_description_source=_source_for_launch_file(mavros_launch),
                launch_arguments={
                    "fcu_url": plan["fcu_url"],
                }.items(),
            )
        )

    return actions


# Backwards-compatible alias
_launch_setup = _create_actions


def generate_launch_description():
    """Entry point invoked by `ros2 launch jetson_sil_launch.py`."""
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.substitutions import LaunchConfiguration

    declared_args = []
    launch_configs = {}
    for name, (default_val, desc) in DEFAULT_LAUNCH_ARGS.items():
        declared_args.append(
            DeclareLaunchArgument(
                name,
                default_value=default_val,
                description=desc,
            )
        )
        launch_configs[name] = LaunchConfiguration(name)

    return LaunchDescription(
        declared_args
        + [
            OpaqueFunction(
                function=_create_actions,
                kwargs={"launch_configs": launch_configs},
            )
        ]
    )
