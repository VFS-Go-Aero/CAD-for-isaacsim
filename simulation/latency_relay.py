#!/usr/bin/env python3
"""
Jetson Orin NX Compute-Latency Relay & ArduPilot/PX4 Obstacle Message Bridge.

Solves two key simulation requirements for Issues #4 and #1:
  1. **Hardware-Representative Latency Relay (Issue #4)**:
       On a cloud GPU (e.g. NVIDIA L40S on `g6e.2xlarge`) or Apple Silicon
       laptop, `visual_obstacle_detection` processes point clouds faster than on
       the physical ZED Box Mini (Jetson Orin NX 16GB). This node subscribes to
       `/mavros/obstacle_distance_3d/send_raw`, buffers `ObstacleDistance3D`
       messages for `--delay-ms` (default `80.0` ms), and republishes them to
       `/mavros/obstacle_distance_3d/send`.

  2. **ArduPilot `OBSTACLE_DISTANCE_3D` vs PX4 72-Sector `OBSTACLE_DISTANCE`
     Bridge (Issue #1, Question 4)**:
       - **ArduPilot** consumes 3D body-frame points via `OBSTACLE_DISTANCE_3D`
         (`MAV_FRAME_BODY_FRD = 12`, `PRX1_TYPE = 2`).
       - **PX4** Collision Prevention (`CP_DIST`) requires the 2D 72-sector
         (`5°` increment) `OBSTACLE_DISTANCE` array (in cm).
       `points_3d_to_px4_obstacle_distance_2d()` bins 3D FRD obstacle points
       into the 72-sector `OBSTACLE_DISTANCE` array and can publish a
       counter-clockwise FLU `sensor_msgs/msg/LaserScan` on `/mavros/obstacle/send`
       (via `frd_sectors_to_flu_laserscan_ranges`) or send `OBSTACLE_DISTANCE`
       directly to PX4 via `pymavlink`.
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import sys
import time
from typing import Any


# MAVLink constants (from common.xml)
MAV_FRAME_BODY_FRD = 12
MAV_DISTANCE_SENSOR_LASER = 0
MAV_DISTANCE_SENSOR_UNKNOWN = 4

PX4_NUM_SECTORS = 72
PX4_SECTOR_INCREMENT_DEG = 5


# =============================================================================
# Pure-Python FIFO Latency Buffer (Unit-Testable)
# =============================================================================

class LatencyBuffer:
    """
    Deterministic FIFO delay queue that holds items for `delay_ms` milliseconds
    before releasing them.
    """

    def __init__(self, delay_ms: float = 80.0, max_queue_size: int = 1024) -> None:
        d_ms = float(delay_ms)
        if not math.isfinite(d_ms) or d_ms < 0.0:
            raise ValueError(f"delay_ms must be finite and non-negative, got {delay_ms}")
        if max_queue_size < 1:
            raise ValueError(f"max_queue_size must be >= 1, got {max_queue_size}")
        self.delay_ms = d_ms
        self.delay_s = d_ms / 1000.0
        self.max_queue_size = int(max_queue_size)
        self._queue: deque[tuple[float, Any]] = deque()

    def __len__(self) -> int:
        return len(self._queue)

    def clear(self) -> None:
        self._queue.clear()

    def push(self, item: Any, now_s: float) -> None:
        """Enqueue `item` received at timestamp `now_s` (seconds)."""
        t_now = float(now_s)
        if not math.isfinite(t_now):
            return
        if len(self._queue) >= self.max_queue_size:
            self._queue.popleft()
        release_time = t_now + self.delay_s
        self._queue.append((release_time, item))

    def pop_ready(self, now_s: float) -> list[Any]:
        """Pop and return all items whose release time is `<= now_s` in FIFO order."""
        t = float(now_s)
        if not math.isfinite(t):
            return []
        ready: list[Any] = []
        while self._queue and self._queue[0][0] <= t + 1e-12:
            _, item = self._queue.popleft()
            ready.append(item)
        return ready


# =============================================================================
# 3D FRD -> PX4 72-Sector 2D `OBSTACLE_DISTANCE` Converter (Issue #1 & #4)
# =============================================================================

def points_3d_to_px4_obstacle_distance_2d(
    points_frd: Any,
    min_distance_cm: int = 20,
    max_distance_cm: int = 500,
    max_z_band_m: float | None = None,
    center_bin_zero: bool = True,
) -> list[int]:
    """
    Bin 3D `MAV_FRAME_BODY_FRD` obstacle points `(x_frd, y_frd, z_frd)` (in
    meters, where `+X=Forward`, `+Y=Right`, `+Z=Down`) into PX4's 72-sector
    (`5°` increment) 2D `OBSTACLE_DISTANCE` array (in centimeters).

    Sector convention (MAVLink `OBSTACLE_DISTANCE` with `frame=MAV_FRAME_BODY_FRD`,
    `increment=5`, `angle_offset=0`):
      - Sector  0 :   0° (Forward, `+X_frd`)
      - Sector 18 :  90° (Right,   `+Y_frd`)
      - Sector 36 : 180° (Behind,  `-X_frd`)
      - Sector 54 : 270° (Left,    `-Y_frd`)
      - Unknown / clear sectors are set to `max_distance_cm + 1` (e.g. `501`).
    """
    min_cm = int(min_distance_cm)
    max_cm = int(max_distance_cm)
    if min_cm < 0 or max_cm <= min_cm:
        raise ValueError(
            f"Invalid distance bounds: min_distance_cm={min_cm}, max_distance_cm={max_cm}"
        )

    unknown_cm = max_cm + 1
    distances = [unknown_cm] * PX4_NUM_SECTORS

    if points_frd is None:
        return distances

    max_range_m = max_cm / 100.0

    for pt in points_frd:
        x_frd = float(pt[0])
        y_frd = float(pt[1])
        z_frd = float(pt[2]) if len(pt) > 2 else 0.0

        if not (math.isfinite(x_frd) and math.isfinite(y_frd) and math.isfinite(z_frd)):
            continue

        if max_z_band_m is not None and abs(z_frd) > float(max_z_band_m):
            continue

        horiz_m = math.hypot(x_frd, y_frd)
        if horiz_m <= 1e-6 or horiz_m > max_range_m + 1e-6:
            continue

        # Clockwise bearing in degrees from +X_frd (Forward) toward +Y_frd (Right)
        angle_deg = math.degrees(math.atan2(y_frd, x_frd)) % 360.0
        if center_bin_zero:
            sector = int(((angle_deg + 2.5) % 360.0) // float(PX4_SECTOR_INCREMENT_DEG)) % PX4_NUM_SECTORS
        else:
            sector = int(angle_deg // float(PX4_SECTOR_INCREMENT_DEG)) % PX4_NUM_SECTORS

        dist_cm = int(round(horiz_m * 100.0))
        dist_cm = max(min_cm, min(max_cm, dist_cm))
        if dist_cm < distances[sector]:
            distances[sector] = dist_cm

    return distances


def frd_sectors_to_flu_laserscan_ranges(
    distances_cm: list[int],
    max_distance_cm: int = 500,
) -> list[float]:
    """
    Convert a 72-sector clockwise (CW) `MAV_FRAME_BODY_FRD` distance array
    (in centimeters) into a 72-element counter-clockwise (CCW) `base_link`
    (FLU) `sensor_msgs/msg/LaserScan.ranges` list (in meters).

    Mapping (`n = len(distances_cm) = 72`):
      - `flu_idx = (n - frd_idx) % n`
      - FRD sector  0 (Forward, 0°)    -> FLU index  0 (0° CCW)
      - FRD sector 54 (Left, 270° CW)  -> FLU index 18 (+90° CCW)
      - FRD sector 36 (Behind, 180°)   -> FLU index 36 (180° CCW)
      - FRD sector 18 (Right, 90° CW)  -> FLU index 54 (+270° CCW)
      - Values `d > max_distance_cm` (unknown / clear) map to `float('inf')`.
    """
    n = len(distances_cm)
    if n == 0:
        return []
    ranges = [float("inf")] * n
    for frd_idx, d_cm in enumerate(distances_cm):
        flu_idx = (n - frd_idx) % n
        ranges[flu_idx] = (
            float("inf") if int(d_cm) > int(max_distance_cm) else float(d_cm) / 100.0
        )
    return ranges


def build_px4_obstacle_distance_mavlink_dict(
    points_frd: Any,
    time_usec: int = 0,
    min_distance_cm: int = 20,
    max_distance_cm: int = 500,
    max_z_band_m: float | None = None,
) -> dict[str, Any]:
    """
    Build a dictionary of fields matching MAVLink `OBSTACLE_DISTANCE` (#330)
    for PX4 Collision Prevention (`CP_DIST`).
    """
    distances = points_3d_to_px4_obstacle_distance_2d(
        points_frd,
        min_distance_cm=min_distance_cm,
        max_distance_cm=max_distance_cm,
        max_z_band_m=max_z_band_m,
    )
    return {
        "time_usec": int(time_usec),
        "sensor_type": MAV_DISTANCE_SENSOR_LASER,
        "distances": distances,
        "increment": PX4_SECTOR_INCREMENT_DEG,
        "min_distance": int(min_distance_cm),
        "max_distance": int(max_distance_cm),
        "increment_f": float(PX4_SECTOR_INCREMENT_DEG),
        "angle_offset": 0.0,
        "frame": MAV_FRAME_BODY_FRD,
    }


def send_px4_obstacle_distance(
    mav_conn: Any,
    points_frd: Any,
    time_usec: int | None = None,
    min_distance_cm: int = 20,
    max_distance_cm: int = 500,
    max_z_band_m: float | None = None,
) -> dict[str, Any]:
    """
    Send a 72-sector `OBSTACLE_DISTANCE` MAVLink message to PX4 via `pymavlink`.
    """
    if time_usec is None:
        time_usec = int(time.time() * 1e6)
    payload = build_px4_obstacle_distance_mavlink_dict(
        points_frd,
        time_usec=time_usec,
        min_distance_cm=min_distance_cm,
        max_distance_cm=max_distance_cm,
        max_z_band_m=max_z_band_m,
    )
    mav_conn.mav.obstacle_distance_send(
        payload["time_usec"],
        payload["sensor_type"],
        payload["distances"],
        payload["increment"],
        payload["min_distance"],
        payload["max_distance"],
        payload["increment_f"],
        payload["angle_offset"],
        payload["frame"],
    )
    return payload


# =============================================================================
# Standalone Self-Test (`--self-test`)
# =============================================================================

def run_self_test(verbose: bool = True) -> dict[str, Any]:
    """Verify `LatencyBuffer`, `points_3d_to_px4_obstacle_distance_2d`, and `frd_sectors_to_flu_laserscan_ranges`."""
    buf = LatencyBuffer(delay_ms=80.0)
    buf.push("msg_1", now_s=1.000)
    buf.push("msg_2", now_s=1.040)
    assert buf.pop_ready(1.050) == []
    assert buf.pop_ready(1.079) == []
    assert buf.pop_ready(1.080) == ["msg_1"]
    assert buf.pop_ready(1.120) == ["msg_2"]
    assert len(buf) == 0

    # Cardinal direction sector checks in FRD:
    #   +X (Forward, 4.0 m) -> sector 0 (400 cm)
    #   +Y (Right,   3.0 m) -> sector 18 (300 cm)
    #   -X (Behind,  2.5 m) -> sector 36 (250 cm)
    #   -Y (Left,    1.5 m) -> sector 54 (150 cm)
    sample_frd = [
        (4.0, 0.0, 0.0),
        (3.5, 0.0, 0.0),   # closer point in sector 0 should win (350 cm)
        (0.0, 3.0, 0.0),
        (-2.5, 0.0, 0.0),
        (0.0, -1.5, 0.0),
    ]
    sectors_2d = points_3d_to_px4_obstacle_distance_2d(
        sample_frd, min_distance_cm=20, max_distance_cm=500
    )
    assert len(sectors_2d) == 72
    assert sectors_2d[0] == 350
    assert sectors_2d[18] == 300
    assert sectors_2d[36] == 250
    assert sectors_2d[54] == 150
    assert sectors_2d[1] == 501  # empty sector = max_distance_cm + 1

    # Verify CW FRD -> CCW FLU LaserScan range conversion:
    flu_ranges = frd_sectors_to_flu_laserscan_ranges(sectors_2d, max_distance_cm=500)
    assert len(flu_ranges) == 72
    assert abs(flu_ranges[0] - 3.5) < 1e-6    # Forward (0 deg)
    assert abs(flu_ranges[18] - 1.5) < 1e-6   # Left (+90 deg CCW = FRD sector 54)
    assert abs(flu_ranges[36] - 2.5) < 1e-6   # Behind (180 deg = FRD sector 36)
    assert abs(flu_ranges[54] - 3.0) < 1e-6   # Right (+270 deg CCW = FRD sector 18)
    assert math.isinf(flu_ranges[1])

    if verbose:
        print("=== latency_relay --self-test PASSED ===")
        print("  LatencyBuffer (80.0 ms)  : FIFO release timing verified")
        print(
            f"  PX4 72-sector 2D binning : "
            f"fwd(s0)={sectors_2d[0]}cm, right(s18)={sectors_2d[18]}cm, "
            f"back(s36)={sectors_2d[36]}cm, left(s54)={sectors_2d[54]}cm, "
            f"clear={sectors_2d[1]}cm"
        )
        print(
            f"  FLU LaserScan CCW ranges : "
            f"fwd(0)={flu_ranges[0]:.2f}m, left(18)={flu_ranges[18]:.2f}m, "
            f"back(36)={flu_ranges[36]:.2f}m, right(54)={flu_ranges[54]:.2f}m"
        )
    return {
        "sectors_2d": sectors_2d,
        "flu_ranges": flu_ranges,
    }


# =============================================================================
# ROS 2 Node Implementation
# =============================================================================

def run_ros2_node(args: argparse.Namespace, ros_args: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from mavros_msgs.msg import ObstacleDistance3D
    from sensor_msgs.msg import LaserScan

    mav_px4 = None
    if args.px4_mavlink_udp:
        from pymavlink import mavutil
        mav_px4 = mavutil.mavlink_connection(f"udpout:{args.px4_mavlink_udp}")

    class LatencyRelayNode(Node):
        def __init__(self) -> None:
            super().__init__("latency_relay")
            self._buffer = LatencyBuffer(delay_ms=args.delay_ms)
            self._recent_frd_points: dict[int, tuple[float, float, float, float]] = {}

            self._sub_3d = self.create_subscription(
                ObstacleDistance3D,
                args.input_topic,
                self._cb_3d,
                20,
            )
            self._pub_3d = self.create_publisher(
                ObstacleDistance3D,
                args.output_topic,
                20,
            )
            self._pub_scan = self.create_publisher(
                LaserScan,
                args.px4_2d_topic,
                10,
            )
            self._timer = self.create_timer(0.005, self._tick)
            self._scan_timer = self.create_timer(0.1, self._publish_px4_2d)

            self.get_logger().info(
                f"LatencyRelay started [delay={args.delay_ms:.1f} ms, "
                f"in={args.input_topic} -> out={args.output_topic}, "
                f"px4_2d_topic={args.px4_2d_topic}]"
            )

        def _cb_3d(self, msg: ObstacleDistance3D) -> None:
            self._buffer.push(msg, time.monotonic())

        def _tick(self) -> None:
            now = time.monotonic()
            for msg in self._buffer.pop_ready(now):
                self._pub_3d.publish(msg)
                self._recent_frd_points[int(msg.obstacle_id)] = (
                    float(msg.position.x),
                    float(msg.position.y),
                    float(msg.position.z),
                    now,
                )

        def _publish_px4_2d(self) -> None:
            now = time.monotonic()
            # Prune obstacle points older than 0.6 s
            self._recent_frd_points = {
                oid: item
                for oid, item in self._recent_frd_points.items()
                if now - item[3] <= 0.6
            }
            pts_frd = [
                (item[0], item[1], item[2])
                for item in self._recent_frd_points.values()
            ]
            distances_cm = points_3d_to_px4_obstacle_distance_2d(
                pts_frd,
                min_distance_cm=args.min_distance_cm,
                max_distance_cm=args.max_distance_cm,
            )

            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = "base_link"
            scan.angle_min = 0.0
            scan.angle_increment = math.radians(PX4_SECTOR_INCREMENT_DEG)
            scan.angle_max = math.radians(360.0 - PX4_SECTOR_INCREMENT_DEG)
            scan.range_min = args.min_distance_cm / 100.0
            scan.range_max = args.max_distance_cm / 100.0
            scan.ranges = frd_sectors_to_flu_laserscan_ranges(
                distances_cm,
                max_distance_cm=args.max_distance_cm,
            )
            self._pub_scan.publish(scan)

            if mav_px4 is not None:
                send_px4_obstacle_distance(
                    mav_px4,
                    pts_frd,
                    time_usec=int(self.get_clock().now().nanoseconds // 1000),
                    min_distance_cm=args.min_distance_cm,
                    max_distance_cm=args.max_distance_cm,
                )

    rclpy.init(args=ros_args)
    node = LatencyRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Jetson Orin NX stereo-to-MAVLink latency relay and PX4 72-sector OBSTACLE_DISTANCE bridge."
    )
    parser.add_argument(
        "--delay-ms",
        type=float,
        default=80.0,
        help="Artificial compute latency in milliseconds (default: 80.0 ms)",
    )
    parser.add_argument(
        "--input-topic",
        type=str,
        default="/mavros/obstacle_distance_3d/send_raw",
        help="Input ObstacleDistance3D topic from obstacle_to_mavlink",
    )
    parser.add_argument(
        "--output-topic",
        type=str,
        default="/mavros/obstacle_distance_3d/send",
        help="Delayed output ObstacleDistance3D topic to MAVROS",
    )
    parser.add_argument(
        "--px4-2d-topic",
        type=str,
        default="/mavros/obstacle/send",
        help="2D 72-sector LaserScan output topic for PX4 Collision Prevention",
    )
    parser.add_argument(
        "--px4-mavlink-udp",
        type=str,
        default="",
        help="Optional host:port (e.g. 127.0.0.1:14580) to send MAVLink OBSTACLE_DISTANCE directly to PX4",
    )
    parser.add_argument(
        "--min-distance-cm",
        type=int,
        default=20,
        help="Minimum obstacle distance in centimeters (default: 20)",
    )
    parser.add_argument(
        "--max-distance-cm",
        type=int,
        default=500,
        help="Maximum obstacle distance in centimeters (default: 500)",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run standalone unit self-test and exit",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args(argv)
    if args.self_test:
        run_self_test(verbose=True)
        return 0
    run_ros2_node(args, ros_args=ros_args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
