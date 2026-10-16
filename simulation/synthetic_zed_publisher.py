#!/usr/bin/env python3
"""
Synthetic ZED X Point-Cloud Publisher & Standalone Obstacle-Detection Harness.

Works in two modes:
  1. **ROS 2 Node mode** (default when launched via `jetson_sil_launch.py` or
     `python3 synthetic_zed_publisher.py` inside the `jetson-sil` container):
       Publishes synthetic 3D point clouds split across:
         - `/zed1/zed_node/point_cloud/cloud_registered` (`frame_id: zed1_camera_link`)
         - `/zed2/zed_node/point_cloud/cloud_registered` (`frame_id: zed2_camera_link`)
       at `--rate-hz` (default 15 Hz, matching `multi_zed.launch.py`), and
       optionally publishes `rosgraph_msgs/msg/Clock` on `/clock` when
       `--publish-clock` is passed.

  2. **Standalone Pure-Python Self-Test / Library mode** (`--self-test`):
       Runs without `rclpy`, `isaacsim`, or even a GPU so developers on macOS
       or Linux can locally verify the complete `visual_obstacle_detection`
       pipeline:
         1. Airframe self-points inside `excluded_boxes.xml` (all 15 boxes,
            including `central_body` `[-0.375..0.375, -0.375..0.375, -0.3008..0.1992]`)
            are 100% filtered out.
         2. Disconnected noise points in the same sector and radial shell are
            rejected by 26-neighbor 10 cm voxel coherence (`coherent_groups`).
         3. Coherent obstacle surfaces at 2.0 m, 3.0 m, 4.0 m, and 4.9 m are
            detected by `build_sector_map`, confirmed on frame 2 of 3 by
            `CandidateConfirmation.stabilize`, and converted from body RFU/FLU
            to `MAV_FRAME_BODY_FRD` (`frame=12`, `x_frd=x, y_frd=-y, z_frd=-z`).
"""

from __future__ import annotations

import argparse
from itertools import product
import math
from pathlib import Path
import struct
import sys
import time
from typing import Any
import xml.etree.ElementTree as ET

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:  # pragma: no cover - fallback for bare system Python without numpy
    np = None  # type: ignore[assignment]
    HAS_NUMPY = False


# =============================================================================
# Constants Matching VFS-Go-Aero/visual-obstacle-detection (@ cba2f42)
# =============================================================================

N_AZ = 32               # azimuth bins (360 / 32 = 11.25 deg each)
N_EL = 8                # elevation bins (180 / 8 = 22.5 deg each)
DIST_BIN_W = 0.1        # radial distance shell width (m)
MIN_POINTS = 100        # required support at distances <= 2.0 m
MIN_POINTS_FLOOR = 20   # minimum support floor at far distances
SUPPORT_BINS = 3        # sliding window of 3 adjacent 0.1 m radial shells
MAX_DETECTION_RANGE_M = 5.0

MAV_FRAME_BODY_FRD = 12

# Static extrinsics from `multi_zed.launch.py` (base_link -> zedN_camera_link)
ZED_CAMERA_EXTRINSICS: dict[str, tuple[float, float, float]] = {
    "zed1_camera_link": (0.1524, 0.0, -0.127),
    "zed2_camera_link": (0.1524, 0.0, -0.127),
    "base_link": (0.0, 0.0, 0.0),
}

# Exact XML content of `visual_obstacle_detection/excluded_boxes.xml` (@ cba2f42)
DEFAULT_EXCLUDED_BOXES_XML = """<?xml version="1.0" encoding="UTF-8"?>
<exclusion_boxes>
  <box name="central_body">
    <min x="-0.375" y="-0.375" z="-0.3008" />
    <max x="0.375" y="0.375" z="0.1992" />
  </box>
  <box name="central_lower1">
    <min x="-0.35" y="-0.3" z="-0.3908" />
    <max x="0.35" y="0.3" z="-0.0908" />
  </box>
  <box name="central_lower2">
    <min x="-0.375" y="-0.325" z="-0.4808" />
    <max x="0.375" y="0.325" z="-0.1808" />
  </box>
  <box name="central_lower3">
    <min x="-0.45" y="-0.35" z="-0.5708" />
    <max x="0.45" y="0.35" z="-0.2708" />
  </box>
  <box name="central_lower4">
    <min x="-0.55" y="-0.45" z="-0.7508" />
    <max x="0.55" y="0.45" z="-0.3508" />
  </box>
  <box name="side_small1_right">
    <min x="0.29" y="0.245" z="-0.2658" />
    <max x="0.69" y="0.645" z="0.1342" />
  </box>
  <box name="side_small2_right">
    <min x="0.245" y="0.205" z="-0.2308" />
    <max x="0.555" y="0.515" z="0.0692" />
  </box>
  <box name="side_small3_right">
    <min x="0.195" y="0.165" z="-0.2308" />
    <max x="0.505" y="0.475" z="0.0692" />
  </box>
  <box name="side_small4_right">
    <min x="0.135" y="0.11" z="-0.2308" />
    <max x="0.445" y="0.42" z="0.0692" />
  </box>
  <box name="side_plate_right">
    <min x="-0.035" y="-0.081" z="-0.1408" />
    <max x="1.065" y="1.019" z="0.1592" />
  </box>
  <box name="side_small1_left">
    <min x="0.29" y="-0.645" z="-0.2658" />
    <max x="0.69" y="-0.245" z="0.1342" />
  </box>
  <box name="side_small2_left">
    <min x="0.245" y="-0.515" z="-0.2308" />
    <max x="0.555" y="-0.205" z="0.0692" />
  </box>
  <box name="side_small3_left">
    <min x="0.195" y="-0.475" z="-0.2308" />
    <max x="0.505" y="-0.165" z="0.0692" />
  </box>
  <box name="side_small4_left">
    <min x="0.135" y="-0.42" z="-0.2308" />
    <max x="0.445" y="-0.11" z="0.0692" />
  </box>
  <box name="side_plate_left">
    <min x="-0.035" y="-1.019" z="-0.1408" />
    <max x="1.065" y="0.081" z="0.1592" />
  </box>
</exclusion_boxes>
"""


# =============================================================================
# Array / Point-List Conversion Helpers (NumPy + Pure-Python Compatible)
# =============================================================================

def _to_point_list(points: Any) -> list[tuple[float, float, float]]:
    if points is None:
        return []
    if HAS_NUMPY and isinstance(points, np.ndarray):
        if points.size == 0:
            return []
        reshaped = points.reshape(-1, 3)
        return [(float(p[0]), float(p[1]), float(p[2])) for p in reshaped]
    return [(float(p[0]), float(p[1]), float(p[2])) for p in points]


def _as_points_output(points_list: list[tuple[float, float, float]]) -> Any:
    if HAS_NUMPY:
        if not points_list:
            return np.empty((0, 3), dtype=np.float32)
        return np.array(points_list, dtype=np.float32).reshape(-1, 3)
    return [list(p) for p in points_list]


def _norm3(p: tuple[float, float, float] | list[float]) -> float:
    return math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])


def _dist3(
    a: tuple[float, float, float] | list[float],
    b: tuple[float, float, float] | list[float],
) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _median_1d(vals: list[float]) -> float:
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


# =============================================================================
# Exclusion Box Loading & Filtering (`point_cloud.py` Parity)
# =============================================================================

ExclusionBox = tuple[str, tuple[float, float, float], tuple[float, float, float]]


def load_exclude_boxes(xml_path: str | Path | None = None) -> list[ExclusionBox]:
    """
    Load axis-aligned airframe exclusion boxes from XML in `base_link` coordinates.

    If `xml_path` is None or does not exist on disk, falls back to the embedded
    `DEFAULT_EXCLUDED_BOXES_XML` from `visual_obstacle_detection/excluded_boxes.xml`.
    """
    if xml_path is not None and Path(xml_path).exists():
        root = ET.parse(Path(xml_path)).getroot()
    else:
        root = ET.fromstring(DEFAULT_EXCLUDED_BOXES_XML)

    boxes: list[ExclusionBox] = []
    for idx, box in enumerate(root.findall("box"), start=1):
        min_node = box.find("min")
        max_node = box.find("max")
        if min_node is None or max_node is None:
            continue
        try:
            c_min = (
                float(min_node.attrib["x"]),
                float(min_node.attrib["y"]),
                float(min_node.attrib["z"]),
            )
            c_max = (
                float(max_node.attrib["x"]),
                float(max_node.attrib["y"]),
                float(max_node.attrib["z"]),
            )
        except (KeyError, ValueError):
            continue
        lo = (min(c_min[0], c_max[0]), min(c_min[1], c_max[1]), min(c_min[2], c_max[2]))
        hi = (max(c_min[0], c_max[0]), max(c_min[1], c_max[1]), max(c_min[2], c_max[2]))
        name = box.attrib.get("name", f"box_{idx}")
        boxes.append((name, lo, hi))
    return boxes


def apply_exclude_boxes(
    points: Any,
    boxes: list[ExclusionBox] | None = None,
    return_box_hits: bool = False,
) -> Any:
    """
    Remove points that fall inside any configured axis-aligned exclusion box
    (inclusive of box boundaries `lo <= p <= hi`, matching `point_cloud.py`).
    """
    if boxes is None:
        boxes = load_exclude_boxes()
    pts = _to_point_list(points)
    box_hits = [0] * len(boxes)
    if not pts or not boxes:
        out = _as_points_output(pts)
        return (out, box_hits) if return_box_hits else out

    kept: list[tuple[float, float, float]] = []
    for x, y, z in pts:
        excluded = False
        for idx, (_name, lo, hi) in enumerate(boxes):
            if (lo[0] <= x <= hi[0]) and (lo[1] <= y <= hi[1]) and (lo[2] <= z <= hi[2]):
                box_hits[idx] += 1
                excluded = True
        if not excluded:
            kept.append((x, y, z))

    out = _as_points_output(kept)
    return (out, box_hits) if return_box_hits else out


# =============================================================================
# Frame Transform Helpers (`base_link` <-> `zedN_camera_link`)
# =============================================================================

def base_link_to_camera_frame(points_base: Any, camera_frame: str = "zed1_camera_link") -> Any:
    """Transform points from `base_link` into `zed1_camera_link` / `zed2_camera_link`."""
    tx, ty, tz = ZED_CAMERA_EXTRINSICS.get(camera_frame, (0.0, 0.0, 0.0))
    pts = _to_point_list(points_base)
    transformed = [(x - tx, y - ty, z - tz) for (x, y, z) in pts]
    return _as_points_output(transformed)


def camera_frame_to_base_link(points_cam: Any, camera_frame: str = "zed1_camera_link") -> Any:
    """Transform points from `zed1_camera_link` / `zed2_camera_link` into `base_link`."""
    tx, ty, tz = ZED_CAMERA_EXTRINSICS.get(camera_frame, (0.0, 0.0, 0.0))
    pts = _to_point_list(points_cam)
    transformed = [(x + tx, y + ty, z + tz) for (x, y, z) in pts]
    return _as_points_output(transformed)


# =============================================================================
# Exact `visual_obstacle_detection` Pipeline Algorithms
# =============================================================================

def required_points(
    distance: Any,
    base: int = MIN_POINTS,
    reference: float = 2.0,
    floor: int = MIN_POINTS_FLOOR,
) -> Any:
    """
    Bound inverse-square scaling so nearby obstacles retain the base count:
      ceil(clip(base * (reference / max(distance, reference))^2, floor, base))
    Matches `visual_obstacle_detection.detection_support.required_points`.
    """
    def _scalar_req(d: float) -> int:
        val = float(d)
        if not math.isfinite(val):
            return int(base)
        d_eff = max(val, float(reference))
        raw = float(base) * ((float(reference) / d_eff) ** 2)
        clamped = min(max(raw, float(floor)), float(base))
        return int(math.ceil(clamped))

    if isinstance(distance, (int, float)):
        return _scalar_req(float(distance))

    vals = [_scalar_req(float(d)) for d in distance]
    if HAS_NUMPY:
        return np.array(vals, dtype=int)
    return vals


def coherent_groups(points: Any, cell_size: float = 0.1):
    """
    Group points in connected occupied 10 cm voxels (26-neighbor adjacency).
    Matches `visual_obstacle_detection.detection_support.coherent_groups`.
    """
    cs = float(cell_size)
    if not math.isfinite(cs) or cs <= 0.0:
        raise ValueError(f"cell_size must be finite and > 0, got {cell_size}")

    pts = _to_point_list(points)
    if not pts:
        return

    cell_to_point_indices: dict[tuple[int, int, int], list[int]] = {}
    for i, (x, y, z) in enumerate(pts):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        cell = (
            int(math.floor(x / cs)),
            int(math.floor(y / cs)),
            int(math.floor(z / cs)),
        )
        cell_to_point_indices.setdefault(cell, []).append(i)

    remaining = set(cell_to_point_indices.keys())
    offsets = list(product((-1, 0, 1), repeat=3))

    while remaining:
        seed = remaining.pop()
        stack = [seed]
        component_cells = [seed]
        while stack:
            current = stack.pop()
            for off in offsets:
                neighbor = (current[0] + off[0], current[1] + off[1], current[2] + off[2])
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component_cells.append(neighbor)
                    stack.append(neighbor)
        member_indices: list[int] = []
        for c in component_cells:
            member_indices.extend(cell_to_point_indices[c])
        member_indices.sort()
        if HAS_NUMPY:
            yield np.array(member_indices, dtype=np.int64)
        else:
            yield member_indices


def build_sector_map(
    points: Any,
    n_az: int = N_AZ,
    n_el: int = N_EL,
    dist_bin_w: float = DIST_BIN_W,
    min_pts: int = MIN_POINTS,
    nearest_ranges: dict[int, float] | None = None,
):
    """
    Build a 32x8 (azimuth x elevation) sector map finding the nearest coherent
    obstacle in each angular sector within a 3-bin sliding radial window.
    Matches `visual_obstacle_detection.obstacle_detection.build_sector_map`.
    """
    pts = _to_point_list(points)
    n_total = len(pts)
    if n_total == 0:
        if HAS_NUMPY:
            return np.zeros(0, dtype=bool), np.zeros(0, dtype=np.uint32)
        return [], []

    dists = [0.0] * n_total
    valid = [False] * n_total
    az_idx = [0] * n_total
    el_idx = [0] * n_total

    for i, (x, y, z) in enumerate(pts):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        d = math.sqrt(x * x + y * y + z * z)
        dists[i] = d
        if 0.0 < d <= MAX_DETECTION_RANGE_M:
            valid[i] = True
            dx, dy, dz = x / d, y / d, z / d
            az = math.atan2(dy, dx)
            el = math.asin(min(max(dz, -1.0), 1.0))
            a = int((az + math.pi) / (2.0 * math.pi) * n_az) % n_az
            e = int((el + math.pi / 2.0) / math.pi * n_el)
            e = min(max(e, 0), n_el - 1)
            az_idx[i] = a
            el_idx[i] = e

    winner_mask = [False] * n_total
    winner_sector = [0] * n_total

    if not any(valid):
        if HAS_NUMPY:
            return np.array(winner_mask, dtype=bool), np.array(winner_sector, dtype=np.uint32)
        return winner_mask, winner_sector

    # Group valid point indices by (az_idx, el_idx)
    sector_buckets: dict[tuple[int, int], list[int]] = {}
    for i in range(n_total):
        if valid[i]:
            sector_buckets.setdefault((az_idx[i], el_idx[i]), []).append(i)

    floor_pts = min(MIN_POINTS_FLOOR, min_pts)

    for a in range(n_az):
        for e in range(n_el):
            sector_indices = sector_buckets.get((a, e))
            if not sector_indices:
                continue

            sector_dists = [dists[idx] for idx in sector_indices]
            max_d = max(sector_dists)
            if not math.isfinite(max_d) or max_d <= 0.0:
                continue

            bin_ids = [int(math.floor(d / dist_bin_w)) for d in sector_dists]
            num_bins = max(bin_ids) + 1
            bin_counts = [0] * num_bins
            for b_id in bin_ids:
                bin_counts[b_id] += 1

            for b in range(num_bins):
                if bin_counts[b] <= 0:
                    continue
                window_count = sum(
                    bin_counts[k] for k in range(b, min(num_bins, b + SUPPORT_BINS))
                )
                # Use the far edge of the sliding window as a conservative lower bound
                # for the fast pre-filter; exact `required_points(near)` is enforced per group below.
                thresh = required_points((b + SUPPORT_BINS) * dist_bin_w, base=min_pts, floor=floor_pts)
                if window_count < thresh:
                    continue

                window_indices = [
                    sector_indices[j]
                    for j, b_id in enumerate(bin_ids)
                    if b <= b_id < b + SUPPORT_BINS
                ]
                window_pts = [pts[idx] for idx in window_indices]

                candidates: list[tuple[float, list[int]]] = []
                for group in coherent_groups(window_pts):
                    group_list = [int(g) for g in group]
                    indices = [window_indices[g] for g in group_list]
                    near = min(dists[idx] for idx in indices)
                    needed = required_points(near, base=min_pts, floor=floor_pts)
                    if len(indices) >= needed:
                        candidates.append((near, indices))

                if not candidates:
                    continue

                near, pts_in_bin = min(candidates, key=lambda c: c[0])
                cx = _median_1d([pts[idx][0] for idx in pts_in_bin])
                cy = _median_1d([pts[idx][1] for idx in pts_in_bin])
                cz = _median_1d([pts[idx][2] for idx in pts_in_bin])
                center = (cx, cy, cz)

                closest = min(pts_in_bin, key=lambda idx: _dist3(pts[idx], center))
                winner_mask[closest] = True
                sector_id = a * n_el + e
                winner_sector[closest] = sector_id
                if nearest_ranges is not None:
                    nearest_ranges[sector_id] = float(near)
                break

    if HAS_NUMPY:
        return np.array(winner_mask, dtype=bool), np.array(winner_sector, dtype=np.uint32)
    return winner_mask, winner_sector


class CandidateConfirmation:
    """
    Confirm sector representatives seen in at least 2 of 3 frames and hold
    confirmed obstacles briefly (`hold_time=0.6` s) across one missed frame.
    Matches `visual_obstacle_detection.detection_support.CandidateConfirmation`.
    """

    def __init__(
        self,
        tolerance: float = 0.5,
        max_gap: float = 1.0,
        hold_time: float = 0.6,
    ) -> None:
        self.previous: dict[int, tuple[float, float, float]] = {}
        self.history: list[dict[int, tuple[float, float, float]]] = []
        self.active: dict[int, tuple[tuple[float, float, float], float]] = {}
        self.last_time: float | None = None
        self.tolerance = float(tolerance)
        self.max_gap = float(max_gap)
        self.hold_time = float(hold_time)

    def update(self, points: Any, sectors: Any, now: float) -> Any:
        t_now = float(now)
        if (
            not math.isfinite(t_now)
            or self.last_time is None
            or not (0.0 <= t_now - self.last_time <= self.max_gap)
        ):
            self.history = []
            self.active = {}

        pts = _to_point_list(points)
        secs = [int(s) for s in sectors]
        confirmed = [False] * len(pts)

        for i, (pt, sec) in enumerate(zip(pts, secs)):
            confirmed[i] = any(
                sec in frame and _dist3(pt, frame[sec]) <= self.tolerance
                for frame in self.history
            )

        self.previous = {int(s): (float(p[0]), float(p[1]), float(p[2])) for p, s in zip(pts, secs)}
        self.history = (self.history + [self.previous])[-2:]
        self.last_time = t_now if math.isfinite(t_now) else None

        if HAS_NUMPY:
            return np.array(confirmed, dtype=bool)
        return confirmed

    def stabilize(self, points: Any, sectors: Any, now: float) -> tuple[Any, Any]:
        pts = _to_point_list(points)
        secs = [int(s) for s in sectors]
        confirmed = self.update(pts, secs, now)

        self.active = {
            s: item
            for s, item in self.active.items()
            if now - item[1] <= self.hold_time
        }

        for pt, sec, accepted in zip(pts, secs, confirmed):
            old = self.active.get(sec)
            matches_active = old is not None and _dist3(pt, old[0]) <= self.tolerance
            if bool(accepted) or matches_active:
                self.active[sec] = ((float(pt[0]), float(pt[1]), float(pt[2])), float(now))

        ordered = sorted(self.active.keys())
        out_pts = [self.active[s][0] for s in ordered]
        if HAS_NUMPY:
            arr_pts = (
                np.array(out_pts, dtype=np.float32).reshape(-1, 3)
                if out_pts
                else np.empty((0, 3), dtype=np.float32)
            )
            arr_secs = np.array(ordered, dtype=np.uint32)
            return arr_pts, arr_secs
        return [list(p) for p in out_pts], ordered


# =============================================================================
# RFU / FLU `base_link` -> `MAV_FRAME_BODY_FRD` Conversion (`obstacle_to_mavlink.py`)
# =============================================================================

def rfu_to_frd_point(point_rfu: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    """
    Convert an obstacle point from `base_link` (`+X=fwd, +Y=left, +Z=up`) to
    `MAV_FRAME_BODY_FRD` (`+X=fwd, +Y=right, +Z=down`) exactly as in
    `obstacle_to_mavlink.py`:
        x_frd =  x
        y_frd = -y
        z_frd = -z
    """
    return (float(point_rfu[0]), -float(point_rfu[1]), -float(point_rfu[2]))


def obstacles_to_mavlink_3d(
    obstacle_points: Any,
    obstacle_sectors: Any,
    min_distance: float = 0.2,
    max_distance: float = 60.0,
    frame_id: str = "base_link",
) -> list[dict[str, Any]]:
    """
    Convert stabilized obstacle representatives into `ObstacleDistance3D`
    message dictionaries matching `obstacle_to_mavlink.py`.
    """
    pts = _to_point_list(obstacle_points)
    secs = [int(s) for s in obstacle_sectors]
    messages: list[dict[str, Any]] = []
    for pt, sec in zip(pts, secs):
        x_frd, y_frd, z_frd = rfu_to_frd_point(pt)
        messages.append(
            {
                "frame_id": frame_id,
                "sensor_type": 1,
                "frame": MAV_FRAME_BODY_FRD,
                "obstacle_id": int(sec % 65536),
                "x": x_frd,
                "y": y_frd,
                "z": z_frd,
                "min_distance": float(min_distance),
                "max_distance": float(max_distance),
            }
        )
    return messages


def run_pipeline_step(
    raw_points_base: Any,
    confirmation: CandidateConfirmation,
    now: float,
    boxes: list[ExclusionBox] | None = None,
) -> dict[str, Any]:
    """
    Execute one full tick of the `visual_obstacle_detection` pipeline on a
    merged `base_link` point cloud:
      1. `apply_exclude_boxes` (removes airframe self-points)
      2. `build_sector_map` + range-preserving representative scaling
      3. `CandidateConfirmation.stabilize` (2-of-3 frame confirmation + 0.6s hold)
      4. `obstacles_to_mavlink_3d` (RFU -> `MAV_FRAME_BODY_FRD`)
    """
    filtered_pts, box_hits = apply_exclude_boxes(
        raw_points_base, boxes=boxes, return_box_hits=True
    )
    pts_list = _to_point_list(filtered_pts)

    nearest_ranges: dict[int, float] = {}
    winner_mask, winner_sector = build_sector_map(
        filtered_pts, nearest_ranges=nearest_ranges
    )

    mask_list = [bool(m) for m in winner_mask]
    sec_list = [int(s) for s in winner_sector]

    rep_points: list[tuple[float, float, float]] = []
    rep_sectors: list[int] = []
    for pt, is_win, sec in zip(pts_list, mask_list, sec_list):
        if is_win:
            d = _norm3(pt)
            scale = nearest_ranges[sec] / d if d > 0.0 else 1.0
            rep_points.append((pt[0] * scale, pt[1] * scale, pt[2] * scale))
            rep_sectors.append(sec)

    stab_pts, stab_secs = confirmation.stabilize(rep_points, rep_sectors, now)
    mav_msgs = obstacles_to_mavlink_3d(stab_pts, stab_secs)

    return {
        "filtered_points": filtered_pts,
        "box_hits": box_hits,
        "candidate_points": _as_points_output(rep_points),
        "candidate_sectors": rep_sectors,
        "confirmed_points": stab_pts,
        "confirmed_sectors": [int(s) for s in stab_secs],
        "mavlink_3d": mav_msgs,
    }


# =============================================================================
# Synthetic Point Cloud Generators
# =============================================================================

def generate_coherent_surface(
    distance_m: float = 4.0,
    n_points: int | None = None,
    y_min: float = 0.02,
    y_max: float = 0.06,
    z_val: float = 0.04,
) -> Any:
    """
    Generate a dense, spatially coherent obstacle surface patch at `distance_m`
    ahead (`+X` in `base_link`), occupying a single azimuth/elevation sector
    and connected 10 cm voxels.
    """
    d_target = float(distance_m)
    if n_points is None:
        # Provide a comfortable margin above `required_points(distance_m)`
        n_points = max(int(required_points(d_target)) + 10, 30)

    if n_points <= 1:
        ys = [0.5 * (y_min + y_max)]
    else:
        step = (y_max - y_min) / float(n_points - 1)
        ys = [y_min + i * step for i in range(n_points)]

    # If `distance_m` is right at `MAX_DETECTION_RANGE_M` (e.g. 5.0 m), ensure
    # the 3D radial norm `sqrt(x^2 + y^2 + z^2)` does not exceed `distance_m`.
    if d_target >= MAX_DETECTION_RANGE_M - 1e-3:
        pts = [
            (
                math.sqrt(max(0.0, d_target * d_target - float(y) * float(y) - float(z_val) * float(z_val))),
                float(y),
                float(z_val),
            )
            for y in ys
        ]
    else:
        pts = [(d_target, float(y), float(z_val)) for y in ys]
    return _as_points_output(pts)


def generate_airframe_self_points(
    boxes: list[ExclusionBox] | None = None,
    points_per_box: int = 8,
) -> Any:
    """
    Generate synthetic airframe self-reflection points strictly inside every
    configured exclusion box in `excluded_boxes.xml` (including `central_body`).
    """
    if boxes is None:
        boxes = load_exclude_boxes()

    pts: list[tuple[float, float, float]] = []
    for _name, lo, hi in boxes:
        for k in range(points_per_box):
            alpha = (k + 1) / float(points_per_box + 1)
            pts.append(
                (
                    lo[0] + alpha * (hi[0] - lo[0]),
                    lo[1] + alpha * (hi[1] - lo[1]),
                    lo[2] + alpha * (hi[2] - lo[2]),
                )
            )
    return _as_points_output(pts)


def generate_disconnected_noise(distance_m: float = 4.5) -> Any:
    """
    Generate isolated noise points at `distance_m` radial range spaced 0.4 m
    apart in Y and Z so no two points occupy 26-adjacent 10 cm voxels.
    Matches `test_disconnected_noise_and_invalid_data` in `visual_obstacle_detection`.
    """
    pts: list[tuple[float, float, float]] = []
    y = 0.1
    while y < 2.0 - 1e-9:
        z = 0.1
        while z < 1.6 - 1e-9:
            r2 = distance_m * distance_m - (y * y + z * z)
            if r2 > 0.25:
                pts.append((math.sqrt(r2), y, z))
            z += 0.4
        y += 0.4
    return _as_points_output(pts)


def generate_synthetic_scene_cloud(
    obstacle_distances: tuple[float, ...] | list[float] = (4.0,),
    include_airframe: bool = True,
    include_noise: bool = True,
    frame_id: str = "base_link",
    points_scale: int = 1,
) -> Any:
    """
    Assemble a synthetic 3D point cloud combining:
      - One or more coherent obstacle surfaces at `obstacle_distances` (m)
      - Optional airframe self-points inside `excluded_boxes.xml`
      - Optional disconnected stereo speckle noise points
    Transformed into `frame_id` (`base_link`, `zed1_camera_link`, or `zed2_camera_link`).
    """
    scale = max(1, int(points_scale))
    all_pts: list[tuple[float, float, float]] = []
    for d in obstacle_distances:
        d_f = float(d)
        n_pts = scale * max(int(required_points(d_f)) + 10, 30)
        all_pts.extend(_to_point_list(generate_coherent_surface(d_f, n_points=n_pts)))
    if include_airframe:
        all_pts.extend(_to_point_list(generate_airframe_self_points()))
    if include_noise:
        all_pts.extend(_to_point_list(generate_disconnected_noise()))

    base_cloud = _as_points_output(all_pts)
    if frame_id != "base_link":
        return base_link_to_camera_frame(base_cloud, camera_frame=frame_id)
    return base_cloud


def split_scene_cloud_for_stereo_cameras(
    obstacle_distances: tuple[float, ...] | list[float] = (4.0,),
    include_airframe: bool = True,
    include_noise: bool = True,
) -> tuple[Any, Any]:
    """
    Generate a synthetic scene cloud in `base_link` and split its points across
    `zed1_camera_link` (`pts[::2]`, even indices) and `zed2_camera_link`
    (`pts[1::2]`, odd indices).

    Each camera half independently contains `>= required_points(d) + 10` points
    so even on the very first single-camera callback in `point_cloud.py` (before
    the second camera has populated its slot), the obstacle meets the point-count
    threshold without duplicating identical coordinates across both cameras.
    """
    base_cloud = generate_synthetic_scene_cloud(
        obstacle_distances=obstacle_distances,
        include_airframe=include_airframe,
        include_noise=include_noise,
        frame_id="base_link",
        points_scale=2,
    )
    pts = _to_point_list(base_cloud)
    cloud_zed1 = base_link_to_camera_frame(_as_points_output(pts[::2]), camera_frame="zed1_camera_link")
    cloud_zed2 = base_link_to_camera_frame(_as_points_output(pts[1::2]), camera_frame="zed2_camera_link")
    return cloud_zed1, cloud_zed2


# =============================================================================
# Standalone Self-Test Suite (`--self-test`)
# =============================================================================

def run_self_test(verbose: bool = True) -> dict[str, Any]:
    """
    Verify the complete `visual_obstacle_detection` math & confirmation pipeline
    without requiring a physical Jetson, ZED camera, Isaac Sim, or ROS 2 daemon.
    """
    boxes = load_exclude_boxes()
    assert len(boxes) == 15, f"Expected 15 exclusion boxes from excluded_boxes.xml, got {len(boxes)}"

    # Verify required_points(2.5) == 65 matching detection_support.py
    assert int(required_points(2.5)) == 65, f"Expected required_points(2.5)==65, got {required_points(2.5)}"

    # 1. Verify 100% of airframe points inside excluded_boxes.xml are removed
    airframe_pts = generate_airframe_self_points(boxes=boxes, points_per_box=10)
    filtered_airframe, box_hits = apply_exclude_boxes(
        airframe_pts, boxes=boxes, return_box_hits=True
    )
    n_airframe_in = len(_to_point_list(airframe_pts))
    n_airframe_out = len(_to_point_list(filtered_airframe))
    assert n_airframe_in == 150
    assert n_airframe_out == 0, f"Expected 0 airframe points after exclusion, got {n_airframe_out}"
    assert all(h > 0 for h in box_hits), "Every exclusion box should be exercised"

    # 2. Verify disconnected noise is 100% rejected by 26-neighbor 10 cm voxel clustering
    noise_pts = generate_disconnected_noise(4.5)
    noise_mask, _ = build_sector_map(noise_pts)
    n_noise_detected = sum(1 for m in noise_mask if bool(m))
    assert n_noise_detected == 0, f"Expected 0 detections on disconnected noise, got {n_noise_detected}"

    # 3. Verify coherent obstacles at 2.0 m, 3.0 m, 4.0 m, 4.9 m across 3 frames:
    #    - Frame 1 (t=1.0s): unconfirmed (0 obstacles output)
    #    - Frame 2 (t=1.1s): confirmed (2 of 3 frames -> 1 obstacle output in FRD)
    #    - Frame 3 (t=1.2s): confirmed (still active)
    test_distances = (2.0, 3.0, 4.0, 4.9)
    distance_results: dict[float, dict[str, Any]] = {}

    for dist_m in test_distances:
        confirm = CandidateConfirmation()
        req_n = int(required_points(dist_m))
        # Generate split stereo clouds in zed1_camera_link (even indices) and
        # zed2_camera_link (odd indices), transform both to base_link, and concatenate
        # exactly as point_cloud.py does.
        zed1_cloud, zed2_cloud = split_scene_cloud_for_stereo_cameras(
            obstacle_distances=(dist_m,),
            include_airframe=True,
            include_noise=True,
        )
        base1 = _to_point_list(camera_frame_to_base_link(zed1_cloud, camera_frame="zed1_camera_link"))
        base2 = _to_point_list(camera_frame_to_base_link(zed2_cloud, camera_frame="zed2_camera_link"))
        base_cloud = _as_points_output(base1 + base2)

        step1 = run_pipeline_step(base_cloud, confirm, now=1.0, boxes=boxes)
        assert len(step1["mavlink_3d"]) == 0, (
            f"Distance {dist_m}m should not be confirmed on frame 1 of 3"
        )

        step2 = run_pipeline_step(base_cloud, confirm, now=1.1, boxes=boxes)
        assert len(step2["mavlink_3d"]) == 1, (
            f"Distance {dist_m}m should be confirmed on frame 2 of 3"
        )

        msg3d = step2["mavlink_3d"][0]
        assert msg3d["frame"] == MAV_FRAME_BODY_FRD
        assert msg3d["sensor_type"] == 1
        # In base_link, obstacle is at (+dist_m, +y, +z) with y > 0, z > 0.
        # In FRD, x_frd = +x > 0, y_frd = -y < 0, z_frd = -z < 0.
        assert msg3d["x"] > 0.0 and msg3d["y"] < 0.0 and msg3d["z"] < 0.0
        measured_norm = math.sqrt(msg3d["x"] ** 2 + msg3d["y"] ** 2 + msg3d["z"] ** 2)
        assert abs(measured_norm - dist_m) < 0.02, (
            f"Expected FRD obstacle range ~{dist_m}m, got {measured_norm:.4f}m"
        )

        distance_results[dist_m] = {
            "required_points": req_n,
            "frame1_confirmed": len(step1["mavlink_3d"]),
            "frame2_confirmed": len(step2["mavlink_3d"]),
            "frd_msg": msg3d,
            "frd_range_m": measured_norm,
        }

    summary = {
        "exclusion_boxes_count": len(boxes),
        "airframe_points_tested": n_airframe_in,
        "airframe_points_remaining": n_airframe_out,
        "disconnected_noise_detected": n_noise_detected,
        "distance_results": distance_results,
    }

    if verbose:
        print("=== synthetic_zed_publisher --self-test PASSED ===")
        print(f"  Exclusion boxes verified : {len(boxes)} boxes ({n_airframe_in} airframe pts -> 0 remaining)")
        print(f"  Disconnected noise check : {len(_to_point_list(noise_pts))} isolated pts -> 0 false positives")
        for d, info in distance_results.items():
            m = info["frd_msg"]
            print(
                f"  Obstacle @ {d:3.1f} m (req>={info['required_points']:3d} pts): "
                f"frame1={info['frame1_confirmed']} -> frame2={info['frame2_confirmed']} | "
                f"MAV_FRAME_BODY_FRD({m['frame']}) pos=({m['x']:+.3f}, {m['y']:+.3f}, {m['z']:+.3f}) m "
                f"sector_id={m['obstacle_id']}"
            )
    return summary


# =============================================================================
# ROS 2 Publisher Node (Used When Running Inside `jetson-sil` Container)
# =============================================================================

def _pack_pointcloud2_xyz32(points: Any, frame_id: str, stamp_msg: Any):
    """Build a `sensor_msgs/msg/PointCloud2` message with float32 x, y, z fields."""
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    pts = _to_point_list(points)
    header = Header()
    header.stamp = stamp_msg
    header.frame_id = frame_id

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", x, y, z) for (x, y, z) in pts)
    return msg


def run_ros2_node(args: argparse.Namespace, ros_args: list[str] | None = None) -> None:
    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Time as RosTime
    from sensor_msgs.msg import PointCloud2

    Clock = None
    if args.publish_clock:
        try:
            from rosgraph_msgs.msg import Clock as _Clock
            Clock = _Clock
        except ImportError:
            Clock = None

    class SyntheticZedPublisher(Node):
        def __init__(self) -> None:
            super().__init__("synthetic_zed_publisher")
            self._pub_zed1 = self.create_publisher(PointCloud2, args.topic_zed1, 10)
            self._pub_zed2 = self.create_publisher(PointCloud2, args.topic_zed2, 10)
            self._pub_clock = (
                self.create_publisher(Clock, "/clock", 10)
                if (args.publish_clock and Clock is not None)
                else None
            )

            # Split the synthetic scene cloud across zed1 (pts[::2], even indices)
            # and zed2 (pts[1::2], odd indices) so point_cloud.py's concatenation
            # produces the exact non-duplicated point density in base_link.
            base_cloud = generate_synthetic_scene_cloud(
                obstacle_distances=args.obstacle_dist,
                include_airframe=args.include_airframe_points,
                include_noise=args.include_noise,
                frame_id="base_link",
            )
            pts = _to_point_list(base_cloud)
            self._cloud_zed1 = base_link_to_camera_frame(
                _as_points_output(pts[::2]),
                camera_frame="zed1_camera_link",
            )
            self._cloud_zed2 = base_link_to_camera_frame(
                _as_points_output(pts[1::2]),
                camera_frame="zed2_camera_link",
            )
            period = 1.0 / max(float(args.rate_hz), 0.5)
            self._timer = self.create_timer(period, self._tick)
            self._clock_timer = (
                self.create_timer(0.02, self._publish_clock_msg)
                if self._pub_clock is not None
                else None
            )
            self.get_logger().info(
                f"SyntheticZedPublisher started at {args.rate_hz:.1f} Hz "
                f"(obstacles={args.obstacle_dist} m, zed1_pts={len(pts[::2])}, "
                f"zed2_pts={len(pts[1::2])}, publish_clock={self._pub_clock is not None})"
            )

        def _make_wall_stamp(self) -> RosTime:
            now_ns = time.time_ns()
            stamp = RosTime()
            stamp.sec = int(now_ns // 1_000_000_000)
            stamp.nanosec = int(now_ns % 1_000_000_000)
            return stamp

        def _publish_clock_msg(self) -> RosTime:
            stamp = self._make_wall_stamp()
            if self._pub_clock is not None and Clock is not None:
                clk = Clock()
                clk.clock = stamp
                self._pub_clock.publish(clk)
            return stamp

        def _tick(self) -> None:
            if self._pub_clock is not None:
                now_msg = self._publish_clock_msg()
            else:
                now_msg = self.get_clock().now().to_msg()
            self._pub_zed1.publish(
                _pack_pointcloud2_xyz32(self._cloud_zed1, "zed1_camera_link", now_msg)
            )
            self._pub_zed2.publish(
                _pack_pointcloud2_xyz32(self._cloud_zed2, "zed2_camera_link", now_msg)
            )

    rclpy.init(args=ros_args)
    node = SyntheticZedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Synthetic ZED X PointCloud2 publisher and visual_obstacle_detection self-test harness."
    )
    parser.add_argument(
        "--obstacle-dist",
        type=float,
        nargs="+",
        default=[4.0],
        help="Obstacle surface distance(s) in meters ahead of base_link (default: 4.0)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=15.0,
        help="PointCloud2 publish rate in Hz (default: 15.0, matching multi_zed.launch.py)",
    )
    parser.add_argument(
        "--topic-zed1",
        type=str,
        default="/zed1/zed_node/point_cloud/cloud_registered",
        help="ROS 2 topic for ZED camera 1 point cloud",
    )
    parser.add_argument(
        "--topic-zed2",
        type=str,
        default="/zed2/zed_node/point_cloud/cloud_registered",
        help="ROS 2 topic for ZED camera 2 point cloud",
    )
    parser.add_argument(
        "--include-airframe-points",
        action="store_true",
        default=True,
        help="Include synthetic airframe self-reflection points inside excluded_boxes.xml",
    )
    parser.add_argument(
        "--include-noise",
        action="store_true",
        default=True,
        help="Include disconnected stereo speckle noise points",
    )
    parser.add_argument(
        "--publish-clock",
        action="store_true",
        default=False,
        help="Publish rosgraph_msgs/msg/Clock on /clock when use_sim_time:=true in standalone mode",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run standalone pure-Python self-test without ROS 2 or Isaac Sim and exit",
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
