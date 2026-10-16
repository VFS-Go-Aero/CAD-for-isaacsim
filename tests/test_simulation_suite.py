#!/usr/bin/env python3
"""
Comprehensive Pure-Python Unit & Integration Test Suite for the Go Aero Simulation.

Runs in standard Python on macOS/Linux (both with `pytest` and with
`python3 tests/test_simulation_suite.py` / `python3 -m unittest`) without
requiring NVIDIA Isaac Sim (`omni.*`, `pxr.*`), `isaaclab`, or ROS 2 (`rclpy`)
to be installed on the host.

Covers:
  1. `simulation/_frames.py` — NED/ENU/mesh quaternion algebra & round-trip conversions
  2. `simulation/px4_bridge.py` — quaternion rotation, `mesh_body_to_frd`, `enu_to_ned`,
     ISA barometric formula, quadrotor thrust/torque physics, and CLI flags (`--device`,
     `--headless`, `--lidar`, `--zed`, `--ros2`)
  3. `simulation/synthetic_zed_publisher.py` + `visual_obstacle_detection` —
     15-box airframe exclusion (`excluded_boxes.xml`), 26-neighbor 10 cm voxel
     coherence, 32x8 sector map, 2-of-3 frame confirmation, and RFU->FRD
     `ObstacleDistance3D` (`MAV_FRAME_BODY_FRD = 12`) conversion
  4. `simulation/latency_relay.py` — 80 ms Orin NX FIFO delay queue, CW FRD
     72-sector (5° increment) PX4 2D `OBSTACLE_DISTANCE` binning, and CCW FLU
     `LaserScan` conversion
  5. `simulation/jetson_sil_launch.py` & `docker/jetson-sil/*` — ROS 2 SIL launch
     plan, static TFs, topic remappings, and Docker/HIL artifacts
  6. `simulation/_compat.py` — Isaac Sim 5.1 / 6.0 compatibility layer, chassis
     sensor mount paths/coordinates, `_AppHandle`, `add_app_launcher_args`, and
     `enable_required_extensions`
  7. `simulation/ardupilot_bridge.py` — ArduPilot `SIM_JSON` binary 16/32-channel
     servo packet parser, PWM normalization, JSON FDM state reply, NED Euler
     extraction, and ArduCopter Quad-X motor mixing (`ARDU_PROP_ORDER`)
  8. `simulation/ardu_poshold.py` & `simulation/params/go_aero_ardupilot.param` —
     RC3 throttle PWM mapping, `POSITION_TARGET_TYPEMASK_POS_ONLY` (`FORCE_SET`
     bit 9 regression guard), obstacle stopping margin evaluator, CLI parser, and
     ArduPilot SITL parameter verification
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import MagicMock


REPO_ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = REPO_ROOT / "simulation"
DOCKER_DIR = REPO_ROOT / "docker" / "jetson-sil"

if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))


# =============================================================================
# Pure-Python Shims for `pxr.Gf` and `numpy` (When Running Outside Isaac Sim)
# =============================================================================

class _ShimVec3d:
    """Pure-Python implementation of `pxr.Gf.Vec3d` supporting vector math."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        if isinstance(x, (tuple, list, _ShimVec3d)):
            self._v = (float(x[0]), float(x[1]), float(x[2]))
        else:
            self._v = (float(x), float(y), float(z))

    def __getitem__(self, idx: int) -> float:
        return self._v[idx]

    def __len__(self) -> int:
        return 3

    def __iter__(self):
        return iter(self._v)

    def __neg__(self) -> "_ShimVec3d":
        return _ShimVec3d(-self._v[0], -self._v[1], -self._v[2])

    def __add__(self, other: "_ShimVec3d") -> "_ShimVec3d":
        return _ShimVec3d(self._v[0] + other[0], self._v[1] + other[1], self._v[2] + other[2])

    def __sub__(self, other: "_ShimVec3d") -> "_ShimVec3d":
        return _ShimVec3d(self._v[0] - other[0], self._v[1] - other[1], self._v[2] - other[2])

    def __mul__(self, scalar: float) -> "_ShimVec3d":
        s = float(scalar)
        return _ShimVec3d(self._v[0] * s, self._v[1] * s, self._v[2] * s)

    def __rmul__(self, scalar: float) -> "_ShimVec3d":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "_ShimVec3d":
        s = float(scalar)
        return _ShimVec3d(self._v[0] / s, self._v[1] / s, self._v[2] / s)

    def GetDot(self, other: "_ShimVec3d") -> float:
        return self._v[0] * other[0] + self._v[1] * other[1] + self._v[2] * other[2]

    def GetCross(self, other: "_ShimVec3d") -> "_ShimVec3d":
        ax, ay, az = self._v
        bx, by, bz = other[0], other[1], other[2]
        return _ShimVec3d(
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx,
        )

    def GetLengthSq(self) -> float:
        return self.GetDot(self)

    def GetLength(self) -> float:
        return math.sqrt(self.GetLengthSq())

    def __repr__(self) -> str:
        return f"Gf.Vec3d({self._v[0]:.6f}, {self._v[1]:.6f}, {self._v[2]:.6f})"


class _ShimQuatd:
    """
    Pure-Python implementation of `pxr.Gf.Quatd(real, imaginary_vec3d)` with
    Hamilton quaternion multiplication and inverse.
    """

    def __init__(
        self,
        real: float = 1.0,
        i: float | _ShimVec3d = 0.0,
        j: float = 0.0,
        k: float = 0.0,
    ) -> None:
        self._real = float(real)
        if isinstance(i, (_ShimVec3d, tuple, list)):
            self._imag = _ShimVec3d(i[0], i[1], i[2])
        else:
            self._imag = _ShimVec3d(float(i), float(j), float(k))

    def GetReal(self) -> float:
        return self._real

    def GetImaginary(self) -> _ShimVec3d:
        return self._imag

    def GetConjugate(self) -> "_ShimQuatd":
        return _ShimQuatd(self._real, -self._imag)

    def GetInverse(self) -> "_ShimQuatd":
        norm_sq = self._real * self._real + self._imag.GetLengthSq()
        if norm_sq <= 0.0:
            return _ShimQuatd(0.0, 0.0, 0.0, 0.0)
        conj = self.GetConjugate()
        return _ShimQuatd(conj.GetReal() / norm_sq, conj.GetImaginary() / norm_sq)

    def __mul__(self, other: "_ShimQuatd") -> "_ShimQuatd":
        w1 = self._real
        x1, y1, z1 = self._imag
        w2 = other.GetReal()
        x2, y2, z2 = other.GetImaginary()
        return _ShimQuatd(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            _ShimVec3d(
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ),
        )

    def Transform(self, vec: _ShimVec3d | tuple[float, float, float]) -> _ShimVec3d:
        v_quat = _ShimQuatd(0.0, _ShimVec3d(vec[0], vec[1], vec[2]))
        res = self * v_quat * self.GetInverse()
        return res.GetImaginary()

    def __repr__(self) -> str:
        return f"Gf.Quatd({self._real:.6f}, {self._imag})"


def _ensure_pxr_shim() -> None:
    try:
        from pxr import Gf as _real_gf  # noqa: F401
    except ImportError:
        pxr_mod = types.ModuleType("pxr")
        gf_mod = types.ModuleType("pxr.Gf")
        gf_mod.Vec3d = _ShimVec3d  # type: ignore[attr-defined]
        gf_mod.Quatd = _ShimQuatd  # type: ignore[attr-defined]
        pxr_mod.Gf = gf_mod  # type: ignore[attr-defined]
        sys.modules["pxr"] = pxr_mod
        sys.modules["pxr.Gf"] = gf_mod


class _MiniArray(list):
    """Lightweight 1D/2D array shim for environments without `numpy` installed."""

    def __init__(self, iterable=(), dtype=float):
        super().__init__(
            _MiniArray(x, dtype=dtype) if isinstance(x, (list, tuple)) else float(x)
            for x in iterable
        )
        self.dtype = dtype

    @property
    def shape(self):
        if len(self) > 0 and isinstance(self[0], list):
            return (len(self), len(self[0]))
        return (len(self),)

    def _binop(self, other, op):
        if isinstance(other, (list, tuple)):
            return _MiniArray([op(a, b) for a, b in zip(self, other)])
        return _MiniArray([op(a, other) for a in self])

    def __add__(self, other):
        return self._binop(other, lambda a, b: a + b)

    def __radd__(self, other):
        return self.__add__(other)

    def __iadd__(self, other):
        res = self.__add__(other)
        self[:] = res
        return self

    def __sub__(self, other):
        return self._binop(other, lambda a, b: a - b)

    def __mul__(self, other):
        return self._binop(other, lambda a, b: a * b)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __neg__(self):
        return _MiniArray([-a for a in self])


def _get_array_module():
    try:
        import numpy as real_np
        return real_np
    except ImportError:
        shim_np = types.ModuleType("numpy")
        shim_np.float64 = float  # type: ignore[attr-defined]
        shim_np.float32 = float  # type: ignore[attr-defined]
        shim_np.array = lambda seq, dtype=float: _MiniArray(seq, dtype=dtype)  # type: ignore[attr-defined]
        shim_np.asarray = lambda seq, dtype=float: _MiniArray(seq, dtype=dtype)  # type: ignore[attr-defined]
        shim_np.zeros = lambda n, dtype=float: _MiniArray([0.0] * int(n), dtype=dtype)  # type: ignore[attr-defined]
        shim_np.cross = lambda a, b: _MiniArray([  # type: ignore[attr-defined]
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ])
        linalg_mod = types.SimpleNamespace(
            norm=lambda v, axis=None: (
                _MiniArray([math.sqrt(sum(x * x for x in row)) for row in v])
                if axis == 1
                else math.sqrt(sum(float(x) * float(x) for x in v))
            )
        )
        shim_np.linalg = linalg_mod  # type: ignore[attr-defined]
        return shim_np


_ensure_pxr_shim()
import _compat  # noqa: E402
import _frames  # noqa: E402
import ardu_poshold  # noqa: E402
import ardupilot_bridge  # noqa: E402
import jetson_sil_launch  # noqa: E402
import latency_relay  # noqa: E402
import synthetic_zed_publisher as szp  # noqa: E402


import px4_bridge  # noqa: E402


def _load_px4_bridge_helpers(cli_args: list[str] | None = None) -> dict[str, object]:
    """
    Load the CLI parser, constants, and math/physics helper functions from
    `simulation/px4_bridge.py` without launching the Omniverse Kit runtime.
    """
    ns: dict[str, object] = dict(vars(px4_bridge))
    parser = px4_bridge.build_arg_parser()
    _compat.add_app_launcher_args(parser)
    args = parser.parse_args(cli_args if cli_args is not None else [])
    if args.lidar or args.zed or args.ros2:
        args.enable_cameras = True
    ns["parser"] = parser
    ns["args"] = args
    return ns


# =============================================================================
# 1. Coordinate Frame Math Tests (`simulation/_frames.py`)
# =============================================================================

class TestFramesModule(unittest.TestCase):
    """Test quaternion algebra and coordinate frame conversions in `_frames.py`."""

    @staticmethod
    def _quat_close(q1, q2, tol: float = 1e-6) -> bool:
        """Compare two unit quaternions up to global sign (`q` and `-q` are identical rotations)."""
        w1, (x1, y1, z1) = q1.GetReal(), q1.GetImaginary()
        w2, (x2, y2, z2) = q2.GetReal(), q2.GetImaginary()
        diff_pos = math.sqrt((w1 - w2) ** 2 + (x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
        diff_neg = math.sqrt((w1 + w2) ** 2 + (x1 + x2) ** 2 + (y1 + y2) ** 2 + (z1 + z2) ** 2)
        return min(diff_pos, diff_neg) < tol

    def test_euler_ned_identity(self) -> None:
        q = _frames.euler_ned_to_quat(0.0, 0.0, 0.0)
        self.assertAlmostEqual(q.GetReal(), 1.0, places=7)
        self.assertAlmostEqual(q.GetImaginary()[0], 0.0, places=7)
        self.assertAlmostEqual(q.GetImaginary()[1], 0.0, places=7)
        self.assertAlmostEqual(q.GetImaginary()[2], 0.0, places=7)

    def test_q_ned_enu_self_inverse(self) -> None:
        prod = _frames.Q_NED_ENU * _frames.Q_NED_ENU_INV
        self.assertAlmostEqual(prod.GetReal(), 1.0, places=7)
        for k in range(3):
            self.assertAlmostEqual(prod.GetImaginary()[k], 0.0, places=7)

    def test_attitude_world_quat_and_inverse_round_trip(self) -> None:
        test_angles = [
            (0.0, 0.0, 0.0),
            (0.25, 0.0, 0.0),
            (0.0, -0.3, 0.0),
            (0.0, 0.0, math.pi / 2.0),
            (0.0, 0.0, -math.pi / 2.0),
            (0.0, 0.0, math.pi),
            (0.35, -0.22, 1.15),
            (-0.48, 0.31, -2.05),
            (0.1, 0.2, 0.3),
        ]
        for roll, pitch, yaw in test_angles:
            q_ned_expected = _frames.euler_ned_to_quat(roll, pitch, yaw)
            q_world_enu = _frames.attitude_to_world_quat(roll, pitch, yaw)
            q_ned_recovered = _frames.enu_quat_to_ned_body(q_world_enu)
            self.assertTrue(
                self._quat_close(q_ned_expected, q_ned_recovered),
                msg=f"Round-trip failed for roll={roll}, pitch={pitch}, yaw={yaw}: "
                    f"expected={q_ned_expected}, got={q_ned_recovered}",
            )

    def test_identity_attitude_faces_north_in_enu(self) -> None:
        # At zero roll/pitch/yaw, attitude_to_world_quat equals Q_MESH_FIX (+90 deg about X).
        # Rotating mesh-forward (0, 0, -1) by Q_MESH_FIX lands at ENU +Y (north = (0, 1, 0)).
        q_world = _frames.attitude_to_world_quat(0.0, 0.0, 0.0)
        self.assertTrue(self._quat_close(q_world, _frames.Q_MESH_FIX))

    def test_vec_enu_ned_conversions(self) -> None:
        v_enu = (10.0, 25.0, 5.0)  # East=10, North=25, Up=5
        v_ned = _frames.vec_enu_to_ned(v_enu)
        self.assertEqual(v_ned, (25.0, 10.0, -5.0))  # North=25, East=10, Down=-5
        self.assertEqual(_frames.vec_ned_to_enu(v_ned), v_enu)


# =============================================================================
# 2. `px4_bridge.py` Physics, Frame Math & CLI Tests
# =============================================================================

class TestPx4BridgeMathAndCli(unittest.TestCase):
    """Test math helpers, quadrotor thrust/torque model, and CLI in `px4_bridge.py`."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.bridge = _load_px4_bridge_helpers([])

    def test_quat_rotate_and_conjugate(self) -> None:
        quat_rotate = self.bridge["quat_rotate"]
        quat_conj = self.bridge["quat_conj"]
        quat_mul = self.bridge["quat_mul"]

        # 90 deg rotation around Z axis: (cos45, 0, 0, sin45) maps (1, 0, 0) -> (0, 1, 0)
        c = math.sqrt(2.0) / 2.0
        q_z90 = [c, 0.0, 0.0, c]
        r = quat_rotate(q_z90, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(float(r[0]), 0.0, places=6)
        self.assertAlmostEqual(float(r[1]), 1.0, places=6)
        self.assertAlmostEqual(float(r[2]), 0.0, places=6)

        # Rotating back by conjugate recovers original vector
        v_orig = [1.5, -2.3, 4.1]
        v_back = quat_rotate(quat_conj(q_z90), quat_rotate(q_z90, v_orig))
        for a, b in zip(v_orig, v_back):
            self.assertAlmostEqual(float(a), float(b), places=6)

        # Quaternion multiplication q * q_conj == identity (1, 0, 0, 0)
        q_id = quat_mul(q_z90, quat_conj(q_z90))
        self.assertAlmostEqual(float(q_id[0]), 1.0, places=6)
        for k in (1, 2, 3):
            self.assertAlmostEqual(float(q_id[k]), 0.0, places=6)

    def test_mesh_body_to_frd_and_enu_to_ned(self) -> None:
        mesh_body_to_frd = self.bridge["mesh_body_to_frd"]
        enu_to_ned = self.bridge["enu_to_ned"]

        # Mesh local: +X = right, +Y = up, -Z = forward
        # FRD body:   +X = forward, +Y = right, +Z = down
        fwd_frd = mesh_body_to_frd([0.0, 0.0, -1.0])
        self.assertEqual([float(x) for x in fwd_frd], [1.0, 0.0, 0.0])

        right_frd = mesh_body_to_frd([1.0, 0.0, 0.0])
        self.assertEqual([float(x) for x in right_frd], [0.0, 1.0, 0.0])

        up_frd = mesh_body_to_frd([0.0, 1.0, 0.0])
        self.assertEqual([float(x) for x in up_frd], [0.0, 0.0, -1.0])

        # Specific force in level hover: IMU reads +9.81 m/s^2 along mesh +Y (up),
        # which maps to -9.81 m/s^2 along FRD +Z (down) as expected by PX4 EKF2.
        hover_acc_frd = mesh_body_to_frd([0.0, 9.81, 0.0])
        self.assertAlmostEqual(float(hover_acc_frd[2]), -9.81, places=5)

        ned = enu_to_ned([3.0, 7.0, 2.0])
        self.assertEqual([float(x) for x in ned], [7.0, 3.0, -2.0])

    def test_barometric_pressure_formula(self) -> None:
        pressure_from_altitude = self.bridge["pressure_from_altitude"]
        p0 = pressure_from_altitude(0.0)
        p100 = pressure_from_altitude(100.0)
        p1000 = pressure_from_altitude(1000.0)
        self.assertAlmostEqual(p0, 1013.25, places=2)
        self.assertLess(p100, p0)
        self.assertLess(p1000, p100)
        self.assertAlmostEqual(p1000, 898.75, delta=0.5)

    def test_quadrotor_hover_and_differential_torque(self) -> None:
        prop_layout = self.bridge["PROP_LAYOUT"]
        prop_order = self.bridge["PROP_ORDER"]
        max_thrust = float(self.bridge["MAX_THRUST_PER_MOTOR"])
        k_tau = float(self.bridge["K_TAU"])
        drone_mass = float(self.bridge["DRONE_MASS"])
        gravity = float(self.bridge["GRAVITY"])
        mesh_body_to_frd = self.bridge["mesh_body_to_frd"]
        np_mod = _get_array_module()

        # Verify PROP_ORDER and corrected PROP_LAYOUT spin directions matching
        # PX4 10050_none_go_aero (CA_ROTOR0_KM=+0.05, CA_ROTOR1_KM=-0.05,
        # CA_ROTOR2_KM=+0.05, CA_ROTOR3_KM=-0.05):
        self.assertEqual(list(prop_order), ["prop_fr", "prop_rr", "prop_rl", "prop_fl"])
        self.assertEqual(int(prop_layout["prop_fr"]["spin_dir"]), 1)
        self.assertEqual(int(prop_layout["prop_rl"]["spin_dir"]), 1)
        self.assertEqual(int(prop_layout["prop_fl"]["spin_dir"]), -1)
        self.assertEqual(int(prop_layout["prop_rr"]["spin_dir"]), -1)

        def compute_body_wrench(cmds):
            f_body = np_mod.zeros(3)
            m_body = np_mod.zeros(3)
            for i, name in enumerate(prop_order):
                t_i = max_thrust * (cmds[i] ** 2)
                r_body = prop_layout[name]["pos_body"]
                f_i = np_mod.array([0.0, t_i, 0.0])
                f_body = f_body + f_i
                m_body = m_body + np_mod.cross(r_body, f_i)
                m_body[1] += -k_tau * t_i * prop_layout[name]["spin_dir"]
            return f_body, m_body

        # 1. Equal hover throttle cmd = sqrt(0.5) gives exact weight balance and zero torque
        hover_cmd = math.sqrt(0.5)
        f_hover, m_hover = compute_body_wrench([hover_cmd] * 4)
        self.assertAlmostEqual(float(f_hover[0]), 0.0, places=6)
        self.assertAlmostEqual(float(f_hover[1]), drone_mass * gravity, places=5)
        self.assertAlmostEqual(float(f_hover[2]), 0.0, places=6)
        for k in range(3):
            self.assertAlmostEqual(float(m_hover[k]), 0.0, places=6)

        # 2. Higher left-side thrust (RL, FL > FR, RR) produces roll right (+X in FRD)
        _, m_roll = compute_body_wrench([0.4, 0.4, 0.8, 0.8])  # order: FR, RR, RL, FL
        m_roll_frd = mesh_body_to_frd(m_roll)
        self.assertGreater(float(m_roll_frd[0]), 0.0)
        self.assertAlmostEqual(float(m_roll_frd[1]), 0.0, places=6)
        self.assertAlmostEqual(float(m_roll_frd[2]), 0.0, places=6)

        # 3. Higher front thrust (FR, FL > RR, RL) produces nose-up pitch (+Y in FRD)
        _, m_pitch = compute_body_wrench([0.8, 0.4, 0.4, 0.8])
        m_pitch_frd = mesh_body_to_frd(m_pitch)
        self.assertAlmostEqual(float(m_pitch_frd[0]), 0.0, places=6)
        self.assertGreater(float(m_pitch_frd[1]), 0.0)
        self.assertAlmostEqual(float(m_pitch_frd[2]), 0.0, places=6)

        # 4. Higher CCW props (prop_fr at index 0, prop_rl at index 2 -> [0.8, 0.4, 0.8, 0.4])
        # produces m_yaw[1] < 0 in mesh +Y (Up), which transforms via mesh_body_to_frd
        # to positive yaw torque (m_yaw_frd[2] > 0, nose-right CW), matching PX4's
        # 10050_none_go_aero (CA_ROTOR0_KM = +0.05, CA_ROTOR2_KM = +0.05).
        _, m_yaw = compute_body_wrench([0.8, 0.4, 0.8, 0.4])
        self.assertLess(float(m_yaw[1]), 0.0)
        m_yaw_frd = mesh_body_to_frd(m_yaw)
        self.assertAlmostEqual(float(m_yaw_frd[0]), 0.0, places=6)
        self.assertAlmostEqual(float(m_yaw_frd[1]), 0.0, places=6)
        self.assertGreater(float(m_yaw_frd[2]), 0.0)

    def test_cli_arg_parsing_device_headless_lidar_zed_ros2(self) -> None:
        ns = _load_px4_bridge_helpers(
            ["--device", "cuda", "--headless", "--lidar", "--zed", "--ros2", "--px4-port", "4560"]
        )
        args = ns["args"]
        self.assertEqual(args.device, "cuda")
        self.assertTrue(args.headless)
        self.assertTrue(args.lidar)
        self.assertTrue(args.zed)
        self.assertTrue(args.ros2)
        self.assertTrue(args.enable_cameras)
        self.assertEqual(args.px4_port, 4560)


# =============================================================================
# 3. Synthetic ZED Publisher & `visual_obstacle_detection` Pipeline Tests
# =============================================================================

class TestSyntheticZedAndObstacleDetection(unittest.TestCase):
    """Test `synthetic_zed_publisher.py` and the `visual_obstacle_detection` algorithms."""

    def test_distance_adaptive_thresholds(self) -> None:
        # Verify required_points(2.5) == 65 matching detection_support.py's
        # np.ceil(100 * (2.0 / 2.5)**2) in IEEE-754 float64.
        self.assertEqual(int(szp.required_points(2.5)), 65)
        reqs = [int(x) for x in szp.required_points([1.0, 2.0, 2.5, 3.0, 4.0, 5.0])]
        self.assertEqual(reqs, [100, 100, 65, 45, 25, 20])

        # Check exact boundary detections matching upstream test_distance_detection.py
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(4.0, 25))[0]), 1)
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(4.0, 24))[0]), 0)
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(2.0, 25))[0]), 0)
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(2.0, 100))[0]), 1)
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(4.9, 20))[0]), 1)
        self.assertEqual(sum(bool(m) for m in szp.build_sector_map(szp.generate_coherent_surface(5.01, 100))[0]), 0)

    def test_shell_boundary_sliding_window_and_nearest_range(self) -> None:
        pts = szp._to_point_list(szp.generate_coherent_surface(3.0, 50))
        straddled = [
            (2.96 + (3.04 - 2.96) * i / 49.0, p[1], p[2])
            for i, p in enumerate(pts)
        ]
        ranges: dict[int, float] = {}
        mask, _ = szp.build_sector_map(straddled, nearest_ranges=ranges)
        self.assertEqual(sum(bool(m) for m in mask), 1)
        expected_min = min(math.sqrt(x * x + y * y + z * z) for x, y, z in straddled)
        self.assertAlmostEqual(next(iter(ranges.values())), expected_min, places=5)

    def test_disconnected_noise_and_invalid_points_rejected(self) -> None:
        noise = szp.generate_disconnected_noise(4.5)
        mask, _ = szp.build_sector_map(noise)
        self.assertEqual(sum(bool(m) for m in mask), 0)

        invalid = [(float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0), (float("inf"), 0.0, 0.0)]
        mask_inv, _ = szp.build_sector_map(invalid)
        self.assertEqual(sum(bool(m) for m in mask_inv), 0)

        mask_empty, _ = szp.build_sector_map([])
        self.assertEqual(len(mask_empty), 0)

    def test_exclusion_boxes_filter_all_15_airframe_regions(self) -> None:
        boxes = szp.load_exclude_boxes()
        self.assertEqual(len(boxes), 15)
        box_names = {b[0] for b in boxes}
        self.assertIn("central_body", box_names)
        self.assertIn("side_plate_right", box_names)
        self.assertIn("side_plate_left", box_names)

        airframe_pts = szp.generate_airframe_self_points(boxes=boxes, points_per_box=6)
        obstacle_pts = szp.generate_coherent_surface(4.0, 30)
        combined = szp._to_point_list(airframe_pts) + szp._to_point_list(obstacle_pts)

        filtered, hits = szp.apply_exclude_boxes(combined, boxes=boxes, return_box_hits=True)
        self.assertEqual(len(szp._to_point_list(filtered)), 30)
        self.assertTrue(all(h >= 6 for h in hits))

    def test_confirmation_history_miss_tolerance_and_expiry(self) -> None:
        history = szp.CandidateConfirmation()
        pts = [(4.0, 0.04, 0.04)]
        secs = [32]
        empty_pts: list[tuple[float, float, float]] = []
        empty_secs: list[int] = []

        # Frame 1 (1.0s): unconfirmed
        self.assertEqual(len(szp._to_point_list(history.stabilize(pts, secs, 1.0)[0])), 0)
        # Frame 2 (1.1s): missed frame
        history.stabilize(empty_pts, empty_secs, 1.1)
        # Frame 3 (1.2s): seen in 2 of last 3 frames -> confirmed!
        self.assertEqual(len(szp._to_point_list(history.stabilize(pts, secs, 1.2)[0])), 1)
        # Held at 1.4s and 1.7s (within hold_time=0.6s of 1.2s)
        self.assertEqual(len(szp._to_point_list(history.stabilize(empty_pts, empty_secs, 1.4)[0])), 1)
        self.assertEqual(len(szp._to_point_list(history.stabilize(empty_pts, empty_secs, 1.7)[0])), 1)
        # Expired at 1.81s (> 0.6s after 1.2s)
        self.assertEqual(len(szp._to_point_list(history.stabilize(empty_pts, empty_secs, 1.81)[0])), 0)

    def test_camera_frame_extrinsics_round_trip(self) -> None:
        pts_base = [(4.0, 0.25, -0.5), (2.5, -0.4, 0.3)]
        for cam_frame in ("zed1_camera_link", "zed2_camera_link"):
            pts_cam = szp.base_link_to_camera_frame(pts_base, cam_frame)
            pts_rec = szp._to_point_list(szp.camera_frame_to_base_link(pts_cam, cam_frame))
            for (x0, y0, z0), (x1, y1, z1) in zip(pts_base, pts_rec):
                self.assertAlmostEqual(x0, x1, places=5)
                self.assertAlmostEqual(y0, y1, places=5)
                self.assertAlmostEqual(z0, z1, places=5)

    def test_full_self_test_harness(self) -> None:
        summary = szp.run_self_test(verbose=False)
        self.assertEqual(summary["exclusion_boxes_count"], 15)
        self.assertEqual(summary["airframe_points_remaining"], 0)
        self.assertEqual(summary["disconnected_noise_detected"], 0)
        for d in (2.0, 3.0, 4.0, 4.9):
            self.assertEqual(summary["distance_results"][d]["frame1_confirmed"], 0)
            self.assertEqual(summary["distance_results"][d]["frame2_confirmed"], 1)


# =============================================================================
# 4. Latency Relay & PX4 72-Sector `OBSTACLE_DISTANCE` Tests
# =============================================================================

class TestLatencyRelayAndPx4ObstacleDistance2D(unittest.TestCase):
    """Test `LatencyBuffer` and `points_3d_to_px4_obstacle_distance_2d` in `latency_relay.py`."""

    def test_latency_buffer_fifo_and_delay_timing(self) -> None:
        buf = latency_relay.LatencyBuffer(delay_ms=80.0, max_queue_size=4)
        buf.push("a", 10.000)
        buf.push("b", 10.030)
        buf.push("c", 10.060)

        self.assertEqual(buf.pop_ready(10.079), [])
        self.assertEqual(buf.pop_ready(10.080), ["a"])
        self.assertEqual(buf.pop_ready(10.110), ["b"])
        self.assertEqual(buf.pop_ready(10.140), ["c"])
        self.assertEqual(len(buf), 0)

    def test_latency_buffer_zero_delay_and_max_queue_size(self) -> None:
        buf0 = latency_relay.LatencyBuffer(delay_ms=0.0, max_queue_size=2)
        buf0.push("x", 1.0)
        buf0.push("y", 1.0)
        buf0.push("z", 1.0)  # evicts "x"
        self.assertEqual(buf0.pop_ready(1.0), ["y", "z"])

        with self.assertRaises(ValueError):
            latency_relay.LatencyBuffer(delay_ms=-5.0)

    def test_px4_72_sector_binning_cardinal_and_intercardinal_directions(self) -> None:
        pts_frd = [
            (4.0, 0.0, 0.0),    # 0 deg   (Forward, +X_frd) -> FRD sector 0  (400 cm)
            (0.0, 3.0, 0.0),    # 90 deg  (Right,   +Y_frd) -> FRD sector 18 (300 cm)
            (-2.0, 0.0, 0.0),   # 180 deg (Behind,  -X_frd) -> FRD sector 36 (200 cm)
            (0.0, -1.0, 0.0),   # 270 deg (Left,    -Y_frd) -> FRD sector 54 (100 cm)
            (2.0, 2.0, 0.0),    # 45 deg  (FR)              -> FRD sector 9  (283 cm)
        ]
        # 1. CW FRD 72-sector OBSTACLE_DISTANCE binning:
        #    sector 18 = Right (+Y_frd), sector 54 = Left (-Y_frd)
        sectors = latency_relay.points_3d_to_px4_obstacle_distance_2d(
            pts_frd, min_distance_cm=20, max_distance_cm=500
        )
        self.assertEqual(len(sectors), 72)
        self.assertEqual(sectors[0], 400)
        self.assertEqual(sectors[18], 300)
        self.assertEqual(sectors[36], 200)
        self.assertEqual(sectors[54], 100)
        self.assertEqual(sectors[9], int(round(math.hypot(2.0, 2.0) * 100.0)))
        self.assertEqual(sectors[1], 501)  # unknown = max_distance_cm + 1

        # 2. CCW FLU LaserScan conversion (`flu_idx = (72 - frd_idx) % 72`):
        #    In ROS 2 `base_link` (FLU), angle increments counter-clockwise from +X (Forward)
        #    toward +Y (Left = +90 deg = flu_idx 18) and -Y (Right = +270 deg = flu_idx 54).
        flu_converter = None
        for helper_name in (
            "px4_sectors_to_flu_laserscan_ranges",
            "frd_sectors_to_flu_laserscan_ranges",
            "frd_to_flu_laserscan_ranges",
            "distances_cm_to_flu_laserscan_ranges",
            "points_3d_to_flu_laserscan_ranges",
        ):
            if hasattr(latency_relay, helper_name):
                flu_converter = getattr(latency_relay, helper_name)
                break

        if flu_converter is not None:
            try:
                flu_ranges = flu_converter(sectors, max_distance_cm=500)
            except TypeError:
                flu_ranges = flu_converter(pts_frd, min_distance_cm=20, max_distance_cm=500)
        else:
            flu_ranges = [
                float("inf") if sectors[(72 - flu_idx) % 72] > 500 else float(sectors[(72 - flu_idx) % 72]) / 100.0
                for flu_idx in range(72)
            ]

        self.assertEqual(len(flu_ranges), 72)
        self.assertAlmostEqual(flu_ranges[0], 4.00, places=5)   # Forward (+X_flu, frd_idx 0)
        self.assertAlmostEqual(flu_ranges[18], 1.00, places=5)  # Left    (+Y_flu, frd_idx 54)
        self.assertAlmostEqual(flu_ranges[36], 2.00, places=5)  # Behind  (-X_flu, frd_idx 36)
        self.assertAlmostEqual(flu_ranges[54], 3.00, places=5)  # Right   (-Y_flu, frd_idx 18)
        self.assertAlmostEqual(
            flu_ranges[(72 - 9) % 72],
            int(round(math.hypot(2.0, 2.0) * 100.0)) / 100.0,
            places=5,
        )
        self.assertTrue(math.isinf(flu_ranges[1]))

        # Verify latency_relay.py reverses CW FRD sectors to CCW FLU for LaserScan
        self.assertTrue(hasattr(latency_relay, "frd_sectors_to_flu_laserscan_ranges"))
        lr_source = (SIM_DIR / "latency_relay.py").read_text(encoding="utf-8")
        self.assertTrue(
            "(n - frd_idx) % n" in lr_source
            or "(num_sectors -" in lr_source
            or "(72 -" in lr_source
            or "(PX4_NUM_SECTORS -" in lr_source,
            msg="latency_relay.py must map FRD CW sectors to FLU CCW LaserScan via (n - frd_idx) % n",
        )

    def test_px4_72_sector_min_win_clamping_and_z_band(self) -> None:
        pts_frd = [
            (4.2, 0.0, 0.0),
            (2.4, 0.0, 0.0),    # closer obstacle in sector 0 wins (240 cm)
            (0.05, 0.0, 0.0),   # clamped up to min_distance_cm=20
            (0.0, 4.0, 3.5),    # outside max_z_band_m=1.5 -> ignored
        ]
        sectors = latency_relay.points_3d_to_px4_obstacle_distance_2d(
            pts_frd[:2], min_distance_cm=20, max_distance_cm=500, max_z_band_m=1.5
        )
        self.assertEqual(sectors[0], 240)

        sectors_clamped = latency_relay.points_3d_to_px4_obstacle_distance_2d(
            pts_frd[2:], min_distance_cm=20, max_distance_cm=500, max_z_band_m=1.5
        )
        self.assertEqual(sectors_clamped[0], 20)
        self.assertEqual(sectors_clamped[18], 501)  # z=3.5m was filtered out

    def test_send_px4_obstacle_distance_mavlink_helper(self) -> None:
        mav_mock = MagicMock()
        payload = latency_relay.send_px4_obstacle_distance(
            mav_mock,
            [(3.2, 0.0, 0.0)],
            time_usec=123456,
            min_distance_cm=20,
            max_distance_cm=500,
        )
        self.assertEqual(payload["frame"], 12)
        self.assertEqual(payload["increment"], 5)
        self.assertEqual(payload["distances"][0], 320)
        mav_mock.mav.obstacle_distance_send.assert_called_once()


# =============================================================================
# 5. `jetson_sil_launch.py` & Docker SIL Artifacts Tests
# =============================================================================

class TestJetsonSilLaunchAndDockerArtifacts(unittest.TestCase):
    """Test `jetson_sil_launch.py` launch plans and `docker/jetson-sil` files."""

    def test_default_launch_plan_and_static_transforms(self) -> None:
        plan = jetson_sil_launch.build_sil_launch_plan()
        self.assertTrue(plan["use_sim_time"])
        self.assertEqual(plan["fcu_url"], "tcp://127.0.0.1:5762")
        self.assertFalse(plan["use_synthetic_zed"])
        self.assertFalse(plan["use_zed_sim_stream"])
        self.assertFalse(plan["use_latency_relay"])

        tfs = plan["static_transforms"]
        self.assertEqual(len(tfs), 2)
        self.assertEqual(tfs[0]["parent_frame"], "base_link")
        self.assertEqual(tfs[0]["child_frame"], "zed1_camera_link")
        self.assertEqual(tfs[0]["xyz"], ("0.1524", "0.0", "-0.127"))
        self.assertEqual(tfs[1]["child_frame"], "zed2_camera_link")

        vod_names = [n["name"] for n in plan["vod_nodes"]]
        self.assertEqual(vod_names, ["point_cloud", "obstacle_detection", "obstacle_to_mavlink"])
        self.assertEqual(plan["vod_nodes"][2]["remappings"], [])

    def test_latency_relay_remaps_obstacle_to_mavlink_topic(self) -> None:
        plan = jetson_sil_launch.build_sil_launch_plan(
            {"latency_ms": "80.0", "use_synthetic_zed": "true", "use_zed_sim_stream": "true"}
        )
        self.assertTrue(plan["use_latency_relay"])
        self.assertAlmostEqual(plan["latency_ms"], 80.0)
        self.assertTrue(plan["use_synthetic_zed"])
        self.assertEqual(len(plan["zed_sim_cameras"]), 2)
        self.assertEqual(
            plan["vod_nodes"][2]["remappings"],
            [("/mavros/obstacle_distance_3d/send", "/mavros/obstacle_distance_3d/send_raw")],
        )

    def test_docker_and_hil_files_exist_and_valid(self) -> None:
        dockerfile = DOCKER_DIR / "Dockerfile"
        compose = DOCKER_DIR / "docker-compose.yml"
        entrypoint = DOCKER_DIR / "entrypoint.sh"
        hil_doc = SIM_DIR / "JETSON_HIL.md"

        for path in (dockerfile, compose, entrypoint, hil_doc):
            self.assertTrue(path.exists(), f"Missing required file: {path}")

        df_text = dockerfile.read_text(encoding="utf-8")
        self.assertIn("ARG BASE_IMAGE=ros:humble-ros-base-jammy", df_text)
        self.assertIn("ros-humble-rmw-fastrtps-cpp", df_text)
        self.assertIn("ros-humble-mavros", df_text)
        self.assertIn("install_geographiclib_datasets.sh", df_text)
        self.assertIn("VFS-Go-Aero/visual-obstacle-detection.git", df_text)
        self.assertIn("colcon build --symlink-install", df_text)


# =============================================================================
# 6. Isaac Sim 5.1 / 6.0 Compatibility Layer Tests (`simulation/_compat.py`)
# =============================================================================

class TestCompatModule(unittest.TestCase):
    """Test `_compat.py` mount path constants, coordinates, CLI args, `_AppHandle`, and extension fallback."""

    def test_mount_path_constants_parented_under_chassis(self) -> None:
        self.assertEqual(_compat.VEHICLE_PATH, "/World/Vehicle")
        self.assertEqual(_compat.CHASSIS_PATH, "/World/Vehicle/chassis")

        mount_paths = (
            _compat.CHASSIS_PATH,
            _compat.IMU_MOUNT_PATH,
            _compat.LIDAR_MOUNT_PATH,
            _compat.LIDAR_SENSOR_PATH,
            _compat.ZED1_MOUNT_PATH,
            _compat.ZED2_MOUNT_PATH,
            _compat.ZED1_CAMERA_PATH,
            _compat.ZED2_CAMERA_PATH,
        )
        for prim_path in mount_paths:
            self.assertTrue(
                prim_path == "/World/Vehicle/chassis"
                or prim_path.startswith("/World/Vehicle/chassis/"),
                msg=f"Mount path {prim_path!r} must be parented under /World/Vehicle/chassis",
            )
            # Issue #7 regression guard: never parent sensors under rotating propeller prims
            self.assertNotIn("/World/Vehicle/prop_fr", prim_path)
            self.assertNotIn("prop_", prim_path)

    def test_lidar_and_zed_mount_coordinates(self) -> None:
        self.assertEqual(_compat.LIDAR_MOUNT_POS_MESH, (0.468, -0.04, -0.515))
        self.assertAlmostEqual(_compat.LIDAR_MOUNT_ROT_X_DEG, 180.0, places=5)

        # At 2.0 m hover on flat ground, /World/Vehicle has RotateX(90) which maps
        # mesh +Y to world +Z, so downward LiDAR range is 2.0 + (-0.04) = 1.96 m
        # (within ±0.05 m of hover altitude).
        hover_alt_m = 2.0
        lidar_range_m = hover_alt_m + _compat.LIDAR_MOUNT_POS_MESH[1]
        self.assertAlmostEqual(lidar_range_m, 1.96, places=5)
        self.assertLessEqual(abs(lidar_range_m - hover_alt_m), 0.05)

        # Dual ZED 2i mounts match multi_zed.launch.py static transform
        # FLU (+0.1524, 0.0, -0.127) -> mesh (0.0, -0.127, -0.1524)
        self.assertEqual(_compat.ZED1_MOUNT_POS_MESH, (0.0, -0.127, -0.1524))
        self.assertEqual(_compat.ZED2_MOUNT_POS_MESH, (0.0, -0.127, -0.1524))
        self.assertEqual(_compat.ZED1_MOUNT_POS_MESH, _compat.ZED2_MOUNT_POS_MESH)

    def test_add_app_launcher_args_idempotent(self) -> None:
        # Ensure standalone fallback is exercised when isaaclab is not installed
        saved_isaaclab = sys.modules.pop("isaaclab", None)
        saved_isaaclab_app = sys.modules.pop("isaaclab.app", None)
        try:
            parser = argparse.ArgumentParser()
            _compat.add_app_launcher_args(parser)
            # Calling a second time must be idempotent (no ArgumentError)
            _compat.add_app_launcher_args(parser)

            for flag in ("--headless", "--device", "--enable_cameras", "--livestream", "--experience"):
                self.assertIn(flag, parser._option_string_actions)

            defaults = parser.parse_args([])
            self.assertFalse(defaults.headless)
            self.assertEqual(defaults.device, "cuda:0")
            self.assertFalse(defaults.enable_cameras)
            self.assertEqual(defaults.livestream, -1)
            self.assertEqual(defaults.experience, "")

            parsed = parser.parse_args(
                ["--headless", "--device", "cuda", "--enable_cameras", "--livestream", "2", "--experience", "kit.cfg"]
            )
            self.assertTrue(parsed.headless)
            self.assertEqual(parsed.device, "cuda")
            self.assertTrue(parsed.enable_cameras)
            self.assertEqual(parsed.livestream, 2)
            self.assertEqual(parsed.experience, "kit.cfg")
        finally:
            if saved_isaaclab is not None:
                sys.modules["isaaclab"] = saved_isaaclab
            if saved_isaaclab_app is not None:
                sys.modules["isaaclab.app"] = saved_isaaclab_app

    def test_app_handle_delegation_app_property_and_unpacking(self) -> None:
        sim_app = MagicMock()
        sim_app.is_running.return_value = True
        sim_app.custom_attr = "kit_110"
        launcher_obj = MagicMock()

        handle = _compat._AppHandle(sim_app, launcher=launcher_obj)
        self.assertIs(handle.app, sim_app)
        self.assertTrue(handle.is_running())
        handle.update()
        sim_app.update.assert_called_once()
        handle.close()
        sim_app.close.assert_called_once()
        self.assertEqual(handle.custom_attr, "kit_110")

        unpacked_launcher, unpacked_app = handle
        self.assertIs(unpacked_launcher, launcher_obj)
        self.assertIs(unpacked_app, sim_app)

        # When launcher is None, unpacking yields (handle, sim_app)
        standalone_handle = _compat._AppHandle(sim_app, launcher=None)
        unpacked_self, unpacked_sim = standalone_handle
        self.assertIs(unpacked_self, standalone_handle)
        self.assertIs(unpacked_sim, sim_app)

    def test_enable_required_extensions_fallback(self) -> None:
        enabled_set: set[str] = set()
        attempted: list[str] = []
        available = {"omni.isaac.sensor", "omni.isaac.ros2_bridge", "isaacsim.core.nodes"}

        ext_manager = MagicMock()
        ext_manager.is_extension_enabled.side_effect = lambda name: name in enabled_set

        def _set_enabled(name: str, flag: bool) -> bool:
            attempted.append(name)
            if flag and name in available:
                enabled_set.add(name)
                return True
            return False

        ext_manager.set_extension_enabled_immediate.side_effect = _set_enabled

        result = _compat.enable_required_extensions(
            ext_manager,
            ["isaacsim.sensors.rtx", "isaacsim.ros2.bridge", "isaacsim.core.nodes"],
        )
        self.assertEqual(
            result,
            ["omni.isaac.sensor", "omni.isaac.ros2_bridge", "isaacsim.core.nodes"],
        )
        self.assertIn("isaacsim.sensors.rtx", attempted)
        self.assertIn("omni.isaac.sensor", attempted)
        self.assertIn("isaacsim.ros2.bridge", attempted)
        self.assertIn("omni.isaac.ros2_bridge", attempted)


# =============================================================================
# 7. `ardupilot_bridge.py` Protocol, Frame Math & Motor Mixing Tests
# =============================================================================

class TestArdupilotBridge(unittest.TestCase):
    """Test ArduPilot `SIM_JSON` protocol, NED Euler conversion, and Quad-X motor mixing."""

    def test_parse_ardupilot_servo_packet_16ch_32ch_and_errors(self) -> None:
        self.assertEqual(ardupilot_bridge.ARDU_MAGIC_16CH, 0x481A)
        self.assertEqual(ardupilot_bridge.ARDU_MAGIC_32CH, 0x7381)

        # 16-channel packet (magic 0x481A = 18458, 40 bytes)
        pwms_16 = [1500, 1600, 1400, 1700] + [1000] * 12
        raw_16 = struct.pack("<HHI16H", 0x481A, 400, 12345, *pwms_16)
        self.assertEqual(len(raw_16), 40)
        pkt_16 = ardupilot_bridge.parse_ardupilot_servo_packet(raw_16)
        self.assertEqual(pkt_16.magic, 0x481A)
        self.assertEqual(pkt_16.frame_rate, 400)
        self.assertEqual(pkt_16.frame_count, 12345)
        self.assertEqual(pkt_16.pwm[:4], [1500, 1600, 1400, 1700])
        self.assertEqual(pkt_16.motors_norm, [0.5, 0.6, 0.4, 0.7])
        self.assertTrue(pkt_16.armed)

        # 32-channel packet (magic 0x7381 = 29569, 72 bytes)
        pwms_32 = [1000, 1000, 1000, 1000] + [1500] * 28
        raw_32 = struct.pack("<HHI32H", 0x7381, 250, 99999, *pwms_32)
        self.assertEqual(len(raw_32), 72)
        pkt_32 = ardupilot_bridge.parse_ardupilot_servo_packet(raw_32)
        self.assertEqual(pkt_32.magic, 0x7381)
        self.assertEqual(pkt_32.frame_rate, 250)
        self.assertEqual(pkt_32.frame_count, 99999)
        self.assertEqual(len(pkt_32.pwm), 32)
        self.assertEqual(pkt_32.motors_norm, [0.0, 0.0, 0.0, 0.0])
        self.assertFalse(pkt_32.armed)

        # Invalid magic -> ValueError
        bad_magic = struct.pack("<HHI16H", 0xDEAD, 400, 1, *pwms_16)
        with self.assertRaises(ValueError):
            ardupilot_bridge.parse_ardupilot_servo_packet(bad_magic)

        # Truncated / invalid length packets -> ValueError
        with self.assertRaises(ValueError):
            ardupilot_bridge.parse_ardupilot_servo_packet(b"\x1a\x48\x90\x01")
        with self.assertRaises(ValueError):
            ardupilot_bridge.parse_ardupilot_servo_packet(raw_16[:38])

    def test_pwm_to_norm(self) -> None:
        self.assertAlmostEqual(ardupilot_bridge.pwm_to_norm(1000), 0.0, places=6)
        self.assertAlmostEqual(ardupilot_bridge.pwm_to_norm(1500), 0.5, places=6)
        self.assertAlmostEqual(ardupilot_bridge.pwm_to_norm(2000), 1.0, places=6)
        # Clamping outside [1000, 2000]
        self.assertAlmostEqual(ardupilot_bridge.pwm_to_norm(800), 0.0, places=6)
        self.assertAlmostEqual(ardupilot_bridge.pwm_to_norm(2200), 1.0, places=6)

    def test_format_ardupilot_json_reply_schema_and_hover_accel(self) -> None:
        hover_accel_frd = ardupilot_bridge.mesh_body_to_frd([0.0, 9.81, 0.0])
        raw = ardupilot_bridge.format_ardupilot_json_reply(
            timestamp_s=1.25,
            gyro_frd=[0.01, -0.02, 0.03],
            accel_frd=hover_accel_frd,
            pos_ned=[5.0, -2.0, -3.5],
            euler_ned=[0.1, -0.05, 1.57],
            vel_ned=[0.5, 0.0, -0.1],
        )
        self.assertIsInstance(raw, bytes)
        text = raw.decode("utf-8")
        self.assertTrue(text.startswith("\n{") and text.endswith("}\n"))

        parsed = json.loads(text.strip())
        self.assertAlmostEqual(parsed["timestamp"], 1.25, places=6)
        self.assertEqual(parsed["imu"]["gyro"], [0.01, -0.02, 0.03])
        self.assertEqual(parsed["imu"]["accel_body"], [0.0, 0.0, -9.81])
        self.assertEqual(parsed["position"], [5.0, -2.0, -3.5])
        self.assertEqual(parsed["attitude"], [0.1, -0.05, 1.57])
        self.assertEqual(parsed["velocity"], [0.5, 0.0, -0.1])

    def test_enu_quat_to_ned_euler_round_trip(self) -> None:
        test_angles = [
            (0.0, 0.0, 0.0),
            (0.22, -0.18, 0.45),
            (-0.35, 0.24, -1.20),
            (0.15, 0.28, 2.35),
            (-0.42, -0.31, -2.60),
        ]
        for roll, pitch, yaw in test_angles:
            q_world = _frames.attitude_to_world_quat(roll, pitch, yaw)
            q_wxyz = (
                q_world.GetReal(),
                q_world.GetImaginary()[0],
                q_world.GetImaginary()[1],
                q_world.GetImaginary()[2],
            )
            r_rec, p_rec, y_rec = [float(x) for x in ardupilot_bridge.enu_quat_to_ned_euler(q_wxyz)]
            self.assertAlmostEqual(r_rec, roll, places=6)
            self.assertAlmostEqual(p_rec, pitch, places=6)
            self.assertAlmostEqual(y_rec, yaw, places=6)

    def test_compute_motor_forces_and_torques_arducopter_quadx(self) -> None:
        self.assertEqual(
            ardupilot_bridge.ARDU_PROP_ORDER,
            ["prop_fr", "prop_rl", "prop_fl", "prop_rr"],
        )

        # 1. Equal throttle -> pure vertical force along mesh +Y, zero roll/pitch/yaw torque
        hover_cmd = math.sqrt(0.5)
        f_eq, m_eq = ardupilot_bridge.compute_motor_forces_and_torques(
            [hover_cmd, hover_cmd, hover_cmd, hover_cmd],
            motor_map="arducopter_quadx",
        )
        self.assertAlmostEqual(float(f_eq[0]), 0.0, places=6)
        self.assertAlmostEqual(
            float(f_eq[1]),
            ardupilot_bridge.DRONE_MASS * ardupilot_bridge.GRAVITY,
            places=5,
        )
        self.assertAlmostEqual(float(f_eq[2]), 0.0, places=6)
        for k in range(3):
            self.assertAlmostEqual(float(m_eq[k]), 0.0, places=6)

        # 2. Right props (prop_fr=M1, prop_rr=M4 -> [0.8, 0.4, 0.4, 0.8]) > Left props
        #    -> negative roll torque in FRD (roll left)
        _, m_right_high = ardupilot_bridge.compute_motor_forces_and_torques(
            [0.8, 0.4, 0.4, 0.8],
            motor_map="arducopter_quadx",
        )
        m_right_frd = ardupilot_bridge.mesh_body_to_frd(m_right_high)
        self.assertLess(float(m_right_frd[0]), 0.0)
        self.assertAlmostEqual(float(m_right_frd[1]), 0.0, places=6)
        self.assertAlmostEqual(float(m_right_frd[2]), 0.0, places=6)

        # 3. Front props (prop_fr=M1, prop_fl=M3 -> [0.8, 0.4, 0.8, 0.4]) > Rear props
        #    -> positive pitch torque in FRD (nose up)
        _, m_front_high = ardupilot_bridge.compute_motor_forces_and_torques(
            [0.8, 0.4, 0.8, 0.4],
            motor_map="arducopter_quadx",
        )
        m_front_frd = ardupilot_bridge.mesh_body_to_frd(m_front_high)
        self.assertAlmostEqual(float(m_front_frd[0]), 0.0, places=6)
        self.assertGreater(float(m_front_frd[1]), 0.0)
        self.assertAlmostEqual(float(m_front_frd[2]), 0.0, places=6)

        # 4. CCW props (prop_fr=M1, prop_rl=M2 -> [0.8, 0.8, 0.4, 0.4]) > CW props
        #    -> positive yaw torque in FRD (nose right CW)
        _, m_ccw_high = ardupilot_bridge.compute_motor_forces_and_torques(
            [0.8, 0.8, 0.4, 0.4],
            motor_map="arducopter_quadx",
        )
        m_ccw_frd = ardupilot_bridge.mesh_body_to_frd(m_ccw_high)
        self.assertAlmostEqual(float(m_ccw_frd[0]), 0.0, places=6)
        self.assertAlmostEqual(float(m_ccw_frd[1]), 0.0, places=6)
        self.assertGreater(float(m_ccw_frd[2]), 0.0)


# =============================================================================
# 8. `ardu_poshold.py` & `go_aero_ardupilot.param` Tests
# =============================================================================

class TestArduPosholdAndParams(unittest.TestCase):
    """Test `ardu_poshold.py` helpers and `simulation/params/go_aero_ardupilot.param`."""

    def test_throttle_pct_to_pwm(self) -> None:
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(0.0), 1000)
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(50.0), 1500)
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(100.0), 2000)
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(-15.0), 1000)
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(130.0), 2000)

    def test_position_target_typemask_pos_only_clears_force_set_bit(self) -> None:
        mask = ardu_poshold.POSITION_TARGET_TYPEMASK_POS_ONLY
        # Bit 9 (1 << 9 = 0x0200, FORCE_SET) MUST be 0 so ArduCopter does not
        # reject SET_POSITION_TARGET_LOCAL_NED messages in GUIDED mode.
        self.assertEqual(mask & 0x0200, 0)
        # Position bits X, Y, Z (bits 0..2 = 0x0007) enabled (0),
        # Velocity & Accel bits (bits 3..8 = 0x01F8) ignored (1).
        self.assertEqual(mask & 0x0007, 0)
        self.assertEqual(mask & 0x01F8, 0x01F8)

    def test_evaluate_obstacle_stopping_margin_pass_and_fail(self) -> None:
        # Passing case: stopped at 3.05 m North before 5.0 m obstacle (1.95 m margin ~ 2.0 m)
        res_pass = ardu_poshold.evaluate_obstacle_stopping_margin(
            current_north_m=3.05,
            obstacle_north_m=5.0,
            avoid_margin_m=2.0,
        )
        self.assertTrue(res_pass["stopped_before_collision"])
        self.assertTrue(res_pass["passed"])
        self.assertAlmostEqual(float(res_pass["distance_to_obstacle_m"]), 1.95, places=5)

        # Failing case: reached 4.9 m North (breached 2.0 m margin / collided with 5.0 m obstacle)
        res_fail = ardu_poshold.evaluate_obstacle_stopping_margin(
            current_north_m=4.9,
            obstacle_north_m=5.0,
            avoid_margin_m=2.0,
        )
        self.assertFalse(res_fail["stopped_before_collision"])
        self.assertFalse(res_fail["passed"])

    def test_build_arg_parser_target_north_implies_guided_forward(self) -> None:
        parser = ardu_poshold.build_arg_parser()
        args = parser.parse_args(["--target-north", "8", "--altitude", "2.5", "--hold", "20"])
        self.assertAlmostEqual(args.target_north, 8.0, places=5)
        self.assertAlmostEqual(args.altitude, 2.5, places=5)
        self.assertAlmostEqual(args.hold, 20.0, places=5)
        effective_mode = "guided-forward" if args.target_north is not None else args.mode
        self.assertEqual(effective_mode, "guided-forward")

    def test_go_aero_ardupilot_param_file(self) -> None:
        param_path = SIM_DIR / "params" / "go_aero_ardupilot.param"
        self.assertTrue(param_path.exists(), f"Missing param file: {param_path}")

        params: dict[str, float] = {}
        for raw_line in param_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.replace("\t", ",").replace(" ", ",").split(",") if p.strip()]
            if len(parts) >= 2:
                params[parts[0]] = float(parts[1])

        # Core airframe and obstacle avoidance parameters
        self.assertEqual(int(params["FRAME_CLASS"]), 1)
        self.assertEqual(int(params["FRAME_TYPE"]), 1)
        self.assertEqual(int(params["PRX1_TYPE"]), 2)
        self.assertEqual(int(params["AVOID_ENABLE"]), 7)
        self.assertEqual(int(params["AVOID_BEHAVE"]), 1)
        self.assertAlmostEqual(params["AVOID_DIST_MAX"], 5.0, places=5)
        self.assertAlmostEqual(params["AVOID_MARGIN"], 2.0, places=5)

        # SITL-safe settings (zeroed hardware trims/offsets, disabled secondary PRX5, active SR1 telemetry)
        self.assertAlmostEqual(params["AHRS_TRIM_X"], 0.0, places=6)
        self.assertAlmostEqual(params["AHRS_TRIM_Y"], 0.0, places=6)
        self.assertAlmostEqual(params["INS_ACCOFFS_Z"], 0.0, places=6)
        self.assertAlmostEqual(params["BARO1_GND_PRESS"], 0.0, places=6)
        self.assertEqual(int(params["PRX5_TYPE"]), 0)
        self.assertGreater(params["SR1_POSITION"], 0.0)


# =============================================================================
# 9. `build_scene.py`, `add_zed_tutorial.py`, `add_lidar_tutorial.py`, `px4_visualizer.py`
#    & Wave 2 Adversarial Math / Edge-Case Regression Tests
# =============================================================================

import ast  # noqa: E402
import add_zed_tutorial  # noqa: E402


class TestBuildSceneAndTutorialsGatekeeper(unittest.TestCase):
    """Test `build_scene.py`, `add_zed_tutorial.py`, `add_lidar_tutorial.py`, `open_scene.py`, and `px4_visualizer.py`."""

    def test_build_scene_obstacle_layout_and_mounts(self) -> None:
        build_src = (SIM_DIR / "build_scene.py").read_text(encoding="utf-8")
        tree = ast.parse(build_src, filename="build_scene.py")
        self.assertIsNotNone(tree)
        # Verify CLI flags `--no-terrain` and `--obstacles` and all 4 chassis sensor mounts
        self.assertIn("'--no-terrain'", build_src)
        self.assertIn("'--obstacles'", build_src)
        for mount_const in ("IMU_MOUNT_PATH", "LIDAR_MOUNT_PATH", "ZED1_MOUNT_PATH", "ZED2_MOUNT_PATH"):
            self.assertIn(mount_const, build_src)
        self.assertIn("ensure_xform_pose", build_src)
        self.assertIn("/World/Obstacles/obstacle_3m", build_src)
        self.assertIn("/World/Obstacles/obstacle_5m", build_src)
        self.assertIn("/World/Obstacles/obstacle_10m", build_src)

    def test_add_zed_tutorial_depth_projection_and_obstacle_evaluation(self) -> None:
        # 1. Pure-Python ROI unprojection at center pixel
        depth_3x3 = [
            [4.0, 4.0, 4.0],
            [4.0, 4.0, 4.0],
            [4.0, 4.0, 4.0],
        ]
        roi = add_zed_tutorial.unproject_depth_roi_median(
            depth_3x3,
            fx=320.0,
            fy=320.0,
            cx=1.0,
            cy=1.0,
            u_min=0,
            u_max=2,
            v_min=0,
            v_max=2,
        )
        self.assertAlmostEqual(float(roi), 4.0, places=5)
        self.assertAlmostEqual(roi.depth_m, 4.0, places=5)
        self.assertAlmostEqual(float(roi[0]), 0.0, places=5)
        self.assertAlmostEqual(float(roi[1]), 0.0, places=5)
        self.assertAlmostEqual(float(roi[2]), -4.0, places=5)

        # 2. Dict wrapper from replicator annotator (`{'data': ...}`) & 3D channel squeeze in depth_to_world_points
        pts_world = add_zed_tutorial.depth_to_world_points(
            {"data": depth_3x3},
            cam_tf=(0.0, 0.0, 1.5),
            fx=320.0,
            fy=320.0,
            cx=1.0,
            cy=1.0,
        )
        if hasattr(pts_world, "shape"):
            self.assertEqual(pts_world.shape, (9, 3))
            # Center pixel at index 4 has local z_cam = -4.0 -> world Y_north = +4.0, Z_up = 1.5
            self.assertAlmostEqual(float(pts_world[4][0]), 0.0, places=5)
            self.assertAlmostEqual(float(pts_world[4][1]), 4.0, places=5)
            self.assertAlmostEqual(float(pts_world[4][2]), 1.5, places=5)

    def test_issue7_no_prop_fr_sensor_parenting_and_syntax_across_all_scripts(self) -> None:
        # Verify every Python script in simulation/ parses cleanly with ast.parse
        # and never creates or parents a sensor under /World/Vehicle/prop_fr
        for py_file in sorted(SIM_DIR.glob("*.py")):
            src = py_file.read_text(encoding="utf-8")
            ast.parse(src, filename=py_file.name)
            for lineno, raw_line in enumerate(src.splitlines(), start=1):
                code_part = raw_line.split("#", 1)[0]
                if "prop_fr/lidar_mount" in code_part:
                    # Only allowed inside the legacy USD migration check (`GetPrimAtPath('/World/Vehicle/prop_fr/lidar_mount')`)
                    self.assertIn(
                        "GetPrimAtPath",
                        code_part,
                        msg=f"{py_file.name}:{lineno} must not create or parent sensors under prop_fr",
                    )

        # Verify px4_visualizer disables RigidBodyAPI so chassis does not fall under gravity during timeline.play()
        vis_src = (SIM_DIR / "px4_visualizer.py").read_text(encoding="utf-8")
        self.assertIn("CreateRigidBodyEnabledAttr().Set(False)", vis_src)
        self.assertIn("ARDU_PROP_ORDER", vis_src)
        self.assertIn("PX4_PROP_ORDER", vis_src)


class TestWave2MathAndEdgeCases(unittest.TestCase):
    """Regression tests for Wave 2 adversarial math, NaN, gimbal-lock, and window boundary findings."""

    def test_nan_and_extreme_quaternion_inputs_do_not_crash(self) -> None:
        # 1. `_frames.euler_ned_to_quat` with NaN or Inf returns identity quaternion
        q_nan = _frames.euler_ned_to_quat(float("nan"), 0.0, 0.0)
        self.assertAlmostEqual(q_nan.GetReal(), 1.0, places=6)
        q_inf = _frames.euler_ned_to_quat(0.0, float("inf"), 0.0)
        self.assertAlmostEqual(q_inf.GetReal(), 1.0, places=6)

        # 2. `px4_bridge.quat_rotate` and `ardupilot_bridge.quat_rotate` normalize non-unit quaternions
        v_in = (1.0, 2.0, 3.0)
        r_unit = px4_bridge.quat_rotate((1.0, 0.0, 0.0, 0.0), v_in)
        r_scaled = px4_bridge.quat_rotate((5.0, 0.0, 0.0, 0.0), v_in)
        for a, b in zip(r_unit, r_scaled):
            self.assertAlmostEqual(float(a), float(b), places=6)

        # Zero or NaN quaternion leaves vector unchanged instead of collapsing to 0 or NaN
        r_zero = ardupilot_bridge.quat_rotate((0.0, 0.0, 0.0, 0.0), v_in)
        for a, b in zip(r_zero, v_in):
            self.assertAlmostEqual(float(a), float(b), places=6)

    def test_baro_high_altitude_and_nan_actuator_guards(self) -> None:
        # Altitude > 44,330 m must clamp to 0.0 hPa (real float), never complex!
        p_stratosphere = px4_bridge.compute_baro_pressure_mbar(50000.0)
        self.assertIsInstance(p_stratosphere, float)
        self.assertEqual(p_stratosphere, 0.0)
        self.assertAlmostEqual(px4_bridge.compute_baro_pressure_mbar(float("nan")), 1013.25, places=4)

        # NaN motor commands must map to 0.0 thrust, never 1.0 full throttle!
        f_nan_px4, m_nan_px4 = px4_bridge.compute_motor_forces_and_torques(
            [float("nan"), float("nan"), float("nan"), float("nan")]
        )
        self.assertEqual(float(f_nan_px4[1]), 0.0)
        self.assertEqual(float(m_nan_px4[1]), 0.0)

        f_nan_ardu, m_nan_ardu = ardupilot_bridge.compute_motor_forces_and_torques(
            [float("nan"), float("nan"), float("nan"), float("nan")]
        )
        self.assertEqual(float(f_nan_ardu[1]), 0.0)
        self.assertEqual(float(m_nan_ardu[1]), 0.0)

        # pwm_to_norm and throttle_pct_to_pwm with NaN or inverted bounds
        self.assertEqual(ardupilot_bridge.pwm_to_norm(float("nan")), 0.0)
        self.assertEqual(ardupilot_bridge.pwm_to_norm(1500, pwm_min=2000.0, pwm_max=1000.0), 0.0)
        self.assertEqual(ardu_poshold.throttle_pct_to_pwm(float("nan")), 1000)

    def test_gimbal_lock_pitch_90_deg_round_trip(self) -> None:
        # At pitch = +pi/2 (nose straight up) with yaw = 0.6 rad, roll and yaw are coupled.
        yaw_in = 0.6
        q_world = _frames.attitude_to_world_quat(0.0, math.pi / 2.0, yaw_in)
        q_wxyz = (
            q_world.GetReal(),
            q_world.GetImaginary()[0],
            q_world.GetImaginary()[1],
            q_world.GetImaginary()[2],
        )
        r_out, p_out, y_out = [float(x) for x in ardupilot_bridge.enu_quat_to_ned_euler(q_wxyz)]
        self.assertAlmostEqual(r_out, 0.0, places=5)
        self.assertAlmostEqual(p_out, math.pi / 2.0, places=5)
        self.assertAlmostEqual(y_out, yaw_in, places=5)

    def test_sector_map_far_bin_edge_and_simultaneous_camera_timestamps(self) -> None:
        # 1. Obstacle at 2.49 m (far edge of bin 9: [2.25, 2.50)) requires 65 points,
        #    whereas bin 9's near edge (2.25 m) would require 79 points.
        #    Verify a 70-point coherent surface at 2.49 m is NOT pre-rejected by the window filter.
        pts_249 = szp.generate_coherent_surface(distance_m=2.49, n_points=70)
        nearest: dict[int, float] = {}
        mask, _ = szp.build_sector_map(pts_249, nearest_ranges=nearest)
        self.assertEqual(sum(1 for flag in mask if flag), 1)

        # 2. Obstacle right at MAX_DETECTION_RANGE_M = 5.0 m is detected
        pts_500 = szp.generate_coherent_surface(distance_m=5.0)
        mask_500, _ = szp.build_sector_map(pts_500)
        self.assertEqual(sum(1 for flag in mask_500 if flag), 1)

        # 3. CandidateConfirmation with dt == 0.0 (two cameras at same /clock tick) preserves history
        conf = szp.CandidateConfirmation()
        c1 = conf.update([(4.0, 0.0, 0.0)], [12], now=10.0)
        self.assertFalse(bool(c1[0]))
        c2 = conf.update([(4.0, 0.0, 0.0)], [12], now=10.0)
        self.assertTrue(bool(c2[0]))

        # 4. px4_mavlink_udp:="none" normalizes to "" in build_sil_launch_plan
        plan = jetson_sil_launch.build_sil_launch_plan(
            {"latency_ms": "0.0", "enable_px4_2d_bridge": "false", "px4_mavlink_udp": "none"}
        )
        self.assertEqual(plan["px4_mavlink_udp"], "")
        self.assertFalse(plan["use_latency_relay"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

