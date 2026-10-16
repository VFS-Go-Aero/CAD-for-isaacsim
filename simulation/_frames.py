"""
Coordinate-frame helpers shared by px4_visualizer.py and px4_bridge.py.

Frames in play:
  PX4 NED body:       +X = nose, +Y = right wing, +Z = down
  PX4 NED world:      +X = north, +Y = east, +Z = down
  Isaac Sim ENU world:+X = east, +Y = north, +Z = up
  STL mesh local:     +X = right, +Y = up, +Z = back (so -Z = forward)
                      Rotated +90 deg around X at /World/Vehicle to sit Z-up in ENU.

Q_MESH_FIX   maps the Y-up STL mesh basis into the Z-up world basis. After
             applying it, mesh-forward (-Z) lands at world +Y (north), so an
             identity-attitude quaternion from PX4 already faces north.
Q_NED_ENU    rebases a NED-world quaternion to ENU-world via similarity
             transform (swaps X/Y and flips Z). This is its own inverse up to
             a sign, which is why Q_NED_ENU_INV is computed explicitly.
"""
import math

try:
    from pxr import Gf
except ImportError:
    class _FallbackGf:
        """Self-contained pure-Python fallback for `pxr.Gf` (`Vec3d`, `Quatd`, `Dot`)
        so `_frames.py` can be imported and tested in pure Python without USD/Kit.
        """

        class Vec3d:
            __slots__ = ('_v',)

            def __init__(self, x=0.0, y=0.0, z=0.0):
                if isinstance(x, (tuple, list)) or hasattr(x, '__iter__'):
                    vals = list(x)
                    self._v = (float(vals[0]), float(vals[1]), float(vals[2]))
                else:
                    self._v = (float(x), float(y), float(z))

            def __getitem__(self, idx):
                return self._v[idx]

            def __len__(self):
                return 3

            def __iter__(self):
                return iter(self._v)

            def __add__(self, other):
                return _FallbackGf.Vec3d(
                    self._v[0] + float(other[0]),
                    self._v[1] + float(other[1]),
                    self._v[2] + float(other[2]),
                )

            def __sub__(self, other):
                return _FallbackGf.Vec3d(
                    self._v[0] - float(other[0]),
                    self._v[1] - float(other[1]),
                    self._v[2] - float(other[2]),
                )

            def __neg__(self):
                return _FallbackGf.Vec3d(-self._v[0], -self._v[1], -self._v[2])

            def __mul__(self, scalar):
                s = float(scalar)
                return _FallbackGf.Vec3d(self._v[0] * s, self._v[1] * s, self._v[2] * s)

            def __rmul__(self, scalar):
                return self.__mul__(scalar)

            def GetLength(self):
                return math.hypot(self._v[0], self._v[1], self._v[2])

            def GetNormalized(self):
                length = self.GetLength()
                if not math.isfinite(length) or length <= 0.0:
                    return _FallbackGf.Vec3d(0.0, 0.0, 0.0)
                return _FallbackGf.Vec3d(
                    self._v[0] / length,
                    self._v[1] / length,
                    self._v[2] / length,
                )

            def __eq__(self, other):
                if not hasattr(other, '__getitem__') or len(other) != 3:
                    return False
                return (
                    abs(self._v[0] - float(other[0])) <= 1e-12
                    and abs(self._v[1] - float(other[1])) <= 1e-12
                    and abs(self._v[2] - float(other[2])) <= 1e-12
                )

            def __repr__(self):
                return f'Gf.Vec3d({self._v[0]!r}, {self._v[1]!r}, {self._v[2]!r})'

        class Quatd:
            __slots__ = ('_real', '_imag')

            def __init__(self, real=1.0, imag=None, y=None, z=None):
                self._real = float(real)
                if imag is None:
                    self._imag = _FallbackGf.Vec3d(0.0, 0.0, 0.0)
                elif y is not None and z is not None:
                    self._imag = _FallbackGf.Vec3d(float(imag), float(y), float(z))
                elif isinstance(imag, _FallbackGf.Vec3d):
                    self._imag = imag
                else:
                    self._imag = _FallbackGf.Vec3d(imag[0], imag[1], imag[2])

            def GetReal(self):
                return self._real

            def GetImaginary(self):
                return self._imag

            def GetConjugate(self):
                return _FallbackGf.Quatd(self._real, -self._imag)

            def GetInverse(self):
                x, y, z = self._imag
                norm = math.hypot(self._real, x, y, z)
                if not math.isfinite(norm) or norm <= 0.0:
                    return _FallbackGf.Quatd(1.0, _FallbackGf.Vec3d(0.0, 0.0, 0.0))
                w_u = self._real / norm
                x_u = x / norm
                y_u = y / norm
                z_u = z / norm
                return _FallbackGf.Quatd(
                    w_u / norm,
                    _FallbackGf.Vec3d(-x_u / norm, -y_u / norm, -z_u / norm),
                )

            def GetNormalized(self):
                x, y, z = self._imag
                norm = math.hypot(self._real, x, y, z)
                if not math.isfinite(norm) or norm <= 0.0:
                    return _FallbackGf.Quatd(1.0, _FallbackGf.Vec3d(0.0, 0.0, 0.0))
                return _FallbackGf.Quatd(
                    self._real / norm,
                    _FallbackGf.Vec3d(x / norm, y / norm, z / norm),
                )

            def Transform(self, vec):
                v_quat = _FallbackGf.Quatd(
                    0.0, _FallbackGf.Vec3d(vec[0], vec[1], vec[2])
                )
                res = self * v_quat * self.GetInverse()
                return res.GetImaginary()

            def __mul__(self, other):
                if isinstance(other, _FallbackGf.Quatd):
                    w1 = self._real
                    x1, y1, z1 = self._imag
                    w2 = other._real
                    x2, y2, z2 = other._imag
                    return _FallbackGf.Quatd(
                        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                        _FallbackGf.Vec3d(
                            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                        ),
                    )
                s = float(other)
                return _FallbackGf.Quatd(self._real * s, self._imag * s)

            def __eq__(self, other):
                if not isinstance(other, _FallbackGf.Quatd):
                    return False
                return (
                    abs(self._real - other._real) <= 1e-12
                    and self._imag == other._imag
                )

            def __repr__(self):
                x, y, z = self._imag
                return f'Gf.Quatd({self._real!r}, {x!r}, {y!r}, {z!r})'

        @staticmethod
        def Dot(a, b):
            return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])

    Gf = _FallbackGf


COS45 = math.sqrt(2) / 2
Q_MESH_FIX = Gf.Quatd(COS45, Gf.Vec3d(COS45, 0.0, 0.0))
Q_NED_ENU = Gf.Quatd(0.0, Gf.Vec3d(COS45, COS45, 0.0))
Q_NED_ENU_INV = Q_NED_ENU.GetInverse()


def euler_ned_to_quat(roll, pitch, yaw):
    """PX4 NED body Tait-Bryan (Z-Y-X) euler -> rotation quaternion."""
    r = float(roll) if math.isfinite(float(roll)) else 0.0
    p = float(pitch) if math.isfinite(float(pitch)) else 0.0
    y = float(yaw) if math.isfinite(float(yaw)) else 0.0
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return Gf.Quatd(
        cr * cp * cy + sr * sp * sy,
        Gf.Vec3d(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ),
    )


def attitude_to_world_quat(roll, pitch, yaw):
    """
    Combine PX4 NED body attitude with the NED->ENU rebase and the mesh-fix
    rotation so the result can be set directly on xformOp:orient and produces
    the correct visual orientation in Isaac Sim.
    """
    q_body_ned = euler_ned_to_quat(roll, pitch, yaw)
    q_body_enu = Q_NED_ENU * q_body_ned * Q_NED_ENU_INV
    return q_body_enu * Q_MESH_FIX


def enu_quat_to_ned_body(q_enu_world):
    """
    Inverse of attitude_to_world_quat's core rebase: take an ENU-world
    orientation of the body (as produced by PhysX / xformOp:orient with the
    mesh fix baked in) and return the equivalent NED-world body quaternion
    expected by PX4.

    q_enu_world is a Gf.Quatd whose axes are ENU. We unwind the mesh fix and
    rebase back into NED.
    """
    q_body_enu = q_enu_world * Q_MESH_FIX.GetInverse()
    q_body_ned = Q_NED_ENU_INV * q_body_enu * Q_NED_ENU
    return q_body_ned


def vec_enu_to_ned(v):
    """ENU (east, north, up) -> NED (north, east, down)."""
    return (v[1], v[0], -v[2])


def vec_ned_to_enu(v):
    """NED (north, east, down) -> ENU (east, north, up)."""
    return (v[1], v[0], -v[2])
