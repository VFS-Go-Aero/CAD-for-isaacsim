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

from pxr import Gf


COS45 = math.sqrt(2) / 2
Q_MESH_FIX = Gf.Quatd(COS45, Gf.Vec3d(COS45, 0.0, 0.0))
Q_NED_ENU = Gf.Quatd(0.0, Gf.Vec3d(COS45, COS45, 0.0))
Q_NED_ENU_INV = Q_NED_ENU.GetInverse()


def euler_ned_to_quat(roll, pitch, yaw):
    """PX4 NED body Tait-Bryan (Z-Y-X) euler -> rotation quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
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
