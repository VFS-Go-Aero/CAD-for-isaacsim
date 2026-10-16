#!/usr/bin/env python3
"""
Build Go Aero eVTOL scene USD with physics and sensor mounts.

Creates:
- Chassis with RigidBody, Collision, ArticulationRoot
- 4 propellers with revolute joints and velocity drives
- Sensor mounts on the rigid chassis:
    * /World/Vehicle/chassis/chassis_IMU
    * /World/Vehicle/chassis/lidar_mount  (Issue #7: fixed to chassis, not prop_fr)
    * /World/Vehicle/chassis/zed1_mount   (Issue #3: forward-facing ZED 2i #1)
    * /World/Vehicle/chassis/zed2_mount   (Issue #3: forward-facing ZED 2i #2)
- Ground plane, lighting, physics scene
- Optional rolling hills terrain (disable with `--no-terrain` for flat-ground tests)
- Optional textured 3D obstacles at 3 m, 5 m, and 10 m ahead (`--obstacles`, works
  cleanly with or without `--no-terrain`)

Usage:
  python build_scene.py --headless
  python build_scene.py --headless --no-terrain
  python build_scene.py --headless --no-terrain --obstacles
  # Then: python open_scene.py --device cuda
"""
import argparse
import os
import sys

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _compat import (  # noqa: E402
    CHASSIS_PATH,
    IMU_MOUNT_PATH,
    LIDAR_MOUNT_PATH,
    LIDAR_MOUNT_POS_MESH,
    LIDAR_MOUNT_ROT_X_DEG,
    VEHICLE_PATH,
    ZED1_MOUNT_PATH,
    ZED1_MOUNT_POS_MESH,
    ZED2_MOUNT_PATH,
    ZED2_MOUNT_POS_MESH,
    add_app_launcher_args,
    ensure_xform_pose,
    launch_app,
)

parser = argparse.ArgumentParser(description='Build Go Aero eVTOL USD scene')
parser.add_argument(
    '--no-terrain',
    action='store_true',
    help='Skip creating /World/Terrain so flat-ground LiDAR altitude tests hit '
         '/World/GroundPlane at z=0.0 (Issue #7). Can be combined with --obstacles.',
)
parser.add_argument(
    '--obstacles',
    action='store_true',
    help='Add textured 3D obstacles under /World/Obstacles at 3m (0, 3, 1.5), '
         '5m (0, 5, 1.5), and 10m (2, 10, 2.0) ahead along world +Y (north) '
         'for ZED 2i stereo depth & point cloud tests (Issue #3). Works cleanly '
         'alongside --no-terrain.',
)
add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = launch_app(args)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
import trimesh  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt  # noqa: E402

try:
    import isaaclab.sim as sim_utils
except (ImportError, ModuleNotFoundError):
    sim_utils = None

app = omni.kit.app.get_app()
stage = omni.usd.get_context().get_stage()

MESH_DIR = os.path.expanduser('~/go_aero/meshes')


# ============================================================
# Helper: load STL and create mesh prim
# ============================================================
def add_mesh(stage, prim_path, stl_file, color=(0.5, 0.5, 0.5)):
    stl = trimesh.load(os.path.join(MESH_DIR, stl_file))
    verts = stl.vertices  # already in meters

    m = UsdGeom.Mesh.Define(stage, prim_path)
    m.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in verts]))
    m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(stl.faces.flatten().tolist()))
    m.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(stl.faces)))
    m.GetSubdivisionSchemeAttr().Set('none')
    m.GetDoubleSidedAttr().Set(True)
    m.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    norms = Vt.Vec3fArray([Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])) for n in stl.vertex_normals])
    m.GetNormalsAttr().Set(norms)
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    mins = verts.min(axis=0)
    maxs = verts.max(axis=0)
    m.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*mins), Gf.Vec3f(*maxs)]))

    return stl


# ============================================================
# Scene setup
# ============================================================
print('[BUILD] Setting up scene...')
physics_scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)

if sim_utils is not None:
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func('/World/GroundPlane', cfg)
    light_cfg = sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
    light_cfg.func('/World/Light', light_cfg)
    dome_cfg = sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0)
    dome_cfg.func('/World/SkyLight', dome_cfg)
else:
    # Standalone Isaac Sim 5.1 / 6.0 fallback when IsaacLab is not installed
    gp_xform = UsdGeom.Xform.Define(stage, '/World/GroundPlane')
    gp_mesh = UsdGeom.Mesh.Define(stage, '/World/GroundPlane/geom')
    half = 50.0
    gp_mesh.GetPointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-half, -half, 0.0),
        Gf.Vec3f(half, -half, 0.0),
        Gf.Vec3f(half, half, 0.0),
        Gf.Vec3f(-half, half, 0.0),
    ]))
    gp_mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 0, 2, 3]))
    gp_mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3, 3]))
    gp_mesh.GetDoubleSidedAttr().Set(True)
    UsdPhysics.CollisionAPI.Apply(gp_mesh.GetPrim())

    distant_light = UsdLux.DistantLight.Define(stage, '/World/Light')
    distant_light.CreateIntensityAttr(3000.0)
    distant_light.CreateColorAttr(Gf.Vec3f(0.75, 0.75, 0.75))

    dome_light = UsdLux.DomeLight.Define(stage, '/World/SkyLight')
    dome_light.CreateIntensityAttr(1000.0)
    dome_light.CreateColorAttr(Gf.Vec3f(0.13, 0.13, 0.13))


# ============================================================
# Uneven terrain mesh for LiDAR testing
# ============================================================
# Generate a heightfield using sine/cosine waves. The mesh is
# centered at the origin and sized to cover the flight area.
# The ground plane remains underneath as a safety net.
# Pass --no-terrain to omit /World/Terrain so downward LiDAR rays hit
# the flat /World/GroundPlane at z = 0.0 (Issue #7 acceptance test).

def add_terrain(stage, prim_path, size=20.0, resolution=100, amplitude=0.5):
    """Create a procedural terrain mesh with rolling hills."""
    x = np.linspace(-size / 2, size / 2, resolution)
    z = np.linspace(-size / 2, size / 2, resolution)
    xx, zz = np.meshgrid(x, z)

    # Combine several frequencies for natural-looking terrain
    heights = (
        amplitude * np.sin(xx * 0.8) * np.cos(zz * 0.6)
        + (amplitude * 0.5) * np.sin(xx * 1.5 + 1.0)
        + (amplitude * 0.3) * np.cos(zz * 2.0 + 0.5)
    )

    # Build vertices (X, Y, Z) in world Z-up frame
    verts = []
    for i in range(resolution):
        for j in range(resolution):
            verts.append(Gf.Vec3f(float(xx[i, j]), float(zz[i, j]), float(heights[i, j])))

    # Build triangle faces from the grid
    faces = []
    for i in range(resolution - 1):
        for j in range(resolution - 1):
            v0 = i * resolution + j
            v1 = v0 + 1
            v2 = v0 + resolution
            v3 = v2 + 1
            faces.extend([v0, v1, v2, v1, v3, v2])
    num_faces = (resolution - 1) * (resolution - 1) * 2

    m = UsdGeom.Mesh.Define(stage, prim_path)
    m.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces))
    m.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * num_faces))
    m.GetSubdivisionSchemeAttr().Set('none')
    m.GetDoubleSidedAttr().Set(True)
    m.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.45, 0.55, 0.35)]))

    UsdPhysics.CollisionAPI.Apply(m.GetPrim())


if not args.no_terrain:
    add_terrain(stage, '/World/Terrain', size=20.0, resolution=100, amplitude=0.5)
    print('[BUILD] Added uneven terrain mesh')
else:
    print('[BUILD] Skipping /World/Terrain (--no-terrain): flat ground at z=0.0')


# ============================================================
# Optional textured 3D obstacles for ZED stereo & LiDAR testing (--obstacles)
# ============================================================
def _add_checkerboard_box_mesh(
    stage,
    prim_path,
    center_xyz,
    size_xyz,
    color_a=(0.85, 0.32, 0.18),
    color_b=(0.18, 0.22, 0.28),
    subdiv_u=12,
    subdiv_v=18,
    window_uv=None,
):
    """Create a subdivided 3D box mesh with per-tile checkerboard vertex/face
    colors and collision API so both stereo disparity (ZED 2i) and raycast /
    depth sensors get rich visual features and dense 3D returns.

    If `window_uv` is provided as `(u_min, u_max, v_min, v_max)` in normalized
    `[0, 1]` face coordinates, tiles inside that window on the front (`-Y`) and
    back (`+Y`) faces are left open so a camera looking through the aperture has
    an unobstructed sightline to obstacles further downrange while preserving
    the exact outer box dimensions and dense returns around the frame.
    """
    cx, cy, cz = (float(center_xyz[0]), float(center_xyz[1]), float(center_xyz[2]))
    sx, sy, sz = (float(size_xyz[0]), float(size_xyz[1]), float(size_xyz[2]))
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    xform = ensure_xform_pose(stage, prim_path, (cx, cy, cz))

    mesh_prim_path = f'{prim_path}/mesh'
    m = UsdGeom.Mesh.Define(stage, mesh_prim_path)

    verts = []
    faces = []
    face_colors = []

    def add_subdivided_quad(p00, p10, p11, p01, nu, nv, allow_window=False):
        for iu in range(nu):
            u0 = iu / float(nu)
            u1 = (iu + 1) / float(nu)
            uc = 0.5 * (u0 + u1)
            for iv in range(nv):
                v0 = iv / float(nv)
                v1 = (iv + 1) / float(nv)
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
                v_00 = bilerp(u0, v0)
                v_10 = bilerp(u1, v0)
                v_11 = bilerp(u1, v1)
                v_01 = bilerp(u0, v1)
                verts.extend([
                    Gf.Vec3f(float(v_00[0]), float(v_00[1]), float(v_00[2])),
                    Gf.Vec3f(float(v_10[0]), float(v_10[1]), float(v_10[2])),
                    Gf.Vec3f(float(v_11[0]), float(v_11[1]), float(v_11[2])),
                    Gf.Vec3f(float(v_01[0]), float(v_01[1]), float(v_01[2])),
                ])
                faces.extend([base, base + 1, base + 2, base, base + 2, base + 3])
                col = color_a if ((iu + iv) % 2 == 0) else color_b
                c_vec = Gf.Vec3f(float(col[0]), float(col[1]), float(col[2]))
                face_colors.extend([c_vec, c_vec])

    # 6 box faces in local coordinates [-hx..hx, -hy..hy, -hz..hz]
    c000 = np.array([-hx, -hy, -hz])
    c100 = np.array([hx, -hy, -hz])
    c110 = np.array([hx, hy, -hz])
    c010 = np.array([-hx, hy, -hz])
    c001 = np.array([-hx, -hy, hz])
    c101 = np.array([hx, -hy, hz])
    c111 = np.array([hx, hy, hz])
    c011 = np.array([-hx, hy, hz])

    # Front face (-Y, facing the vehicle at origin looking along +Y)
    add_subdivided_quad(c000, c100, c101, c001, subdiv_u, subdiv_v, allow_window=True)
    # Back face (+Y)
    add_subdivided_quad(c110, c010, c011, c111, subdiv_u, subdiv_v, allow_window=True)
    # Left face (-X)
    add_subdivided_quad(c010, c000, c001, c011, max(4, subdiv_u // 2), subdiv_v)
    # Right face (+X)
    add_subdivided_quad(c100, c110, c111, c101, max(4, subdiv_u // 2), subdiv_v)
    # Top face (+Z)
    add_subdivided_quad(c001, c101, c111, c011, subdiv_u, max(4, subdiv_u // 2))
    # Bottom face (-Z)
    add_subdivided_quad(c010, c110, c100, c000, subdiv_u, max(4, subdiv_u // 2))

    num_tris = len(faces) // 3
    m.GetPointsAttr().Set(Vt.Vec3fArray(verts))
    m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(faces))
    m.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * num_tris))
    m.GetSubdivisionSchemeAttr().Set('none')
    m.GetDoubleSidedAttr().Set(True)
    m.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)]))

    primvars_api = UsdGeom.PrimvarsAPI(m)
    color_primvar = primvars_api.CreatePrimvar(
        'displayColor', Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform
    )
    color_primvar.Set(Vt.Vec3fArray(face_colors))

    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    return xform


def add_obstacles(stage):
    """Add 3 textured 3D obstacles under /World/Obstacles along world +Y (north):
      - Obstacle 1 at 3.0 m ahead: (0.0, 3.0, 1.5), box 1.0 x 0.4 x 3.0 m
      - Obstacle 2 at 5.0 m ahead: (0.0, 5.0, 1.5), box 1.2 x 0.4 x 3.0 m
      - Obstacle 3 at 10.0 m ahead: (2.0, 10.0, 2.0), box 1.5 x 0.5 x 4.0 m
    """
    UsdGeom.Xform.Define(stage, '/World/Obstacles')

    # Obstacle 1 at 3.0 m ahead: includes a central sightline aperture so a
    # forward-facing camera at (0, -0.127, -0.1524) simultaneously gets dense
    # returns on Obstacle 1 at 3.0 m AND sees Obstacle 2 directly behind it at 5.0 m.
    _add_checkerboard_box_mesh(
        stage,
        '/World/Obstacles/obstacle_3m',
        center_xyz=(0.0, 3.0, 1.5),
        size_xyz=(1.0, 0.4, 3.0),
        color_a=(0.90, 0.35, 0.15),
        color_b=(0.20, 0.25, 0.30),
        window_uv=(0.28, 0.72, 0.35, 0.75),
    )

    # Obstacle 2 at 5.0 m ahead
    _add_checkerboard_box_mesh(
        stage,
        '/World/Obstacles/obstacle_5m',
        center_xyz=(0.0, 5.0, 1.5),
        size_xyz=(1.2, 0.4, 3.0),
        color_a=(0.20, 0.75, 0.35),
        color_b=(0.15, 0.20, 0.25),
    )

    # Obstacle 3 at 10.0 m ahead (offset +2.0 m in X / east)
    _add_checkerboard_box_mesh(
        stage,
        '/World/Obstacles/obstacle_10m',
        center_xyz=(2.0, 10.0, 2.0),
        size_xyz=(1.5, 0.5, 4.0),
        color_a=(0.25, 0.55, 0.90),
        color_b=(0.18, 0.18, 0.24),
    )


if args.obstacles:
    add_obstacles(stage)
    print('[BUILD] Added textured obstacles at 3.0m, 5.0m, and 10.0m under /World/Obstacles')


# ============================================================
# Vehicle root — Y-up to Z-up rotation, start above ground
# ============================================================
vehicle = ensure_xform_pose(stage, VEHICLE_PATH, (0.0, 0.0, 1.5), rotate_x_deg=90.0)

# Mark as articulation root
UsdPhysics.ArticulationRootAPI.Apply(vehicle.GetPrim())

# ============================================================
# Chassis — rigid body with collision
# ============================================================
print('[BUILD] Adding chassis...')
chassis_xform = UsdGeom.Xform.Define(stage, CHASSIS_PATH)
chassis_xform.ClearXformOpOrder()
chassis_prim = chassis_xform.GetPrim()

# Visual mesh
add_mesh(stage, f'{CHASSIS_PATH}/mesh', 'frame.stl', color=(0.35, 0.35, 0.4))

# Rigid body
UsdPhysics.RigidBodyAPI.Apply(chassis_prim)
mass_api = UsdPhysics.MassAPI.Apply(chassis_prim)
mass_api.CreateMassAttr(5.0)  # 5 kg drone
mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0, -0.1, 0))  # slightly below center
# Diagonal inertia tensor for a 5 kg, roughly 1 m x 1 m x 0.3 m airframe.
# Approximating as a thin plate: Ixx = Iyy ~= (1/12) m (w^2 + h^2),
# Izz ~= (1/12) m (w^2 + d^2). Values chosen round and slightly conservative
# so PX4's attitude loop converges quickly; tune against real vehicle later.
mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.5, 0.5, 0.8))

# Box collider for chassis (simplified). Scale is in the mesh Y-up frame, so
# (X, Y, Z) ~ (width, height, depth). Keep it conservative - slightly smaller
# than the visual hull so props and arms never clip into it.
collider = UsdGeom.Cube.Define(stage, f'{CHASSIS_PATH}/collider')
collider.CreateSizeAttr(1.0)
collider.ClearXformOpOrder()
collider.AddScaleOp().Set(Gf.Vec3f(0.9, 0.25, 1.0))
collider.AddTranslateOp().Set(Gf.Vec3d(0, -0.15, 0))
UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
collider.GetPurposeAttr().Set('guide')  # invisible collider

# ============================================================
# IMU mount point — child Xform of the chassis rigid body.
# The IsaacLab IMU sensor reads root body state via this prim, so it must
# live under the chassis. Orientation matches the chassis body frame (after
# the vehicle's RotateXOp(90)), so sensor outputs are already in FRD-ish
# body frame. px4_bridge.py attaches the IMUSensor at runtime.
# ============================================================
imu_xform = ensure_xform_pose(stage, IMU_MOUNT_PATH, (0.0, 0.0, 0.0))

# ============================================================
# LiDAR mount point (Issue #7) — child Xform of the rigid chassis.
#
# IMPORTANT: The LiDAR mount MUST be parented to `/World/Vehicle/chassis`
# (`/World/Vehicle/chassis/lidar_mount`), NEVER to `/World/Vehicle/prop_fr`.
# `prop_fr` is a revolute-jointed rigid body that spins at up to ~1500 deg/s
# every physics step; parenting a sensor to `prop_fr` caused the sensor frame
# to spin at motor RPM.
#
# In the STL mesh frame (+X=right, +Y=up, -Z=forward):
#   translation = (0.468, -0.04, -0.515)
# places the mount directly beneath the front-right arm (`X=0.468, Z=-0.515`),
# 4 cm below the chassis origin (`Y=-0.04`), clear of the chassis collider and
# spinning propeller blades while keeping the downward range to flat ground
# (`z=0.0`) at `hover_z - 0.04 m` (within ±5 cm of vehicle hover altitude).
# Combined with `/World/Vehicle`'s `RotateX(90)`, `RotateX(180)` on `lidar_mount`
# flips the sensor's local `+Y` axis to point straight down (`-Z` world) and
# local `+Z` to point forward (`+Y` world), orienting `Example_Rotary`'s XY scan
# plane vertically to measure ground range directly beneath the arm.
# ============================================================
lidar_mount = ensure_xform_pose(
    stage,
    LIDAR_MOUNT_PATH,
    LIDAR_MOUNT_POS_MESH,
    rotate_x_deg=LIDAR_MOUNT_ROT_X_DEG,
)
print(f'[BUILD] Added lidar_mount under chassis at {LIDAR_MOUNT_PATH}')

# ============================================================
# Stereolabs ZED 2i camera mount points (Issue #3) — child Xforms of chassis.
#
# Matches the static transform in `multi_zed.launch.py` from `base_link` (FLU)
# to `zed1_camera_link` / `zed2_camera_link`:
#   FLU: (+0.1524 m forward, 0.0 m left, -0.127 m up)
#   STL mesh frame (+X=right, +Y=up, -Z=forward): (0.0, -0.127, -0.1524)
# Because `/World/Vehicle` has `RotateX(90)`, a child Xform of `chassis` with
# identity rotation has local +Y pointing along world +Z (up) and local -Z
# pointing along world +Y (north / forward), so USD cameras mounted here are
# horizontal to the ground plane and look forward along world +Y.
# ============================================================
zed1_mount = ensure_xform_pose(stage, ZED1_MOUNT_PATH, ZED1_MOUNT_POS_MESH)
print(f'[BUILD] Added zed1_mount under chassis at {ZED1_MOUNT_PATH}')

zed2_mount = ensure_xform_pose(stage, ZED2_MOUNT_PATH, ZED2_MOUNT_POS_MESH)
print(f'[BUILD] Added zed2_mount under chassis at {ZED2_MOUNT_PATH}')

# ============================================================
# Propellers — revolute joints with velocity drives
# ============================================================
# Prop positions in the Y-up frame (before vehicle rotation)
# The Y-axis in the STL frame = spin axis for props
props = {
    'prop_fr': {'file': 'FR.stl', 'pos': (0.468, 0.048, -0.515)},
    'prop_fl': {'file': 'FL.stl', 'pos': (-0.468, 0.048, -0.515)},
    'prop_rr': {'file': 'BR.stl', 'pos': (0.468, 0.048, 0.515)},
    'prop_rl': {'file': 'BL.stl', 'pos': (-0.468, 0.048, 0.515)},
}

for name, info in props.items():
    print(f'[BUILD] Adding {name}...')

    # Prop xform
    prop_xform = ensure_xform_pose(stage, f'{VEHICLE_PATH}/{name}', info['pos'])
    prop_prim = prop_xform.GetPrim()

    # Visual mesh
    add_mesh(stage, f'{VEHICLE_PATH}/{name}/mesh', info['file'], color=(0.12, 0.12, 0.12))

    # Rigid body for prop
    UsdPhysics.RigidBodyAPI.Apply(prop_prim)
    prop_mass = UsdPhysics.MassAPI.Apply(prop_prim)
    prop_mass.CreateMassAttr(0.1)  # 100g per prop

    # Revolute joint — spin around Y axis (which becomes Z after vehicle rotation)
    joint_path = f'{VEHICLE_PATH}/joints/{name}_joint'
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr('Y')
    joint.CreateBody0Rel().SetTargets([Sdf.Path(CHASSIS_PATH)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(f'{VEHICLE_PATH}/{name}')])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*info['pos']))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # Velocity drive — allows setting target angular velocity
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'angular')
    drive.CreateTypeAttr('force')
    drive.CreateDampingAttr(0.5)
    drive.CreateStiffnessAttr(0.0)  # velocity drive: stiffness=0, damping>0
    drive.CreateMaxForceAttr(100.0)
    # Default target velocity = 0 (will be set at runtime)
    drive.CreateTargetVelocityAttr(0.0)

print('[BUILD] Physics setup complete.')

# Let stage compose
for _ in range(30):
    app.update()

# ============================================================
# Save
# ============================================================
out = os.path.expanduser('~/go_aero/vehicle_scene.usd')
stage.GetRootLayer().Export(out)
print(f'[BUILD] Saved: {out} ({os.path.getsize(out)} bytes)')
simulation_app.close()
