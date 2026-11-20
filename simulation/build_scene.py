#!/usr/bin/env python3
"""
Build Go Aero eVTOL scene USD with physics.

Creates:
- Chassis with RigidBody, Collision, ArticulationRoot
- 4 propellers with revolute joints and velocity drives
- Ground plane, lighting, physics scene

Usage:
  python build_scene.py --headless
  # Then: python open_scene.py --device cuda
"""
import os
import shutil
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
# Isaac Lab 3 no longer registers --headless (headless unless --viz kit)
# but still honors a 'headless' launcher arg. Register it before
# AppLauncher so `python build_scene.py --headless` is accepted.
parser.add_argument('--headless', action='store_true', help='Run without a viewport')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import trimesh
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, Vt, PhysxSchema, Sdf
import isaaclab.sim as sim_utils

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

# Isaac Lab 3 reads the stage from a thread-local context. The Kit stage
# is not visible to GroundPlaneCfg / light spawners until it is bound.
with sim_utils.use_stage(stage):
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func('/World/GroundPlane', cfg)
    light_cfg = sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
    light_cfg.func('/World/Light', light_cfg)
    dome_cfg = sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0)
    dome_cfg.func('/World/SkyLight', dome_cfg)

# ============================================================
# Uneven terrain mesh for LiDAR testing
# ============================================================
# Generate a heightfield using sine/cosine waves. The mesh is
# centered at the origin and sized to cover the flight area.
# The ground plane remains underneath as a safety net.

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

add_terrain(stage, '/World/Terrain', size=20.0, resolution=100, amplitude=0.5)
print('[BUILD] Added uneven terrain mesh')

# ============================================================
# Vehicle root — Y-up to Z-up rotation, start above ground
# ============================================================
vehicle = UsdGeom.Xform.Define(stage, '/World/Vehicle')
vehicle.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.5))
vehicle.AddRotateXOp().Set(90.0)

# Mark as articulation root
UsdPhysics.ArticulationRootAPI.Apply(vehicle.GetPrim())

# ============================================================
# Chassis — rigid body with collision
# ============================================================
print('[BUILD] Adding chassis...')
chassis_xform = UsdGeom.Xform.Define(stage, '/World/Vehicle/chassis')
chassis_prim = chassis_xform.GetPrim()

# Visual mesh
add_mesh(stage, '/World/Vehicle/chassis/mesh', 'frame.stl', color=(0.35, 0.35, 0.4))

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
collider = UsdGeom.Cube.Define(stage, '/World/Vehicle/chassis/collider')
collider.CreateSizeAttr(1.0)
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
imu_xform = UsdGeom.Xform.Define(stage, '/World/Vehicle/chassis/chassis_IMU')
imu_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

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
    prop_xform = UsdGeom.Xform.Define(stage, f'/World/Vehicle/{name}')
    prop_xform.AddTranslateOp().Set(Gf.Vec3d(*info['pos']))
    prop_prim = prop_xform.GetPrim()

    # Visual mesh
    add_mesh(stage, f'/World/Vehicle/{name}/mesh', info['file'], color=(0.12, 0.12, 0.12))

    # Rigid body for prop
    UsdPhysics.RigidBodyAPI.Apply(prop_prim)
    prop_mass = UsdPhysics.MassAPI.Apply(prop_prim)
    prop_mass.CreateMassAttr(0.1)  # 100g per prop

    # Revolute joint — spin around Y axis (which becomes Z after vehicle rotation)
    joint_path = f'/World/Vehicle/joints/{name}_joint'
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr('Y')
    joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/Vehicle/chassis')])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(f'/World/Vehicle/{name}')])
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

# ============================================================
# LiDAR mount point — child Xform under the chassis, not a propeller.
# prop_fr spins, and a child of that prim would spin with it.
# Translate (0.468, -0.04, -0.515) sits on the front-right arm
# (prop_fr is at X=0.468, Z=-0.515) and 4 cm below the chassis origin.
# Mesh -Y is down before the vehicle's RotateX(90), so that 4 cm keeps
# a downward range on flat ground within 5 cm of hover altitude.
# RotateX(180) aims the sensor's local +Z at the ground, the default
# ray direction for Isaac Sim RTX LiDAR. The sensor prim is created
# at runtime by px4_bridge.py or the tutorial script.
# ============================================================
lidar_mount = UsdGeom.Xform.Define(stage, '/World/Vehicle/chassis/lidar_mount')
lidar_mount.AddTranslateOp().Set(Gf.Vec3d(0.468, -0.04, -0.515))
lidar_mount.AddRotateXOp().Set(180.0)
print('[BUILD] Added lidar_mount under chassis')

print('[BUILD] Physics setup complete.')

# Let stage compose
for _ in range(30):
    app.update()

def anchor_ground_plane(stage, scene_path):
    """Point the ground plane at a copy that lives beside the scene file.

    GroundPlaneCfg localizes the Isaac Lab plane into a new /tmp directory
    on every run. Export stores that path, and the collider vanishes when
    the directory is removed. The saved copy is one USDA plus a textures
    folder, without the numbered /tmp directories.
    """
    from pxr import UsdUtils

    prim = stage.GetPrimAtPath('/World/GroundPlane')
    if not prim.IsValid():
        raise RuntimeError('Ground plane prim is missing')

    asset_paths = []
    for spec in prim.GetPrimStack():
        for ref in list(spec.referenceList.prependedItems) + list(spec.referenceList.explicitItems):
            asset_paths.append(ref.assetPath)
    if len(asset_paths) != 1:
        raise RuntimeError(f'Expected one ground-plane reference, found {asset_paths}')

    src = asset_paths[0]
    if not os.path.isabs(src):
        src = os.path.normpath(os.path.join(os.path.dirname(stage.GetRootLayer().identifier), src))
    if not os.path.isfile(src):
        raise RuntimeError(f'Ground plane asset is missing: {src}')
    src = os.path.abspath(src)
    source_dir = os.path.dirname(src)

    plane_dir = os.path.join(os.path.dirname(scene_path), 'ground_plane')
    textures_dir = os.path.join(plane_dir, 'textures')
    if os.path.isdir(plane_dir):
        shutil.rmtree(plane_dir)
    os.makedirs(textures_dir)
    dest_usda = os.path.join(plane_dir, 'default_ground_plane.usda')
    shutil.copy2(src, dest_usda)

    image_exts = {'.png', '.jpg', '.jpeg', '.exr', '.hdr', '.tif', '.tiff'}
    copied = {src: dest_usda}
    leftovers = []

    def place_file(abs_path):
        abs_path = os.path.normpath(abs_path)
        if abs_path in copied:
            return copied[abs_path]
        ext = os.path.splitext(abs_path)[1].lower()
        folder = textures_dir if ext in image_exts else plane_dir
        dest = os.path.join(folder, os.path.basename(abs_path))
        if os.path.exists(dest):
            stem, suffix = os.path.splitext(os.path.basename(abs_path))
            dest = os.path.join(folder, f'{stem}_{len(copied)}{suffix}')
        shutil.copy2(abs_path, dest)
        copied[abs_path] = dest
        return dest

    def resolve_existing(asset_path):
        if not asset_path or '://' in asset_path:
            return None
        if os.path.isabs(asset_path):
            candidate = os.path.normpath(asset_path)
        else:
            candidate = os.path.normpath(os.path.join(source_dir, asset_path))
        return candidate if os.path.isfile(candidate) else None

    layer = Sdf.Layer.FindOrOpen(dest_usda)
    if layer is None:
        raise RuntimeError(f'Could not open ground-plane layer: {dest_usda}')
    changed = False

    def rewrite(asset_path):
        nonlocal changed
        existing = resolve_existing(asset_path)
        if existing is None:
            if isinstance(asset_path, str) and asset_path.startswith('/tmp'):
                leftovers.append(asset_path)
            return asset_path
        dest = place_file(existing)
        rel = os.path.relpath(dest, plane_dir).replace(os.sep, '/')
        if rel != asset_path:
            changed = True
        if rel.startswith('/tmp'):
            leftovers.append(rel)
        return rel

    UsdUtils.ModifyAssetPaths(layer, rewrite)
    if leftovers:
        raise RuntimeError('Ground plane still references /tmp: ' + '; '.join(leftovers))
    if changed and not layer.Export(dest_usda):
        raise RuntimeError(f'Could not save ground-plane layer: {dest_usda}')
    if not os.path.isfile(dest_usda):
        raise RuntimeError(f'Copied ground plane is missing: {dest_usda}')

    rel_asset = os.path.relpath(dest_usda, os.path.dirname(scene_path)).replace(os.sep, '/')
    prim.GetReferences().SetReferences([Sdf.Reference(rel_asset)])
    print(f'[BUILD] Ground plane reference: {rel_asset}', flush=True)


def verify_ground_plane(scene_path):
    from pxr import Usd
    opened = Usd.Stage.Open(scene_path)
    if not opened:
        raise RuntimeError(f'Could not open {scene_path}')
    plane = None
    for prim in opened.Traverse():
        if str(prim.GetPath()).startswith('/World/GroundPlane') and prim.GetTypeName() == 'Plane':
            plane = prim
            break
    if plane is None:
        raise RuntimeError('Exported scene has no ground-plane collider')
    print(
        f'[BUILD] Ground collider: {plane.GetPath()} schemas={list(plane.GetAppliedSchemas())}',
        flush=True,
    )


# ============================================================
# Save
# ============================================================
out = os.path.expanduser('~/go_aero/assets/vehicle_scene.usd')
os.makedirs(os.path.dirname(out), exist_ok=True)
anchor_ground_plane(stage, out)
stage.GetRootLayer().Export(out)
print(f'[BUILD] Saved: {out} ({os.path.getsize(out)} bytes)')
verify_ground_plane(out)
simulation_app.close()
