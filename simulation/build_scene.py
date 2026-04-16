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
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
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

cfg = sim_utils.GroundPlaneCfg()
cfg.func('/World/GroundPlane', cfg)
light_cfg = sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
light_cfg.func('/World/Light', light_cfg)
dome_cfg = sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0)
dome_cfg.func('/World/SkyLight', dome_cfg)

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
