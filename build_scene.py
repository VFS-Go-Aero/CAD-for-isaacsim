#!/usr/bin/env python3
"""Build vehicle scene USD with correct orientation (Y-up STL -> Z-up Isaac Sim)."""
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
from pxr import UsdGeom, Gf, UsdPhysics, Vt
import isaaclab.sim as sim_utils

app = omni.kit.app.get_app()
stage = omni.usd.get_context().get_stage()

# Scene (same as carter demo)
physics_scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)
cfg = sim_utils.GroundPlaneCfg()
cfg.func('/World/GroundPlane', cfg)
light_cfg = sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0)
light_cfg.func('/World/Light', light_cfg)
dome_cfg = sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0)
dome_cfg.func('/World/SkyLight', dome_cfg)

# Vehicle root - rotate -90 around X to convert Y-up to Z-up, raise above ground
vehicle = UsdGeom.Xform.Define(stage, '/World/Vehicle')
vehicle.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.6))
vehicle.AddRotateXOp().Set(90.0)

MESH_DIR = os.path.expanduser('~/go_aero/meshes')

parts = {
    'chassis': {'file': 'frame.stl', 'pos': (0, 0, 0), 'color': (0.35, 0.35, 0.4)},
    'wheel_fr': {'file': 'FR.stl', 'pos': (0.468, 0.048, -0.515), 'color': (0.1, 0.1, 0.1)},
    'wheel_fl': {'file': 'FL.stl', 'pos': (-0.468, 0.048, -0.515), 'color': (0.1, 0.1, 0.1)},
    'wheel_rr': {'file': 'BR.stl', 'pos': (0.468, 0.048, 0.515), 'color': (0.1, 0.1, 0.1)},
    'wheel_rl': {'file': 'BL.stl', 'pos': (-0.468, 0.048, 0.515), 'color': (0.1, 0.1, 0.1)},
}

for name, info in parts.items():
    print(f'Adding {name}...', flush=True)
    stl = trimesh.load(os.path.join(MESH_DIR, info['file']))
    verts = stl.vertices  # already in meters

    xform = UsdGeom.Xform.Define(stage, f'/World/Vehicle/{name}')
    xform.AddTranslateOp().Set(Gf.Vec3d(*info['pos']))

    m = UsdGeom.Mesh.Define(stage, f'/World/Vehicle/{name}/mesh')
    m.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in verts]))
    m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(stl.faces.flatten().tolist()))
    m.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(stl.faces)))
    m.GetSubdivisionSchemeAttr().Set('none')
    m.GetDoubleSidedAttr().Set(True)
    m.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*info['color'])]))

    norms = Vt.Vec3fArray([Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])) for n in stl.vertex_normals])
    m.GetNormalsAttr().Set(norms)
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    mins = verts.min(axis=0)
    maxs = verts.max(axis=0)
    m.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*mins), Gf.Vec3f(*maxs)]))
    print(f'  size: {maxs - mins}')

out = os.path.expanduser('~/go_aero/vehicle_scene.usd')
stage.GetRootLayer().Export(out)
print(f'Saved: {out} ({os.path.getsize(out)} bytes)')
simulation_app.close()
