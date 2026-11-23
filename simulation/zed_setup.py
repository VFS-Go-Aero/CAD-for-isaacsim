#!/usr/bin/env python3
"""ZED X mounts, Stereolabs stream, and textured obstacles.

``sl.sensor.camera`` renders a virtual ZED X and streams left and right
images over IPC. A ZED SDK process on this machine opens that stream with
``InitParameters.set_from_stream``.

One ZED X sits on the nose. Its left and right eyes are the stereo pair
inside that body. The asset looks along local +X. The camera prims look
along -Z of the camera, which is that +X, and camera +Y is image up. On
this vehicle, mesh -Z is the nose. RotateY(90) sends asset +X along the
nose. That alone leaves camera +Y pointing mesh +X (world east) and the
12 cm baseline vertical. RotateZ(90) after that roll levels the body:
image up is world up, the baseline is horizontal, and there is no pitch.
Translate is authored first so the rotations do not move the mount. The
pose is the uncalibrated placeholder from the ROS wrapper (0.1524 m
forward, 0.127 m down), mapped into the mesh frame as (0, -0.127, -0.1524).
"""
from __future__ import annotations

import os
from typing import Sequence

ZED_MOUNT_PATH = '/World/Vehicle/chassis/zed1_mount'
ZED1_MOUNT_PATH = ZED_MOUNT_PATH
ZED_MOUNT_POS_MESH = (0.0, -0.127, -0.1524)
ZED1_MOUNT_POS_MESH = ZED_MOUNT_POS_MESH
ZED_MOUNT_ROT_Y_DEG = 90.0
# Rolls the body about the nose so image up is world up. See the module note.
ZED_MOUNT_ROT_Z_DEG = 90.0
ZED_STREAM_PORT = 30000
ZED1_STREAM_PORT = ZED_STREAM_PORT
# Older scenes authored a second mount on top of this one. Drop it.
_RETIRED_MOUNT_PATH = '/World/Vehicle/chassis/zed2_mount'
ZED_CAMERA_MODEL = 'ZED_X'
ZED_RESOLUTION = 'SVGA'
ZED_FPS = 15
# World +Y is the nose after the vehicle's RotateX(90). Boards are offset in
# X so a nearer board does not hide the one behind it. From the left camera
# (x≈-0.06), the 5 m board's shadow reaches about x=1.3 on the y=10 plane,
# so the 10 m board starts to the right of that.
OBSTACLE_POSES_M = (
    (-1.5, 3.0, 0.6),
    (0.0, 5.0, 0.6),
    (2.4, 10.0, 0.6),
)


def resolve_zed_ext_folder(explicit: str | None = None) -> str:
    """Return the built ``zed-isaac-sim/exts`` directory."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    env = os.environ.get('ZED_ISAAC_SIM_EXTS')
    if env:
        candidates.append(env)
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates.extend([
        os.path.expanduser('~/zed-isaac-sim/exts'),
        os.path.join(os.path.dirname(repo_root), 'zed-isaac-sim', 'exts'),
    ])
    for candidate in candidates:
        folder = os.path.abspath(os.path.expanduser(candidate))
        marker = os.path.join(folder, 'sl.sensor.camera', 'config', 'extension.toml')
        if os.path.isfile(marker):
            return folder
    searched = ', '.join(os.path.abspath(os.path.expanduser(c)) for c in candidates)
    raise FileNotFoundError(
        'Built Stereolabs extension not found. Pass --zed-ext or set '
        f'ZED_ISAAC_SIM_EXTS. Looked in: {searched}'
    )


def zed_usd_path(ext_folder: str) -> str:
    path = os.path.join(ext_folder, 'sl.sensor.camera', 'data', 'usd', 'ZED_X.usdc')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'ZED X asset missing: {path}')
    return path


def append_kit_args(args, ext_folder: str) -> None:
    """Register the extension folder before Kit starts.

    ``--enable sl.sensor.camera`` at launch fails the dependency solver on
    Kit 110.4, because the extension lists 110.1.2 and 110.3.0. The folder
    is registered here and the extension is enabled after Kit is up.
    """
    extra = f'--ext-folder {ext_folder}'
    existing = getattr(args, 'kit_args', None) or ''
    args.kit_args = f'{existing} {extra}'.strip()
    print(f'[ZED] Kit will search {ext_folder}', flush=True)


def ensure_zed_mounts(stage) -> None:
    """Author the nose mount. The parent is the chassis, never a propeller."""
    from pxr import Gf, UsdGeom

    retired = stage.GetPrimAtPath(_RETIRED_MOUNT_PATH)
    if retired.IsValid():
        stage.RemovePrim(retired.GetPath())
    xform = UsdGeom.Xform.Define(stage, ZED_MOUNT_PATH)
    prim = xform.GetPrim()
    xform.ClearXformOpOrder()
    for prop_name in (
        'xformOp:translate',
        'xformOp:rotateX',
        'xformOp:rotateY',
        'xformOp:rotateZ',
    ):
        if prim.HasAttribute(prop_name):
            prim.RemoveProperty(prop_name)
    # First op is the outer one: translation stays in the mesh frame,
    # then the roll, then the yaw that aims asset +X down the nose.
    xform.AddTranslateOp().Set(Gf.Vec3d(*ZED_MOUNT_POS_MESH))
    xform.AddRotateZOp().Set(float(ZED_MOUNT_ROT_Z_DEG))
    xform.AddRotateYOp().Set(float(ZED_MOUNT_ROT_Y_DEG))


def _reference_zed(stage, mount_path: str, usd_path: str) -> str:
    """Reference the asset under the mount so the asset's own xform ops stay intact."""
    prim_path = f'{mount_path}/ZED_X'
    prim = stage.DefinePrim(prim_path, 'Xform')
    prim.GetReferences().ClearReferences()
    prim.GetReferences().AddReference(usd_path)
    return prim_path


def attach_zed_streams(stage, ext_folder: str) -> list[dict]:
    """Reference one ZED X on the nose and stream it on playback ticks.

    The stream port is 30000. Left and right images are the two eyes of that
    camera. Frames start after ``timeline.play()``. The first RTX compile
    can take several minutes.
    """
    import omni.graph.core as og
    import omni.kit.app
    from pxr import Sdf

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    # The ZED annotator builds an OmniGraph that reads sim time and an IMU.
    # Isaac Sim 6.2 ships those node types on isaacsim.sensors.physics.nodes
    # (the node id is still isaacsim.sensors.physics.IsaacReadIMU). Enabling
    # sl.sensor.camera with --enable at process start fails the Kit 110.4
    # solver, so it is enabled here, after the ext folder was registered.
    for name in (
        'isaacsim.sensors.physics.nodes',
        'sl.sensor.camera',
    ):
        if ext_manager.is_extension_enabled(name):
            continue
        if not ext_manager.set_extension_enabled_immediate(name, True):
            raise RuntimeError(
                f'{name} did not enable. For sl.sensor.camera, rebuild '
                'stereolabs/zed-isaac-sim and pass that exts folder with --zed-ext.'
            )

    usd_path = zed_usd_path(ext_folder)
    ensure_zed_mounts(stage)
    cameras = (
        ('zed1', ZED_MOUNT_PATH, ZED_STREAM_PORT),
    )
    keys = og.Controller.Keys
    create_nodes = [('OnTick', 'omni.graph.action.OnPlaybackTick')]
    connect = []
    set_values = []
    attached = []
    for name, mount_path, port in cameras:
        prim_path = _reference_zed(stage, mount_path, usd_path)
        node = f'ZED_{name}'
        create_nodes.append((node, 'sl.sensor.camera.ZED_Camera'))
        connect.append(('OnTick.outputs:tick', f'{node}.inputs:execIn'))
        set_values.extend([
            (f'{node}.inputs:streamingPort', int(port)),
            (f'{node}.inputs:transportLayerMode', 'IPC'),
            (f'{node}.inputs:cameraModel', ZED_CAMERA_MODEL),
            (f'{node}.inputs:resolution', ZED_RESOLUTION),
            (f'{node}.inputs:fps', int(ZED_FPS)),
            (f'{node}.inputs:streamDepth', False),
        ])
        attached.append({
            'name': name,
            'mount': mount_path,
            'prim': prim_path,
            'port': int(port),
        })

    og.Controller.edit(
        {'graph_path': '/World/ZedStreamGraph', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CONNECT: connect,
            keys.SET_VALUES: set_values,
        },
    )
    for camera in attached:
        node_path = f"/World/ZedStreamGraph/ZED_{camera['name']}"
        stage.GetPrimAtPath(node_path).GetRelationship('inputs:cameraPrim').SetTargets(
            [Sdf.Path(camera['prim'])]
        )
        print(
            f"[ZED] {camera['name']} at {camera['prim']} streaming IPC port {camera['port']}",
            flush=True,
        )
    _print_left_camera_basis(stage, ZED1_MOUNT_PATH)
    return attached


def _print_left_camera_basis(stage, mount_path: str) -> None:
    """Print the left eye in world, so a level nose mount is visible in the log."""
    from pxr import Gf, Usd, UsdGeom

    mount = stage.GetPrimAtPath(mount_path)
    camera = None
    for prim in Usd.PrimRange(mount):
        if prim.GetName() == 'CameraLeft' and prim.GetTypeName() == 'Camera':
            camera = prim
            break
    if camera is None:
        print(f'[ZED] CameraLeft not found under {mount_path}', flush=True)
        return
    matrix = UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    origin = matrix.ExtractTranslation()
    # USD cameras look along -Z. +Y is image up and +X is image right.
    look = matrix.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    up = matrix.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    right = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))

    def fmt(vector) -> str:
        return f'({vector[0]:+.3f},{vector[1]:+.3f},{vector[2]:+.3f})'

    print(
        f'[ZED] CameraLeft origin={fmt(origin)} look={fmt(look)} '
        f'up={fmt(up)} right={fmt(right)}',
        flush=True,
    )


def write_checkerboard(path: str, cells: int = 8, cell_px: int = 32) -> str:
    """Write a high-contrast checkerboard PNG. Stereo matching needs texture."""
    import struct
    import zlib

    size = cells * cell_px
    raw = bytearray()
    for y in range(size):
        raw.append(0)
        row_on = (y // cell_px) % 2 == 0
        for x in range(size):
            on = row_on == ((x // cell_px) % 2 == 0)
            raw.extend(b'\xff\xff\xff' if on else b'\x11\x11\x11')

    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)

    ihdr = struct.pack('>IIBBBBB', size, size, 8, 2, 0, 0, 0)
    png = (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', ihdr)
        + chunk(b'IDAT', zlib.compress(bytes(raw), 9))
        + chunk(b'IEND', b'')
    )
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as handle:
        handle.write(png)
    return path


def add_textured_obstacles(
    stage,
    poses: Sequence[tuple[float, float, float]] = OBSTACLE_POSES_M,
) -> list[str]:
    """Add vertical checkerboard boards at known positions ahead of the nose.

    ``poses`` are world translates in meters. The board is 1.2 m wide and
    1.2 m tall, facing the vehicle at the origin. A flat ``displayColor`` is
    not enough for stereo matching, so each board samples a checkerboard.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

    png_path = write_checkerboard(os.path.expanduser('~/go_aero/zed_checker.png'))
    material_path = '/World/ZedObstacles/Checker'
    material = UsdShade.Material.Define(stage, material_path)
    preview = UsdShade.Shader.Define(stage, material_path + '/Preview')
    preview.CreateIdAttr('UsdPreviewSurface')
    preview.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.8)
    preview.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    preview.CreateOutput('surface', Sdf.ValueTypeNames.Token)
    texture = UsdShade.Shader.Define(stage, material_path + '/Tex')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(png_path)
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
    texture.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
    texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    preview.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), 'rgb'
    )
    material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), 'surface')

    root = UsdGeom.Xform.Define(stage, '/World/ZedObstacles')
    paths = []
    for index, (x, y, z) in enumerate(poses):
        prim_path = f'/World/ZedObstacles/board_{index}'
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        mesh.CreatePointsAttr([
            Gf.Vec3f(-0.6, 0.0, 0.0),
            Gf.Vec3f(0.6, 0.0, 0.0),
            Gf.Vec3f(0.6, 0.0, 1.2),
            Gf.Vec3f(-0.6, 0.0, 1.2),
        ])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateDoubleSidedAttr(True)
        primvar = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            'st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        primvar.Set(Vt.Vec2fArray([
            Gf.Vec2f(0.0, 0.0),
            Gf.Vec2f(4.0, 0.0),
            Gf.Vec2f(4.0, 4.0),
            Gf.Vec2f(0.0, 4.0),
        ]))
        mesh.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        collision.CreateApproximationAttr('convexHull')
        paths.append(prim_path)
        print(
            f'[ZED] textured obstacle {prim_path} at world '
            f'({x:.1f}, {y:.1f}, {z:.1f}) m, 1.2 m tall',
            flush=True,
        )
    root.GetPrim()
    return paths
