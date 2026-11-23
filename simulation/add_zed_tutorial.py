#!/usr/bin/env python3
"""
ZED X stream tutorial for the Go Aero vehicle.

Loads ~/go_aero/assets/vehicle_scene.usd, references a ZED X on each chassis mount,
and streams left and right images to the ZED SDK over IPC.

  python add_zed_tutorial.py --headless --obstacles --duration 60

A second process opens the stream. See ZED_TUTORIAL.md. Gravity is set to
zero so the cameras stay on the authored pose. Closed-loop hover is
ardupilot_bridge.py --zed with ardu_poshold.py.
"""
import os

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

import argparse

from isaaclab.app import AppLauncher

from zed_setup import (
    ZED1_STREAM_PORT,
    add_textured_obstacles,
    append_kit_args,
    attach_zed_streams,
    resolve_zed_ext_folder,
)

parser = argparse.ArgumentParser(description='Stream two simulated ZED X cameras from the vehicle')
parser.add_argument('--headless', action='store_true', help='Run without a viewport')
parser.add_argument('--duration', type=float, default=60.0, help='Seconds to keep the stream open')
parser.add_argument('--obstacles', action='store_true', help='Add checkerboard boards 3, 5, and 10 m ahead')
parser.add_argument('--zed-ext', default=None, help='Built stereolabs/zed-isaac-sim exts folder')
parser.add_argument('--ros2', action='store_true', help='Also publish /clock for use_sim_time')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
ext_folder = resolve_zed_ext_folder(args.zed_ext)
append_kit_args(args, ext_folder)

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdPhysics

print(f'[TUTORIAL] ZED extension folder: {ext_folder}', flush=True)
scene_path = os.path.expanduser('~/go_aero/assets/vehicle_scene.usd')
print(f'[TUTORIAL] Opening {scene_path}', flush=True)
omni.usd.get_context().open_stage(scene_path)
app = omni.kit.app.get_app()
for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath('/World/Vehicle/chassis').IsValid():
    raise RuntimeError('Vehicle chassis missing from the scene. Run build_scene.py first.')
if args.obstacles:
    add_textured_obstacles(stage)

physics = stage.GetPrimAtPath('/World/PhysicsScene')
if physics.IsValid():
    UsdPhysics.Scene(physics).CreateGravityMagnitudeAttr(0.0)
    print('[TUTORIAL] Gravity off so the vehicle holds its start pose', flush=True)

cameras = attach_zed_streams(stage, ext_folder)

if not getattr(args, 'headless', False):
    try:
        from pxr import Gf
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        # Nose is world +Y. Sit in front of it so the level camera bodies are in frame.
        view = ViewportCameraState('/OmniverseKit_Persp')
        view.set_position_world(Gf.Vec3d(1.2, 2.8, 1.8), True)
        view.set_target_world(Gf.Vec3d(0.0, 0.15, 1.35), True)
        print('[TUTORIAL] Viewport aimed at the nose cameras', flush=True)
    except Exception as exc:
        print(f'[TUTORIAL] Could not aim the viewport: {exc}', flush=True)

if args.ros2:
    import omni.graph.core as og

    ext_manager = app.get_extension_manager()
    for name in ('isaacsim.core.nodes', 'isaacsim.ros2.bridge'):
        ext_manager.set_extension_enabled_immediate(name, True)
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': '/World/ZedClockGraph', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ],
            keys.CONNECT: [
                ('OnTick.outputs:tick', 'PublishClock.inputs:execIn'),
                ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
            ],
            keys.SET_VALUES: [
                ('PublishClock.inputs:topicName', '/clock'),
            ],
        },
    )
    print('[TUTORIAL] Publishing /clock for use_sim_time', flush=True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(
    f'[TUTORIAL] Streaming the nose camera on {ZED1_STREAM_PORT}. '
    f'Open it with simulation/zed_stream_check.py. Duration {args.duration:.0f}s.',
    flush=True,
)

step = 0
max_steps = max(1, int(args.duration * 60))
try:
    while simulation_app.is_running() and step < max_steps:
        app.update()
        step += 1
        if step % 60 == 0:
            print(f'[TUTORIAL] step={step} cameras={len(cameras)}', flush=True)
except KeyboardInterrupt:
    print('\n[TUTORIAL] Interrupted.', flush=True)

timeline.stop()
simulation_app.close()
print('[TUTORIAL] Done.', flush=True)
