#!/usr/bin/env python3
"""Open the pre-built vehicle scene USD."""
import os
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
from pxr import Gf

app = omni.kit.app.get_app()

print('[INFO] Opening vehicle_scene.usd...')
omni.usd.get_context().open_stage(os.path.expanduser('~/go_aero/vehicle_scene.usd'))

for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()
meshes = [p for p in stage.Traverse() if p.GetTypeName() == 'Mesh']
print(f'[INFO] Meshes: {len(meshes)}')
for m in meshes:
    print(f'  {m.GetPath()}')

from omni.kit.viewport.utility.camera_state import ViewportCameraState
cam = ViewportCameraState("/OmniverseKit_Persp")
cam.set_position_world(Gf.Vec3d(3.0, -3.0, 2.5), True)
cam.set_target_world(Gf.Vec3d(0.0, 0.0, 0.5), True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(120):
    app.update()

print('[INFO] Running. Ctrl+C to stop.', flush=True)
step = 0
try:
    while simulation_app.is_running():
        app.update()
        step += 1
        if step % 600 == 0:
            print(f'[INFO] step {step}', flush=True)
except KeyboardInterrupt:
    pass

timeline.stop()
simulation_app.close()
