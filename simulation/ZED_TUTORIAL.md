# ZED X stream — Go Aero

Isaac Sim renders one ZED X on the nose of the vehicle. The left and right
views are the two eyes of that camera. The ZED SDK is a separate process
that opens the stream. Detection scripts stay unmodified and talk to that
SDK, or to `zed-ros2-wrapper` with `sim_mode:=true`.

## Versions used on this machine

| Piece | Version |
|---|---|
| Isaac Sim | 6.2.0.0 |
| Isaac Lab | 3.0.0 |
| Kit | 110.4.0 |
| `sl.sensor.camera` | 5.3.1 (`stereolabs/zed-isaac-sim` `a8f3447`) |
| ZED SDK / pyzed | 5.5.1 / 5.5 |
| CUDA toolkit installed for the SDK | 13.5.0 |
| Driver | 595.91.07 |
| `zed-ros2-wrapper` | not installed on this machine |

`sl.sensor.camera` 5.3.1 lists Kit 110.1.2 and 110.3.0. Passing `--enable` at
Kit startup makes the 110.4 dependency solver reject it. The scripts register
the `exts` folder with `--ext-folder` and enable the extension after Kit is
up. It then loads and streams on this Kit 110.4 install. The vehicle's Jetson stays on ZED SDK 5.1.0. The sim
host needs a receiver SDK of at least 5.5.0 for this extension.

## One-time setup

Build the extension (sibling of this repo is fine):

```bash
git clone https://github.com/stereolabs/zed-isaac-sim.git ~/zed-isaac-sim
cd ~/zed-isaac-sim
./build.sh
```

Install ZED SDK 5.5.1 for the host CUDA major version, then the Python API
into `env_isaaclab_3`. The SDK directory is group `zed`:

```bash
sudo usermod -aG zed "$USER"
# new login, or: sg zed -c '...'
python /usr/local/zed/get_python_api.py
```

`import pyzed.sl` needs `LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64`.
Set `LC_ALL=C` in that process. The calibration file parser warns without it.

Rebuild the vehicle scene so the chassis mounts exist:

```bash
python simulation/build_scene.py --headless
```

`zed1_mount` is a child of `/World/Vehicle/chassis`, on the nose. A
propeller parent would spin the camera with the prop. One ZED X uses the
uncalibrated placeholder `(0, -0.127, -0.1524)` in the mesh frame
(15.24 cm forward, 12.7 cm down). Its left and right eyes are the stereo
pair inside that body. RotateY(+90°) aims the asset's +X look axis along
the nose (mesh −Z), so the optical axis is horizontal. An extra
RotateZ(+90°) levels the body: image up is world up and the stereo baseline
is parallel to the ground. There is no tilt. Replace that pose when a
calibration exists. The runtime `ensure_zed_mounts()` writes the same ops,
so a tutorial or `--zed` flight uses this pose even before the next
`build_scene.py`.

## Run

Terminal 1, Isaac Lab env:

```bash
python simulation/add_zed_tutorial.py --headless --obstacles --ros2 --duration 120
```

`--zed-ext` or `ZED_ISAAC_SIM_EXTS` points at the built `exts` folder when it
is not `~/zed-isaac-sim/exts` or a sibling `../zed-isaac-sim/exts`. The first
RTX shader compile can take several minutes before the ports open.

Terminal 2:

```bash
sg zed -c 'LC_ALL=C LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64 \
  python simulation/zed_stream_check.py --port 30000'
```

The camera streams on port 30000. The check prints left and right shapes
from the two eyes. SVGA is 960×600. The SDK log may say the UDP port is
already used and switch ports. The frames still arrive on the IPC stream
for the port you passed. The tutorial turns gravity off so the vehicle
stays at its start pose.

`--obstacles` adds 1.2 m checkerboard boards at world `(x, y, z)` meters:

| Board | Position | Distance ahead |
|---|---|---|
| `board_0` | (−1.5, 3, 0.6) | 3 m |
| `board_1` | (0, 5, 0.6) | 5 m |
| `board_2` | (2.4, 10, 0.6) | 10 m |

World +Y is the nose. The boards are offset in X so they do not hide each
other. A flat color has no features for stereo matching, so each board uses
`~/go_aero/zed_checker.png`.

`--ros2` publishes `/clock` from Isaac Sim's ROS 2 bridge (Jazzy on this
host). The wrapper can set `use_sim_time`. Camera images travel on the ZED
IPC stream.

## With ArduPilot

```bash
python simulation/ardupilot_bridge.py --headless --zed --obstacles --ros2 --port 9002
```

`--zed` uses the same mount and IPC port. `zed-ros2-wrapper`
(`sim_mode:=true`) or pyzed opens 30000 and publishes
`/zed1/zed_node/point_cloud/cloud_registered`.

Depth against the boards, with the tutorial still running:

```bash
python simulation/zed_stream_check.py --port 30000 --depth
```

The SDK depth is the distance along the camera axis. The left camera looks
along world +Y from y=0.160 m, level with the ground, so the boards at 3,
5, and 10 m are 2.840, 4.840, and 9.840 m away. One headless run with the
level mount measured 2.800 m (1.39%), 4.817 m (0.47%), and 9.840 m (0.00%).

## Still to measure

- Unmodified `visual-obstacle-detection` seeing an obstacle 5 m ahead. That repo is not on this machine.
- ROS image, depth, and point-cloud topics stamped from `/clock`. The wrapper is not installed, so those topics are not published. `/clock` itself is sim time, about 30 Hz. The ZED image timestamps are host time at 66.7 ms (15 Hz). A consumer that also retrieved neural depth and the XYZ cloud kept up at 12.6 Hz.

One headless `ardupilot_bridge.py --zed` plus `ardu_poshold.py --mode guided --altitude 5 --hold 30` armed after `EKF3 IMU0 is using GPS`, climbed to 4.64 m, held between 4.82 m and 4.99 m, landed, and disarmed. No lockstep stall and no EKF failsafe. From the first lockstep status to disarm the bridge advanced about 78 s of sim time in 615 s of wall time, 0.12× realtime (about 30 of the 250 Hz physics steps per wall second). The 30 s hold is wall time, so the hover was about 4 s of sim time.
