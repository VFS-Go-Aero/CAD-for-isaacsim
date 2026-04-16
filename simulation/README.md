# Go Aero Isaac Sim Simulation

This folder contains the Python scripts that drive the Go Aero eVTOL simulation in NVIDIA Isaac Sim, with a PX4 SITL autopilot in the loop.

For installation and one-time setup, see the [root README](../README.md).
For internals, see [ARCHITECTURE.md](./ARCHITECTURE.md) and [PX4_API.md](./PX4_API.md).

---

## What This Is

A real-time visual simulation of the Go Aero quadrotor flying under PX4 autopilot control. Two modes are supported:

| Mode | Physics | Use case |
|------|---------|----------|
| **Visualizer** (`px4_visualizer.py`) | PX4 SIH (internal) | Smooth visualization, demos, autonomy testing against PX4's generic quadrotor model |
| **Bridge** (`px4_bridge.py`) | Isaac Sim PhysX | Hardware-In-The-Loop (HIL) — full physics, real airframe parameters (work in progress) |

Most current usage is the Visualizer mode — it just mirrors PX4's internal pose into Isaac Sim, no HIL required.

---

## Where It Lives

### In this repo (GitHub)

```
CAD-for-isaacsim/
├── simulation/                ← you are here
│   ├── README.md
│   ├── ARCHITECTURE.md
│   ├── PX4_API.md
│   ├── build_scene.py         build vehicle_scene.usd from STL meshes
│   ├── open_scene.py          open scene with PID hover controller (no PX4)
│   ├── px4_visualizer.py      mirror PX4 SIH pose into Isaac Sim   (default mode)
│   ├── px4_bridge.py          full HIL bridge (sensors out, motors in)  (WIP)
│   └── fly_demo.py            MAVSDK script: takeoff, fly square, land
├── meshes/                    STL files: frame.stl, FR.stl, FL.stl, BR.stl, BL.stl
├── go_aero_vehicle.urdf       reference URDF (not used at runtime)
├── XML2_fixed.urdf            alternate URDF
└── README.md                  install + setup guide
```

### On the EC2 instance (`isaac-sim-massrobotics-01`, IP `52.91.203.66`)

```
/home/ubuntu/
├── miniconda3/                                   conda + env_isaaclab_jazzy env
├── PX4-Autopilot/                                PX4 source + built SITL binary
└── go_aero/                                      runtime working dir
    ├── meshes/                                   copy of repo meshes
    ├── vehicle_scene.usd                         built by build_scene.py
    ├── build_scene.py
    ├── open_scene.py
    ├── px4_visualizer.py
    ├── px4_bridge.py
    └── fly_demo.py
```

The scripts read the scene from `~/go_aero/vehicle_scene.usd` (a hard-coded path inside each script — see the *Updating* section if you need to change this).

---

## How to Run

Three terminals (or `tmux` panes). Activate the conda env in each first:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate env_isaaclab_jazzy
```

| Terminal | Command | Wait for |
|---|---|---|
| 1 — PX4 | `cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d` | `Ready for takeoff!` |
| 2 — Isaac Sim | `python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550` | `Visualizer running!` (~90 s) |
| 3 — Mission | `python ~/go_aero/fly_demo.py --loops 3 --altitude 5 --size 8` | takeoff → square → land |

Connect a viewer with NICE DCV at `https://52.91.203.66:8443` to see the Isaac Sim viewport.

---

## How to Update

The scripts on the EC2 instance live in `~/go_aero/` — that's a **copy** of the repo files, not a clone. Three workflows:

### Workflow A — Edit on EC2, push back to git (typical)

```bash
# On the EC2 instance
cd ~/go_aero
nano px4_visualizer.py         # or vim/code-server

# When happy, copy back into the repo clone
cd ~/go_aero_repo               # the cloned CAD-for-isaacsim
git checkout sgoss/isaac-sim-import
cp ~/go_aero/{build_scene,open_scene,px4_visualizer,px4_bridge,fly_demo}.py simulation/

# Commit and push
git add simulation/
git commit -m "Tweak visualizer: <what changed>"
git push origin sgoss/isaac-sim-import
```

### Workflow B — Edit locally, sync to EC2 via scp

```bash
# Local clone
git clone -b sgoss/isaac-sim-import git@github.com:VFS-Go-Aero/CAD-for-isaacsim.git
cd CAD-for-isaacsim
# ...edit simulation/*.py...

# Push edits to EC2
scp -i isaac-sim-keypair.pem simulation/*.py ubuntu@52.91.203.66:~/go_aero/

# Then commit + push as usual
git add simulation/ && git commit -m "..." && git push
```

### Workflow C — Pull from git on EC2

```bash
ssh -i isaac-sim-keypair.pem ubuntu@52.91.203.66
cd ~/go_aero_repo && git pull
cp simulation/*.py ~/go_aero/
```

### When you change the scene structure (`build_scene.py`)

You must rebuild the USD scene before the visualizer can see the change:

```bash
conda activate env_isaaclab_jazzy
python ~/go_aero/build_scene.py --headless     # ~60 s
```

### When you change `px4_visualizer.py` or `fly_demo.py`

Just restart the corresponding terminal — no rebuild needed.

### When you change PX4 airframe parameters

```bash
cd ~/PX4-Autopilot
make px4_sitl                                   # rebuild PX4 (~3 min first time, ~30 s incremental)
```

Then restart the PX4 terminal.

---

## File Map

| File | Reads | Writes | Other side |
|---|---|---|---|
| `build_scene.py` | `~/go_aero/meshes/*.stl` | `~/go_aero/vehicle_scene.usd` | — |
| `open_scene.py` | `~/go_aero/vehicle_scene.usd` | viewport | — (standalone, no PX4) |
| `px4_visualizer.py` | USD scene + MAVLink UDP `:14550` | viewport | PX4 SIH |
| `px4_bridge.py` | USD scene + MAVLink UDP `:14560` | UDP `:14560` (HIL_SENSOR, HIL_GPS) | PX4 SITL (HIL mode) |
| `fly_demo.py` | — | MAVSDK UDP `:14540` | PX4 (offboard) |

See [PX4_API.md](./PX4_API.md) for the MAVLink message and MAVSDK method details, and [ARCHITECTURE.md](./ARCHITECTURE.md) for the full data flow.

---

## Common Tasks

### Change the demo flight pattern
Edit `fly_demo.py` — the `waypoints` list and the `for loop in range(args.loops)` body.

### Add a new MAVLink message handler in the visualizer
Edit `px4_visualizer.py` `mavlink_receiver()` — add an `elif msg_type == 'YOUR_MSG':` branch and store fields into `vehicle_state`.

### Move the simulated home location
Edit `px4_bridge.py` constants `HOME_LAT`, `HOME_LON`, `HOME_ALT` (currently set to Boston for the Mass Robotics demo).

### Change vehicle mass / inertia / motor layout
Edit `build_scene.py` — `mass_api.CreateMassAttr(...)`, the `props` dict (motor positions), and prop mass. Then **rebuild the scene**.

### Change the camera angle
Edit the `cam.set_position_world(...)` and `cam.set_target_world(...)` calls near the top of `px4_visualizer.py` or `open_scene.py`.

---

## Stopping & Costs

The instance is `g6e.2xlarge` (NVIDIA L40S, 46 GB VRAM) — about **$2/hr on-demand** when running. An EC2 instance scheduler tag (`Schedule = isaacsim`) auto-stops it nightly, but stop it manually when done:

```bash
aws ec2 stop-instances --profile vividcloud-exp --region us-east-1 --instance-ids i-00f42bf1ca1f91f99
```
