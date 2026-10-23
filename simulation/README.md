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
| **Bridge** (`px4_bridge.py`) | Isaac Sim PhysX | Hardware-In-The-Loop (HIL) — full physics, real airframe parameters |

Use the Visualizer mode for quick demos and autonomy work; use the Bridge when you need PhysX to be the source of truth (airframe tuning, real motor/prop characterisation, collision and ground-effect experiments).

---

## Where It Lives

### In this repo (GitHub)

```
CAD-for-isaacsim/
├── simulation/                ← you are here
│   ├── README.md
│   ├── ARCHITECTURE.md
│   ├── PX4_API.md
│   ├── _frames.py             shared NED/ENU + mesh-fix quaternion helpers
│   ├── build_scene.py         build vehicle_scene.usd from STL meshes
│   ├── open_scene.py          open scene with PID hover controller (no PX4)
│   ├── px4_visualizer.py      mirror PX4 SIH pose into Isaac Sim   (default mode)
│   ├── px4_bridge.py          full HIL bridge (sensors out, motors in)
│   ├── power_model.py         battery + Jetson brownout model (px4_bridge.py --power-sim)
│   ├── power_config.example.json  template config for power_model.py
│   ├── joystick_control.py    (Phase 2) Xbox controller → MANUAL_CONTROL to PX4
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
    ├── power_model.py
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

### Visualizer mode (PX4 SIH physics)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — PX4 | `cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d` | `Ready for takeoff!` |
| 2 — Isaac Sim | `python ~/go_aero/px4_visualizer.py --device cuda --mavlink-port 14550` | `Visualizer running!` (~90 s) |
| 3 — Mission | `python ~/go_aero/fly_demo.py --loops 3 --altitude 5 --size 8` | takeoff → square → land |

### HIL bridge mode (Isaac Sim PhysX is the physics)

| Terminal | Command | Wait for |
|---|---|---|
| 1 — Bridge | `python ~/go_aero/px4_bridge.py --device cuda --px4-port 4560` | `Waiting for PX4 on TCP 4560...` |
| 2 — PX4 | `cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d` | bridge prints `PX4 connected. Bridge running.`; PX4 then prints `Ready for takeoff!` after EKF converges (~3 s) |
| 3 — Mission | `python ~/go_aero/fly_demo.py --loops 2 --altitude 5 --size 8` | takeoff → square → land |

The bridge requires the `chassis_IMU` prim to exist in the USD — rebuild the scene first (`python ~/go_aero/build_scene.py --headless`) if you're upgrading from an earlier checkout.

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
cp ~/go_aero/{build_scene,open_scene,px4_visualizer,px4_bridge,fly_demo,_frames,power_model}.py simulation/

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
| `px4_bridge.py` | USD scene + MAVLink UDP `:4560` (bidirectional) | HIL_SENSOR / HIL_GPS out, HIL_ACTUATOR_CONTROLS in | PX4 SITL (HIL mode) |
| `fly_demo.py` | — | MAVSDK UDP `:14540` | PX4 (offboard) |
| `power_model.py` | JSON power config + per-motor thrust from `px4_bridge.py` | optional CSV log + summary | — (imported by the bridge, or standalone) |
| `joystick_control.py` | Xbox controller (USB) | MAVLink `MANUAL_CONTROL` over UDP `:14550` | PX4 GCS endpoint |

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

## HIL Bridge — Airframe Setup

The `none_go_aero` airframe (`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero`) must tell PX4 to run in HIL mode and disable its internal sensor sims. The PX4 rc script `px4-rc.mavlinksim` already starts `simulator_mavlink` on TCP port 4560 when `PX4_SIM_MODEL` is not `sihsim`/`gz`, so no extra `simulator_mavlink` line is needed — we just need the param flips:

```sh
# HIL mode: simulator provides sensors, PX4 provides actuators
param set-default SYS_HITL 1
param set-default SENS_EN_GPSSIM 0
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 0
```

Re-apply after a fresh PX4 checkout with:

```bash
sed -i '/^set AUTOCNF/a param set-default SYS_HITL 1\nparam set-default SENS_EN_GPSSIM 0\nparam set-default SENS_EN_BAROSIM 0\nparam set-default SENS_EN_MAGSIM 0' \
  ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10050_none_go_aero
cd ~/PX4-Autopilot && make px4_sitl
```

The bridge defaults to `--protocol tcp` to match PX4's out-of-the-box TCP client. If you patch `px4-rc.mavlinksim` to launch `simulator_mavlink` with `-u`, also pass `--protocol udp` to the bridge.

The SIH visualizer mode is unaffected — it uses a different airframe (`10040_sihsim_quadx`) that leaves `SYS_HITL=0`.

---

## HIL Bridge — Verification

After a rebuild + PX4 restart, work through these steps to confirm the loop is actually closed:

1. **Rebuild scene** — `python ~/go_aero/build_scene.py --headless`. Grep the resulting USD for `chassis_IMU` to confirm the sensor prim exists.
2. **Start bridge first** — `python ~/go_aero/px4_bridge.py --device cuda --px4-port 4560`. The bridge binds the TCP port and prints `Waiting for PX4 on TCP 4560...`. It must be up before PX4, because PX4's `simulator_mavlink` is the TCP *client*.
3. **Start PX4** — `PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero bin/px4 -d`. PX4 connects into the bridge; the bridge prints `PX4 connected. Bridge running.` and ~3 s later PX4 prints `Ready for takeoff!` (EKF converged on our sensors).
4. **Fly offboard** — `python ~/go_aero/fly_demo.py --altitude 5 --size 8 --loops 2`. Expect the bridge's status line to show motors hovering around ~0.5–0.6 during hover, not pegged at 0 or 1. In the DCV viewport the vehicle should visibly pitch forward/back while moving.
5. **Sanity kill** — stop the bridge mid-flight. PX4 must enter failsafe within `COM_OF_LOSS_T` seconds and print `Sensor timeout`. If it doesn't, PX4 is still using its internal SIH and the HIL airframe changes above didn't take effect.
6. **Regression** — the old visualizer path (`px4_visualizer.py`) should still work unchanged against the SIH airframe.

## Xbox Manual Flight (`joystick_control.py`)

`joystick_control.py` runs on your **local Windows machine** (not on EC2) and feeds an Xbox controller's sticks into PX4 as `MANUAL_CONTROL` MAVLink messages over UDP. PX4 already binds 14550 for its default GCS instance, so no PX4-side changes are needed — just open UDP 14550 inbound on the VividCloud security group to your home IP.

### Setup

1. Plug Xbox controller into the Windows machine, confirm Windows sees it under *Set up USB game controllers*.
2. `pip install pygame pymavlink`
3. Open the EC2 security group `isaac-sim-sg` (in the VividCloud `us-east-1` account) and add an inbound UDP rule for port 14550 from your current IP (`curl ifconfig.me`). Rule source: `your.ip.here/32`.
4. Start PX4 on EC2 (either SIH or HIL mode works):
   ```bash
   cd ~/PX4-Autopilot/build/px4_sitl_default
   PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx bin/px4 -d    # SIH path
   # or the HIL path — also start px4_bridge.py in a second terminal
   ```
5. From Windows: `python simulation/joystick_control.py --ec2-ip 52.91.203.66`
6. **A** = arm, **B** = disarm, **Y** = cycle mode (STABILIZED → ALTITUDE → POSITION), sticks = control.

### Stick layout

| Xbox axis | PX4 field | Range |
|---|---|---|
| Left stick X | roll | [-1000, 1000] |
| Left stick Y | pitch (inverted) | [-1000, 1000] |
| Right stick X | yaw | [-1000, 1000] |
| Right stick Y | throttle (remapped) | [0, 1000] |

5 % deadzone on each axis by default; pass `--deadzone 0.1` to widen.

### Failsafe behaviour

If the controller disconnects mid-flight, the pygame axes return zero and the script keeps streaming neutral sticks. If the script itself dies, PX4's `COM_RC_LOSS_T` failsafe (default 0.5 s) triggers and the autopilot auto-lands / RTLs depending on the param.

On graceful shutdown (Ctrl-C) the script sends 1 s of neutral sticks followed by a disarm command, so the vehicle settles rather than latching the last stick input.

---

## HIL Bridge — Tuning

The bridge exposes four tuning knobs at the top of `px4_bridge.py`:

| Constant | Default | Notes |
|---|---|---|
| `MAX_THRUST_PER_MOTOR` | `2 × hover ≈ 24.5 N` | Bump to 3× hover for acro-style response |
| `K_TAU` | `0.016 m` | Moment-per-unit-thrust. Adjust if yaw response is too soft/twitchy |
| `ACC_NOISE_STD`, `GYRO_NOISE_STD`, ... | small | Pass `--no-noise` to disable entirely while debugging |
| `SENSOR_RATE` | `250` Hz | Physics step rate + HIL_SENSOR send rate. Keep them equal |

Inertia and mass live in `build_scene.py` — edit `mass_api.CreateMassAttr` / `CreateDiagonalInertiaAttr` and rebuild the scene.

---

## HIL Bridge — Jetson Power Budget (`--power-sim`)

Simulates the battery and the Jetson companion computer's supply during a HIL flight. It shows whether a given battery, harness, regulator and hold-up capacitor keep the Jetson powered through gradual sag, scripted brownouts and random voltage transients. `power_model.py` implements it.

```
battery: OCV(SoC), R_cells
  └─ harness R ── main bus ─┬─ 4x ESC + motor  (power from the bridge's per-motor thrust)
                            ├─ other avionics  (constant W)
                            └─ [injected drop] ─ buck regulator ─ hold-up cap ─ Jetson
```

The model is **observational**. It reads motor thrust but never feeds voltage back into thrust or PX4, so enabling it doesn't change how the vehicle flies. PX4's own battery readout is separate and unaffected.

### Run it

```bash
cp ~/go_aero_repo/simulation/power_config.example.json ~/go_aero/power_config.json   # then edit
python ~/go_aero/px4_bridge.py --device cuda --power-sim \
    --power-config ~/go_aero/power_config.json --power-log ~/go_aero/power_log.csv
```

`--power-sim` alone uses the built-in placeholder values. The bridge status line gains `bus=…V soc=…% jetson=ON|BOOTING|OFF`, and stopping the bridge (Ctrl-C) prints a summary like this:

```
[POWER] bus voltage      min 19.25 V at t=149.8s
[POWER] regulator input  min 7.98 V (including injected drops)
[POWER] Jetson headroom  min +1.03 V vs its 9 V cutoff
[POWER] hold-up time     1.26 ms at 25 W (1000 uF)
[POWER] Jetson brownouts 1 (first at t=45.0s), downtime 30.2 s
[POWER] verdict          FAIL: see brownouts / collapse above
```

A `VOLTAGE COLLAPSE` line means the pack plus harness can't deliver the load at all (past its maximum power point), so the battery or wiring is undersized.

**Quick sweeps without Isaac Sim.** `power_model.py` also runs standalone at constant hover thrust in under a second, with no GPU or PX4. It exits with status 1 on a FAIL, so you can script it:

```bash
python simulation/power_model.py --config simulation/power_config.example.json --duration 600
python simulation/power_model.py --dt 0.0001 ...   # finer step to resolve sub-4 ms transients
```

### What it models

| Part | Model | Key config |
|---|---|---|
| Battery | OCV-vs-SoC table (generic LiPo by default) minus `I × (cells × cell_r_ohm + harness_r_ohm)`; constant-power loads, so current rises as voltage sags | `battery.*` |
| Motors | Momentum theory `P = T^1.5 / sqrt(2ρA) / (FM × η_drive)` per rotor, using the same thrust the bridge applies to PhysX | `propulsion.*` |
| Regulator | Buck: holds `output_v` until input < `output_v + dropout_v`, then output tracks input | `regulator.*` |
| Hold-up | Capacitor at the Jetson input rides through dips shorter than `C (V² − Vmin²) / 2P` | `jetson.holdup_uf` |
| Jetson | Browns out when its input < `min_input_v`, then reboots for `boot_time_s` once supply is back above `min_input_v + restart_margin_v` | `jetson.*` |

### Voltage events

All are configured under `faults` in the JSON. They subtract volts at the regulator input. Fault times count from when **PX4 first arms**, so they line up with the flight. Overlapping events don't stack: the largest active drop applies.

| Profile | How |
|---|---|
| Gradual sag | Emerges from load + discharge. Set `battery.initial_soc` low for an end-of-flight test, and `battery.drain_speedup` > 1 to squeeze a full discharge into a short sim |
| Step brownout | `faults.steps`: list of `{t_start_s, duration_s, drop_v}` |
| Random transients | `faults.transients`: Poisson arrivals at `rate_hz`, drop and duration uniform in `[min, max]`, reproducible via `seed` |

Events are applied for whole steps, so at the bridge's 250 Hz anything shorter than 4 ms counts as 4 ms. That errs on the conservative side for sizing. Use the standalone runner with a smaller `--dt` to resolve shorter transients.

### Filling in real numbers

Every default is a **placeholder**, and the verdict means nothing until these are real:

- **Jetson `power_w`**: measure with `tegrastats` on the real unit at your `nvpmodel` mode under your actual workload, and add cameras, NVMe and USB devices. Approximate NVIDIA module envelopes are Orin Nano 7–25 W, Orin NX 10–40 W and AGX Orin 15–60 W, depending on module variant and power mode.
- **Jetson `min_input_v`, `holdup_uf`**: from the carrier board datasheet, or measure them.
- **Battery**: cell count, capacity, per-cell DC internal resistance and the datasheet OCV curve (`battery.ocv_table`, `[[soc, cell_v], ...]` from 0 to 1).
- **Regulator**: output voltage, dropout and efficiency from the converter datasheet.
- **Propulsion**: once real prop and motor data exist, also update `MAX_THRUST_PER_MOTOR` in `px4_bridge.py`.

The config loader accepts plain JSON only and is strict. Unknown keys, wrong types, `NaN`/`Infinity` and out-of-range values are rejected before Isaac Sim boots, so a typo can't silently fall back to a default. Keys starting with `_` are treated as comments.

---

## Stopping & Costs

The instance is `g6e.2xlarge` (NVIDIA L40S, 46 GB VRAM) — about **$2/hr on-demand** when running. An EC2 instance scheduler tag (`Schedule = isaacsim`) auto-stops it nightly, but stop it manually when done:

```bash
aws ec2 stop-instances --profile vividcloud-exp --region us-east-1 --instance-ids i-00f42bf1ca1f91f99
```
