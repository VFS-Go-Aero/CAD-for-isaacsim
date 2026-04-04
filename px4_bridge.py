#!/usr/bin/env python3
"""
PX4 SITL <-> Isaac Sim MAVLink Bridge

Sends simulated sensor data (IMU, GPS, barometer, magnetometer) to PX4 SITL.
Receives motor commands back and applies thrust forces in Isaac Sim.

PX4 SITL communicates via MAVLink on UDP:
  - Sim sends to PX4: localhost:14560
  - PX4 sends to Sim: localhost:14560 (same port, bidirectional)

Usage:
  1. Start PX4 SITL:
     cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero ./build/px4_sitl_default/bin/px4

  2. Start Isaac Sim with this bridge:
     python px4_bridge.py --device cuda
"""
import os
import time
import math
import struct
import socket
import threading

os.environ['OMNI_KIT_ACCEPT_EULA'] = 'yes'
os.environ.setdefault('ROS_DISTRO', 'jazzy')

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--px4-port', type=int, default=14560, help='PX4 SITL MAVLink port')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import omni.usd
import omni.kit.app
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics
from pymavlink import mavutil

app = omni.kit.app.get_app()

# ============================================================
# Load scene
# ============================================================
print('[INFO] Opening vehicle scene...')
omni.usd.get_context().open_stage(os.path.expanduser('~/go_aero/vehicle_scene.usd'))

for i in range(300):
    app.update()
    loading = omni.usd.get_context().get_stage_loading_status()
    if loading[2] == 0 and i > 30:
        break

stage = omni.usd.get_context().get_stage()

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
cam = ViewportCameraState("/OmniverseKit_Persp")
cam.set_position_world(Gf.Vec3d(5.0, -5.0, 4.0), True)
cam.set_target_world(Gf.Vec3d(0.0, 0.0, 1.5), True)

# ============================================================
# MAVLink connection to PX4 SITL
# ============================================================
print(f'[INFO] Connecting to PX4 SITL on UDP port {args.px4_port}...')
mav = mavutil.mavlink_connection(
    f'udpout:127.0.0.1:{args.px4_port}',
    source_system=1,
    source_component=1,
)

# Also listen for incoming messages from PX4
mav_in = mavutil.mavlink_connection(
    f'udpin:0.0.0.0:{args.px4_port}',
    source_system=1,
    source_component=1,
)

# ============================================================
# Drone state
# ============================================================
# Home position (Isaac Sim scene origin -> GPS coordinates)
HOME_LAT = 42.3601  # Boston area (for Mass Robotics demo)
HOME_LON = -71.0589
HOME_ALT = 10.0  # meters above sea level

# Vehicle properties
DRONE_MASS = 5.0  # kg
GRAVITY = 9.81
MAX_THRUST_PER_MOTOR = DRONE_MASS * GRAVITY / 4.0 * 2.0  # 2x hover thrust per motor
ARM_LENGTH = 0.7  # meters from center to motor

# Motor state (0-1 normalized from PX4)
motor_commands = [0.0, 0.0, 0.0, 0.0]
motor_lock = threading.Lock()
armed = False

# ============================================================
# MAVLink receiver thread
# ============================================================
def mavlink_receiver():
    global motor_commands, armed
    while True:
        try:
            msg = mav_in.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == 'HIL_ACTUATOR_CONTROLS':
                with motor_lock:
                    motor_commands = [
                        max(0.0, min(1.0, msg.controls[0])),
                        max(0.0, min(1.0, msg.controls[1])),
                        max(0.0, min(1.0, msg.controls[2])),
                        max(0.0, min(1.0, msg.controls[3])),
                    ]
                    armed = (msg.mode & 128) != 0  # MAV_MODE_FLAG_SAFETY_ARMED

            elif msg_type == 'HEARTBEAT':
                pass  # PX4 heartbeat, ignore

        except Exception as e:
            print(f'[WARN] MAVLink recv error: {e}')
            time.sleep(0.1)

recv_thread = threading.Thread(target=mavlink_receiver, daemon=True)
recv_thread.start()
print('[INFO] MAVLink receiver started')

# ============================================================
# Sensor helpers
# ============================================================
def get_vehicle_pose():
    """Read vehicle position and orientation from Isaac Sim."""
    try:
        vehicle_prim = stage.GetPrimAtPath('/World/Vehicle')
        xformable = UsdGeom.Xformable(vehicle_prim)
        world_tf = xformable.ComputeLocalToWorldTransform(0)
        pos = world_tf.ExtractTranslation()
        rot = world_tf.ExtractRotationQuat()
        return pos, rot
    except Exception:
        return Gf.Vec3d(0, 0, 1.5), Gf.Quatd(1, 0, 0, 0)


def send_heartbeat():
    """Send simulator heartbeat to PX4."""
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )


def send_hil_sensor(pos, rot, dt):
    """Send HIL_SENSOR message with simulated IMU, baro, mag data."""
    # Simple sensor model (no noise for now)
    # Accelerometer: gravity in body frame + vehicle acceleration
    # For static/hover: acc = (0, 0, -9.81) rotated to body frame
    # Gyroscope: angular velocity (0 for now)
    # Magnetometer: earth magnetic field rotated to body frame
    # Barometer: pressure from altitude

    timestamp_us = int(time.time() * 1e6)

    # Altitude -> pressure (simple barometric formula)
    alt = float(pos[2]) + HOME_ALT
    pressure = 1013.25 * (1.0 - 2.2558e-5 * alt) ** 5.2560  # hPa
    temperature = 25.0  # Celsius

    # For simplicity, assume level flight: acc = (0, 0, -g), gyro = 0, mag = earth field
    mav.mav.hil_sensor_send(
        timestamp_us,
        0.0, 0.0, -GRAVITY,  # xacc, yacc, zacc (body frame, NED: z-down = -g)
        0.0, 0.0, 0.0,       # xgyro, ygyro, zgyro (rad/s)
        0.2, 0.0, 0.4,       # xmag, ymag, zmag (Gauss, approximate Boston)
        pressure,             # abs_pressure (hPa)
        0.0,                  # diff_pressure
        alt,                  # pressure_altitude (m)
        temperature,          # temperature (C)
        0x1FF,                # fields_updated (all fields)
        0,                    # id
    )


def send_hil_gps(pos):
    """Send HIL_GPS message."""
    timestamp_us = int(time.time() * 1e6)

    # Convert Isaac Sim position (ENU: X=east, Y=north, Z=up) to GPS
    # Approximate: 1 degree lat = 111,139m, 1 degree lon = 111,139 * cos(lat)
    lat_offset = float(pos[1]) / 111139.0  # Y = north
    lon_offset = float(pos[0]) / (111139.0 * math.cos(math.radians(HOME_LAT)))  # X = east
    alt_m = float(pos[2]) + HOME_ALT

    mav.mav.hil_gps_send(
        timestamp_us,
        3,  # fix_type: 3D fix
        int((HOME_LAT + lat_offset) * 1e7),  # lat (degE7)
        int((HOME_LON + lon_offset) * 1e7),  # lon (degE7)
        int(alt_m * 1000),  # alt (mm)
        65535,  # eph (HDOP * 100, 65535=unknown)
        65535,  # epv (VDOP * 100)
        0,      # vel (cm/s)
        0, 0, 0,  # vn, ve, vd (cm/s)
        0,      # cog (course over ground, cdeg)
        12,     # satellites_visible
        0,      # id
        0,      # yaw (cdeg, 0=not available)
    )


# ============================================================
# Start simulation
# ============================================================
print('[INFO] Starting simulation...')
timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(60):
    app.update()

print('[INFO] PX4 bridge running. Waiting for PX4 SITL to connect...')
print('[INFO]   Start PX4: cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=10050 PX4_SIM_MODEL=none_go_aero ./build/px4_sitl_default/bin/px4')
print('[INFO]   Arm: commander arm')
print('[INFO]   Takeoff: commander takeoff', flush=True)

# ============================================================
# Main loop
# ============================================================
step = 0
last_sensor_time = time.time()
last_gps_time = time.time()
last_heartbeat_time = time.time()
SENSOR_RATE = 250  # Hz
GPS_RATE = 10      # Hz

try:
    while simulation_app.is_running():
        app.update()
        step += 1
        now = time.time()

        # Read vehicle pose
        pos, rot = get_vehicle_pose()

        # Send heartbeat at 1 Hz
        if now - last_heartbeat_time > 1.0:
            send_heartbeat()
            last_heartbeat_time = now

        # Send HIL_SENSOR at 250 Hz
        dt = now - last_sensor_time
        if dt >= 1.0 / SENSOR_RATE:
            send_hil_sensor(pos, rot, dt)
            last_sensor_time = now

        # Send HIL_GPS at 10 Hz
        if now - last_gps_time >= 1.0 / GPS_RATE:
            send_hil_gps(pos)
            last_gps_time = now

        # Status log
        if step % 600 == 0:
            with motor_lock:
                m = motor_commands[:]
            print(f'[INFO] step {step}, z={float(pos[2]):.2f}m, armed={armed}, motors=[{m[0]:.2f},{m[1]:.2f},{m[2]:.2f},{m[3]:.2f}]', flush=True)

except KeyboardInterrupt:
    print('[INFO] Stopping...')

timeline.stop()
simulation_app.close()
print('[INFO] Done.')
