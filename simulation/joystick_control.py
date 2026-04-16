#!/usr/bin/env python3
"""
Xbox controller -> PX4 MANUAL_CONTROL MAVLink bridge.

Runs on the user's Windows machine (not inside Isaac Sim), reads the first
attached joystick via pygame, and sends MANUAL_CONTROL messages to the PX4
instance running on the EC2 host over UDP. PX4 listens for GCS MAVLink on
port 14550 by default, and MANUAL_CONTROL is the standard way a GCS feeds
stick data for stabilize/altitude/position modes.

Network topology:

    [Xbox ctrl] --USB--> [this script] ===== UDP 14550 =====> [PX4 on EC2]

The EC2 security group must allow UDP 14550 inbound from the machine
running this script. PX4 already binds 14550 (it's the default GCS port),
so no additional PX4 config is required.

Stick mapping (Xbox / pygame defaults):

    axis 0 (left-X)   -> roll      [-1000, 1000]
    axis 1 (left-Y)   -> pitch     [-1000, 1000] (inverted: stick fwd -> nose down)
    axis 2 (right-X)  -> yaw       [-1000, 1000]
    axis 3 (right-Y)  -> throttle  [0, 1000]     (stick fwd -> full throttle)

    button 0 (A)      -> arm     (on press)
    button 1 (B)      -> disarm  (on press)
    button 3 (Y)      -> cycle mode: stabilize -> altitude -> position -> stabilize

Usage:
  pip install pygame pymavlink
  python joystick_control.py --ec2-ip 52.91.203.66

The script runs at 50 Hz, sends MANUAL_CONTROL at 20 Hz effective (every
other tick), and emits a GCS heartbeat at 1 Hz. Ctrl-C sends neutral sticks
+ disarm on the way out.
"""
import argparse
import math
import signal
import sys
import time

import pygame
from pymavlink import mavutil


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--ec2-ip', required=True, help='Public IP of the PX4/EC2 host')
parser.add_argument('--port', type=int, default=14550, help='PX4 GCS MAVLink UDP port (default 14550)')
parser.add_argument('--rate-hz', type=float, default=50.0, help='Joystick poll rate (default 50 Hz)')
parser.add_argument('--send-hz', type=float, default=20.0, help='MANUAL_CONTROL send rate (default 20 Hz)')
parser.add_argument('--deadzone', type=float, default=0.05,
                    help='Stick deadzone on each axis (0..1), default 0.05')
parser.add_argument('--invert-pitch', action='store_true', default=True,
                    help='Invert pitch axis so stick-forward = nose-down (default true)')
parser.add_argument('--no-invert-pitch', dest='invert_pitch', action='store_false')
parser.add_argument('--target-system', type=int, default=1,
                    help='PX4 MAVLink system ID (default 1)')
parser.add_argument('--verbose', action='store_true', help='Print every send')
args = parser.parse_args()


# ------------------------------------------------------------------
# PX4 custom flight modes (base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
# custom_mode layout (little-endian):
#   byte 3 = main_mode (PX4 main mode)
#   byte 2 = sub_mode  (PX4 sub mode, 0 unless main=AUTO)
# Reference: Firmware/src/modules/commander/px4_custom_mode.h
# ------------------------------------------------------------------
PX4_MAIN_MANUAL = 1
PX4_MAIN_ALTCTL = 2
PX4_MAIN_POSCTL = 3
PX4_MAIN_STABILIZED = 7

# Cycle order for the Y-button mode toggle.
MODE_CYCLE = [
    ('STABILIZED', PX4_MAIN_STABILIZED),
    ('ALTITUDE',   PX4_MAIN_ALTCTL),
    ('POSITION',   PX4_MAIN_POSCTL),
]


def px4_custom_mode(main_mode, sub_mode=0):
    """Pack PX4 main/sub into the 32-bit custom_mode field."""
    return (main_mode << 16) | (sub_mode << 24)


# ------------------------------------------------------------------
# Joystick
# ------------------------------------------------------------------
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print('[ERROR] No joystick detected. Plug in an Xbox controller and retry.', file=sys.stderr)
    sys.exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f'[INFO] Joystick: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)')


def deadzone(v, dz=args.deadzone):
    if abs(v) < dz:
        return 0.0
    # Rescale so that motion is smooth outside the deadzone.
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


# ------------------------------------------------------------------
# MAVLink connection (GCS identity: system=255, component=MAV_COMP_ID_MISSIONPLANNER=190)
# ------------------------------------------------------------------
mav_url = f'udpout:{args.ec2_ip}:{args.port}'
print(f'[INFO] Connecting MAVLink: {mav_url}')
mav = mavutil.mavlink_connection(
    mav_url,
    source_system=255,
    source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
)


def send_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def send_manual_control(x, y, z, r, buttons):
    """x=pitch, y=roll, z=throttle, r=yaw, all in [-1000..1000] (z in [0..1000])."""
    mav.mav.manual_control_send(
        args.target_system,
        int(max(-1000, min(1000, x))),
        int(max(-1000, min(1000, y))),
        int(max(0, min(1000, z))),
        int(max(-1000, min(1000, r))),
        buttons,
    )


def send_arm(armed):
    """MAV_CMD_COMPONENT_ARM_DISARM; param1 = 1 (arm) / 0 (disarm)."""
    mav.mav.command_long_send(
        args.target_system, 1,                            # autopilot is component 1
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,                                                # confirmation
        1.0 if armed else 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


def send_set_mode(main_mode):
    """Switch to a PX4 custom flight mode via MAV_CMD_DO_SET_MODE."""
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    mav.mav.command_long_send(
        args.target_system, 1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,
        float(main_mode),
        0.0, 0.0, 0.0, 0.0, 0.0,
    )


# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------
tick_period = 1.0 / max(args.rate_hz, 1.0)
send_period = 1.0 / max(args.send_hz, 1.0)
last_send = 0.0
last_hb = 0.0

prev_buttons = [False] * js.get_numbuttons()
mode_idx = 0

running = True


def shutdown(signum=None, frame=None):
    global running
    running = False


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

print('[INFO] Streaming MANUAL_CONTROL. A=arm, B=disarm, Y=mode cycle, Ctrl-C to quit.')

try:
    while running:
        t = time.time()
        pygame.event.pump()

        # Axes: apply deadzone, clamp to [-1, 1].
        lx = deadzone(js.get_axis(0))     # roll
        ly = deadzone(js.get_axis(1))     # pitch (raw)
        rx = deadzone(js.get_axis(2))     # yaw
        ry = deadzone(js.get_axis(3))     # throttle (raw)

        if args.invert_pitch:
            ly = -ly

        # Map to PX4 MANUAL_CONTROL ranges.
        roll_cmd = lx * 1000.0
        pitch_cmd = ly * 1000.0
        yaw_cmd = rx * 1000.0
        # Right-stick Y at rest = 0; forward = -1 (pygame convention), back = +1.
        # Remap to throttle [0..1000] with stick-forward -> full throttle.
        throttle_cmd = ((-ry) + 1.0) * 0.5 * 1000.0

        # Buttons: rising edge detection.
        buttons_now = [js.get_button(i) == 1 for i in range(js.get_numbuttons())]
        button_bits = 0

        def pressed(idx):
            return idx < len(buttons_now) and buttons_now[idx] and not prev_buttons[idx]

        if pressed(0):
            print('[INFO] A pressed -> ARM')
            send_arm(True)
            button_bits |= (1 << 0)
        if pressed(1):
            print('[INFO] B pressed -> DISARM')
            send_arm(False)
            button_bits |= (1 << 1)
        if pressed(3):
            mode_idx = (mode_idx + 1) % len(MODE_CYCLE)
            name, main_mode = MODE_CYCLE[mode_idx]
            print(f'[INFO] Y pressed -> mode {name}')
            send_set_mode(main_mode)
            button_bits |= (1 << 2)

        prev_buttons = buttons_now

        # Send MANUAL_CONTROL at send-Hz.
        if t - last_send >= send_period:
            last_send = t
            # The MANUAL_CONTROL field names confusingly map x->pitch, y->roll.
            # See https://mavlink.io/en/messages/common.html#MANUAL_CONTROL.
            send_manual_control(
                x=pitch_cmd,
                y=roll_cmd,
                z=throttle_cmd,
                r=yaw_cmd,
                buttons=button_bits,
            )
            if args.verbose:
                print(f'  pitch={pitch_cmd:+5.0f} roll={roll_cmd:+5.0f} '
                      f'yaw={yaw_cmd:+5.0f} thr={throttle_cmd:5.0f}')

        # Heartbeat at 1 Hz.
        if t - last_hb >= 1.0:
            last_hb = t
            send_heartbeat()

        # Sleep until next tick.
        sleep_t = tick_period - (time.time() - t)
        if sleep_t > 0:
            time.sleep(sleep_t)

finally:
    # Graceful shutdown: 1 s of neutral sticks so PX4 doesn't latch the
    # final command, then a disarm for safety.
    print('[INFO] Shutdown: sending neutral sticks + disarm...')
    try:
        for _ in range(int(args.send_hz)):
            send_manual_control(0, 0, 500, 0, 0)
            time.sleep(send_period)
        send_arm(False)
    except Exception as e:
        print(f'[WARN] Shutdown send failed: {e}', file=sys.stderr)

    try:
        pygame.quit()
    except Exception:
        pass
    print('[INFO] Done.')
