#!/usr/bin/env python3
"""
Xbox controller -> PX4 MANUAL_CONTROL MAVLink bridge.

Runs on the user's Windows/local machine (not inside Isaac Sim), reads the first
attached joystick via pygame, and sends MANUAL_CONTROL messages to the PX4
instance running on the EC2 host over UDP.

Network topology & UDP port note (`18570` vs `14550`):

    [Xbox ctrl] --USB--> [this script] ===== UDP 18570 / 14550 =====> [PX4 on EC2]

In PX4 SITL (`px4-rc.mavlink`), PX4 binds local UDP port `18570` (`mavlink start -x -u 18570`)
and streams telemetry *out* to GCS port `14550` until a GCS sends packets to `18570`
(or unless a MAVLink router / custom `mavlink start -u 14550` listener is bound on
`14550`). When sending directly to an unpatched PX4 SITL instance without a router,
pass `--port 18570` (and open UDP 18570 in the EC2 security group); when sending
to a MAVLink router or custom PX4 listener on 14550, use the default `--port 14550`.

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
  # Direct to PX4 SITL GCS listener port 18570:
  python joystick_control.py --ec2-ip 52.91.203.66 --port 18570
  # Or via MAVLink router / custom listener on 14550:
  python joystick_control.py --ec2-ip 52.91.203.66 --port 14550

The script runs at 50 Hz, sends MANUAL_CONTROL at 20 Hz effective, and emits a
GCS heartbeat at 1 Hz. Ctrl-C sends neutral sticks + disarm on the way out.
"""
import argparse
import signal
import sys
import time

from pymavlink import mavutil

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

# Module-level state (populated in main(), or overridable in tests)
args = None
mav = None
is_armed = False


def px4_custom_mode(main_mode: int, sub_mode: int = 0) -> int:
    """Pack PX4 main/sub into the 32-bit custom_mode field."""
    return (int(main_mode) << 16) | (int(sub_mode) << 24)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ec2-ip', required=True, help='Public IP of the PX4/EC2 host')
    parser.add_argument(
        '--port',
        type=int,
        default=14550,
        help='PX4 inbound MAVLink UDP port (default 14550; use --port 18570 for '
             'direct PX4 SITL GCS link from px4-rc.mavlink)',
    )
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
    return parser


def deadzone(v: float, dz: float | None = None) -> float:
    if dz is None:
        dz = args.deadzone if args is not None else 0.05
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def send_heartbeat(mav_conn=None):
    conn = mav_conn or mav
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def send_manual_control(x, y, z, r, buttons, mav_conn=None, target_system=None):
    """x=pitch, y=roll, z=throttle, r=yaw, all in [-1000..1000] (z in [0..1000])."""
    conn = mav_conn or mav
    tgt = target_system if target_system is not None else (args.target_system if args is not None else 1)
    conn.mav.manual_control_send(
        tgt,
        int(max(-1000, min(1000, x))),
        int(max(-1000, min(1000, y))),
        int(max(0, min(1000, z))),
        int(max(-1000, min(1000, r))),
        buttons,
    )


def send_arm(armed: bool, mav_conn=None, target_system=None):
    """MAV_CMD_COMPONENT_ARM_DISARM; param1 = 1 (arm) / 0 (disarm)."""
    global is_armed
    is_armed = bool(armed)
    conn = mav_conn or mav
    tgt = target_system if target_system is not None else (args.target_system if args is not None else 1)
    conn.mav.command_long_send(
        tgt, 1,                                           # autopilot is component 1
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,                                                # confirmation
        1.0 if armed else 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )


def send_set_mode(main_mode: int, sub_mode: int = 0, mav_conn=None, armed: bool | None = None, target_system=None):
    """Switch to a PX4 custom flight mode using `px4_custom_mode(main_mode, sub_mode)`
    while preserving `MAV_MODE_FLAG_SAFETY_ARMED` when armed.
    """
    conn = mav_conn or mav
    tgt = target_system if target_system is not None else (args.target_system if args is not None else 1)
    currently_armed = is_armed if armed is None else bool(armed)

    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if currently_armed:
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    custom_mode = px4_custom_mode(main_mode, sub_mode)
    conn.mav.set_mode_send(
        tgt,
        base_mode,
        custom_mode,
    )


def main() -> None:
    global args, mav, is_armed
    import pygame

    parser = build_arg_parser()
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print('[ERROR] No joystick detected. Plug in an Xbox controller and retry.', file=sys.stderr)
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f'[INFO] Joystick: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)')

    mav_url = f'udpout:{args.ec2_ip}:{args.port}'
    print(f'[INFO] Connecting MAVLink: {mav_url}')
    mav = mavutil.mavlink_connection(
        mav_url,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )

    tick_period = 1.0 / max(args.rate_hz, 1.0)
    send_period = 1.0 / max(args.send_hz, 1.0)
    last_send = 0.0
    last_hb = 0.0

    prev_buttons = [False] * js.get_numbuttons()
    mode_idx = 0
    running = True

    def shutdown(signum=None, frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print('[INFO] Streaming MANUAL_CONTROL. A=arm, B=disarm, Y=mode cycle, Ctrl-C to quit.')

    try:
        while running:
            t = time.time()
            pygame.event.pump()

            # Check for any incoming HEARTBEAT from PX4 to keep armed flag synced
            try:
                hb = mav.recv_match(type='HEARTBEAT', blocking=False)
                if hb is not None and hb.get_srcSystem() == args.target_system:
                    is_armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            except Exception:
                pass

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
                print(f'[INFO] Y pressed -> mode {name} (armed={is_armed})')
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


if __name__ == '__main__':
    main()
