#!/usr/bin/env python3
"""
ArduPilot Copter SITL flight & obstacle avoidance test script (`ardu_poshold.py`).

Supports two modes of operation:
  1. `poshold` (default):
     - Connects via pymavlink on `udpin:0.0.0.0:14540` (configurable via `--port`).
     - Waits for heartbeat and EKF GPS readiness.
     - Switches to `GUIDED`, arms, and takes off to `--altitude` (default 5.0 m).
     - Sets RC3 throttle override (`--throttle-pct 50` -> 1500 PWM), switches to
       `POSHOLD`, holds for `--hold` seconds while streaming periodic RC override
       and GCS heartbeats, then switches to `LAND`, clears RC overrides, and disarms.

  2. `guided-forward` (or when `--target-north N` is passed):
     - Takes off in `GUIDED` to `--altitude` (default 5.0 m).
     - Commands a forward position setpoint `(north=target_north, east=0, down=-altitude)`
       past the obstacle at `--obstacle-north` (default 5.0 m).
     - Logs `LOCAL_POSITION_NED` and any `OBSTACLE_DISTANCE_3D` / `OBSTACLE_DISTANCE`
       telemetry, verifying that ArduPilot's Simple Avoidance stops the vehicle at
       `AVOID_MARGIN` (default 2.0 m) in front of the obstacle before landing.

Usage:
  # Standard PosHold hover test:
  python3 ~/go_aero/ardu_poshold.py --altitude 5 --throttle-pct 50 --hold 30

  # Obstacle avoidance forward stop test (target 8m north toward 5m obstacle):
  python3 ~/go_aero/ardu_poshold.py --mode guided-forward --target-north 8 --altitude 2.5 --hold 20
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

# Standard ArduCopter custom_mode mapping
ARDUCOPTER_MODES: dict[str, int] = {
    'STABILIZE': 0,
    'ACRO': 1,
    'ALT_HOLD': 2,
    'AUTO': 3,
    'GUIDED': 4,
    'LOITER': 5,
    'RTL': 6,
    'CIRCLE': 7,
    'LAND': 9,
    'DRIFT': 11,
    'SPORT': 13,
    'FLIP': 14,
    'AUTOTUNE': 15,
    'POSHOLD': 16,
    'BRAKE': 17,
    'THROW': 18,
    'AVOID_ADSB': 19,
    'GUIDED_NOGPS': 20,
    'SMART_RTL': 21,
}

# SET_POSITION_TARGET_LOCAL_NED type_mask:
# Bit 9 (FORCE_SET = 0x0200) MUST be 0; ignore vel (0x0038), accel (0x01C0),
# and yaw_rate (0x0800), while using position (X, Y, Z) + yaw -> 0x09F8 = 2552.
POSITION_TARGET_TYPEMASK_POS_ONLY = 0b0000_1001_1111_1000


def throttle_pct_to_pwm(
    throttle_pct: float,
    pwm_min: int = 1000,
    pwm_max: int = 2000,
) -> int:
    """Convert a throttle percentage [0, 100] to RC PWM microseconds [1000, 2000]."""
    if pwm_min > pwm_max:
        pwm_min, pwm_max = pwm_max, pwm_min
    val = float(throttle_pct)
    pct = max(0.0, min(100.0, val)) if math.isfinite(val) else 0.0
    pwm = int(round(pwm_min + (pct / 100.0) * (pwm_max - pwm_min)))
    return max(pwm_min, min(pwm_max, pwm))


def evaluate_obstacle_stopping_margin(
    current_north_m: float,
    obstacle_north_m: float = 5.0,
    avoid_margin_m: float = 2.0,
    tolerance_m: float = 0.75,
) -> dict[str, float | bool]:
    """
    Evaluate whether the vehicle stopped at `avoid_margin_m` in front of an obstacle
    at `obstacle_north_m` along the North (+X NED) axis.
    """
    dist_to_obstacle = float(obstacle_north_m) - float(current_north_m)
    expected_stop_north = float(obstacle_north_m) - float(avoid_margin_m)
    stopped_before_collision = (
        dist_to_obstacle > 0.0
        and dist_to_obstacle >= (float(avoid_margin_m) - float(tolerance_m))
    )
    near_expected_margin = abs(float(current_north_m) - expected_stop_north) <= float(tolerance_m)
    return {
        'current_north_m': float(current_north_m),
        'obstacle_north_m': float(obstacle_north_m),
        'avoid_margin_m': float(avoid_margin_m),
        'distance_to_obstacle_m': dist_to_obstacle,
        'expected_stop_north_m': expected_stop_north,
        'stopped_before_collision': stopped_before_collision,
        'passed': bool(stopped_before_collision and near_expected_margin),
    }


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the CLI argument parser for `ardu_poshold.py`."""
    parser = argparse.ArgumentParser(
        description='ArduPilot Copter SITL PosHold & GUIDED Obstacle Avoidance flight script'
    )
    parser.add_argument(
        '--altitude',
        type=float,
        default=5.0,
        help='Target takeoff altitude in meters (default: 5.0)',
    )
    parser.add_argument(
        '--throttle-pct',
        type=float,
        default=50.0,
        help='RC3 throttle override percentage during POSHOLD (default: 50 -> 1500 PWM)',
    )
    parser.add_argument(
        '--hold',
        type=float,
        default=30.0,
        help='Duration in seconds to hold in POSHOLD or GUIDED forward test (default: 30.0)',
    )
    parser.add_argument(
        '--port',
        type=int,
        default=14540,
        help='MAVLink UDP input port (default: 14540)',
    )
    parser.add_argument(
        '--mode',
        choices=('poshold', 'guided-forward'),
        default='poshold',
        help='Flight test mode: poshold (default) or guided-forward (obstacle avoidance test)',
    )
    parser.add_argument(
        '--target-north',
        type=float,
        default=None,
        help='Target North position (m) in GUIDED mode (implies --mode guided-forward; default 8.0m)',
    )
    parser.add_argument(
        '--obstacle-north',
        type=float,
        default=5.0,
        help='North position (m) of the obstacle in the scene (default: 5.0)',
    )
    parser.add_argument(
        '--avoid-margin',
        type=float,
        default=2.0,
        help='Expected ArduPilot AVOID_MARGIN in meters (default: 2.0)',
    )
    return parser


def send_gcs_heartbeat(mav: Any, mavutil_mod: Any) -> None:
    """Send a GCS HEARTBEAT to keep ArduPilot's GCS link active."""
    mav.mav.heartbeat_send(
        mavutil_mod.mavlink.MAV_TYPE_GCS,
        mavutil_mod.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def set_copter_mode(mav: Any, mavutil_mod: Any, mode_name: str, timeout: float = 10.0) -> bool:
    """Switch ArduCopter to `mode_name` and wait for HEARTBEAT confirmation."""
    mapping = mav.mode_mapping() or ARDUCOPTER_MODES
    mode_id = mapping.get(mode_name, ARDUCOPTER_MODES.get(mode_name))
    if mode_id is None:
        raise ValueError(f'Unknown ArduCopter mode: {mode_name}')

    print(f'[INFO] Switching mode to {mode_name} (mode_id={mode_id})...', flush=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil_mod.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if hb is not None and hb.get_srcSystem() == mav.target_system:
            if hb.custom_mode == mode_id:
                print(f'[INFO] Mode confirmed: {mode_name}', flush=True)
                return True
    print(f'[WARN] Timed out waiting for mode {mode_name} confirmation', flush=True)
    return False


def wait_for_ekf_gps(mav: Any, timeout: float = 45.0) -> None:
    """Wait until ArduPilot reports a 3D GPS fix and valid EKF horizontal position."""
    print('[INFO] Waiting for EKF + 3D GPS readiness...', flush=True)
    deadline = time.time() + timeout
    gps_ok = False
    ekf_ok = False

    while time.time() < deadline:
        msg = mav.recv_match(
            type=['GPS_RAW_INT', 'EKF_STATUS_REPORT', 'STATUSTEXT'],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype == 'STATUSTEXT':
            text = getattr(msg, 'text', '')
            if text:
                print(f'[AUTOPILOT] {text}', flush=True)
            if 'using GPS' in text:
                ekf_ok = True
        elif mtype == 'GPS_RAW_INT':
            if msg.fix_type >= 3:
                gps_ok = True
        elif mtype == 'EKF_STATUS_REPORT':
            # EKF_POS_HORIZ_ABS = 0x10 (16), EKF_CONST_POS_MODE = 0x80 (128)
            if (msg.flags & 0x10) != 0 and (msg.flags & 0x80) == 0:
                ekf_ok = True

        if gps_ok and ekf_ok:
            print('[INFO] EKF and 3D GPS fix ready.', flush=True)
            return

    print('[WARN] Proceeding after EKF wait timeout; attempting arm anyway.', flush=True)


def arm_and_takeoff(mav: Any, mavutil_mod: Any, target_alt_m: float, timeout: float = 45.0) -> None:
    """Arm ArduCopter in GUIDED mode and climb to `target_alt_m`."""
    set_copter_mode(mav, mavutil_mod, 'GUIDED')

    print('[INFO] Arming motors...', flush=True)
    arm_deadline = time.time() + 30.0
    armed = False
    last_hb = 0.0
    while time.time() < arm_deadline and not armed:
        now = time.time()
        if now - last_hb >= 1.0:
            last_hb = now
            send_gcs_heartbeat(mav, mavutil_mod)

        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil_mod.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        t_wait = time.time() + 2.0
        while time.time() < t_wait:
            msg = mav.recv_match(type=['HEARTBEAT', 'COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                print(f'[AUTOPILOT] {msg.text}', flush=True)
            elif msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == mav.target_system:
                if (msg.base_mode & mavutil_mod.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
                    armed = True
                    break

    if not armed:
        raise RuntimeError('Failed to arm ArduCopter within 30s (check EKF/pre-arm status)')
    print('[INFO] Motors ARMED.', flush=True)

    # Send MAV_CMD_NAV_TAKEOFF and check COMMAND_ACK (retrying if needed)
    takeoff_accepted = False
    for attempt in range(1, 6):
        print(
            f'[INFO] Sending MAV_CMD_NAV_TAKEOFF to {target_alt_m:.1f} m (attempt {attempt}/5)...',
            flush=True,
        )
        send_gcs_heartbeat(mav, mavutil_mod)
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil_mod.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            float(target_alt_m),
        )
        ack_deadline = time.time() + 2.0
        while time.time() < ack_deadline:
            ack = mav.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=0.5)
            if ack is None:
                continue
            if ack.get_type() == 'STATUSTEXT':
                print(f'[AUTOPILOT] {ack.text}', flush=True)
            elif (
                ack.get_type() == 'COMMAND_ACK'
                and ack.command == mavutil_mod.mavlink.MAV_CMD_NAV_TAKEOFF
            ):
                if ack.result == mavutil_mod.mavlink.MAV_RESULT_ACCEPTED:
                    takeoff_accepted = True
                    print('[INFO] MAV_CMD_NAV_TAKEOFF accepted.', flush=True)
                else:
                    print(f'[WARN] MAV_CMD_NAV_TAKEOFF ACK result={ack.result}', flush=True)
                break
        if takeoff_accepted:
            break

    climb_deadline = time.time() + timeout
    last_log = 0.0
    last_hb = 0.0
    while time.time() < climb_deadline:
        now = time.time()
        if now - last_hb >= 1.0:
            last_hb = now
            send_gcs_heartbeat(mav, mavutil_mod)

        msg = mav.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED':
            alt_m = -float(msg.z)
        else:
            alt_m = float(msg.relative_alt) / 1000.0

        now = time.time()
        if now - last_log >= 1.5:
            last_log = now
            print(f'[TAKEOFF] altitude={alt_m:.2f}m / {target_alt_m:.2f}m', flush=True)

        if alt_m >= target_alt_m - 0.4:
            print(f'[INFO] Reached target altitude ({alt_m:.2f} m).', flush=True)
            return

    print(f'[WARN] Takeoff timeout reached before {target_alt_m:.1f} m; continuing.', flush=True)


def send_rc_override(
    mav: Any,
    roll_pwm: int = 1500,
    pitch_pwm: int = 1500,
    throttle_pwm: int = 1500,
    yaw_pwm: int = 1500,
) -> None:
    """Send RC_CHANNELS_OVERRIDE for channels 1..4."""
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        int(roll_pwm),
        int(pitch_pwm),
        int(throttle_pwm),
        int(yaw_pwm),
        0, 0, 0, 0,
    )


def send_guided_position_ned(
    mav: Any,
    mavutil_mod: Any,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_rad: float = 0.0,
) -> None:
    """Command a local NED position target in ArduCopter GUIDED mode."""
    mav.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        mav.target_system,
        mav.target_component,
        mavutil_mod.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_TARGET_TYPEMASK_POS_ONLY,
        float(north_m),
        float(east_m),
        float(down_m),
        0.0, 0.0, 0.0,  # vx, vy, vz
        0.0, 0.0, 0.0,  # afx, afy, afz
        float(yaw_rad),
        0.0,            # yaw_rate
    )


def run_poshold_phase(mav: Any, mavutil_mod: Any, throttle_pct: float, hold_s: float) -> None:
    """Set RC3 override, switch to POSHOLD, maintain hover for `hold_s` seconds,
    then switch to LAND mode BEFORE clearing RC overrides.
    """
    rc3_pwm = throttle_pct_to_pwm(throttle_pct)
    print(
        f'[POSHOLD] Setting RC3 throttle override to {throttle_pct:.1f}% ({rc3_pwm} PWM)',
        flush=True,
    )
    send_rc_override(mav, 1500, 1500, rc3_pwm, 1500)
    set_copter_mode(mav, mavutil_mod, 'POSHOLD')

    start_t = time.time()
    last_log = 0.0
    pos_ned = [0.0, 0.0, 0.0]

    while time.time() - start_t < hold_s:
        # Keep RC override and GCS heartbeat alive at 10 Hz
        send_rc_override(mav, 1500, 1500, rc3_pwm, 1500)
        send_gcs_heartbeat(mav, mavutil_mod)

        msg = mav.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'OBSTACLE_DISTANCE_3D'],
            blocking=True,
            timeout=0.1,
        )
        if msg is not None and msg.get_type() == 'LOCAL_POSITION_NED':
            pos_ned = [float(msg.x), float(msg.y), float(msg.z)]

        elapsed = time.time() - start_t
        if elapsed - last_log >= 2.0:
            last_log = elapsed
            print(
                f'[POSHOLD] t={elapsed:4.1f}/{hold_s:.0f}s  '
                f'N={pos_ned[0]:+.2f}m E={pos_ned[1]:+.2f}m Alt={-pos_ned[2]:+.2f}m  '
                f'RC3={rc3_pwm}',
                flush=True,
            )

    # Switch to LAND mode BEFORE clearing RC overrides so POSHOLD never sees a
    # sudden zero/default throttle jump while still active.
    set_copter_mode(mav, mavutil_mod, 'LAND')
    send_rc_override(mav, 0, 0, 0, 0)


def run_guided_forward_obstacle_phase(
    mav: Any,
    mavutil_mod: Any,
    altitude_m: float,
    target_north_m: float,
    obstacle_north_m: float,
    avoid_margin_m: float,
    hold_s: float,
) -> dict[str, float | bool]:
    """
    Command a forward position target (`target_north_m` North) in GUIDED mode and
    verify that ArduPilot's Simple Avoidance stops at `avoid_margin_m` in front
    of the obstacle at `obstacle_north_m`.
    """
    # Record initial North position before commanding forward motion
    start_north = 0.0
    pos_received = False
    init_deadline = time.time() + 2.0
    while time.time() < init_deadline:
        msg = mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if msg is not None:
            start_north = float(msg.x)
            pos_received = True
            break

    print(
        f'[AVOID-TEST] start_north={start_north:+.2f}m -> Commanding GUIDED position target: '
        f'North={target_north_m:.1f}m (obstacle at {obstacle_north_m:.1f}m, '
        f'AVOID_MARGIN={avoid_margin_m:.1f}m)',
        flush=True,
    )
    down_m = -abs(float(altitude_m))
    start_t = time.time()
    last_cmd = 0.0
    last_log = 0.0

    current_north = start_north
    current_east = 0.0
    current_alt = altitude_m
    current_vx = 0.0
    max_north = start_north
    min_obs_3d_m: float | None = None

    while time.time() - start_t < hold_s:
        now = time.time()
        if now - last_cmd >= 0.2:
            last_cmd = now
            send_guided_position_ned(mav, mavutil_mod, target_north_m, 0.0, down_m, 0.0)
            send_gcs_heartbeat(mav, mavutil_mod)

        msg = mav.recv_match(
            type=['LOCAL_POSITION_NED', 'OBSTACLE_DISTANCE_3D', 'OBSTACLE_DISTANCE', 'STATUSTEXT'],
            blocking=True,
            timeout=0.1,
        )
        if msg is not None:
            mtype = msg.get_type()
            if mtype == 'LOCAL_POSITION_NED':
                pos_received = True
                current_north = float(msg.x)
                current_east = float(msg.y)
                current_alt = -float(msg.z)
                current_vx = float(msg.vx)
                max_north = max(max_north, current_north)
            elif mtype == 'OBSTACLE_DISTANCE_3D':
                d3 = math.sqrt(float(msg.x) ** 2 + float(msg.y) ** 2 + float(msg.z) ** 2)
                if min_obs_3d_m is None or d3 < min_obs_3d_m:
                    min_obs_3d_m = d3
            elif mtype == 'OBSTACLE_DISTANCE':
                valid = [d / 100.0 for d in msg.distances if 10 < d < 65535]
                if valid:
                    dmin = min(valid)
                    if min_obs_3d_m is None or dmin < min_obs_3d_m:
                        min_obs_3d_m = dmin
            elif mtype == 'STATUSTEXT':
                print(f'[AUTOPILOT] {msg.text}', flush=True)

        elapsed = now - start_t
        if elapsed - last_log >= 1.0:
            last_log = elapsed
            dist_to_obs = obstacle_north_m - current_north
            obs_str = f'{min_obs_3d_m:.2f}m' if min_obs_3d_m is not None else 'n/a'
            print(
                f'[AVOID-TEST] t={elapsed:4.1f}s  N={current_north:+.2f}m '
                f'E={current_east:+.2f}m Alt={current_alt:+.2f}m  '
                f'vx={current_vx:+.2f}m/s  dist_to_obstacle={dist_to_obs:.2f}m  '
                f'OBSTACLE_DISTANCE_3D_min={obs_str}',
                flush=True,
            )

    if not pos_received:
        print('[ERROR] No LOCAL_POSITION_NED telemetry received during forward phase!', flush=True)

    # Evaluate settled stopping margin and also verify max_north never collided
    settled_eval = evaluate_obstacle_stopping_margin(
        current_north_m=current_north,
        obstacle_north_m=obstacle_north_m,
        avoid_margin_m=avoid_margin_m,
    )
    peak_eval = evaluate_obstacle_stopping_margin(
        current_north_m=max_north,
        obstacle_north_m=obstacle_north_m,
        avoid_margin_m=avoid_margin_m,
    )
    no_collision = bool(max_north < obstacle_north_m and peak_eval['stopped_before_collision'])
    moved_forward = bool((max_north - start_north) >= 0.5)
    passed = bool(
        pos_received
        and moved_forward
        and no_collision
        and settled_eval['passed']
    )

    result: dict[str, float | bool] = {
        **settled_eval,
        'start_north_m': float(start_north),
        'max_north_m': float(max_north),
        'settled_north_m': float(current_north),
        'pos_received': pos_received,
        'no_collision': no_collision,
        'passed': passed,
    }
    status = 'PASS' if result['passed'] else 'FAIL'
    print(
        f'[{status}] Obstacle stop result: start_north={start_north:.2f}m, '
        f'settled_north={current_north:.2f}m, max_north={max_north:.2f}m, '
        f'distance_to_obstacle={result["distance_to_obstacle_m"]:.2f}m '
        f'(expected ~{avoid_margin_m:.2f}m margin before {obstacle_north_m:.2f}m obstacle)',
        flush=True,
    )
    return result


def land_and_disarm(mav: Any, mavutil_mod: Any, timeout: float = 45.0) -> None:
    """Switch to LAND mode, clear RC overrides, wait for touchdown/disarm, and disarm cleanly."""
    set_copter_mode(mav, mavutil_mod, 'LAND')
    send_rc_override(mav, 0, 0, 0, 0)
    print('[INFO] Waiting for landing...', flush=True)

    deadline = time.time() + timeout
    last_hb = 0.0
    while time.time() < deadline:
        now = time.time()
        if now - last_hb >= 1.0:
            last_hb = now
            send_gcs_heartbeat(mav, mavutil_mod)

        msg = mav.recv_match(
            type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'HEARTBEAT'],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype == 'HEARTBEAT' and msg.get_srcSystem() == mav.target_system:
            if (msg.base_mode & mavutil_mod.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == 0:
                print('[INFO] Landed and auto-disarmed.', flush=True)
                return
        elif mtype == 'LOCAL_POSITION_NED':
            if -float(msg.z) < 0.25 and abs(float(msg.vz)) < 0.2:
                break
        elif mtype == 'GLOBAL_POSITION_INT':
            if float(msg.relative_alt) / 1000.0 < 0.25:
                break

    print('[INFO] Disarming motors...', flush=True)
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil_mod.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    time.sleep(1.0)
    print('[INFO] Flight complete.', flush=True)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    from pymavlink import mavutil

    effective_mode = 'guided-forward' if args.target_north is not None else args.mode
    target_north = args.target_north if args.target_north is not None else 8.0

    print(f'[INFO] Connecting to ArduPilot on udpin:0.0.0.0:{args.port}...', flush=True)
    mav = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.port}', source_system=255)

    print('[INFO] Waiting for ArduPilot heartbeat...', flush=True)
    hb = mav.wait_heartbeat(timeout=30)
    if hb is None:
        print('[ERROR] No heartbeat received within 30s.', flush=True)
        sys.exit(1)
    print(
        f'[INFO] Connected to system {mav.target_system}, component {mav.target_component}',
        flush=True,
    )

    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        20,
        1,
    )

    result: dict[str, float | bool] | None = None
    try:
        wait_for_ekf_gps(mav)
        arm_and_takeoff(mav, mavutil, args.altitude)

        if effective_mode == 'guided-forward':
            result = run_guided_forward_obstacle_phase(
                mav=mav,
                mavutil_mod=mavutil,
                altitude_m=args.altitude,
                target_north_m=target_north,
                obstacle_north_m=args.obstacle_north,
                avoid_margin_m=args.avoid_margin,
                hold_s=args.hold,
            )
        else:
            run_poshold_phase(
                mav=mav,
                mavutil_mod=mavutil,
                throttle_pct=args.throttle_pct,
                hold_s=args.hold,
            )
    except KeyboardInterrupt:
        print('\n[INFO] Interrupted! Switching to LAND and clearing overrides...', flush=True)
    finally:
        try:
            land_and_disarm(mav, mavutil)
        except Exception as exc:
            print(f'[WARN] Landing cleanup error: {exc}', flush=True)

    if result is not None and not result.get('passed', False):
        sys.exit(1)


if __name__ == '__main__':
    main()
