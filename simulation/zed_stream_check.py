#!/usr/bin/env python3
"""Open one simulated ZED stream and print a left/right grab.

The Isaac Lab env does not see the SDK libraries until LD_LIBRARY_PATH is set,
and /usr/local/zed is readable by the zed group:

  sg zed -c 'LC_ALL=C LD_LIBRARY_PATH=/usr/local/zed/lib:/usr/local/cuda/lib64 \
    python zed_stream_check.py --port 30000'
"""
from __future__ import annotations

import argparse
import sys
import time


def _save_png(path: str, bgra) -> None:
    """Write a BGRA frame as an RGB PNG."""
    import struct
    import zlib

    height, width = bgra.shape[:2]
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        row = bgra[y]
        for x in range(width):
            b, g, r = int(row[x, 0]), int(row[x, 1]), int(row[x, 2])
            raw.extend((r, g, b))

    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', crc)

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    png = (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', ihdr)
        + chunk(b'IDAT', zlib.compress(bytes(raw), 9))
        + chunk(b'IEND', b'')
    )
    with open(path, 'wb') as handle:
        handle.write(png)


def _report_board_depth(depth_m, finite, intrinsics):
    """Compare SDK depth with the distance along the camera axis to each board.

    The left camera looks along world +Y with image up along world +Z.
    A ZED depth sample is that axial distance, so a board at world y is
    truth ``y - camera_y``. Points are kept only when they land inside that
    board's rectangle. The pose is CameraLeft of the ZED X asset on
    zed1_mount: mount (0, -0.127, -0.1524), RotateZ(90), RotateY(90),
    vehicle at the authored (0, 0, 1.5) with RotateX(90).
    """
    import numpy as np

    fx, fy, cx, cy = intrinsics
    origin = np.array([-0.06, 0.1600742591458515, 1.3889235917185454])
    right = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    look = np.array([0.0, 1.0, 0.0])
    # name, board y, x limits, z limits, tolerance
    boards = (
        ('3 m', 3.0, (-2.2, -0.8), (0.5, 1.9), 0.05),
        ('5 m', 5.0, (-0.7, 0.7), (0.5, 1.9), 0.05),
        ('10 m', 10.0, (1.7, 3.1), (0.5, 1.9), 0.10),
    )
    height, width = depth_m.shape
    vs, us = np.mgrid[0:height, 0:width]
    depth = np.where(finite, depth_m, 0.0).astype(np.float64)
    cam_x = (us - cx) / fx * depth
    cam_y = -(vs - cy) / fy * depth
    points = origin + cam_x[..., None] * right + cam_y[..., None] * up + depth[..., None] * look

    passed = True
    ready = False
    short = False
    for name, board_y, x_limits, z_limits, tolerance in boards:
        truth = board_y - origin[1]
        on_board = (
            finite
            & (points[:, :, 0] >= x_limits[0]) & (points[:, :, 0] <= x_limits[1])
            & (points[:, :, 1] >= board_y - 0.15) & (points[:, :, 1] <= board_y + 0.15)
            & (points[:, :, 2] >= z_limits[0]) & (points[:, :, 2] <= z_limits[1])
        )
        count = int(on_board.sum())
        if count < 30:
            # Axial window is tighter than the 10% band, so also report the
            # depth of whatever falls in the board's x/z footprint.
            footprint = (
                finite
                & (points[:, :, 0] >= x_limits[0]) & (points[:, :, 0] <= x_limits[1])
                & (points[:, :, 2] >= z_limits[0]) & (points[:, :, 2] <= z_limits[1])
            )
            n_foot = int(footprint.sum())
            extra = ''
            if n_foot:
                extra = f' footprint n={n_foot} median={float(np.median(depth[footprint])):.3f} m'
            print(f'[ZED] {name}: {count} depth samples on the board{extra}', flush=True)
            short = True
            continue
        ready = True
        measured = float(np.median(depth[on_board]))
        error = abs(measured - truth) / truth
        ok = error <= tolerance
        passed = passed and ok
        print(
            f'[ZED] {name}: n={count} median={measured:.3f} m truth={truth:.3f} m '
            f'error={100.0 * error:.2f}% limit={100.0 * tolerance:.0f}% '
            f'{"PASS" if ok else "FAIL"}',
            flush=True,
        )
    # A far board can be missing on the first neural frame. Wait for all three.
    if not ready or short:
        return None
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description='Grab one stereo pair from a ZED SDK stream')
    parser.add_argument('--ip', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=30000)
    parser.add_argument('--open-timeout', type=float, default=90.0)
    parser.add_argument('--grab-timeout', type=float, default=30.0)
    parser.add_argument('--save', default=None, help='Write the left view to this PNG path')
    parser.add_argument(
        '--depth',
        action='store_true',
        help='Measure SDK depth on the checkerboards at 3, 5, and 10 m',
    )
    args = parser.parse_args()
    if args.depth and args.grab_timeout == 30.0:
        args.grab_timeout = 480.0

    import numpy as np
    import pyzed.sl as sl

    camera = sl.Camera()
    deadline = time.time() + args.open_timeout
    last_error = None
    while time.time() < deadline:
        init = sl.InitParameters()
        init.set_from_stream(args.ip, args.port)
        if args.depth:
            init.depth_mode = sl.DEPTH_MODE.NEURAL
            init.coordinate_units = sl.UNIT.METER
            init.depth_minimum_distance = 0.3
            init.depth_maximum_distance = 20.0
        status = camera.open(init)
        print(f'[ZED] open {args.ip}:{args.port} -> {status}', flush=True)
        if status == sl.ERROR_CODE.SUCCESS:
            break
        last_error = status
        time.sleep(2.0)
    else:
        print(f'[ZED] stream did not open: {last_error}', flush=True)
        return 1

    calib = camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
    intrinsics = (calib.fx, calib.fy, calib.cx, calib.cy)
    print(
        f'[ZED] intrinsics fx={calib.fx:.1f} fy={calib.fy:.1f} '
        f'cx={calib.cx:.1f} cy={calib.cy:.1f}',
        flush=True,
    )

    runtime = sl.RuntimeParameters()
    left = sl.Mat()
    right = sl.Mat()
    depth = sl.Mat()
    grab_deadline = time.time() + args.grab_timeout
    ok = False
    last_report = 0.0
    while time.time() < grab_deadline:
        grab = camera.grab(runtime)
        if grab != sl.ERROR_CODE.SUCCESS:
            print(f'[ZED] grab={grab}', flush=True)
            time.sleep(0.5)
            continue
        camera.retrieve_image(left, sl.VIEW.LEFT)
        camera.retrieve_image(right, sl.VIEW.RIGHT)
        left_np = left.get_data()
        right_np = right.get_data()
        ok = left_np.size > 0 and right_np.size > 0 and left_np.shape[1] > 1
        if not ok:
            time.sleep(0.2)
            continue
        if not args.depth:
            print(
                f'[ZED] left={tuple(left_np.shape)} mean={float(np.mean(left_np)):.1f} '
                f'right={tuple(right_np.shape)} mean={float(np.mean(right_np)):.1f}',
                flush=True,
            )
            if args.save:
                _save_png(args.save, left_np)
                print(f'[ZED] saved {args.save}', flush=True)
            break
        camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
        depth_np = np.squeeze(depth.get_data())
        finite = np.isfinite(depth_np) & (depth_np > 0.3) & (depth_np < 20.0)
        now = time.time()
        if now - last_report > 15.0:
            last_report = now
            print(
                f'[ZED] left={tuple(left_np.shape)} mean={float(np.mean(left_np)):.1f} '
                f'depth finite={int(finite.sum())}/{depth_np.size}',
                flush=True,
            )
        if int(finite.sum()) < 1000:
            time.sleep(1.0)
            continue
        verdict = _report_board_depth(depth_np, finite, intrinsics)
        if verdict is None:
            time.sleep(1.0)
            continue
        if args.save:
            _save_png(args.save, left_np)
            print(f'[ZED] saved {args.save}', flush=True)
        ok = verdict
        break
    camera.close()
    print(f'[ZED] ok={ok}', flush=True)
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
