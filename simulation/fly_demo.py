#!/usr/bin/env python3
"""
Fly the Go Aero drone in a demo loop: takeoff, fly a square, land, repeat.
Run this AFTER px4_visualizer.py is running.

Usage:
  python fly_demo.py [--loops 3] [--altitude 5] [--size 8] [--speed 3]
"""
import asyncio
import argparse
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

parser = argparse.ArgumentParser()
parser.add_argument('--loops', type=int, default=3, help='Number of loops')
parser.add_argument('--altitude', type=float, default=5.0, help='Flight altitude (m)')
parser.add_argument('--size', type=float, default=8.0, help='Square size (m)')
parser.add_argument('--speed', type=float, default=3.0, help='Flight speed (m/s)')
parser.add_argument('--port', type=int, default=14540, help='MAVSDK UDP port')
args = parser.parse_args()


async def run():
    drone = System()
    await drone.connect(system_address=f"udpin://0.0.0.0:{args.port}")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Wait for GPS fix
    print("Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS fix OK")
            break

    alt = -args.altitude  # NED: negative = up
    half = args.size / 2.0

    # Square waypoints (NED: x=north, y=east)
    waypoints = [
        ("Front-Right", half, half),
        ("Front-Left", half, -half),
        ("Back-Left", -half, -half),
        ("Back-Right", -half, half),
    ]

    # Set initial setpoint before starting offboard
    print("Setting initial setpoint...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, alt, 0.0))

    print("Arming...")
    await drone.action.arm()

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        await drone.action.disarm()
        return

    # Takeoff: go to altitude at center
    print(f"Taking off to {args.altitude}m...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, alt, 0.0))
    await wait_for_position(drone, 0, 0, alt, tolerance=0.5)
    print(f"At altitude!")
    await asyncio.sleep(2)

    # Fly loops
    for loop in range(args.loops):
        print(f"\n=== Loop {loop + 1}/{args.loops} ===")

        for name, north, east in waypoints:
            yaw = 0.0  # face north
            print(f"  Flying to {name} ({north:.1f}, {east:.1f})...")
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, alt, yaw))
            await wait_for_position(drone, north, east, alt, tolerance=1.0)
            print(f"  Reached {name}")
            await asyncio.sleep(0.5)

        # Return to center
        print("  Returning to center...")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, alt, 0.0))
        await wait_for_position(drone, 0, 0, alt, tolerance=0.5)
        await asyncio.sleep(1)

    # Land
    print("\nLanding...")
    await drone.offboard.stop()
    await drone.action.land()

    # Wait for landing
    print("Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed!")
            break

    await asyncio.sleep(2)
    await drone.action.disarm()
    print("Disarmed. Demo complete!")


async def wait_for_position(drone, target_n, target_e, target_d, tolerance=1.0, timeout=30):
    """Wait until drone is within tolerance of target position."""
    for _ in range(int(timeout * 10)):
        async for pos in drone.telemetry.position_velocity_ned():
            n = pos.position.north_m
            e = pos.position.east_m
            d = pos.position.down_m
            dist = ((n - target_n)**2 + (e - target_e)**2 + (d - target_d)**2)**0.5
            if dist < tolerance:
                return
            break
        await asyncio.sleep(0.1)


asyncio.run(run())
