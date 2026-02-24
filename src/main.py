#!/usr/bin/env python3
"""
Intercept flight controller — follow a dynamic GPS target via offboard NED velocity.
"""
import argparse
import asyncio
import math
import time

from mavsdk.offboard import VelocityNedYaw

from config import DEFAULT_SYSTEM_ADDRESS, DEFAULT_HZ, DEFAULT_ENABLE_AFTER
from drone.connection import connect, start_offboard, watch_armed
from navigation.flight import compute_horizontal, compute_vertical, YawController, SpeedController
from UDP_GPS import get_current_lat_lon_alt


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--system-address", default=DEFAULT_SYSTEM_ADDRESS)
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ)
    ap.add_argument("--enable-after", type=float, default=DEFAULT_ENABLE_AFTER)
    args = ap.parse_args()

    dt = 1.0 / args.hz

    # --- Setup ---
    drone = await connect(args.system_address)
    shared = {"armed": False}
    asyncio.create_task(watch_armed(drone, shared))
    await start_offboard(drone)

    yaw = YawController()
    speed_ctrl = SpeedController()
    pos_stream = drone.telemetry.position()

    t0 = time.monotonic()
    last_print = 0.0
    ground_alt = None

    print("\n=== WAYPOINT FOLLOW (DYNAMIC TARGET) ===")
    print("1) REMOVE PROPS for testing")
    print("2) ARM")
    print("3) Drone will follow get_current_lat_lon_alt()")
    print("4) Using relative altitude from first reading\n")

    # --- Loop ---
    while True:
        now = time.monotonic()
        armed = shared["armed"]
        enabled = (now - t0) >= args.enable_after

        vn = ve = vd = yaw_deg = 0.0

        # Current drone position
        pos = await pos_stream.__anext__()
        lat = pos.latitude_deg
        lon = pos.longitude_deg
        abs_alt = pos.absolute_altitude_m

        if ground_alt is None:
            ground_alt = abs_alt
            print(f"[INFO] Ground reference altitude = {ground_alt:.2f} m")

        rel_alt = abs_alt - ground_alt

        # Dynamic target
        target_lat, target_lon, target_abs_alt = get_current_lat_lon_alt()
        target_rel_alt = target_abs_alt - ground_alt

        dist = 0.0
        bearing_rad = 0.0
        target_yaw_deg = 0.0

        if armed and enabled:
            # 1) Get desired speed and bearing to target
            desired_speed, dist, bearing_rad = compute_horizontal(lat, lon, target_lat, target_lon)

            # 2) Yaw controller (dynamic rate: big error → fast turn)
            target_yaw_deg = math.degrees(bearing_rad)
            yaw_deg = yaw.update(target_yaw_deg, dt)

            # 3) Speed controller (yaw coupling: big yaw error → slow down, + acceleration)
            speed = speed_ctrl.update(desired_speed, yaw.yaw_error_deg, dt)

            # 4) Decompose speed into NED
            vn = speed * math.cos(bearing_rad)
            ve = speed * math.sin(bearing_rad)

            # 5) Vertical
            vd = compute_vertical(rel_alt, target_rel_alt)

        # Send command
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))

        # Debug print every 0.5s
        if now - last_print > 0.5:
            last_print = now
            print(f"armed={armed} enabled={enabled}")
            print(f"  Drone:  lat={lat:.6f} lon={lon:.6f} rel_alt={rel_alt:.2f}")
            print(f"  Target: lat={target_lat:.6f} lon={target_lon:.6f} rel_alt={target_rel_alt:.2f}")
            print(f"  Distance: {dist:.2f} m | Bearing: {math.degrees(bearing_rad):.2f} deg")
            print(f"  Yaw: target={target_yaw_deg:.1f} current={yaw.current_yaw_deg:.1f} error={yaw.yaw_error_deg:.1f}")
            print(f"  Speed: {speed_ctrl.current_speed:.2f} m/s")
            print(f"  Velocity cmd: vn={vn:+.2f} ve={ve:+.2f} vd={vd:+.2f}\n")

        await asyncio.sleep(dt)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
