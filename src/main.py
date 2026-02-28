#!/usr/bin/env python3
"""
Main flight controller
- GPS follow mode
- Camera chase mode
- Seamless switching
"""

import argparse
import asyncio
import math
import time
import sys
import termios
import tty
import select

from mavsdk.offboard import VelocityNedYaw

from config import DEFAULT_SYSTEM_ADDRESS, DEFAULT_HZ, DEFAULT_ENABLE_AFTER
from drone.connection import connect, start_offboard, watch_armed
from navigation.flight import compute_horizontal, compute_vertical, YawController, SpeedController
from UDP_GPS import get_current_lat_lon_alt

from camera_chase import CameraChaseController


# ============================================================
# Non-blocking keyboard
# ============================================================

def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


# ============================================================

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

    # GPS controllers
    yaw = YawController()
    speed_ctrl = SpeedController()
    pos_stream = drone.telemetry.position()

    # Camera controller
    camera_ctrl = CameraChaseController()

    t0 = time.monotonic()
    ground_alt = None
    mode = "GPS"

    print("\n==============================")
    print("Press 'c' → CAMERA MODE")
    print("Press 'g' → GPS MODE")
    print("==============================\n")

    # Enable non-blocking terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            now = time.monotonic()
            armed = shared["armed"]
            enabled = (now - t0) >= args.enable_after

            # -------------------------
            # Keyboard Mode Switching
            # -------------------------
            key = get_key_nonblocking()
            if key == "c" and mode != "CAMERA":
                mode = "CAMERA"
                camera_ctrl.current_yaw_deg = yaw.current_yaw_deg
                print(">>> CAMERA MODE")

            elif key == "g" and mode != "GPS":
                mode = "GPS"
                yaw.current_yaw_deg = camera_ctrl.current_yaw_deg
                print(">>> GPS MODE")

            vn = ve = vd = yaw_deg = 0.0

            # ====================================================
            # ACTIVE CONTROL
            # ====================================================
            if armed and enabled:

                # ------------------------------------------------
                # CAMERA CHASE MODE
                # ------------------------------------------------
                if mode == "CAMERA":

                    vn, ve, vd, yaw_deg = camera_ctrl.update()

                # ------------------------------------------------
                # GPS FOLLOW MODE
                # ------------------------------------------------
                else:
                    pos = await pos_stream.__anext__()

                    lat = pos.latitude_deg
                    lon = pos.longitude_deg
                    abs_alt = pos.absolute_altitude_m

                    if ground_alt is None:
                        ground_alt = abs_alt
                        print(f"[INFO] Ground ref alt = {ground_alt:.2f} m")

                    rel_alt = abs_alt - ground_alt

                    target_lat, target_lon, target_abs_alt = get_current_lat_lon_alt()
                    target_rel_alt = target_abs_alt - ground_alt

                    desired_speed, dist, bearing_rad = compute_horizontal(
                        lat, lon, target_lat, target_lon
                    )

                    target_yaw_deg = math.degrees(bearing_rad)
                    yaw_deg = yaw.update(target_yaw_deg, dt)

                    speed = speed_ctrl.update(desired_speed, yaw.yaw_error_deg, dt)

                    vn = speed * math.cos(bearing_rad)
                    ve = speed * math.sin(bearing_rad)
                    vd = compute_vertical(rel_alt, target_rel_alt)

            # ====================================================
            # Send Offboard Command
            # ====================================================
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, yaw_deg)
            )

            # Optional debug
            # print(f"{mode} | vn={vn:.2f} ve={ve:.2f} vd={vd:.2f} yaw={yaw_deg:.1f}")

            await asyncio.sleep(dt)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    asyncio.run(main())