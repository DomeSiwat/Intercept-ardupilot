#!/usr/bin/env python3
import asyncio
import math
import time
from typing import Tuple

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import FlightMode

# Your target provider (must return lat, lon, alt)
from fake_gps_multiple_vihecle import get_current_lat_lon_alt

# ===================== CONFIG =====================
CONNECT_ADDR = "udpin://0.0.0.0:14540"   # ArduPilot SITL usually outputs to 14540
HZ = 20
DT = 1.0 / HZ

MAX_SPEED = 20.0        # m/s horizontal cap
MIN_SPEED = 1.0         # m/s minimum chase speed when moving
SLOW_RADIUS = 10.0      # m (not strictly used, but kept for tuning)
STOP_RADIUS = 0.20      # m (consider "arrived")

MAX_VD = 1.5         # m/s vertical cap (up/down)
ALT_HOLD_BAND = 10.0     # m, if within this band -> vd=0

# If your target altitude is NOT AMSL (absolute), set this False and use REL alt logic instead.
TARGET_ALT_IS_ABSOLUTE_AMSL = False

# =================================================


def clamp(x, x_min, x_max):
    return max(x_min, min(x, x_max))


def distance_and_bearing(lat1_deg, lon1_deg, lat2_deg, lon2_deg) -> Tuple[float, float]:
    """
    Returns (distance_m, bearing_rad_from_north_clockwise)
    Great-circle distance + initial bearing.
    """
    R = 6371000.0
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (math.sin(dlat / 2.0) ** 2 +
         math.cos(lat1) * math.cos(lat2) * (math.sin(dlon / 2.0) ** 2))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    distance = R * c

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)

    return distance, bearing


def get_target_coordinate() -> Tuple[float, float, float]:
    """
    Returns (lat, lon, alt).
    alt meaning:
      - If TARGET_ALT_IS_ABSOLUTE_AMSL=True: altitude is AMSL meters (same frame as telemetry.absolute_altitude_m)
      - Else: altitude is RELATIVE meters (you must change the altitude control section)
    """
    result = get_current_lat_lon_alt()

    if result is not None and hasattr(result, "__len__") and len(result) >= 3:
        lat, lon, alt = float(result[0]), float(result[1]), float(result[2])
        return lat, lon, alt

    # fallback hardcode
    return (-35.360204, 149.164830 , 50.0)


def is_guided_like(mode) -> bool:
    """
    On ArduPilot, RC GUIDED often appears as OFFBOARD in MAVSDK.
    If your MAVSDK shows a different mode when you flip to GUIDED,
    add it here after you print Mode=... in console.
    """
    return mode == FlightMode.OFFBOARD


async def watch_mode(drone, shared):
    async for mode in drone.telemetry.flight_mode():
        shared["mode"] = mode


async def watch_armed(drone, shared):
    async for armed in drone.telemetry.armed():
        shared["armed"] = armed


async def watch_in_air(drone, shared):
    async for in_air in drone.telemetry.in_air():
        shared["in_air"] = in_air


async def main():
    drone = System()

    print(f"-- Connecting to {CONNECT_ADDR}")
    await drone.connect(system_address=CONNECT_ADDR)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            break

    shared = {"mode": None, "armed": False, "in_air": False}
    asyncio.create_task(watch_mode(drone, shared))
    asyncio.create_task(watch_armed(drone, shared))
    asyncio.create_task(watch_in_air(drone, shared))

    print("\n=== PROCEDURE (ArduPilot) ===")
    print("1) ARM + TAKEOFF ด้วย RC ให้ลอยนิ่ง")
    print("2) Flip RC ไป GUIDED")
    print("   → สคริปต์จะเริ่มไล่ target ด้วย GPS")
    print("3) ออก GUIDED → หยุด (ส่ง 0 velocity ตลอด)\n")

    # ------------------------------------------------
    # MAVSDK OFFBOARD REQUIREMENT
    # ------------------------------------------------
    print("-- Warm-up setpoints")
    for _ in range(30):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(0.05)

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
        print("✅ Offboard started")
    except OffboardError as e:
        # In ArduPilot, sometimes OFFBOARD start is denied until certain conditions are met.
        # But usually in SITL it should work after warm-up.
        print("❌ Offboard start failed:", e._result.result_str)
        return

    # We'll read GPS continuously
    pos_stream = drone.telemetry.position()

    loop = asyncio.get_running_loop()
    next_time = loop.time()
    last_print = 0.0

    try:
        while True:
            mode = shared["mode"]
            armed = shared["armed"]
            in_air = shared["in_air"]
            guided_like = (mode is not None) and is_guided_like(mode)

            # Always read own position (so you see updates even when not guided)
            pos = await pos_stream.__anext__()
            own_lat = pos.latitude_deg
            own_lon = pos.longitude_deg
            own_abs_alt = pos.absolute_altitude_m
            own_rel_alt = pos.relative_altitude_m

            # Default command = ZERO (safety)
            vn = ve = vd = 0.0
            yaw_deg = 0.0

            if armed and in_air and guided_like:
                # Target
                tgt_lat, tgt_lon, tgt_alt = get_target_coordinate()

                # Distance + bearing
                distance_m, bearing_rad = distance_and_bearing(own_lat, own_lon, tgt_lat, tgt_lon)

                # Altitude control
                if TARGET_ALT_IS_ABSOLUTE_AMSL:
                   alt_error = tgt_alt - own_abs_alt
                else:
                    # If your provider gives "altitude relative", use relative frame:
                    alt_error = tgt_alt - own_rel_alt

                if abs(alt_error) <= abs(ALT_HOLD_BAND):
                    vd = 0.0
                else:
                    # NED Down positive; climb => negative
                    vd = -clamp(alt_error, -MAX_VD, MAX_VD)

                # Horizontal chase
                if distance_m < STOP_RADIUS:
                    vn = 0.0
                    ve = 0.0
                else:
                    desired = distance_m / 1.5   # same idea as your PX4 code
                    speed_cmd = clamp(desired, MIN_SPEED, MAX_SPEED)
                    vn = speed_cmd * math.cos(bearing_rad)
                    ve = speed_cmd * math.sin(bearing_rad)

                yaw_deg = math.degrees(bearing_rad)

                await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))
            else:
                # Not in GUIDED-like mode -> hold still (keep sending zeros)
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

            now = time.monotonic()
            if now - last_print > 0.5:
                last_print = now
                print(
                    f"Mode={mode} armed={armed} in_air={in_air} guided_like={guided_like} | "
                    f"VN={vn:+.2f} VE={ve:+.2f} VD={vd:+.2f} yaw={yaw_deg:+.1f} | "
                    f"own=({own_lat:.6f},{own_lon:.6f}) alt_abs={own_abs_alt:.1f} alt_rel={own_rel_alt:.1f}"
                )

            next_time += DT
            await asyncio.sleep(max(0.0, next_time - loop.time()))

    except KeyboardInterrupt:
        print("\n-- Keyboard interrupt")
    finally:
        print("-- Stopping offboard (best effort)")
        try:
            await drone.offboard.stop()
        except OffboardError as e:
            print("-- Offboard stop failed:", e._result.result_str)

        # In ArduPilot SITL you can RTL, but if you prefer manual RC, you can comment this out.
        try:
            await drone.action.return_to_launch()
        except Exception as e:
            print("-- RTL failed (ignored):", e)


if __name__ == "__main__":
    asyncio.run(main())
