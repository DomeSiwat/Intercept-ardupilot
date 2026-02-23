#!/usr/bin/env python3
import argparse
import asyncio
import time
import math

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ===================== TARGET PROVIDER (DYNAMIC) =====================
# This function returns (lat_deg, lon_deg, alt_m) for the moving target
from fake_gps_square import get_current_lat_lon_alt
# ====================================================================

# ===================== TUNING =====================
MAX_SPEED = 2.0      # m/s horizontal
MIN_SPEED = 0.5      # m/s
MAX_VD = 0.5         # m/s vertical (Down positive in NED)
STOP_RADIUS = 0.3    # meters
ALT_HOLD_BAND = 0.3  # meters
# ==================================================

def clamp(x, lo, hi):
    return max(lo, min(x, hi))

def distance_and_bearing(lat1, lon1, lat2, lon2):
    """Returns (distance_m, bearing_rad_from_north)"""
    R = 6371000.0
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    y = math.sin(dlon) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2) -
         math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    bearing = math.atan2(y, x)
    return distance, bearing

async def watch_armed(drone, shared):
    async for armed in drone.telemetry.armed():
        shared["armed"] = armed

async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--system-address", default="serial:///dev/ttyAMA0:115200")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--enable-after", type=float, default=0.0)
    args = ap.parse_args()

    dt = 1.0 / args.hz

    drone = System()
    print(f"-- Connecting to {args.system_address}")
    await drone.connect(system_address=args.system_address)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            break

    shared = {"armed": False}
    asyncio.create_task(watch_armed(drone, shared))

    # ------------------------------------------------
    # Warm-up setpoints (REQUIRED)
    # ------------------------------------------------
    print("-- Priming setpoints...")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(0.05)

    print("-- Starting offboard...")
    try:
        await drone.offboard.start()
        print("✅ Offboard started")
    except OffboardError as e:
        print("❌ Offboard start failed:", e)
        return

    pos_stream = drone.telemetry.position()
    t0 = time.monotonic()
    last_print = 0.0

    print("\n=== DYNAMIC TARGET FOLLOW (OFFBOARD VELOCITY) ===")
    print("1) REMOVE PROPS for testing")
    print("2) ARM")
    print("3) Vehicle will chase dynamic target from fake_gps_square.get_current_lat_lon_alt()\n")

    while True:
        now = time.monotonic()
        armed = shared["armed"]
        enabled = (now - t0) >= args.enable_after

        # Default: zero velocity
        vn = ve = vd = yaw_deg = 0.0
        dist = 0.0
        bearing = 0.0

        # Get own position
        pos = await pos_stream.__anext__()
        lat = pos.latitude_deg
        lon = pos.longitude_deg
        rel_alt = pos.relative_altitude_m
        abs_alt = pos.absolute_altitude_m

        # Get dynamic target position
        tgt_lat, tgt_lon, tgt_abs_alt = get_current_lat_lon_alt()

        # Convert target absolute altitude to target relative altitude
        # (so we can compare to telemetry.relative_altitude_m)
        tgt_rel_alt = tgt_abs_alt - abs_alt + rel_alt

        if armed and enabled:
            # -------- Horizontal --------
            dist, bearing = distance_and_bearing(lat, lon, tgt_lat, tgt_lon)

            if dist > STOP_RADIUS:
                speed = clamp(dist / 0.2, MIN_SPEED, MAX_SPEED)
                vn = speed * math.cos(bearing)
                ve = speed * math.sin(bearing)
            else:
                vn = ve = 0.0

            # -------- Vertical --------
            alt_error = tgt_rel_alt - rel_alt
            if abs(alt_error) > ALT_HOLD_BAND:
                # NED: +Down, so to go UP we need negative vd
                vd = -clamp(alt_error, -MAX_VD, MAX_VD)
            else:
                vd = 0.0

            yaw_deg = math.degrees(bearing)

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, yaw_deg)
            )

        if now - last_print > 0.5:
            last_print = now
            print(
                f"armed={armed} enabled={enabled} | "
                f"VN={vn:+.2f} VE={ve:+.2f} VD={vd:+.2f} | "
                f"dist={dist:.2f}m | "
                f"me(lat={lat:.6f},lon={lon:.6f},rel={rel_alt:.1f}) | "
                f"tgt(lat={tgt_lat:.6f},lon={tgt_lon:.6f},rel={tgt_rel_alt:.1f})"
            )

        await asyncio.sleep(dt)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
