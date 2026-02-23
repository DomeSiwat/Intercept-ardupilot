#!/usr/bin/env python3
import argparse
import asyncio
import time
import math

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ------------------ Import dynamic target ------------------
from fake_gps_square import get_current_lat_lon_alt

# ===================== TUNING =====================
MAX_SPEED = 1.0       # m/s horizontal
MIN_SPEED = 0.5       # m/s
MAX_VD = 0.1          # m/s vertical
STOP_RADIUS = 0.3     # meters
ALT_HOLD_BAND = 0.3   # meters
MAX_YAW_RATE_DEG_S = 20  # deg/s
# ======================================================

def clamp(x, lo, hi):
    return max(lo, min(x, hi))

def distance_and_bearing(lat1, lon1, lat2, lon2):
    """
    Returns (distance_m, bearing_rad_from_north)
    Bearing is measured clockwise from North (0 rad)
    """
    R = 6371000.0  # Earth radius in meters
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula for distance
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    # Bearing
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)  # radians from North

    return distance, bearing

def wrap_to_180(angle):
    """Wrap angle in degrees to [-180, 180]"""
    return (angle + 180) % 360 - 180

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
            print("  ^|^s Connected")
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
        print("  ^|^s Offboard started")
    except OffboardError as e:
        print("  ^|^w Offboard start failed:", e)
        return

    pos_stream = drone.telemetry.position()  # For debugging current position
    t0 = time.monotonic()
    last_print = 0.0

    print("\n=== WAYPOINT FOLLOW (DYNAMIC TARGET) ===")
    print("1) REMOVE PROPS for testing")
    print("2) ARM")
    print("3) Drone will follow the moving target from get_current_lat_lon_alt()")
    print("4) All intermediate calculations will be printed\n")

    current_yaw_deg = 0.0

    while True:
        now = time.monotonic()
        armed = shared["armed"]
        enabled = (now - t0) >= args.enable_after

        # Default: zero velocity
        vn = ve = vd = yaw_deg = 0.0

        # ------------------ Current drone position ------------------
        pos = await pos_stream.__anext__()
        lat = pos.latitude_deg
        lon = pos.longitude_deg
        rel_alt = pos.relative_altitude_m

        # ------------------ Dynamic target ------------------
        target_lat, target_lon, target_alt = get_current_lat_lon_alt()

        if armed:  # and enabled
            # --- Horizontal ---
            dist, bearing_rad = distance_and_bearing(lat, lon, target_lat, target_lon)

            # Convert bearing to yaw angle (deg)
            target_yaw_deg = math.degrees(bearing_rad)
            yaw_error_deg = wrap_to_180(target_yaw_deg - current_yaw_deg)

            # Limit yaw change per timestep (simulate yaw-rate)
            yaw_change = clamp(yaw_error_deg, -MAX_YAW_RATE_DEG_S * dt, MAX_YAW_RATE_DEG_S * dt)
            current_yaw_deg += yaw_change
            yaw_deg = current_yaw_deg

            # Horizontal velocity toward target
            if dist > STOP_RADIUS:
                speed = clamp(dist / 0.2, MIN_SPEED, MAX_SPEED)
                vn = speed * math.cos(bearing_rad)
                ve = speed * math.sin(bearing_rad)
            else:
                vn = ve = 0.0

            # --- Vertical ---
            alt_error = target_alt - rel_alt
            if abs(alt_error) > ALT_HOLD_BAND:
                vd = -clamp(alt_error, -MAX_VD, MAX_VD)
            else:
                vd = 0.0

        # ------------------ Send command ------------------
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, yaw_deg)
        )

        # ------------------ DEBUG PRINT ------------------
        if now - last_print > 0.5:
            last_print = now
            print(f"armed={armed} enabled={enabled}")
            print(f"  Drone GPS pos: lat={lat:.6f} lon={lon:.6f} alt={rel_alt:.2f}")
            print(f"  Target pos: lat={target_lat:.6f} lon={target_lon:.6f} alt={target_alt:.2f}")
            print(f"  Distance to target: {dist:.2f} m")
            print(f"  Bearing to target: {math.degrees(bearing_rad):.2f} deg")
            print(f"  Target yaw: {target_yaw_deg:.2f} deg")
            print(f"  Current yaw: {current_yaw_deg:.2f} deg (yaw-rate limited)")
            print(f"  Velocity command: vn={vn:+.2f} ve={ve:+.2f} vd={vd:+.2f}\n")

        await asyncio.sleep(dt)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
