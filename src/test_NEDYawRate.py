#!/usr/bin/env python3
import argparse
import asyncio
import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


async def watch_armed(drone, shared):
    async for armed in drone.telemetry.armed():
        shared["armed"] = armed


async def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--system-address", default="serial:///dev/ttyAMA0:115200")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--enable-after", type=float, default=0.0,
                    help="Wait N seconds before commanding velocity")

    # 🔹 Direction parameters (NED + yaw)
    ap.add_argument("--vn", type=float, default=0.0, help="North velocity (m/s)")
    ap.add_argument("--ve", type=float, default=0.0, help="East velocity (m/s)")
    ap.add_argument("--vd", type=float, default=0.0, help="Down velocity (m/s)")
    ap.add_argument("--yaw", type=float, default=0.0, help="Yaw rate (deg/s)")

    args = ap.parse_args()
    dt = 1.0 / args.hz

    drone = System()
    print(f"-- Connecting to {args.system_address}")
    await drone.connect(system_address=args.system_address)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" ✓ Connected")
            break

    shared = {"armed": False}
    asyncio.create_task(watch_armed(drone, shared))

    # Warm-up setpoints (required by Offboard)
    print("-- Priming setpoints...")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

    print("-- Starting offboard...")
    try:
        await drone.offboard.start()
        print(" ✓ Offboard started")
    except OffboardError as e:
        print(" ✗ Offboard start failed:", e)
        return

    t0 = time.monotonic()
    last_print = 0.0

    print("\n=== PROP-OFF MOTOR TEST ===")
    print("1) REMOVE PROPS")
    print("2) ARM (RC or QGC)")
    print("3) Velocity will be applied after enable-after seconds")
    print("4) DISARM to stop motors\n")

    while True:
        now = time.monotonic()
        armed = shared["armed"]
        enabled = (now - t0) >= args.enable_after

        if armed and enabled:
            vn, ve, vd, yaw = args.vn, args.ve, args.vd, args.yaw
        else:
            vn = ve = vd = yaw = 0.0

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, yaw)
        )

        if now - last_print > 0.5:
            last_print = now
            print(
                f"armed={armed} enabled={enabled} | "
                f"vn={vn:.2f} ve={ve:.2f} vd={vd:.2f} yaw={yaw:.1f}"
            )

        await asyncio.sleep(dt)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
