#!/usr/bin/env python3
import argparse
import asyncio
import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

UP_VD = -0.3   # gentle "up" command (NED down is +, so up is -)

async def watch_armed(drone, shared):
    async for armed in drone.telemetry.armed():
        shared["armed"] = armed

async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--system-address", default="serial:///dev/ttyAMA0:115200")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--enable-after", type=float, default=0.0,
                    help="Wait N seconds before commanding UP (default 0)")
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

    # Warm-up setpoints
    print("-- Priming setpoints...")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.05)

    # Start offboard
    print("-- Starting offboard...")
    try:
        await drone.offboard.start()
        print("✅ Offboard started")
    except OffboardError as e:
        print("❌ Offboard start failed:", e)
        return

    t0 = time.monotonic()
    last_print = 0.0

    print("\n=== PROP-OFF MOTOR TEST ===")
    print("1) REMOVE PROPS")
    print("2) ARM (RC or QGC)")
    print("3) Script will command UP after enable-after seconds")
    print("4) DISARM to stop motors\n")

    while True:
        now = time.monotonic()
        armed = shared["armed"]

        enabled = (now - t0) >= args.enable_after
        v_d = 0.0

        if armed and enabled:
            v_d = UP_VD  # motors should ramp together a bit

        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, v_d, 0.0))

        if now - last_print > 0.5:
            last_print = now
            cmd = "UP (vd<0)" if v_d < 0 else "ZERO"
            print(f"armed={armed} enabled={enabled} cmd={cmd}")

        await asyncio.sleep(dt)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
