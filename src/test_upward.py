#!/usr/bin/env python3
import asyncio
import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError
from mavsdk.telemetry import FlightMode

# ===================== CONFIG =====================
UP_VD = -0.5        # m/s  (NED: negative Down = UP)
HZ = 20
DT = 1.0 / HZ
CONNECT_ADDR = "udpin://0.0.0.0:14540"
# =================================================


def is_guided_like(mode) -> bool:
    """
    ArduPilot GUIDED often appears as OFFBOARD in MAVSDK
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
            print("✅ Connected to ArduPilot SITL")
            break

    shared = {
        "mode": None,
        "armed": False,
        "in_air": False,
    }

    # Telemetry watchers
    asyncio.create_task(watch_mode(drone, shared))
    asyncio.create_task(watch_armed(drone, shared))
    asyncio.create_task(watch_in_air(drone, shared))

    print("\n=== TEST PROCEDURE ===")
    print("1) ARM + TAKEOFF ด้วย RC ให้ลอยนิ่ง")
    print("2) Flip RC ไปที่ GUIDED")
    print("   → Drone จะไต่ขึ้น")
    print("3) ออก GUIDED → หยุดทันที\n")

    # ------------------------------------------------
    # MAVSDK OFFBOARD REQUIREMENT (VERY IMPORTANT)
    # ------------------------------------------------
    print("-- Sending warm-up setpoints")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.05)

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
        print("✅ Offboard started")
    except OffboardError as e:
        print("❌ Offboard start failed:", e._result.result_str)
        return

    # ------------------------------------------------
    # CONTROL LOOP
    # ------------------------------------------------
    loop = asyncio.get_running_loop()
    next_time = loop.time()
    last_print = 0.0

    while True:
        mode = shared["mode"]
        armed = shared["armed"]
        in_air = shared["in_air"]

        guided_like = (mode is not None) and is_guided_like(mode)

        # Default = ZERO
        v_d = 0.0

        # Only climb if REALLY flying + GUIDED
        if armed and in_air and guided_like:
            v_d = UP_VD

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, v_d, 0.0)
        )

        now = time.monotonic()
        if now - last_print > 0.5:
            last_print = now
            cmd = "UP ↑" if v_d < 0 else "ZERO"
            print(
                f"Mode={mode} | armed={armed} in_air={in_air} | CMD={cmd}"
            )

        next_time += DT
        await asyncio.sleep(max(0.0, next_time - loop.time()))


if __name__ == "__main__":
    asyncio.run(main())
