#!/usr/bin/env python3
import asyncio
from mavsdk import System

ADDR = "udpin://0.0.0.0:14540"

async def main():
    drone = System()
    print(f"-- Connecting to {ADDR}")

    try:
        await asyncio.wait_for(drone.connect(system_address=ADDR), timeout=5.0)
        print("-- connect() returned ✅")
    except asyncio.TimeoutError:
        print("❌ connect() timed out: no MAVLink packets received on 14540")
        return

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        print("   is_connected =", state.is_connected)
        if state.is_connected:
            print("-- Connected ✅")
            break

if __name__ == "__main__":
    asyncio.run(main())
