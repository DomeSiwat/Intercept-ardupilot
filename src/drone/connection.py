import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


async def connect(system_address):
    """Connect to the drone and wait until connected. Returns the System object."""
    drone = System()
    print(f"-- Connecting to {system_address}")
    await drone.connect(system_address=system_address)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected")
            break
    return drone


async def start_offboard(drone):
    """Prime setpoints and start offboard mode."""
    # Warm-up setpoints (REQUIRED for Offboard)
    print("-- Priming setpoints...")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(0.05)

    print("-- Starting offboard...")
    try:
        await drone.offboard.start()
        print("✓ Offboard started")
    except OffboardError as e:
        print("Offboard start failed:", e)
        raise


async def watch_armed(drone, shared):
    """Background task: keep shared['armed'] up to date."""
    async for armed in drone.telemetry.armed():
        shared["armed"] = armed
