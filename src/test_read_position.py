#!/usr/bin/env python3
import argparse
import asyncio
import time
from typing import Optional

from mavsdk import System


def age_str(ts: Optional[float]) -> str:
    if ts is None:
        return "N/A"
    return f"{(time.monotonic() - ts):.1f}s"


async def watch_mode(drone: System, s: dict):
    try:
        async for v in drone.telemetry.flight_mode():
            s["mode"] = v
            s["mode_ts"] = time.monotonic()
    except Exception as e:
        s["mode_err"] = repr(e)


async def watch_armed(drone: System, s: dict):
    try:
        async for v in drone.telemetry.armed():
            s["armed"] = v
            s["armed_ts"] = time.monotonic()
    except Exception as e:
        s["armed_err"] = repr(e)


async def watch_in_air(drone: System, s: dict):
    try:
        async for v in drone.telemetry.in_air():
            s["in_air"] = v
            s["in_air_ts"] = time.monotonic()
    except Exception as e:
        s["in_air_err"] = repr(e)


async def watch_position(drone: System, s: dict):
    try:
        async for p in drone.telemetry.position():
            s["pos"] = p
            s["pos_ts"] = time.monotonic()
    except Exception as e:
        s["pos_err"] = repr(e)


async def watch_gps_info(drone: System, s: dict):
    try:
        async for gi in drone.telemetry.gps_info():
            s["gps_info"] = gi
            s["gps_info_ts"] = time.monotonic()
    except Exception as e:
        s["gps_info_err"] = repr(e)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--system-address", default="serial:///dev/ttyAMA0:115200",
                    help="serial:///dev/ttyAMA0:115200 or serial:///dev/serial0:115200 or udpin://0.0.0.0:14540")
    ap.add_argument("--print-hz", type=float, default=2.0)
    args = ap.parse_args()

    drone = System()
    print(f"-- Connecting to {args.system_address}", flush=True)
    await drone.connect(system_address=args.system_address)

    print("-- Waiting for heartbeat...", flush=True)
    async for st in drone.core.connection_state():
        if st.is_connected:
            print("✅ Connected", flush=True)
            break

    s = {
        "mode": None, "mode_ts": None, "mode_err": None,
        "armed": None, "armed_ts": None, "armed_err": None,
        "in_air": None, "in_air_ts": None, "in_air_err": None,
        "pos": None, "pos_ts": None, "pos_err": None,
        "gps_info": None, "gps_info_ts": None, "gps_info_err": None,
    }

    tasks = [
        asyncio.create_task(watch_mode(drone, s)),
        asyncio.create_task(watch_armed(drone, s)),
        asyncio.create_task(watch_in_air(drone, s)),
        asyncio.create_task(watch_position(drone, s)),
        asyncio.create_task(watch_gps_info(drone, s)),
    ]

    dt = 1.0 / max(0.1, args.print_hz)
    print("\n=== TELEMETRY MONITOR (MODE / ARMED / IN_AIR / GPS / POSITION) ===")
    print("No offboard. No commands.\n")

    try:
        while True:
            mode = s["mode"]
            armed = s["armed"]
            in_air = s["in_air"]
            pos = s["pos"]
            gi = s["gps_info"]

            mode_str = str(mode) if mode is not None else "N/A"
            armed_str = str(armed) if armed is not None else "N/A"
            in_air_str = str(in_air) if in_air is not None else "N/A"

            if gi is None:
                gps_str = "fix=N/A sats=N/A"
            else:
                gps_str = f"fix={gi.fix_type} sats={gi.num_satellites}"

            if pos is None:
                pos_str = "lat=N/A lon=N/A abs_alt=N/A rel_alt=N/A"
            else:
                pos_str = (
                    f"lat={pos.latitude_deg:.6f} lon={pos.longitude_deg:.6f} "
                    f"abs_alt={pos.absolute_altitude_m:.1f} rel_alt={pos.relative_altitude_m:.1f}"
                )

            print(
                f"mode={mode_str} (age {age_str(s['mode_ts'])}) | "
                f"armed={armed_str} (age {age_str(s['armed_ts'])}) | "
                f"in_air={in_air_str} (age {age_str(s['in_air_ts'])}) | "
                f"gps[{gps_str}] (age {age_str(s['gps_info_ts'])}) | "
                f"pos[{pos_str}] (age {age_str(s['pos_ts'])})"
            )

            # Show stream errors if any
            for k in ["mode_err", "armed_err", "in_air_err", "gps_info_err", "pos_err"]:
                if s.get(k):
                    print(f"⚠️ {k}: {s[k]}")

            await asyncio.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        for t in tasks:
            t.cancel()


if __name__ == "__main__":
    asyncio.run(main())
