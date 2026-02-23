#!/usr/bin/env python3
import asyncio
import time
import socket
import argparse
from mavsdk import System

CONNECT_ADDR_DEFAULT = "udp://:14540"


async def main(connect_addr: str, host: str, port: int, fmt: str):
    drone = System()
    await drone.connect(system_address=connect_addr)

    print("[mavsdk_udp] Connecting...")
    async for st in drone.core.connection_state():
        if st.is_connected:
            print("[mavsdk_udp] connected=True")
            break

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)

    t0 = time.time()
    seq = 0

    print(f"[mavsdk_udp] sending to {host}:{port} format={fmt} (send-on-receive)")

    async for gps in drone.telemetry.raw_gps():
        lat = gps.latitude_deg
        lon = gps.longitude_deg
        alt = gps.absolute_altitude_m

        unix_time = time.time()
        time_boot_s = unix_time - t0

        if fmt == "csv":
            payload = f"{unix_time:.6f},{time_boot_s:.3f},{seq},{lat:.7f},{lon:.7f},{alt:.2f}\n"
        else:  # json
            payload = (
                f'{{"unix_time":{unix_time:.6f},"time_boot_s":{time_boot_s:.3f},"seq":{seq},'
                f'"lat":{lat:.7f},"lon":{lon:.7f},"alt_m":{alt:.2f}}}\n'
            )

        sock.sendto(payload.encode("utf-8"), addr)
        seq += 1


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Read MAVSDK raw_gps and send over UDP immediately")
    p.add_argument("--connect", default=CONNECT_ADDR_DEFAULT, help="MAVSDK connect address")
    p.add_argument("--host", default="127.0.0.1", help="Destination IP/host")
    p.add_argument("--port", type=int, default=16000, help="Destination UDP port")
    p.add_argument("--fmt", choices=["csv", "json"], default="csv", help="Payload format")
    args = p.parse_args()

    asyncio.run(main(args.connect, args.host, args.port, args.fmt))
