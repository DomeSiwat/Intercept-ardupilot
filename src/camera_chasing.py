#!/usr/bin/env python3
"""
SAFE visual tracking controller (benign): keep a detected object centered for filming/training.
- Receives YOLO detections via UDP JSON (from your yolo_udp sender)
- Outputs MAVSDK VelocityNedYaw to ArduPilot
- Lost-target => stop (safe)

Expected UDP JSON (example):
{
  "t": 1700000000.12,
  "found": true,
  "ex": -0.08,
  "ey": 0.04,
  "conf": 0.91,
  "cx": 312,
  "cy": 241,
  "w": 640,
  "h": 480
}

Run example:
  Terminal 1 (YOLO sender):
    python3 yolo_udp.py --udp-ip 127.0.0.1 --udp-port 5600 --udp-send-always

  Terminal 2 (this controller):
    python3 mavsdk_track_udp.py --udp-port 5600 --system-address udp://:14540
"""

import argparse
import asyncio
import json
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


# --------------------------
# Detection interface
# --------------------------

@dataclass
class Detection:
    # We store both pixel center and normalized error.
    cx: float
    cy: float
    ex: float
    ey: float
    conf: float
    w: int
    h: int
    t: float  # monotonic timestamp when we RECEIVED this packet


# --------------------------
# Controller parameters
# --------------------------

HZ = 20.0
DT = 1.0 / HZ

# Deadband to avoid jitter (applied to ex/ey)
DEADBAND_X = 0.05
DEADBAND_Y = 0.05

# Gains (conservative)
K_YAW_DEG_PER_S = 60.0     # yaw integration gain based on x-error
K_ALT_MPS = 0.6            # vertical speed response to y-error

# Speeds
VF_BASE = 1.0              # forward m/s when centered
VF_MAX = 2.0
VZ_MAX = 0.8               # |v_d| clamp (NED down)

# Yaw limits
YAW_RATE_LIMIT_DEG_PER_S = 90.0  # software yaw delta limiter

# Detection freshness
DETECT_TIMEOUT_S = 0.35     # if last packet older than this => lost

# Lost target behavior
LOST_HOLD_SECONDS = 9999    # keep stopped; set smaller if you want scan
SCAN_YAW_RATE_DEG_PER_S = 20.0

# Safety: do nothing unless in a guided-like mode
GUIDED_LIKE = {"FlightMode.OFFBOARD", "FlightMode.GUIDED"}  # adjust if MAVSDK shows something else


# --------------------------
# Helpers
# --------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_360(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def apply_deadband(ex: float, ey: float) -> Tuple[float, float]:
    if abs(ex) < DEADBAND_X:
        ex = 0.0
    if abs(ey) < DEADBAND_Y:
        ey = 0.0
    return ex, ey

def forward_speed_from_alignment(ex: float) -> float:
    # slow down when target is far from center horizontally
    scale = 1.0 - min(abs(ex), 1.0)  # 1 when centered, 0 when far off
    vf = VF_BASE * (0.3 + 0.7 * scale)
    return clamp(vf, 0.0, VF_MAX)


# --------------------------
# UDP receiver (async)
# --------------------------

class UdpDetections:
    """
    Stores the most recent detection from UDP.
    """
    def __init__(self):
        self._latest: Optional[Detection] = None

    def update_from_packet(self, payload: dict) -> None:
        # Expect fields: found, ex, ey, conf, cx, cy, w, h
        found = bool(payload.get("found", False))

        # If sender is not sending always and it disappears, we will timeout.
        # If sender sends always, it will send found=false.
        if not found:
            # still update time so "fresh but not found" can be represented as None detection
            self._latest = Detection(
                cx=float(payload.get("cx") or 0.0),
                cy=float(payload.get("cy") or 0.0),
                ex=0.0,
                ey=0.0,
                conf=0.0,
                w=int(payload.get("w") or 0),
                h=int(payload.get("h") or 0),
                t=time.monotonic(),
            )
            return

        ex = float(payload.get("ex", 0.0))
        ey = float(payload.get("ey", 0.0))
        conf = float(payload.get("conf", 0.0))
        cx = float(payload.get("cx", 0.0))
        cy = float(payload.get("cy", 0.0))
        w = int(payload.get("w", 0))
        h = int(payload.get("h", 0))

        # clamp errors
        ex = clamp(ex, -1.0, 1.0)
        ey = clamp(ey, -1.0, 1.0)

        self._latest = Detection(cx=cx, cy=cy, ex=ex, ey=ey, conf=conf, w=w, h=h, t=time.monotonic())

    def get_latest_detection(self) -> Optional[Detection]:
        # Return detection only if "found" in last packet and fresh
        if self._latest is None:
            return None
        age = time.monotonic() - self._latest.t
        if age > DETECT_TIMEOUT_S:
            return None
        # If conf==0 and ex/ey==0 and found was false, treat as no detection
        if self._latest.conf <= 0.0:
            return None
        return self._latest


class UdpJsonProtocol(asyncio.DatagramProtocol):
    def __init__(self, store: UdpDetections, debug: bool = False):
        self.store = store
        self.debug = debug

    def datagram_received(self, data: bytes, addr):
        try:
            msg = json.loads(data.decode("utf-8", errors="ignore"))
            if isinstance(msg, dict):
                self.store.update_from_packet(msg)
                if self.debug:
                    print(f"[UDP] from {addr} found={msg.get('found')} ex={msg.get('ex')} ey={msg.get('ey')} conf={msg.get('conf')}")
        except Exception:
            # ignore malformed packets
            return


async def start_udp_listener(bind_ip: str, bind_port: int, store: UdpDetections, debug: bool = False):
    loop = asyncio.get_running_loop()
    transport, _protocol = await loop.create_datagram_endpoint(
        lambda: UdpJsonProtocol(store, debug=debug),
        local_addr=(bind_ip, bind_port),
    )
    return transport


# --------------------------
# Telemetry watchers
# --------------------------

async def watch_flight_mode(drone: System, shared: dict):
    async for mode in drone.telemetry.flight_mode():
        shared["mode_str"] = str(mode)

async def watch_yaw(drone: System, shared: dict):
    async for att in drone.telemetry.attitude_euler():
        shared["yaw_est_deg"] = float(att.yaw_deg)


# --------------------------
# Main control loop
# --------------------------

async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--system-address", default="udp://:14540", help="MAVSDK system address (default: udp://:14540)")
    ap.add_argument("--udp-bind", default="0.0.0.0", help="UDP bind IP (default: 0.0.0.0)")
    ap.add_argument("--udp-port", type=int, default=5601, help="UDP port to listen (default: 5601)")
    ap.add_argument("--udp-debug", action="store_true", help="Print received UDP packets (debug)")
    args = ap.parse_args()

    # UDP store + listener
    store = UdpDetections()
    udp_transport = await start_udp_listener(args.udp_bind, args.udp_port, store, debug=args.udp_debug)
    print(f"[OK] UDP listening on {args.udp_bind}:{args.udp_port}")

    drone = System()
    await drone.connect(system_address=args.system_address)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected")
            break

    shared = {"mode_str": "", "yaw_est_deg": 0.0}
    asyncio.create_task(watch_flight_mode(drone, shared))
    asyncio.create_task(watch_yaw(drone, shared))

    # Prime setpoints
    print("-- Priming setpoints...")
    for _ in range(20):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.05)

    try:
        print("-- Starting offboard...")
        await drone.offboard.start()
        print("-- Offboard started")
    except OffboardError as e:
        print("Offboard start failed:", e)
        print("Tip: Ensure ArduPilot is in GUIDED and armed (via RC/GCS) and offboard is allowed.")
        udp_transport.close()
        return

    yaw_cmd = wrap_360(shared.get("yaw_est_deg", 0.0))
    lost_since = None

    print("-- Control loop running")
    try:
        while True:
            tnow = time.monotonic()
            det = store.get_latest_detection()

            mode_ok = shared.get("mode_str", "") in GUIDED_LIKE

            if not mode_ok:
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_cmd))
                await asyncio.sleep(DT)
                

            if det is not None:
                lost_since = None

                ex, ey = apply_deadband(det.ex, det.ey)

                # Yaw integration with rate limiting
                yaw_delta = (K_YAW_DEG_PER_S * ex) * DT
                yaw_delta = clamp(yaw_delta, -YAW_RATE_LIMIT_DEG_PER_S * DT, YAW_RATE_LIMIT_DEG_PER_S * DT)
                yaw_cmd = wrap_360(yaw_cmd + yaw_delta)

                # Forward speed: safer when aligned
                vf = forward_speed_from_alignment(ex)

                # Convert forward in heading direction -> NED
                yaw_rad = math.radians(yaw_cmd)
                v_n = vf * math.cos(yaw_rad)
                v_e = vf * math.sin(yaw_rad)

                # Vertical speed from ey (positive ey means target below center -> go down)
                v_d = clamp(K_ALT_MPS * ey, -VZ_MAX, VZ_MAX)

                await drone.offboard.set_velocity_ned(VelocityNedYaw(v_n, v_e, v_d, yaw_cmd))

            else:
                # Lost target: stop (safe). Optionally scan yaw slowly.
                if lost_since is None:
                    lost_since = tnow
                dt_lost = tnow - lost_since

                if dt_lost < LOST_HOLD_SECONDS:
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_cmd))
                else:
                    yaw_cmd = wrap_360(yaw_cmd + SCAN_YAW_RATE_DEG_PER_S * DT)
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_cmd))

            await asyncio.sleep(DT)

    finally:
        udp_transport.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
