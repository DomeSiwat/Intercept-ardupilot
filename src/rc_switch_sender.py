#!/usr/bin/env python3
"""
rc_switch_sender.py
-------------------
Reads CH10 PWM from ArduPilot via pymavlink and sends the camera-following
state (True/False) over UDP as a JSON packet.

CH10 PWM mapping:
    ~1951 µs  →  camera_following = True
    ~1051 µs  →  camera_following = False

UDP JSON sent to receiver (e.g. main.py):
    {"camera_following": true}
    {"camera_following": false}

Usage:
    python3 rc_switch_sender.py
    python3 rc_switch_sender.py --connection serial:/dev/ttyAMA0:115200
    python3 rc_switch_sender.py --connection udp:127.0.0.1:14550
    python3 rc_switch_sender.py --udp-ip 127.0.0.1 --udp-port 5700

Requirements:
    pip install pymavlink pyserial
"""

import json
import socket
import time
import argparse
from pymavlink import mavutil

try:
    from serial import SerialException
except ImportError:
    from serial.serialutil import SerialException

# ── RC Switch config ──────────────────────────────────────────────────────────
RC_CHANNEL        = 10       # channel to monitor
PWM_HIGH          = 1951     # µs → camera_following = True
PWM_LOW           = 1051     # µs → camera_following = False
PWM_THRESHOLD     = 200      # ±tolerance in µs for matching

# ── ArduPilot connection ──────────────────────────────────────────────────────
DEFAULT_CONNECTION = "serial:/dev/ttyAMA0:115200"

# ── UDP output ────────────────────────────────────────────────────────────────
DEFAULT_UDP_IP   = "127.0.0.1"
DEFAULT_UDP_PORT = 5700


def parse_args():
    parser = argparse.ArgumentParser(description="RC CH10 switch → UDP sender")
    parser.add_argument(
        "--connection",
        type=str,
        default=DEFAULT_CONNECTION,
        help="ArduPilot connection: 'serial:/dev/ttyAMA0:115200' or 'udp:127.0.0.1:14550'",
    )
    parser.add_argument("--udp-ip",   type=str, default=DEFAULT_UDP_IP,
                        help=f"UDP destination IP (default: {DEFAULT_UDP_IP})")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT,
                        help=f"UDP destination port (default: {DEFAULT_UDP_PORT})")
    return parser.parse_args()


def connect_mavlink(connection: str):
    """Open a pymavlink connection from a 'serial:port:baud' or 'udp:...' string."""
    if connection.startswith("udp:"):
        return mavutil.mavlink_connection(connection)
    elif connection.startswith("serial:"):
        parts = connection.split(":")
        port = parts[1]
        baud = int(parts[2]) if len(parts) > 2 else 115200
        return mavutil.mavlink_connection(port, baud=baud)
    else:
        raise ValueError(f"Unknown connection format: {connection}")


def pwm_to_camera_following(pwm: int) -> bool | None:
    """
    Decode CH10 PWM to camera_following state.
    Returns True, False, or None if PWM does not match either position.
    """
    if abs(pwm - PWM_HIGH) <= PWM_THRESHOLD:
        return True
    if abs(pwm - PWM_LOW) <= PWM_THRESHOLD:
        return False
    return None   # unknown / mid-switch position


def send_udp(sock: socket.socket, addr: tuple, camera_following: bool) -> None:
    """Send a JSON UDP packet with the camera_following state."""
    payload = json.dumps({"camera_following": camera_following}).encode("utf-8")
    try:
        sock.sendto(payload, addr)
    except OSError as e:
        print(f"[WARN] UDP send failed: {e}")


def main():
    args = parse_args()
    udp_addr = (args.udp_ip, args.udp_port)

    # ── Connect to ArduPilot ──────────────────────────────────────────────────
    print(f"[INFO] Connecting to ArduPilot: {args.connection}")
    try:
        master = connect_mavlink(args.connection)
    except (SerialException, ValueError, Exception) as e:
        print(f"[ERROR] Connection failed: {e}")
        return

    print("[INFO] Waiting for heartbeat...")
    try:
        master.wait_heartbeat()
    except Exception as e:
        print(f"[ERROR] Heartbeat failed: {e}")
        return
    print(f"[INFO] Connected  (sysid={master.target_system})")

    # ── UDP socket ────────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[INFO] Sending UDP → {udp_addr[0]}:{udp_addr[1]}")
    print(f"[INFO] Monitoring CH{RC_CHANNEL}  "
          f"(HIGH={PWM_HIGH}µs → True  |  LOW={PWM_LOW}µs → False)\n")

    last_state = None   # track changes to avoid spamming unchanged state

    try:
        while True:
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2.0)

            if msg is None:
                print("[WARN] No RC_CHANNELS – check connection.")
                continue

            d   = msg.to_dict()
            pwm = d.get(f"chan{RC_CHANNEL}_raw", 0)

            if pwm == 0 or pwm == 65535:
                print(f"[WARN] CH{RC_CHANNEL} not available (PWM={pwm})")
                continue

            state = pwm_to_camera_following(pwm)

            if state is None:
                print(f"[INFO] CH{RC_CHANNEL} PWM={pwm} µs  →  (mid/unknown, not sent)")
                continue

            # Always send the current state over UDP
            send_udp(sock, udp_addr, state)

            # Print only when state changes
            if state != last_state:
                label = "ON  ✓" if state else "OFF ✗"
                print(f"[CH{RC_CHANNEL}] PWM={pwm} µs  →  camera_following={state}  [{label}]")
                last_state = state

    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
    finally:
        master.close()
        sock.close()


if __name__ == "__main__":
    main()
