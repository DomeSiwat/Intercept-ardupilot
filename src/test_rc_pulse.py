#!/usr/bin/env python3
"""
test_rc_pulse.py
----------------
Read RC channel pulse widths (PWM in microseconds) from ArduPilot via pymavlink.
Supports both serial and UDP connections.
"""

import time
import argparse
from pymavlink import mavutil

# Fix SerialException import for pyserial v3.x+
try:
    from serial import SerialException
except ImportError:
    from serial.serialutil import SerialException

# Default serial port and baud
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200

CHANNEL_NAMES = {
    1:  "Roll", 2:  "Pitch", 3:  "Throttle", 4:  "Yaw",
    5:  "Flight Mode", 6:  "CH6", 7: "CH7", 8: "CH8",
    9:  "CH9", 10: "CH10", 11: "CH11", 12: "CH12",
    13: "CH13", 14: "CH14", 15: "CH15", 16: "CH16",
    17: "CH17", 18: "CH18",
}

def parse_args():
    parser = argparse.ArgumentParser(description="Read RC channel pulse widths from ArduPilot")
    parser.add_argument(
        "--connection",
        type=str,
        default=f"serial:{DEFAULT_SERIAL_PORT}:{DEFAULT_BAUD}",
        help="Connection string: 'serial:/dev/ttyAMA0:115200' or 'udp:host:port'"
    )
    return parser.parse_args()

def main():
    args = parse_args()
    connection = args.connection

    print(f"[INFO] Connecting to: {connection}")

    if connection.startswith("udp:"):
        try:
            master = mavutil.mavlink_connection(connection)
        except Exception as e:
            print(f"[ERROR] Failed to connect via UDP: {e}")
            return
    elif connection.startswith("serial:"):
        try:
            import serial
        except ImportError:
            print("[ERROR] pyserial not installed. Run: pip install pyserial")
            return

        # Extract serial port and baud
        try:
            _, port, baud = connection.split(":")
            baud = int(baud)
            master = mavutil.mavlink_connection(port, baud=baud)
        except SerialException as e:
            print(f"[ERROR] Failed to connect via Serial: {e}")
            return
        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            return
    else:
        print("[ERROR] Invalid connection string. Use 'serial:/dev/ttyAMA0:115200' or 'udp:host:port'")
        return

    print("[INFO] Waiting for heartbeat...")
    try:
        master.wait_heartbeat()
    except Exception as e:
        print(f"[ERROR] Failed to receive heartbeat: {e}")
        return

    print(f"[INFO] Connected  (sysid={master.target_system})\n")

    try:
        while True:
            msg = master.recv_match(type="RC_CHANNELS", blocking=True, timeout=2.0)
            if msg is None:
                print("[WARN] No RC_CHANNELS received – check connection.")
                continue

            d = msg.to_dict()
            for ch in range(1, 19):
                pwm = d.get(f"chan{ch}_raw", 0)
                if pwm == 0 or pwm == 65535:
                    continue
                name = CHANNEL_NAMES.get(ch, f"CH{ch}")
                print(f"  CH{ch:>2} ({name:<12}): {pwm} µs")
            print("-" * 40)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
    finally:
        master.close()

if __name__ == "__main__":
    main()