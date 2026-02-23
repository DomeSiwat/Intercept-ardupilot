import time
import math
from pymavlink import mavutil

# Default base position for the fake vehicle (can be overridden)
DEFAULT_BASE_LAT = 14.705569   # deg
DEFAULT_BASE_LON = 101.5726240   # deg
DEFAULT_BASE_ALT_M = 5         # meters AMSL

_START_TIME = time.time()

# Motion config (used by get_current_lat_lon_alt so it matches the running sender)
_SQUARE_SIZE_M = 10.0   # side length (meters)
_SPEED_MPS = 2.0        # meters/second


def _meters_to_deg(base_lat_deg: float, d_north_m: float, d_east_m: float):
    # Convert meters to degrees (approx)
    dlat_deg = d_north_m / 111000.0

    lat_rad = math.radians(base_lat_deg)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    if meters_per_deg_lon == 0:
        dlon_deg = 0.0
    else:
        dlon_deg = d_east_m / meters_per_deg_lon

    return dlat_deg, dlon_deg


def compute_position_square(now_seconds: float,
                            base_lat: float = DEFAULT_BASE_LAT,
                            base_lon: float = DEFAULT_BASE_LON,
                            base_alt_m: float = DEFAULT_BASE_ALT_M,
                            square_size_m: float = 50.0,
                            speed_m_per_s: float = 2.0,
                            start_corner: str = "SW",
                            clockwise: bool = True):
    """
    Square path aligned to North/East axes.

    square_size_m: side length in meters (L)
    speed_m_per_s: constant ground speed in m/s
    start_corner: one of {"SW","SE","NE","NW"} (where the square is anchored)
    clockwise: direction around the square
    """
    L = float(square_size_m)
    if L <= 0:
        raise ValueError("square_size_m must be > 0")
    v = float(speed_m_per_s)
    if v <= 0:
        raise ValueError("speed_m_per_s must be > 0")

    # Distance traveled along perimeter
    perimeter = 4.0 * L
    s = (v * now_seconds) % perimeter  # meters along perimeter [0, 4L)

    # Square corners in (north,east) offsets relative to the chosen anchor corner
    # We define the square as spanning [0..L] in north and [0..L] in east from SW.
    # Then we shift depending on start_corner.
    # Path sequence (clockwise) from SW: SW->SE->NE->NW->SW
    # That corresponds to: +E, +N, -E, -N
    # For counter-clockwise: +N, +E, -N, -E (from SW)

    # Base offsets for SW-anchored square:
    if clockwise:
        # segment 0: east, 1: north, 2: west, 3: south
        if s < L:
            dn, de = 0.0, s
        elif s < 2 * L:
            dn, de = (s - L), L
        elif s < 3 * L:
            dn, de = L, (3 * L - s)
        else:
            dn, de = (4 * L - s), 0.0
    else:
        # segment 0: north, 1: east, 2: south, 3: west
        if s < L:
            dn, de = s, 0.0
        elif s < 2 * L:
            dn, de = L, (s - L)
        elif s < 3 * L:
            dn, de = (3 * L - s), L
        else:
            dn, de = 0.0, (4 * L - s)

    # Shift square so that the chosen start_corner sits at (0,0)
    # Our dn,de currently assume SW is (0,0) and NE is (L,L).
    start_corner = start_corner.upper()
    if start_corner == "SW":
        pass
    elif start_corner == "SE":
        de -= L
    elif start_corner == "NE":
        dn -= L
        de -= L
    elif start_corner == "NW":
        dn -= L
    else:
        raise ValueError('start_corner must be one of {"SW","SE","NE","NW"}')

    dlat_deg, dlon_deg = _meters_to_deg(base_lat, dn, de)

    lat = base_lat + dlat_deg
    lon = base_lon + dlon_deg
    alt = base_alt_m
    return lat, lon, alt


def get_current_lat_lon_alt():
    """Return the current fake GPS lat, lon, alt using the same square config as the sender."""
    now = time.time() - _START_TIME
    return compute_position_square(
        now_seconds=now,
        square_size_m=_SQUARE_SIZE_M,
        speed_m_per_s=_SPEED_MPS,
    )


# If QGC is on the same machine:
TARGET = "udpout:127.0.0.1:14550"
# If QGC is on another machine:
# TARGET = "udpout:192.168.1.50:14550"

FAKE_SYSID = 2
FAKE_COMPID = 1


def fake_gps(square_size_m: float = 50.0,
             speed_m_per_s: float = 2.0,
             clockwise: bool = True,
             start_corner: str = "SW"):
    global _SQUARE_SIZE_M, _SPEED_MPS
    _SQUARE_SIZE_M = float(square_size_m)
    _SPEED_MPS = float(speed_m_per_s)

    master = mavutil.mavlink_connection(
        TARGET,
        source_system=FAKE_SYSID,
        source_component=FAKE_COMPID
    )

    print(f"Fake vehicle (sysid=2) starting... square={_SQUARE_SIZE_M}m speed={_SPEED_MPS}m/s "
          f"dir={'CW' if clockwise else 'CCW'} start={start_corner}")

    # Send some heartbeats so QGC detects vehicle 2
    for _ in range(20):
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0
        )
        time.sleep(0.2)

    print("Sending fake GPS for vehicle 2 (square path)...")

    last_print_time = 0.0

    while True:
        now = time.time() - _START_TIME

        lat, lon, alt = compute_position_square(
            now_seconds=now,
            square_size_m=_SQUARE_SIZE_M,
            speed_m_per_s=_SPEED_MPS,
            clockwise=clockwise,
            start_corner=start_corner,
        )

        real_now = time.time()
        if real_now - last_print_time >= 2.0:
            print(f"[fake_gps] lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}")
            last_print_time = real_now

        # Convert to MAVLink units
        lat_i = int(lat * 1e7)
        lon_i = int(lon * 1e7)
        alt_mm = int(alt * 1000)

        eph = 100  # cm
        epv = 100  # cm
        vel = 0    # cm/s
        cog = 0    # cdeg

        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0
        )

        master.mav.gps_raw_int_send(
            int(time.time() * 1e6),
            3,        # 3D fix
            lat_i,
            lon_i,
            alt_mm,
            eph,
            epv,
            vel,
            cog,
            8
        )

        master.mav.global_position_int_send(
            int(now * 1000),  # time_boot_ms
            lat_i,
            lon_i,
            alt_mm,           # alt MSL
            alt_mm,           # alt relative
            0, 0, 0,          # vx, vy, vz
            0
        )

        time.sleep(0.2)  # 5 Hz


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run fake GPS sender with square motion")
    parser.add_argument("--square", "-q", type=float, default=50.0,
                        help="Square side length in meters.")
    parser.add_argument("--speed", "-s", type=float, default=2.0,
                        help="Ground speed in m/s.")
    parser.add_argument("--ccw", action="store_true",
                        help="Go counter-clockwise (default: clockwise).")
    parser.add_argument("--start", type=str, default="SW",
                        help='Start corner: SW, SE, NE, NW (default: SW).')

    args = parser.parse_args()

    fake_gps(
        square_size_m=args.square,
        speed_m_per_s=args.speed,
        clockwise=(not args.ccw),
        start_corner=args.start
    )
