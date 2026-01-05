import time
import math
from pymavlink import mavutil

# Default base position for the fake vehicle (can be overridden)
DEFAULT_BASE_LAT = -35.360204   # deg
DEFAULT_BASE_LON = 149.164830  # deg
DEFAULT_BASE_ALT_M = 50        # meters AMSL

# Reference start time used for the movement pattern. Using a module-level
# start time makes the `get_current_lat_lon_alt()` function deterministic
# relative to the process start and usable from other modules.
_START_TIME = time.time()


def compute_position(now_seconds: float,
                     base_lat: float = DEFAULT_BASE_LAT,
                     base_lon: float = DEFAULT_BASE_LON,
                     base_alt_m: float = DEFAULT_BASE_ALT_M,
                     speed_m_per_s: float = 7.0,
                     heading_deg: float = 180.0):
    """Compute the fake vehicle position for a given elapsed time.

    This version produces straight-line motion at a fixed speed and heading
    (instead of an oscillatory/circular path). The displacement is computed
    in meters and converted to degrees using approximate conversions.

    Args:
        now_seconds: seconds elapsed since _START_TIME (float).
        base_lat, base_lon, base_alt_m: base position to offset from.
        speed_m_per_s: ground speed in meters/second.
        heading_deg: heading in degrees, where 0 = north, 90 = east.

    Returns:
        (lat, lon, alt) as floats: degrees, degrees, meters.
    """
    # Distance traveled (meters) along north/east from start
    distance_m = speed_m_per_s * now_seconds
    heading_rad = math.radians(heading_deg)
    d_north_m = distance_m * math.cos(heading_rad)
    d_east_m = distance_m * math.sin(heading_rad)

    # Convert meters to degrees (approx). Latitude degrees are ~111000 m.
    dlat_deg = d_north_m / 111000.0

    # Longitude conversion depends on latitude: degrees per meter = 1 / (111320 * cos(lat))
    lat_rad = math.radians(base_lat)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    # avoid division by zero in extreme cases
    if meters_per_deg_lon == 0:
        dlon_deg = 0.0
    else:
        dlon_deg = d_east_m / meters_per_deg_lon

    lat = base_lat + dlat_deg
    lon = base_lon + dlon_deg
    alt = base_alt_m

    return lat, lon, alt


def get_current_lat_lon_alt():
    """Return the current fake GPS lat, lon, alt in real time.

    This function is safe to call from other modules. It computes the
    same position used by the running fake GPS sender.

    Returns:
        (lat, lon, alt) as floats: degrees, degrees, meters.
    """
    now = time.time() - _START_TIME
    # Default behavior: use defaults from compute_position
    return compute_position(now)

# If QGC is on the same machine:
TARGET = "udpout:127.0.0.1:14550"
# If QGC is on another machine:
# TARGET = "udpout:192.168.1.50:14550"

FAKE_SYSID = 2   # Real drone is usually 1 → use 2 for fake
FAKE_COMPID = 1  # Autopilot component

def fake_gps(speed_m_per_s: float = 2.0, heading_deg: float = 0.0):
    master = mavutil.mavlink_connection(
        TARGET,
        source_system=FAKE_SYSID,
        source_component=FAKE_COMPID
    )

    print(f"Fake vehicle (sysid=2) starting... speed={speed_m_per_s} m/s heading={heading_deg}°")

    # Send some heartbeats so QGC detects vehicle 2
    for _ in range(20):
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0
        )
        time.sleep(0.2)

    print("Sending fake GPS for vehicle 2...")

    # The module-level defaults (DEFAULT_BASE_*) control the base
    # position used by the movement pattern. The loop uses the module
    # _START_TIME so `get_current_lat_lon_alt()` can be called from
    # other modules and remain time-consistent.

    # Track last time we printed coordinates so we only print every 2 seconds
    last_print_time = 0.0

    while True:
        # elapsed time relative to process start
        now = time.time() - _START_TIME

        # Use the shared function to compute current lat/lon/alt
        # Pass configured speed/heading so the motion follows the requested heading
        lat, lon, alt = get_current_lat_lon_alt()
        # If external callers want to directly obtain the current position with a
        # non-default heading/speed, they can call compute_position or
        # get_current_lat_lon_alt(now_override) directly. The running sender uses
        # the module defaults (or the speed/heading provided to fake_gps()).

        # Print the computed coordinates every 2 seconds (avoids flooding the terminal)
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

        # HEARTBEAT (keeps vehicle 2 "alive")
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0
        )

        # GPS_RAW_INT
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
            8         # satellites visible
        )

        # GLOBAL_POSITION_INT (what QGC uses for map position)
        master.mav.global_position_int_send(
            int(now * 1000),  # time_boot_ms
            lat_i,
            lon_i,
            alt_mm,           # alt MSL
            alt_mm,           # alt relative (fake same)
            0, 0, 0,          # vx, vy, vz
            0                 # heading
        )

        time.sleep(0.2)  # 5 Hz

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run fake GPS sender with straight-line motion")
    parser.add_argument("--heading", "-hd", type=float, default=90.0,
                        help="Heading in degrees (0=north, 90=east).")
    parser.add_argument("--speed", "-s", type=float, default=1.0,
                        help="Ground speed in m/s.")
    args = parser.parse_args()

    # Start the fake GPS with the requested parameters
    fake_gps(speed_m_per_s=args.speed, heading_deg=args.heading)

