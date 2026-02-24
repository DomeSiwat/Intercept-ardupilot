import math
from config import (
    MAX_SPEED, MIN_SPEED, STOP_RADIUS,
    MAX_VD, ALT_HOLD_BAND,
    MAX_YAW_RATE_DEG_S,
)


# -------------------- Utility --------------------

def clamp(x, lo, hi):
    return max(lo, min(x, hi))


def distance_and_bearing(lat1, lon1, lat2, lon2):
    """
    Returns (distance_m, bearing_rad_from_north)
    Bearing measured clockwise from North (0 rad)
    """
    R = 6371000.0
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    return distance, bearing


def wrap_to_180(angle):
    return (angle + 180) % 360 - 180


# -------------------- Horizontal --------------------

def compute_horizontal(lat, lon, target_lat, target_lon):
    """
    Compute horizontal velocity toward target.
    Returns (vn, ve, distance_m, bearing_rad).
    """
    dist, bearing_rad = distance_and_bearing(lat, lon, target_lat, target_lon)
    if dist > STOP_RADIUS:
        speed = clamp(dist / 0.2, MIN_SPEED, MAX_SPEED)
        vn = speed * math.cos(bearing_rad)
        ve = speed * math.sin(bearing_rad)
    else:
        vn = ve = 0.0
    return vn, ve, dist, bearing_rad


# -------------------- Vertical --------------------

def compute_vertical(rel_alt, target_rel_alt):
    """
    Compute vertical velocity (NED: positive = down).
    Returns vd.
    """
    alt_error = target_rel_alt - rel_alt
    if abs(alt_error) > ALT_HOLD_BAND:
        vd = -clamp(alt_error, -MAX_VD, MAX_VD)  # NED: Down positive
    else:
        vd = 0.0
    return vd


# -------------------- Yaw --------------------

class YawController:
    """Rate-limited yaw controller."""

    def __init__(self):
        self.current_yaw_deg = 0.0

    def update(self, target_yaw_deg, dt):
        """
        Step toward target_yaw_deg with rate limiting.
        Returns the smoothed yaw command in degrees.
        """
        yaw_error_deg = wrap_to_180(target_yaw_deg - self.current_yaw_deg)
        max_step = MAX_YAW_RATE_DEG_S * dt
        yaw_change = clamp(yaw_error_deg, -max_step, max_step)
        self.current_yaw_deg += yaw_change
        return self.current_yaw_deg
