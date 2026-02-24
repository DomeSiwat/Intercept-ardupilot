import math
from config import (
    MAX_SPEED, MIN_SPEED, STOP_RADIUS,
    MAX_VD, ALT_HOLD_BAND,
    MAX_YAW_RATE_DEG_S, MIN_YAW_RATE_DEG_S,
    ACCELERATION, YAW_FULL_SLOW_DEG,
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


# -------------------- Yaw --------------------

class YawController:
    """Dynamic rate-limited yaw controller.

    Yaw rate scales with yaw error:
      - Large error → fast turn  (MAX_YAW_RATE_DEG_S)
      - Small error → slow turn  (MIN_YAW_RATE_DEG_S)
    """

    def __init__(self):
        self.current_yaw_deg = 0.0
        self.yaw_error_deg = 0.0   # exposed so speed controller can read it

    def update(self, target_yaw_deg, dt):
        """
        Step toward target_yaw_deg with dynamic rate limiting.
        Returns the smoothed yaw command in degrees.
        """
        self.yaw_error_deg = wrap_to_180(target_yaw_deg - self.current_yaw_deg)

        # Dynamic yaw rate: interpolate based on error magnitude
        error_ratio = clamp(abs(self.yaw_error_deg) / YAW_FULL_SLOW_DEG, 0.0, 1.0)
        dynamic_rate = MIN_YAW_RATE_DEG_S + (MAX_YAW_RATE_DEG_S - MIN_YAW_RATE_DEG_S) * error_ratio

        max_step = dynamic_rate * dt
        yaw_change = clamp(self.yaw_error_deg, -max_step, max_step)
        self.current_yaw_deg += yaw_change
        return self.current_yaw_deg


# -------------------- Speed (with acceleration & yaw coupling) --------------------

class SpeedController:
    """Acceleration-limited speed with yaw-error coupling.

    - Speed ramps up/down at ACCELERATION m/s².
    - When yaw error is large, target speed is reduced (drone slows down to turn).
    - YAW_FULL_SLOW_DEG controls how aggressively yaw error reduces speed.
    """

    def __init__(self):
        self.current_speed = 0.0

    def update(self, desired_speed, yaw_error_deg, dt):
        """
        Compute the actual speed after applying yaw-coupling and acceleration.
        Returns clamped speed in m/s.
        """
        # --- Yaw-speed coupling: reduce target speed when yaw error is large ---
        yaw_factor = 1.0 - clamp(abs(yaw_error_deg) / YAW_FULL_SLOW_DEG, 0.0, 1.0)
        # yaw_factor: 1.0 = facing target, 0.0 = 90°+ off
        target_speed = MIN_SPEED + (desired_speed - MIN_SPEED) * yaw_factor

        # --- Acceleration limiting ---
        speed_error = target_speed - self.current_speed
        max_change = ACCELERATION * dt
        self.current_speed += clamp(speed_error, -max_change, max_change)
        self.current_speed = clamp(self.current_speed, 0.0, MAX_SPEED)

        return self.current_speed


# -------------------- Horizontal --------------------

def compute_horizontal(lat, lon, target_lat, target_lon):
    """
    Compute raw desired speed and bearing toward target.
    Returns (desired_speed, distance_m, bearing_rad).
    Does NOT apply yaw coupling or acceleration — that is SpeedController's job.
    """
    dist, bearing_rad = distance_and_bearing(lat, lon, target_lat, target_lon)
    if dist > STOP_RADIUS:
        desired_speed = clamp(dist / 0.2, MIN_SPEED, MAX_SPEED)
    else:
        desired_speed = 0.0
    return desired_speed, dist, bearing_rad


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
