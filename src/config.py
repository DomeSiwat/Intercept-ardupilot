# ===================== TUNING =====================
MAX_SPEED = 5.0           # m/s horizontal
MIN_SPEED = 0.5           # m/s
MAX_VD = 0.2              # m/s vertical (Down positive)
STOP_RADIUS = 1           # meters
ALT_HOLD_BAND = 5         # meters
# ==================================================

# ================== YAW TUNING ====================
MAX_YAW_RATE_DEG_S = 30   # deg/s  (yaw rate when error is large)
MIN_YAW_RATE_DEG_S = 5    # deg/s  (yaw rate when error is small)
# ==================================================

# ============== ACCELERATION TUNING ===============
ACCELERATION = 2.0        # m/s²  (how fast speed ramps up/down)
# ==================================================

# =========== SPEED-YAW COUPLING TUNING ============
# Controls how much yaw error reduces speed.
# At YAW_FULL_SLOW_DEG error, speed is reduced to MIN_SPEED.
# At 0 error, speed is unrestricted.
# Higher value = drone tolerates more yaw error before slowing down.
YAW_FULL_SLOW_DEG = 90.0  # degrees of yaw error for full speed reduction
# ==================================================

# ===================== DEFAULTS ===================
DEFAULT_SYSTEM_ADDRESS = "serial:///dev/ttyAMA0:115200"
DEFAULT_HZ = 20.0
DEFAULT_ENABLE_AFTER = 0.0
# ==================================================
