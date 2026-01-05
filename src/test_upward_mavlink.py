from pymavlink import mavutil
import time

PORT = 14540
HZ = 20
DT = 1.0 / HZ
UP_VD = -0.5  # NED: negative is up

master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{PORT}")
master.wait_heartbeat()
print("✅ Heartbeat received")

def is_guided():
    hb = master.messages.get("HEARTBEAT")
    if not hb:
        return False
    mode = mavutil.mode_string_v10(hb)  # e.g. 'GUIDED', 'LOITER', ...
    return mode.upper().startswith("GUIDED")

# type_mask: use only velocity (vx,vy,vz); ignore pos, accel, yaw, yaw_rate
TYPE_MASK_VEL_ONLY = 0b0000111111000111

while True:
    # keep receiving messages so HEARTBEAT updates
    master.recv_match(blocking=False)

    vz = UP_VD if is_guided() else 0.0

    time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_VEL_ONLY,
        0, 0, 0,      # x,y,z ignored
        0, 0, vz,     # vx,vy,vz (vz is "down" in NED)
        0, 0, 0,      # afx,afy,afz ignored
        0, 0          # yaw, yaw_rate ignored
    )

    # optional print
    # print("MODE:", mavutil.mode_string_v10(master.messages.get("HEARTBEAT")), "CMD:", "UP" if vz < 0 else "ZERO")

    time.sleep(DT)
