#!/usr/bin/env python3
"""
Camera-based object chase controller.

Receives UDP JSON:
    {
        "found": bool,
        "ex": float,   # normalized horizontal error [-1,1]
        "ey": float,   # normalized vertical error [-1,1]
        ...
    }

Outputs:
    vn, ve, vd, yaw_deg
"""

import json
import math
import socket
import time
from dataclasses import dataclass


# ===============================
# Configurable Parameters
# ===============================

@dataclass
class ChaseParams:
    forward_speed: float = 3.0          # constant forward speed (m/s)
    yaw_kp: float = 60.0                # deg per unit error
    yaw_max_rate: float = 120.0         # deg/s clamp
    vertical_kp: float = 2.0            # m/s per unit error
    vertical_max: float = 2.0           # m/s clamp
    search_yaw_rate: float = 30.0       # deg/s when target lost
    lost_timeout: float = 0.5           # seconds before considered lost


# ===============================
# Main Chase Controller
# ===============================

class CameraChaseController:
    def __init__(self, udp_ip="127.0.0.1", udp_port=5600, params=None):
        self.params = params or ChaseParams()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.sock.setblocking(False)

        self.current_yaw_deg = 0.0
        self.last_target_time = 0.0
        self.last_ex = 0.0
        self.last_ey = 0.0

        self.last_time = time.time()

    # ----------------------------------

    def _read_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = json.loads(data.decode("utf-8"))

            if msg.get("found", False):
                self.last_ex = float(msg.get("ex", 0.0))
                self.last_ey = float(msg.get("ey", 0.0))
                self.last_target_time = time.time()

        except BlockingIOError:
            pass
        except Exception:
            pass

    # ----------------------------------

    def update(self):
        """
        Returns:
            vn, ve, vd, yaw_deg
        """

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self._read_udp()

        p = self.params

        target_visible = (now - self.last_target_time) < p.lost_timeout

        # --------------------------
        # YAW CONTROL
        # --------------------------
        if target_visible:
            yaw_rate = p.yaw_kp * self.last_ex
            yaw_rate = max(-p.yaw_max_rate, min(p.yaw_max_rate, yaw_rate))
        else:
            yaw_rate = p.search_yaw_rate  # slow scan

        self.current_yaw_deg += yaw_rate * dt

        # --------------------------
        # FORWARD VELOCITY
        # --------------------------
        speed = p.forward_speed

        yaw_rad = math.radians(self.current_yaw_deg)

        vn = speed * math.cos(yaw_rad)
        ve = speed * math.sin(yaw_rad)

        # --------------------------
        # VERTICAL CONTROL
        # --------------------------
        if target_visible:
            vd = -p.vertical_kp * self.last_ey
            vd = max(-p.vertical_max, min(p.vertical_max, vd))
        else:
            vd = 0.0

        return vn, ve, vd, self.current_yaw_deg