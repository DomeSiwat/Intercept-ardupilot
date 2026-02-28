#!/usr/bin/env python3

import cv2
import json
import socket
import time
from ultralytics import YOLO

INPUT = "/dev/video10"
MODEL = "yolo/drone_target_ncnn_model"
UDP_IP = "127.0.0.1"
UDP_PORT = 5600

def pixel_error_norm(cx, cy, w, h):
    ex = (cx - w / 2) / (w / 2)
    ey = (cy - h / 2) / (h / 2)
    return max(-1, min(1, ex)), max(-1, min(1, ey))

def main():
    cap = cv2.VideoCapture(INPUT)
    model = YOLO(MODEL)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]

        results = model.predict(frame, conf=0.25, imgsz=640, verbose=False)
        result0 = results[0] if results else None

        found = False
        ex = ey = 0.0
        conf_val = 0.0
        cx = cy = None

        if result0 and result0.boxes is not None and len(result0.boxes) > 0:
            best_i = int(result0.boxes.conf.argmax().item())
            xyxy = result0.boxes.xyxy[best_i]
            conf_val = float(result0.boxes.conf[best_i].item())

            x1, y1, x2, y2 = map(int, xyxy.tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            ex, ey = pixel_error_norm(cx, cy, w, h)
            found = True

        msg = {
            "t": time.time(),
            "found": found,
            "ex": ex,
            "ey": ey,
            "conf": conf_val,
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
        }

        try:
            sock.sendto(json.dumps(msg).encode(), (UDP_IP, UDP_PORT))
        except:
            pass

if __name__ == "__main__":
    main()