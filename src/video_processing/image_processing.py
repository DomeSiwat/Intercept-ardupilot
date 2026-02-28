#!/usr/bin/env python3

import argparse
import json
import socket
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


# -----------------------------
# RTSP input pipeline
# -----------------------------

def build_rtsp_input(rtsp_url):
    return (
        f"rtspsrc location={rtsp_url} latency=0 buffer-mode=0 ! "
        "rtph264depay ! "
        "h264parse ! "
        "v4l2h264dec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true sync=false"
    )


# -----------------------------
# RTSP output pipeline
# -----------------------------

def build_rtsp_output(width, height, fps, rtsp_url):
    return (
        f"appsrc ! "
        f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! "
        "v4l2h264enc extra-controls=\"controls,video_bitrate=2000000;\" ! "
        "h264parse ! "
        f"rtspclientsink location={rtsp_url} protocols=tcp"
    )


# -----------------------------
# Main
# -----------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtsp-in", required=True)
    ap.add_argument("--rtsp-out", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--udp-ip", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=5600)
    args = ap.parse_args()

    if not Path(args.model).exists():
        raise FileNotFoundError("Model directory not found")

    print("Loading YOLO...")
    model = YOLO(args.model)

    cap = cv2.VideoCapture(
        build_rtsp_input(args.rtsp_in),
        cv2.CAP_GSTREAMER,
    )

    if not cap.isOpened():
        raise RuntimeError("Failed to open RTSP input")

    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Failed to read first frame")

    h, w = frame.shape[:2]
    fps = 30

    out = cv2.VideoWriter(
        build_rtsp_output(w, h, fps, args.rtsp_out),
        cv2.CAP_GSTREAMER,
        0,
        fps,
        (w, h),
        True,
    )

    if not out.isOpened():
        raise RuntimeError("Failed to open RTSP output")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    udp_addr = (args.udp_ip, args.udp_port)

    print("Running YOLO → RTSP out")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        results = model.predict(frame, conf=args.conf, verbose=False)
        result0 = results[0] if results else None

        if result0 and result0.boxes is not None and len(result0.boxes) > 0:
            best_i = int(result0.boxes.conf.argmax().item())
            xyxy = result0.boxes.xyxy[best_i]

            x1, y1, x2, y2 = map(int, xyxy.tolist())
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        out.write(frame)

    cap.release()
    out.release()
    sock.close()


if __name__ == "__main__":
    main()