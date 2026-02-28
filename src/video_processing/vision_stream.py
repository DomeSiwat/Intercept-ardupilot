#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

INPUT = "/dev/video10"
OUTPUT = "/dev/video11"
MODEL = "yolo/drone_target_ncnn_model"

def main():
    cap = cv2.VideoCapture(INPUT)
    model = YOLO(MODEL)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = 30

    out = cv2.VideoWriter(
        OUTPUT,
        cv2.VideoWriter_fourcc(*"YUYV"),
        fps,
        (w, h)
    )

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        results = model.predict(frame, conf=0.25, imgsz=640, verbose=False)
        result0 = results[0] if results else None

        if result0 and result0.boxes is not None and len(result0.boxes) > 0:
            best_i = int(result0.boxes.conf.argmax().item())
            xyxy = result0.boxes.xyxy[best_i]
            x1, y1, x2, y2 = map(int, xyxy.tolist())
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        out.write(frame)

if __name__ == "__main__":
    main()