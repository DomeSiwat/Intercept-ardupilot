#!/usr/bin/env python3
"""
Single-file YOLO (Ultralytics) NCNN inference + coordinate Hz measurement
+ UDP sender for normalized pixel error.

- Loads NCNN-exported YOLO model from: yolo/drone_target_ncnn_model
- Reads frames from /dev/video10
- Picks ONLY ONE object per frame: highest-confidence detection
- Draws GREEN box around that object
- Writes annotated stream to /dev/video11 (FFmpeg v4l2 writer)
- Prints:
  - LOOP_HZ  : how fast the whole loop runs (roughly "inference FPS")
  - COORD_HZ : how often you get a VALID coordinate (cx, cy)

NEW:
- Sends UDP JSON packets containing:
    found, ex, ey, conf, cx, cy, w, h, t

Where:
  ex = normalized x error in [-1, 1]
  ey = normalized y error in [-1, 1]

Optional:
- --target-hz N : rate-limit the loop to a fixed Hz
- --print-every N : how often to print (seconds)
- --udp-ip, --udp-port : where to send UDP
- --udp-send-always : send even when no detection (found=false)

Requirements:
  pip install ultralytics opencv-python
  sudo apt-get install -y ffmpeg
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
from ultralytics import YOLO

# ---------- Paths / defaults ----------
BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR_DEFAULT = BASE_DIR / "yolo" / "drone_target_ncnn_model"
INPUT_DEV_DEFAULT = "/dev/video10"
OUTPUT_DEV_DEFAULT = "/dev/video11"


# ---------- Data structures ----------
@dataclass
class BestDetection:
    """Represents the single best detection in a frame."""
    xyxy: Tuple[int, int, int, int]  # (x1, y1, x2, y2) pixels
    conf: float
    cls: int


# ---------- Public helper you will reuse in other file later ----------
def get_best_xy_pixel_from_result(result) -> Tuple[Optional[int], Optional[int]]:
    """
    Returns the (x, y) pixel center of the single best detection.
    If no detection -> (None, None)
    """
    best = get_best_detection_from_result(result)
    if best is None:
        return None, None
    x1, y1, x2, y2 = best.xyxy
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    return cx, cy


# ---------- Core logic ----------
def get_best_detection_from_result(result) -> Optional[BestDetection]:
    """Picks ONLY ONE object: the highest-confidence detection in the result."""
    if result is None:
        return None

    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return None

    confs = boxes.conf
    if confs is None or len(confs) == 0:
        return None

    # best index by confidence
    best_i = int(confs.argmax().item())

    xyxy = boxes.xyxy[best_i]
    cls = boxes.cls[best_i] if boxes.cls is not None else 0
    conf = confs[best_i]

    # Convert to Python primitives
    x1 = int(xyxy[0].item())
    y1 = int(xyxy[1].item())
    x2 = int(xyxy[2].item())
    y2 = int(xyxy[3].item())
    conf_f = float(conf.item())
    cls_i = int(cls.item() if hasattr(cls, "item") else cls)

    return BestDetection(xyxy=(x1, y1, x2, y2), conf=conf_f, cls=cls_i)


def draw_best_box(frame, best: Optional[BestDetection], show_label: bool = True) -> None:
    """Draws a green box around the best detection (if any)."""
    if best is None:
        return

    x1, y1, x2, y2 = best.xyxy
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # green

    if show_label:
        label = f"best {best.conf:.2f} cls={best.cls}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), (0, 255, 0), -1)
        cv2.putText(
            frame,
            label,
            (x1 + 3, y1 - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
        )


def open_v4l2_input(device_path: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device_path)
    return cap


# ---------- UDP helpers ----------
def pixel_error_norm(cx: int, cy: int, w: int, h: int) -> Tuple[float, float]:
    """
    Normalize pixel error to roughly [-1, +1]
      ex < 0 => target left of center
      ex > 0 => target right of center
      ey < 0 => target above center
      ey > 0 => target below center
    """
    ex = (cx - (w / 2.0)) / (w / 2.0)
    ey = (cy - (h / 2.0)) / (h / 2.0)
    # clamp
    ex = max(-1.0, min(1.0, float(ex)))
    ey = max(-1.0, min(1.0, float(ey)))
    return ex, ey


def send_udp_error(
    sock: socket.socket,
    addr,
    *,
    found: bool,
    ex: float,
    ey: float,
    conf: float,
    cx: Optional[int],
    cy: Optional[int],
    w: int,
    h: int,
) -> None:
    """
    Sends a small JSON message. Non-blocking; drops if OS buffer is full.
    """
    msg = {
        "t": time.time(),  # wall time seconds
        "found": bool(found),
        "ex": float(ex),
        "ey": float(ey),
        "conf": float(conf),
        "cx": cx,
        "cy": cy,
        "w": int(w),
        "h": int(h),
    }
    data = json.dumps(msg, separators=(",", ":")).encode("utf-8")
    try:
        sock.sendto(data, addr)
    except (BlockingIOError, OSError):
        pass


# ---------- Output writer: FFmpeg -> v4l2 ----------
class FFmpegV4L2Writer:
    """
    Reliable writer to /dev/video11 using ffmpeg (v4l2).

    Pipes raw BGR frames to ffmpeg, and ffmpeg outputs to v4l2 in yuyv422.
    """

    def __init__(self, device_path: str, w: int, h: int, fps: float):
        self.device_path = device_path
        self.w = w
        self.h = h
        self.fps = fps

        cmd = [
            "ffmpeg",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{w}x{h}",
            "-r",
            str(int(round(fps))),
            "-i",
            "pipe:0",
            "-an",
            "-pix_fmt",
            "yuyv422",
            "-f",
            "v4l2",
            device_path,
        ]

        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

        if self.proc.stdin is None:
            raise RuntimeError("Failed to open ffmpeg stdin pipe.")

    def isOpened(self) -> bool:
        return self.proc.poll() is None and self.proc.stdin is not None

    def write(self, frame_bgr) -> None:
        if not self.isOpened():
            raise RuntimeError("ffmpeg process is not running (pipe closed).")
        self.proc.stdin.write(frame_bgr.tobytes())

    def release(self) -> None:
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
        except Exception:
            pass


def open_v4l2_output_writer(device_path: str, w: int, h: int, fps: float) -> FFmpegV4L2Writer:
    return FFmpegV4L2Writer(device_path, w, h, fps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--model",
        default=str(MODEL_DIR_DEFAULT),
        help="Path to NCNN model directory (default: yolo/drone_target_ncnn_model relative to this file)",
    )
    ap.add_argument("--input", default=INPUT_DEV_DEFAULT, help="Input video device (default: /dev/video10)")
    ap.add_argument("--output", default=OUTPUT_DEV_DEFAULT, help="Output video device (default: /dev/video11)")
    ap.add_argument("--conf", type=float, default=0.25, help="Confidence threshold for YOLO (default: 0.25)")
    ap.add_argument("--imgsz", type=int, default=640, help="Inference image size (default: 640)")
    ap.add_argument("--show", action="store_true", help="Show a local preview window (optional)")

    # Hz measurement / control
    ap.add_argument("--print-every", type=float, default=1.0, help="Print stats every N seconds (default: 1.0)")
    ap.add_argument(
        "--target-hz",
        type=float,
        default=0.0,
        help="Rate-limit the loop to this Hz (0 = no limit). Useful for stable control (default: 0)",
    )

    # UDP output
    ap.add_argument("--udp-ip", default="127.0.0.1", help="UDP target IP (default: 127.0.0.1)")
    ap.add_argument("--udp-port", type=int, default=5600, help="UDP target port (default: 5600)")
    ap.add_argument(
        "--udp-send-always",
        action="store_true",
        help="Send UDP every loop (even when no detection). Default: send only when detection exists.",
    )

    args = ap.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f"Model directory does not exist: {model_path}")

    # Load YOLO model (NCNN export directory)
    model = YOLO(str(model_path))

    cap = open_v4l2_input(args.input)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open input device: {args.input}")

    # Read one frame to get size
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Could not read first frame from input device.")

    h, w = frame.shape[:2]

    # Use capture FPS if available, else 30
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps != fps or fps < 1:
        fps = 30.0
    fps = float(int(round(fps)))

    out = open_v4l2_output_writer(args.output, w, h, fps)
    if not out.isOpened():
        raise RuntimeError(
            f"Cannot open output device via ffmpeg: {args.output}\n"
            f"Check permissions (group 'video') and that ffmpeg is installed."
        )

    # UDP socket
    udp_addr = (args.udp_ip, args.udp_port)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setblocking(False)

    print(f"[OK] Model:  {model_path}")
    print(f"[OK] Input:  {args.input}  ({w}x{h})")
    print(f"[OK] Output: {args.output}  (ffmpeg -> v4l2)")
    if args.target_hz and args.target_hz > 0:
        print(f"[OK] Rate limit: {args.target_hz:.1f} Hz")
    print(f"[OK] Printing every: {args.print_every:.2f}s")
    print(f"[OK] UDP:    {udp_addr[0]}:{udp_addr[1]}  (JSON)")
    print(f"[OK] UDP send always: {bool(args.udp_send_always)}\n")

    # --------- Stats counters ----------
    loop_count = 0
    loop_t0 = time.time()

    coord_count = 0
    coord_t0 = time.time()

    last_cx, last_cy = None, None
    last_conf = None

    # Rate limiting
    target_dt = (1.0 / args.target_hz) if args.target_hz and args.target_hz > 0 else 0.0

    try:
        while True:
            loop_start = time.time()

            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            # YOLO inference
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            result0 = results[0] if results else None

            best = get_best_detection_from_result(result0)
            cx, cy = get_best_xy_pixel_from_result(result0)

            found = (best is not None and cx is not None and cy is not None)

            if found:
                last_conf = best.conf
                coord_count += 1
                last_cx, last_cy = cx, cy
                ex, ey = pixel_error_norm(cx, cy, w, h)
                conf_val = float(best.conf)
            else:
                last_conf = None
                ex, ey = 0.0, 0.0
                conf_val = 0.0

            # Draw + output
            draw_best_box(frame, best, show_label=True)
            out.write(frame)

            if args.show:
                cv2.imshow("video11 preview", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # UDP send
            if args.udp_send_always or found:
                send_udp_error(
                    udp_sock,
                    udp_addr,
                    found=found,
                    ex=ex,
                    ey=ey,
                    conf=conf_val,
                    cx=cx,
                    cy=cy,
                    w=w,
                    h=h,
                )

            # ---- Stats ----
            loop_count += 1
            now = time.time()

            if (now - loop_t0) >= args.print_every:
                loop_hz = loop_count / (now - loop_t0)
                coord_hz = coord_count / (now - coord_t0)

                conf_str = f"{last_conf:.2f}" if last_conf is not None else "None"
                print(
                    f"[STATS] LOOP_HZ={loop_hz:5.1f} Hz | COORD_HZ={coord_hz:5.1f} Hz | "
                    f"last_xy=({last_cx},{last_cy}) | conf={conf_str}"
                )

                loop_t0 = now
                coord_t0 = now
                loop_count = 0
                coord_count = 0

            # ---- Optional rate limit ----
            if target_dt > 0:
                elapsed = time.time() - loop_start
                sleep_time = target_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    finally:
        cap.release()
        out.release()
        try:
            udp_sock.close()
        except Exception:
            pass
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
