#!/usr/bin/env python3
"""
flight_recorder.py  —  Lightweight ROS frame saver for Jetson
=============================================================
Subscribes to a ROS camera topic and saves frames as JPEG at a fixed rate.
Also logs drone X,Y,Z position from MAVROS into a CSV alongside each frame.
Designed to have minimal CPU impact so VIO is not affected.

Usage:
  # Color camera at 2 fps (default) — saves to ~/seed_tracker/detections
  python3 flight_recorder.py

  # Infra camera at 3 fps, custom save dir
  python3 flight_recorder.py --camera infra --fps 3 --save-dir /tmp/recordings

  # Explicit topic override
  python3 flight_recorder.py --topic /camera/color/image_raw --fps 2

Output:
  ~/seed_tracker/detections/
      frame_000000.jpg   ← HD (1280x720) JPEG
      frame_000001.jpg
      ...
      positions.csv      ← frame_file, timestamp, pos_x, pos_y, pos_z, heading_deg
"""

import argparse
import csv
import math
import os
import signal
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# ──────────────────────────────────────────────────────────────
# CAMERA PRESETS
# ──────────────────────────────────────────────────────────────
CAMERA_PRESETS = {
    "color": {
        "topic":    "/camera/color/image_raw",
        "encoding": "bgr8",
    },
    "infra": {
        "topic":    "/camera/infra1/image_rect_raw",
        "encoding": "mono8",
    },
}
DEFAULT_CAMERA   = "color"
SAVE_WIDTH       = 1280         # always save at this resolution
SAVE_HEIGHT      = 720
BLUR_THRESHOLD   = 20.0         # Laplacian variance below this → skip frame
JPEG_QUALITY     = 85
MAVROS_NS        = "/mavros"

# ──────────────────────────────────────────────────────────────
# GLOBALS
# ──────────────────────────────────────────────────────────────
latest_frame   = None
frame_lock     = threading.Lock()

latest_pos     = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
pos_lock       = threading.Lock()
pos_received   = False

running        = True
saved_count    = 0
skipped_blur   = 0
total_received = 0


# ──────────────────────────────────────────────────────────────
# CALLBACKS
# ──────────────────────────────────────────────────────────────
def image_callback(msg):
    global latest_frame, total_received
    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        enc  = msg.encoding
        if enc == "mono8":
            gray  = data.reshape(msg.height, msg.width)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif enc == "rgb8":
            frame = data.reshape(msg.height, msg.width, 3)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8", "8UC3"):
            frame = data.reshape(msg.height, msg.width, 3).copy()
        else:
            channels = len(data) // (msg.height * msg.width)
            frame = data.reshape(msg.height, msg.width, channels).copy()
            if channels == 1:
                frame = cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)

        # Resize to HD if not already 1280×720
        h, w = frame.shape[:2]
        if w != SAVE_WIDTH or h != SAVE_HEIGHT:
            frame = cv2.resize(frame, (SAVE_WIDTH, SAVE_HEIGHT), interpolation=cv2.INTER_LINEAR)

        with frame_lock:
            latest_frame = frame
        total_received += 1
    except Exception as e:
        rospy.logwarn(f"[recorder] image_callback error: {e}")


def pose_callback(msg):
    global pos_received
    try:
        p = msg.pose.position
        q = msg.pose.orientation
        # ENU: x=East, y=North — convert yaw from quaternion
        yaw_rad = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        with pos_lock:
            latest_pos["x"]       = p.x   # East (ENU)
            latest_pos["y"]       = p.y   # North (ENU)
            latest_pos["z"]       = p.z   # Up (ENU)
            latest_pos["yaw_deg"] = math.degrees(yaw_rad)
        pos_received = True
    except Exception as e:
        rospy.logwarn(f"[recorder] pose_callback error: {e}")


# ──────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────
def is_blurry(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var() < BLUR_THRESHOLD


def get_pos_snapshot():
    with pos_lock:
        return dict(latest_pos)


# ──────────────────────────────────────────────────────────────
# SAVE LOOP
# ──────────────────────────────────────────────────────────────
def save_loop(save_dir, fps):
    global saved_count, skipped_blur, running

    csv_path = os.path.join(save_dir, "positions.csv")
    interval  = 1.0 / fps
    last_save = time.time() - interval   # save first frame immediately
    last_stat = time.time()

    with open(csv_path, "w", newline="") as csvf:
        writer = csv.writer(csvf)
        writer.writerow(["frame_file", "timestamp", "pos_x_east_m",
                         "pos_y_north_m", "pos_z_up_m", "heading_deg"])

        while running and not rospy.is_shutdown():
            now = time.time()
            if now - last_save < interval:
                time.sleep(0.01)
                continue

            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None

            if frame is None:
                time.sleep(0.05)
                continue

            if is_blurry(frame):
                skipped_blur += 1
                last_save = now
                continue

            pos   = get_pos_snapshot()
            fname = f"frame_{saved_count:06d}.jpg"
            fpath = os.path.join(save_dir, fname)

            cv2.imwrite(fpath, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

            writer.writerow([
                fname,
                f"{now:.3f}",
                f"{pos['x']:.4f}",
                f"{pos['y']:.4f}",
                f"{pos['z']:.4f}",
                f"{pos['yaw_deg']:.2f}",
            ])
            csvf.flush()

            saved_count += 1
            last_save = now

            # Stats every 10s
            if now - last_stat >= 10.0:
                size_mb = sum(
                    os.path.getsize(os.path.join(save_dir, f))
                    for f in os.listdir(save_dir) if f.endswith('.jpg')
                ) / 1e6
                print(f"[recorder] saved={saved_count}  blur_skip={skipped_blur}"
                      f"  rx={total_received}  disk={size_mb:.1f}MB"
                      f"  pos=({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
                last_stat = now


# ──────────────────────────────────────────────────────────────
# SIGNAL + ARGS
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    global running
    print(f"\n[recorder] Stopping. Saved {saved_count} frames ({skipped_blur} blur-skipped).")
    running = False
    rospy.signal_shutdown("User quit")


def parse_args():
    parser = argparse.ArgumentParser(description="Lightweight ROS frame recorder with position logging")

    cam_group = parser.add_mutually_exclusive_group()
    cam_group.add_argument(
        "--camera", choices=list(CAMERA_PRESETS.keys()), default=DEFAULT_CAMERA,
        help=f"camera preset (default: {DEFAULT_CAMERA})"
    )
    cam_group.add_argument(
        "--topic", default=None,
        help="explicit ROS image topic to subscribe to"
    )

    parser.add_argument("--fps",      type=float, default=2.0,
                        help="save rate in fps (default: 2)")
    parser.add_argument("--save-dir", default=os.path.expanduser("~/seed_tracker/detections"),
                        help="directory to save frames and positions.csv (default: ~/seed_tracker/detections)")
    return parser.parse_args()


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    topic     = args.topic if args.topic else CAMERA_PRESETS[args.camera]["topic"]
    cam_label = args.topic if args.topic else args.camera

    # Save directly to the specified directory (no timestamped subfolder)
    save_dir = os.path.expanduser(args.save_dir)
    os.makedirs(save_dir, exist_ok=True)

    signal.signal(signal.SIGINT, sigint_handler)

    rospy.init_node("flight_recorder", anonymous=True, disable_signals=True)

    rospy.Subscriber(topic, Image, image_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose", PoseStamped, pose_callback, queue_size=1)

    print(f"[recorder] camera    : {cam_label}")
    print(f"[recorder] topic     : {topic}")
    print(f"[recorder] fps       : {args.fps}")
    print(f"[recorder] save res  : {SAVE_WIDTH}x{SAVE_HEIGHT} (HD)")
    print(f"[recorder] save_dir  : {save_dir}")
    print(f"[recorder] positions : {save_dir}/positions.csv")
    print("[recorder] Waiting for first frame... (Ctrl+C to stop)")

    # Wait for first frame
    t0 = time.time()
    while latest_frame is None and not rospy.is_shutdown():
        if time.time()-t0 > 10.0:
            print("[recorder] ERROR: No frames received after 10s. Is camera node running?")
            return
        time.sleep(0.1)

    if not pos_received:
        print("[recorder] WARNING: No MAVROS position yet — frames will be logged with pos=(0,0,0)")
    else:
        pos = get_pos_snapshot()
        print(f"[recorder] MAVROS position OK: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")

    h, w = latest_frame.shape[:2]
    print(f"[recorder] Camera frames: {w}x{h} -> saving as {SAVE_WIDTH}x{SAVE_HEIGHT}")
    print("[recorder] Recording started.\n")

    save_loop(save_dir, args.fps)

    print(f"[recorder] Done. {saved_count} frames saved to: {save_dir}")


if __name__ == "__main__":
    main()
