#!/usr/bin/env python3
"""
yellow_border_node.py  —  Background ROS node for arena border detection
=========================================================================
Subscribes to the color camera, detects yellow in 3 regions (left, right, front),
publishes a comma-separated string of active regions.

Published topic:
    /yellow_border/status  (std_msgs/String)
    Values: "none", "left", "right", "front", "left,right", "left,front", etc.

Usage:
    python3 yellow_border_node.py                      # defaults
    python3 yellow_border_node.py --debug              # show live mask window
    python3 yellow_border_node.py --rate 10            # faster publish rate

Run this in a separate terminal BEFORE starting the navigation code.
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ──────────────────────────────────────────────────────────────
#  YELLOW HSV RANGE  —  calibrate with hsv_tuner.py
# ──────────────────────────────────────────────────────────────
YELLOW_H_LO = 15
YELLOW_H_HI = 38
YELLOW_S_LO = 80
YELLOW_S_HI = 255
YELLOW_V_LO = 80
YELLOW_V_HI = 255

# ──────────────────────────────────────────────────────────────
#  DETECTION REGIONS  (fractions of frame after flip correction)
# ──────────────────────────────────────────────────────────────
# Camera is upside-down → we flip 180° first.
# Camera is tilted forward → top of corrected frame = ground ahead.
#
#   ┌──────────────────────────────┐
#   │        FRONT STRIP           │  top FRONT_FRAC of frame
#   ├──────┬──────────────┬────────┤
#   │ LEFT │              │ RIGHT  │  left/right SIDE_FRAC strips
#   │      │   (center)   │        │
#   └──────┴──────────────┴────────┘

FRONT_FRAC  = 0.30    # top 30% = front
SIDE_FRAC   = 0.20    # left/right 20% each

# Minimum yellow pixel fraction in a region to count as "detected"
MIN_YELLOW_FRAC = 0.1   # 10% of the region's pixels — tune if needed

# ──────────────────────────────────────────────────────────────
#  CAMERA
# ──────────────────────────────────────────────────────────────
DEFAULT_TOPIC = "/camera/color/image_raw"
FLIP_IMAGE    = True    # True = camera is upside-down

# ──────────────────────────────────────────────────────────────
#  GLOBALS
# ──────────────────────────────────────────────────────────────
latest_frame = None
frame_lock   = threading.Lock()
debug_mode   = False


# ──────────────────────────────────────────────────────────────
#  IMAGE CALLBACK
# ──────────────────────────────────────────────────────────────
def image_callback(msg):
    global latest_frame
    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        enc  = msg.encoding

        if enc == "mono8":
            # Can't do color detection on mono — warn and skip
            return
        elif enc == "rgb8":
            frame = data.reshape(msg.height, msg.width, 3)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8", "8UC3"):
            frame = data.reshape(msg.height, msg.width, 3).copy()
        else:
            frame = data.reshape(msg.height, msg.width, 3).copy()

        if FLIP_IMAGE:
            frame = cv2.flip(frame, -1)

        with frame_lock:
            latest_frame = frame
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[yellow] callback error: {e}")


# ──────────────────────────────────────────────────────────────
#  DETECTION
# ──────────────────────────────────────────────────────────────
def detect_yellow_regions(frame):
    """Returns set of detected regions: {'left', 'right', 'front'} or empty."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([YELLOW_H_LO, YELLOW_S_LO, YELLOW_V_LO])
    upper = np.array([YELLOW_H_HI, YELLOW_S_HI, YELLOW_V_HI])
    mask  = cv2.inRange(hsv, lower, upper)

    regions = set()

    # Front strip: top FRONT_FRAC
    front_h  = int(h * FRONT_FRAC)
    front_mask = mask[0:front_h, :]
    if _region_active(front_mask):
        regions.add("front")

    # Left strip: left SIDE_FRAC, below front strip
    side_w   = int(w * SIDE_FRAC)
    left_mask = mask[front_h:, 0:side_w]
    if _region_active(left_mask):
        regions.add("left")

    # Right strip: right SIDE_FRAC, below front strip
    right_mask = mask[front_h:, w - side_w:]
    if _region_active(right_mask):
        regions.add("right")

    return regions, mask


def _region_active(region_mask):
    """Check if yellow pixel fraction exceeds threshold."""
    total = region_mask.shape[0] * region_mask.shape[1]
    if total == 0:
        return False
    yellow = cv2.countNonZero(region_mask)
    return (yellow / total) >= MIN_YELLOW_FRAC


# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global debug_mode

    parser = argparse.ArgumentParser(description="Yellow border detection ROS node")
    parser.add_argument("--topic", default=DEFAULT_TOPIC,
                        help=f"camera topic (default: {DEFAULT_TOPIC})")
    parser.add_argument("--rate",  type=float, default=5.0,
                        help="publish rate in Hz (default: 5)")
    parser.add_argument("--debug", action="store_true",
                        help="show live detection window (needs display)")
    parser.add_argument("--no-flip", action="store_true",
                        help="disable 180° flip (if camera is right-side-up)")
    args = parser.parse_args()

    global FLIP_IMAGE
    if args.no_flip:
        FLIP_IMAGE = False

    debug_mode = args.debug

    rospy.init_node("yellow_border_node", anonymous=True)
    rospy.Subscriber(args.topic, Image, image_callback, queue_size=1, buff_size=2**24)
    pub = rospy.Publisher("/yellow_border/status", String, queue_size=5)

    print(f"[yellow] topic     : {args.topic}")
    print(f"[yellow] rate      : {args.rate} Hz")
    print(f"[yellow] flip      : {FLIP_IMAGE}")
    print(f"[yellow] HSV range : H[{YELLOW_H_LO}-{YELLOW_H_HI}] "
          f"S[{YELLOW_S_LO}-{YELLOW_S_HI}] V[{YELLOW_V_LO}-{YELLOW_V_HI}]")
    print(f"[yellow] min frac  : {MIN_YELLOW_FRAC}")
    print(f"[yellow] debug     : {debug_mode}")
    print("[yellow] Waiting for frames...")

    # Wait for first frame
    while latest_frame is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    if rospy.is_shutdown():
        return

    fh, fw = latest_frame.shape[:2]
    print(f"[yellow] Frame size: {fw}x{fh} (after flip={FLIP_IMAGE})")
    print("[yellow] Publishing on /yellow_border/status")

    rate = rospy.Rate(args.rate)
    last_status = ""

    while not rospy.is_shutdown():
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            rate.sleep()
            continue

        regions, mask = detect_yellow_regions(frame)
        status_str = ",".join(sorted(regions)) if regions else "none"

        pub.publish(String(data=status_str))

        # Print only on change
        if status_str != last_status:
            rospy.loginfo(f"[yellow] border: {status_str}")
            last_status = status_str

        if debug_mode:
            h, w = frame.shape[:2]
            front_h = int(h * FRONT_FRAC)
            side_w  = int(w * SIDE_FRAC)

            # Draw region boundaries on frame
            debug_frame = frame.copy()
            cv2.line(debug_frame, (0, front_h), (w, front_h), (0, 255, 255), 2)
            cv2.line(debug_frame, (side_w, front_h), (side_w, h), (0, 255, 255), 2)
            cv2.line(debug_frame, (w - side_w, front_h), (w - side_w, h), (0, 255, 255), 2)

            cv2.putText(debug_frame, f"FRONT", (w // 2 - 40, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(debug_frame, "L", (5, front_h + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(debug_frame, "R", (w - 25, front_h + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.putText(debug_frame, f"Status: {status_str}", (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if regions else (0, 200, 0), 2)

            # Color the mask for display
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            combined = np.hstack([debug_frame, mask_color])
            cv2.imshow("Yellow Border Detection", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    if debug_mode:
        cv2.destroyAllWindows()
    print("[yellow] Node stopped.")


if __name__ == "__main__":
    main()
