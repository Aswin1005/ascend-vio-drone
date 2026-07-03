#!/usr/bin/env python3
"""
hsv_tuner.py  —  Laptop utility for calibrating yellow border HSV range
========================================================================
Opens a recorded frame and provides trackbar sliders to find the
exact HSV range for the yellow arena borders. When done, prints the
values to paste into yellow_border_node.py.

Usage:
  python hsv_tuner.py --image ./recordings/2026-06-17_08-30/frame_000010.jpg
  python hsv_tuner.py --image ./some_photo.jpg

Controls:
  - Adjust sliders until ONLY the yellow border is white in the mask
  - Press 'q' to quit — values are printed to console
  - Press 's' to save a screenshot of the current mask
"""

import argparse
import cv2
import numpy as np
import sys


def nothing(x):
    pass


def main():
    parser = argparse.ArgumentParser(description="HSV tuner for yellow border calibration")
    parser.add_argument("--image", required=True, help="path to a frame with visible yellow border")
    parser.add_argument("--flip", action="store_true",
                        help="flip image 180° (if camera was upside-down when captured)")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"ERROR: Could not load '{args.image}'")
        sys.exit(1)

    if args.flip:
        img = cv2.flip(img, -1)

    # Resize for display if too large
    h, w = img.shape[:2]
    if w > 1000:
        scale = 800.0 / w
        img = cv2.resize(img, (int(w * scale), int(h * scale)))

    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)

    # Default yellow range — good starting point
    cv2.createTrackbar("H Lo", "HSV Tuner", 18, 179, nothing)
    cv2.createTrackbar("H Hi", "HSV Tuner", 38, 179, nothing)
    cv2.createTrackbar("S Lo", "HSV Tuner", 80, 255, nothing)
    cv2.createTrackbar("S Hi", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("V Lo", "HSV Tuner", 80, 255, nothing)
    cv2.createTrackbar("V Hi", "HSV Tuner", 255, 255, nothing)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    print("Adjust sliders. Press 'q' when yellow border is cleanly isolated.")
    print("Press 's' to save mask screenshot.\n")

    while True:
        h_lo = cv2.getTrackbarPos("H Lo", "HSV Tuner")
        h_hi = cv2.getTrackbarPos("H Hi", "HSV Tuner")
        s_lo = cv2.getTrackbarPos("S Lo", "HSV Tuner")
        s_hi = cv2.getTrackbarPos("S Hi", "HSV Tuner")
        v_lo = cv2.getTrackbarPos("V Lo", "HSV Tuner")
        v_hi = cv2.getTrackbarPos("V Hi", "HSV Tuner")

        lower = np.array([h_lo, s_lo, v_lo])
        upper = np.array([h_hi, s_hi, v_hi])
        mask = cv2.inRange(hsv, lower, upper)

        # Show yellow pixel count
        yellow_px = cv2.countNonZero(mask)
        total_px = mask.shape[0] * mask.shape[1]
        pct = 100.0 * yellow_px / total_px

        display = img.copy()
        cv2.putText(display, f"Yellow: {yellow_px}px ({pct:.1f}%)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display, f"H:[{h_lo}-{h_hi}] S:[{s_lo}-{s_hi}] V:[{v_lo}-{v_hi}]",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("HSV Tuner", display)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite("hsv_mask.jpg", mask)
            print("Saved mask to hsv_mask.jpg")

    cv2.destroyAllWindows()

    print("\n" + "=" * 50)
    print("PASTE THESE INTO yellow_border_node.py:")
    print("=" * 50)
    print(f"YELLOW_H_LO = {h_lo}")
    print(f"YELLOW_H_HI = {h_hi}")
    print(f"YELLOW_S_LO = {s_lo}")
    print(f"YELLOW_S_HI = {s_hi}")
    print(f"YELLOW_V_LO = {v_lo}")
    print(f"YELLOW_V_HI = {v_hi}")
    print("=" * 50)


if __name__ == "__main__":
    main()
