#!/usr/bin/env python3
"""
offline_detector.py  —  Run on LAPTOP after flight
===================================================
Loads recorded JPEG frames from flight_recorder.py and runs ORB-based
seed detection. No ROS required — pure OpenCV + numpy.

Usage:
  python offline_detector.py ^
      --frames ./recordings/2026-06-15_14-30 ^
      --seeds  ./seed_tracker/seeds/seeds_clamp ^
      --save-dir ./detections_offline ^
      --display

  # Run against multiple seed folders at once
  python offline_detector.py ^
      --frames ./recordings/2026-06-15_14-30 ^
      --seeds ./seed_tracker/seeds/seeds_clamp ./seed_tracker/seeds/seeds_mouse ^
      --display --stop-on-first
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

# ──────────────────────────────────────────────────────────────
# DETECTION CONFIG  (edit if needed)
# ──────────────────────────────────────────────────────────────
MAX_FEATURES      = 1000
FAST_THRESHOLD    = 8
MATCH_RATIO       = 0.85
MIN_MATCH_COUNT   = 10
MIN_INLIERS       = 8
MIN_INLIER_RATIO  = 0.30
LR_SIZE           = 128      # patch matching resolution
PATCH_SIZES       = [256, 512]
PATCH_OVERLAP     = 0.5
BLUR_THRESHOLD    = 30.0
SEED_CENTER_CROP  = 1.0      # 0.75 if seed photos have noisy backgrounds
JPEG_QUALITY      = 95
# ──────────────────────────────────────────────────────────────


# ── Seed loading ──────────────────────────────────────────────
class SeedTemplate:
    def __init__(self, name, keypoints, descriptors, width, height):
        self.name        = name
        self.keypoints   = keypoints
        self.descriptors = descriptors
        self.width       = width
        self.height      = height


def load_seeds_from_dirs(seed_dirs, orb, lr_size=128, center_crop=1.0):
    seeds = []
    exts  = {".jpg", ".jpeg", ".png", ".bmp"}

    for seed_dir in seed_dirs:
        if not os.path.isdir(seed_dir):
            print(f"[seeds] WARNING: '{seed_dir}' is not a directory — skipping")
            continue

        for fname in sorted(os.listdir(seed_dir)):
            if os.path.splitext(fname)[1].lower() not in exts:
                continue
            path = os.path.join(seed_dir, fname)
            img  = cv2.imread(path)
            if img is None:
                print(f"[seeds] WARNING: Could not load {path}")
                continue

            h, w = img.shape[:2]
            if center_crop < 1.0:
                ch = int(h * center_crop); cw = int(w * center_crop)
                y0 = (h - ch) // 2;        x0 = (w - cw) // 2
                img = img[y0:y0+ch, x0:x0+cw]
                h, w = img.shape[:2]

            lr = cv2.resize(img, (lr_size, lr_size), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray  = clahe.apply(gray)
            kps, descs = orb.detectAndCompute(gray, None)

            if descs is None or len(kps) < 4:
                print(f"[seeds] WARNING: Too few features in {fname} ({len(kps) if kps else 0}) — skipping")
                continue

            name = os.path.splitext(fname)[0]
            seeds.append(SeedTemplate(name, kps, descs, lr_size, lr_size))
            print(f"[seeds] Loaded: {fname}  ({len(kps)} keypoints)")

    return seeds


# ── Detection pipeline ────────────────────────────────────────
class OfflineDetector:
    def __init__(self, seeds):
        self.seeds   = seeds
        self.orb     = cv2.ORB_create(
            nfeatures=MAX_FEATURES,
            fastThreshold=FAST_THRESHOLD,
            scoreType=cv2.ORB_HARRIS_SCORE,
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _axis_positions(self, length, patch_size):
        if patch_size > length:
            return []
        stride = max(int(patch_size * (1.0 - PATCH_OVERLAP)), 1)
        positions = list(range(0, length - patch_size + 1, stride))
        last = length - patch_size
        if not positions or positions[-1] != last:
            positions.append(last)
        return sorted(set(positions))

    def _patches(self, frame, patch_size):
        h, w = frame.shape[:2]
        result = []
        for y in self._axis_positions(h, patch_size):
            for x in self._axis_positions(w, patch_size):
                crop  = frame[y:y+patch_size, x:x+patch_size]
                lr    = cv2.resize(crop, (LR_SIZE, LR_SIZE), interpolation=cv2.INTER_AREA)
                result.append((x, y, lr))
        return result

    def _is_blurry(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(gray, cv2.CV_64F).var() < BLUR_THRESHOLD

    def _match_seed(self, seed, kps, descs):
        if descs is None or len(kps) < 4:
            return None
        try:
            knn = self.matcher.knnMatch(seed.descriptors, descs, k=2)
        except cv2.error:
            return None

        good = [m for pair in knn if len(pair)==2
                for m, n in [pair] if m.distance < MATCH_RATIO * n.distance]

        if len(good) < MIN_MATCH_COUNT:
            return None

        src = np.float32([seed.keypoints[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        dst = np.float32([kps[m.trainIdx].pt for m in good]).reshape(-1,1,2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
        if H is None or mask is None:
            return None

        inliers = int(mask.ravel().sum())
        if inliers < MIN_INLIERS:
            return None
        score = inliers / max(len(good), 1)
        if score < MIN_INLIER_RATIO:
            return None
        return inliers, score, H

    def _project_quad(self, H, seed, px, py, patch_size):
        scale = patch_size / LR_SIZE
        corners = np.float32([[0,0],[seed.width-1,0],
                               [seed.width-1,seed.height-1],[0,seed.height-1]]).reshape(-1,1,2)
        try:
            proj = cv2.perspectiveTransform(corners, H).reshape(-1,2)
            proj[:,0] = proj[:,0]*scale + px
            proj[:,1] = proj[:,1]*scale + py
            return proj
        except Exception:
            return None

    def process(self, frame):
        """Returns list of detections: [{seed_name, inliers, score, quad, bbox}]"""
        if self._is_blurry(frame):
            return None, True   # (detections, is_blurry)

        # Build patch features
        patch_features = {}
        for ps in PATCH_SIZES:
            for px, py, lr_bgr in self._patches(frame, ps):
                gray = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2GRAY)
                gray = self.clahe.apply(gray)
                kps, descs = self.orb.detectAndCompute(gray, None)
                if descs is not None and len(kps) >= 4:
                    patch_features[(px, py, ps)] = (kps, descs)

        best = None
        for seed in self.seeds:
            for (px, py, ps), (kps, descs) in patch_features.items():
                m = self._match_seed(seed, kps, descs)
                if m is None:
                    continue
                inliers, score, H = m
                if best is not None and inliers <= best["inliers"]:
                    continue

                quad = self._project_quad(H, seed, px, py, ps)
                if quad is None:
                    continue

                fh, fw = frame.shape[:2]
                x1 = max(0, int(quad[:,0].min())); y1 = max(0, int(quad[:,1].min()))
                x2 = min(fw-1, int(quad[:,0].max())); y2 = min(fh-1, int(quad[:,1].max()))
                if x2-x1 < 8 or y2-y1 < 8:
                    continue

                best = {"seed_name": seed.name, "inliers": inliers,
                        "score": score, "quad": quad,
                        "bbox": (x1, y1, x2-x1, y2-y1)}

        return best, False


def draw_detection(frame, det):
    canvas = frame.copy()
    if det is None:
        cv2.putText(canvas, "No detection", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,200), 2)
        return canvas
    pts = det["quad"].astype(np.int32).reshape(-1,1,2)
    cv2.polylines(canvas, [pts], True, (0,255,0), 3)
    x, y, w, h = det["bbox"]
    cv2.rectangle(canvas, (x,y), (x+w, y+h), (0,220,80), 2)
    label = f"{det['seed_name']}  inl:{det['inliers']}  score:{det['score']:.2f}"
    cv2.putText(canvas, label, (x, max(24, y-8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
    return canvas


def parse_args():
    parser = argparse.ArgumentParser(description="Offline seed detector — run on laptop after flight")
    parser.add_argument("--frames",   required=True, nargs="+",
                        help="folder(s) containing recorded JPEG frames")
    parser.add_argument("--seeds",    required=True, nargs="+",
                        help="seed image folder(s)")
    parser.add_argument("--save-dir", default="./detections_offline",
                        help="output directory for annotated frames + CSV (default: ./detections_offline)")
    parser.add_argument("--display",  action="store_true",
                        help="show live window while processing")
    parser.add_argument("--stop-on-first", action="store_true",
                        help="stop processing after first confirmed detection")
    parser.add_argument("--seed-center-crop", type=float, default=SEED_CENTER_CROP,
                        help=f"center crop fraction for seeds (default: {SEED_CENTER_CROP})")
    return parser.parse_args()


def main():
    args = parse_args()

    os.makedirs(args.save_dir, exist_ok=True)

    # Collect frame files from all specified folders
    frame_files = []
    for folder in args.frames:
        if not os.path.isdir(folder):
            print(f"[offline] WARNING: '{folder}' not found — skipping")
            continue
        files = sorted(
            os.path.join(folder, f)
            for f in os.listdir(folder)
            if f.lower().endswith((".jpg",".jpeg",".png"))
        )
        frame_files.extend(files)
        print(f"[offline] Found {len(files)} frames in {folder}")

    if not frame_files:
        print("[offline] ERROR: No frames found. Check --frames path.")
        sys.exit(1)

    # Load seeds
    orb   = cv2.ORB_create(nfeatures=MAX_FEATURES, fastThreshold=FAST_THRESHOLD,
                            scoreType=cv2.ORB_HARRIS_SCORE)
    seeds = load_seeds_from_dirs(args.seeds, orb, LR_SIZE, args.seed_center_crop)

    if not seeds:
        print("[offline] ERROR: No seeds loaded.")
        sys.exit(1)

    print(f"\n[offline] {len(frame_files)} frames | {len(seeds)} seeds")
    print(f"[offline] Saving to: {args.save_dir}")
    print("[offline] Processing... (press Q in display window to quit)\n")

    detector   = OfflineDetector(seeds)
    csv_path   = os.path.join(args.save_dir, "detections.csv")
    detections = []
    blurry_ct  = 0
    t_start    = time.time()

    with open(csv_path, "w", newline="") as csvf:
        writer = csv.writer(csvf)
        writer.writerow(["frame_file","seed_name","inliers","score","bbox_x","bbox_y","bbox_w","bbox_h"])

        for i, fpath in enumerate(frame_files):
            frame = cv2.imread(fpath)
            if frame is None:
                print(f"[offline] WARNING: Could not read {fpath}")
                continue

            det, blurry = detector.process(frame)

            if blurry:
                blurry_ct += 1
                if args.display:
                    canvas = frame.copy()
                    cv2.putText(canvas, "BLURRY - skipped", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)
                    cv2.imshow("Offline Detector", canvas)
                    if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                        break
                continue

            fname_stem = os.path.splitext(os.path.basename(fpath))[0]
            canvas = draw_detection(frame, det)

            if det is not None:
                x, y, w, h = det["bbox"]
                writer.writerow([os.path.basename(fpath), det["seed_name"],
                                  det["inliers"], f"{det['score']:.4f}",
                                  x, y, w, h])
                csvf.flush()
                detections.append(det)

                # Save annotated frame
                out_path = os.path.join(args.save_dir, f"det_{fname_stem}.jpg")
                cv2.imwrite(out_path, canvas, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                print(f"[offline] DETECTION  frame={i+1}/{len(frame_files)}"
                      f"  seed={det['seed_name']}  inliers={det['inliers']}"
                      f"  score={det['score']:.2f}")

                if args.stop_on_first:
                    print("[offline] --stop-on-first reached.")
                    if args.display: cv2.destroyAllWindows()
                    break

            else:
                # Progress print every 50 frames
                if (i+1) % 50 == 0:
                    elapsed = time.time()-t_start
                    rate    = (i+1)/elapsed
                    print(f"[offline] {i+1}/{len(frame_files)} frames  "
                          f"({rate:.1f} fps)  detections so far: {len(detections)}"
                          f"  blurry: {blurry_ct}")

            if args.display:
                cv2.imshow("Offline Detector", canvas)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    print("[offline] User quit.")
                    break

    if args.display:
        cv2.destroyAllWindows()

    elapsed = time.time()-t_start
    print("\n" + "="*50)
    print("OFFLINE DETECTION SUMMARY")
    print("="*50)
    print(f"  Frames processed : {len(frame_files)}")
    print(f"  Blurry skipped   : {blurry_ct}")
    print(f"  Total detections : {len(detections)}")
    print(f"  Time elapsed     : {elapsed:.1f}s ({len(frame_files)/elapsed:.1f} fps)")
    print(f"  CSV saved to     : {csv_path}")
    if detections:
        seeds_found = set(d["seed_name"] for d in detections)
        print(f"  Seeds detected   : {', '.join(seeds_found)}")
    print("="*50)


if __name__ == "__main__":
    main()
