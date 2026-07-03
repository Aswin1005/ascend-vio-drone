"""Texture-based survey pipeline for IRoC detection.

Replaces ORB feature matching with multi-descriptor histogram matching.
Designed for texture/region features that ORB cannot handle:
  - Layered rock formations
  - Red-oxide patches
  - Satellite imagery printed on flex sheets (with reflections)
  - Cardboard craters

Two-stage matching:
  Stage 1 (fast): HSV colour histogram pre-filter  (~0.3 ms / patch)
  Stage 2 (full): HSV + LBP + gradient fusion       (~2 ms / patch)

Reuses the existing capture, seeds, config, and runner modules.
"""

import cv2
import numpy as np
import os
import time

from .config import TrackingConfig
from .texture_match import (
    TextureDescriptor,
    bhattacharyya_coeff,
    match_score,
)


# ---------------------------------------------------------------------------
# Config extension (inherits base config, adds texture-specific params)
# ---------------------------------------------------------------------------

class TextureConfig(TrackingConfig):
    """Configuration for the texture-based survey pipeline.

    Inherits all base TrackingConfig params (resolution, patch sizes, fps,
    display, save directory, etc.) and adds texture-specific thresholds.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # ---- Colour pre-filter (Stage 1) ----
        # Minimum Bhattacharyya coefficient (HSV) to pass the fast colour check.
        # Lower = more patches proceed to Stage 2 (slower but fewer missed).
        # Random noise scores ~0.27-0.33; real matches score ~0.7+.
        self.color_prefilter = kwargs.get('color_prefilter', 0.55)

        # ---- Full-match confidence threshold (Stage 2) ----
        # Minimum weighted score to accept a detection.
        # True matches score ~0.85+; background noise ~0.50-0.60.
        self.confidence_threshold = kwargs.get('confidence_threshold', 0.75)

        # ---- Descriptor fusion weights (must sum to 1) ----
        self.w_color = kwargs.get('w_color', 0.60)
        self.w_texture = kwargs.get('w_texture', 0.25)
        self.w_gradient = kwargs.get('w_gradient', 0.15)

        # ---- Non-maximum suppression ----
        self.nms_iou_threshold = kwargs.get('nms_iou_threshold', 0.30)

        # ---- Minimum gradient magnitude threshold ----
        self.min_gradient_magnitude = kwargs.get('min_gradient_magnitude', 30.0)


# ---------------------------------------------------------------------------
# Detection result
# ---------------------------------------------------------------------------

class TextureResult:
    """Single detection from texture matching."""

    def __init__(self, seed_name, patch_xy, patch_size, confidence,
                 color_sim, texture_sim, gradient_sim, frame_index, hd_bbox):
        self.seed_name = seed_name
        self.patch_xy = patch_xy          # (x, y) top-left in HD frame
        self.patch_size = patch_size      # which scale (256 or 512)
        self.confidence = confidence      # weighted similarity score
        self.color_sim = color_sim
        self.texture_sim = texture_sim
        self.gradient_sim = gradient_sim
        self.frame_index = frame_index
        self.hd_bbox = hd_bbox            # (x, y, w, h) in HD frame coords

        # Compatibility with SurveyRunner summary print
        self.inliers = int(confidence * 100)
        self.score = confidence


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

class TexturePipeline:
    """Texture-based survey scanner.

    Interface-compatible with SurveyPipeline so SurveyRunner works unchanged.
    """

    def __init__(self, seeds, seed_descriptors, config):
        """
        Args:
            seeds:            list of SeedTemplate (loaded in texture mode).
            seed_descriptors: list of TextureDescriptor (precomputed).
            config:           TextureConfig instance.
        """
        self.seeds = seeds
        self.seed_descriptors = seed_descriptors
        self.config = config

        # State tracking (same interface as SurveyPipeline)
        self.confirmed = None
        self.first_confirmed = None
        self.frame_count = 0
        self.frames_processed = 0
        self.last_tick = time.perf_counter()
        self._fps_times = []

        if config.hd_save_dir:
            os.makedirs(config.hd_save_dir, exist_ok=True)

        # Position logging
        import threading
        self.latest_pos = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
        self.pos_lock = threading.Lock()
        self._pos_sub = None
        
        try:
            import rospy
            from geometry_msgs.msg import PoseStamped
            import math
            
            def pose_callback(msg):
                try:
                    p = msg.pose.position
                    q = msg.pose.orientation
                    # ENU: x=East, y=North — convert yaw from quaternion
                    yaw_rad = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
                    with self.pos_lock:
                        self.latest_pos["x"]       = p.x
                        self.latest_pos["y"]       = p.y
                        self.latest_pos["z"]       = p.z
                        self.latest_pos["yaw_deg"] = math.degrees(yaw_rad)
                except Exception as e:
                    rospy.logwarn(f"[texture_pipeline] pose_callback error: {e}")
                    
            self._pos_sub = rospy.Subscriber(
                "/mavros/local_position/pose", PoseStamped, pose_callback, queue_size=1
            )
            print("[texture_pipeline] Subscribed to /mavros/local_position/pose for logging coordinates.")
        except Exception as e:
            print(f"[texture_pipeline] ROS Pose subscription skipped or failed: {e}")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _compute_fps(self):
        now = time.perf_counter()
        elapsed = now - self.last_tick
        self.last_tick = now
        self._fps_times.append(elapsed)
        if len(self._fps_times) > 8:
            self._fps_times.pop(0)
        avg = sum(self._fps_times) / len(self._fps_times)
        return 1.0 / avg if avg > 0.0 else 0.0

    def _is_blurry(self, gray_frame):
        variance = cv2.Laplacian(gray_frame, cv2.CV_64F).var()
        return variance < self.config.blur_threshold

    def _axis_positions(self, length, patch_size, stride):
        """Sliding-window start positions, including the edge-aligned patch."""
        if patch_size > length:
            return []
        positions = list(range(0, length - patch_size + 1, stride))
        last = length - patch_size
        if not positions or positions[-1] != last:
            positions.append(last)
        return sorted(set(positions))

    def _extract_patches_at_scale(self, frame_bgr, patch_size):
        """Slide a window of lr_size x lr_size across the downsampled (LR) frame.
        Returns list of (x_hd, y_hd, patch_size, lr_bgr).
        """
        h, w = frame_bgr.shape[:2]
        lr = self.config.lr_size
        scale = lr / float(patch_size)

        # Downsample the entire frame to LR once
        w_lr = int(round(w * scale))
        h_lr = int(round(h * scale))
        lr_frame = cv2.resize(frame_bgr, (w_lr, h_lr), interpolation=cv2.INTER_AREA)

        overlap = self.config.patch_overlap
        stride_lr = max(int(lr * (1.0 - overlap)), 1)

        patches = []
        for y_lr in self._axis_positions(h_lr, lr, stride_lr):
            for x_lr in self._axis_positions(w_lr, lr, stride_lr):
                crop_lr = lr_frame[y_lr: y_lr + lr, x_lr: x_lr + lr]

                # If for some rounding reason the crop is not exactly lr x lr, resize it just in case
                if crop_lr.shape[0] != lr or crop_lr.shape[1] != lr:
                    crop_lr = cv2.resize(crop_lr, (lr, lr), interpolation=cv2.INTER_AREA)

                # Map back to HD coordinates
                x_hd = int(round(x_lr / scale))
                y_hd = int(round(y_lr / scale))

                # Ensure we don't exceed HD dimensions
                x_hd = min(x_hd, w - patch_size)
                y_hd = min(y_hd, h - patch_size)

                patches.append((x_hd, y_hd, patch_size, crop_lr))
        return patches


    @staticmethod
    def _iou(box1, box2):
        """Intersection over Union for (x, y, w, h) boxes."""
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        inter = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        union = w1 * h1 + w2 * h2 - inter
        return inter / max(union, 1)

    def _nms(self, candidates, iou_threshold):
        """Non-maximum suppression.  Keeps highest-confidence non-overlapping."""
        if not candidates:
            return []
        candidates.sort(key=lambda c: c['confidence'], reverse=True)
        accepted = []
        for c in candidates:
            if any(self._iou(c['hd_bbox'], a['hd_bbox']) > iou_threshold
                   for a in accepted):
                continue
            accepted.append(c)
        return accepted

    def _save_hd_frame(self, hd_frame_original, detections, frame_index):
        """Save original 1280x720 HD frame -- raw and annotated."""
        if not self.config.hd_save_dir:
            return

        stem = f"tex_frame{frame_index:06d}"
        raw_name = f"{stem}.jpg"
        raw_path = os.path.join(self.config.hd_save_dir, raw_name)
        boxed_path = os.path.join(self.config.hd_save_dir, f"{stem}_boxed.jpg")

        # Raw HD (no annotations)
        cv2.imwrite(raw_path, hd_frame_original, [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Annotated copy with all detection bboxes
        boxed = hd_frame_original.copy()
        for det in detections:
            x, y, bw, bh = det.hd_bbox
            cv2.rectangle(boxed, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            label = (f"{det.seed_name}  conf:{det.confidence:.2f}  "
                     f"[c:{det.color_sim:.2f} t:{det.texture_sim:.2f} g:{det.gradient_sim:.2f}]")
            cv2.putText(boxed, label, (x, max(24, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imwrite(boxed_path, boxed, [cv2.IMWRITE_JPEG_QUALITY, 95])

        print(f"[texture] HD frame saved -> {raw_path}  "
              f"({hd_frame_original.shape[1]}x{hd_frame_original.shape[0]})")

        # Save coordinates to positions.csv
        csv_path = os.path.join(self.config.hd_save_dir, "positions.csv")
        file_exists = os.path.exists(csv_path)
        with self.pos_lock:
            pos = dict(self.latest_pos)

        try:
            import csv
            with open(csv_path, "a", newline="") as csvf:
                writer = csv.writer(csvf)
                if not file_exists:
                    writer.writerow([
                        "frame_file", "timestamp", "pos_x_east_m",
                        "pos_y_north_m", "pos_z_up_m", "heading_deg",
                        "seed_name", "confidence", "patch_x", "patch_y", "patch_size"
                    ])
                best = detections[0]
                writer.writerow([
                    raw_name,
                    f"{time.time():.3f}",
                    f"{pos['x']:.4f}",
                    f"{pos['y']:.4f}",
                    f"{pos['z']:.4f}",
                    f"{pos['yaw_deg']:.2f}",
                    best.seed_name,
                    f"{best.confidence:.3f}",
                    best.patch_xy[0],
                    best.patch_xy[1],
                    best.patch_size
                ])
                csvf.flush()
            print(f"[texture] CSV log written -> {csv_path} with coordinates x={pos['x']:.2f}, y={pos['y']:.2f}")
        except Exception as e:
            print(f"[texture] failed to write CSV log: {e}")

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------

    def process(self, hd_frame, hd_frame_original=None):
        """Process one frame.

        Args:
            hd_frame:          Working-resolution frame (patch extraction).
            hd_frame_original: Original 1280x720 HD frame saved on detection.
                               If None, hd_frame is used.

        Returns dict compatible with SurveyRunner:
            latest_detection  -- TextureResult or None
            all_detections    -- list[TextureResult]
            detected          -- bool
            blurry            -- bool
            fps               -- float
            frame_index       -- int
            patches_total     -- int  (total patches extracted)
            patches_passed    -- int  (patches that passed colour pre-filter)
        """
        self.frame_count += 1
        fps = self._compute_fps()

        result = {
            'latest_detection': None,
            'all_detections': [],
            'detected': False,
            'blurry': False,
            'fps': fps,
            'frame_index': self.frame_count,
            'patches_total': 0,
            'patches_passed': 0,
        }

        # Blur check
        gray_full = cv2.cvtColor(hd_frame, cv2.COLOR_BGR2GRAY)
        if self._is_blurry(gray_full):
            result['blurry'] = True
            return result

        frame_for_save = hd_frame_original if hd_frame_original is not None else hd_frame
        self.frames_processed += 1

        # ------------------------------------------------------------------
        # Step 1 -- Extract all patches at all scales
        # ------------------------------------------------------------------
        all_patches = []
        for ps in self.config.patch_sizes:
            all_patches.extend(self._extract_patches_at_scale(hd_frame, ps))
        result['patches_total'] = len(all_patches)

        if not all_patches:
            return result

        # Matching weights and thresholds
        weights = (self.config.w_color, self.config.w_texture, self.config.w_gradient)
        color_pf = self.config.color_prefilter
        conf_thr = self.config.confidence_threshold

        candidates = []

        for px, py, ps, lr_bgr in all_patches:
            # --------------------------------------------------------------
            # Stage 1: Fast colour pre-filter (HSV histogram only)
            # --------------------------------------------------------------
            hsv = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2HSV)
            patch_hsv = TextureDescriptor.compute_hsv_histogram(hsv)

            best_color_sim = 0.0
            for sd in self.seed_descriptors:
                csim = bhattacharyya_coeff(patch_hsv, sd.hsv_hist)
                if csim > best_color_sim:
                    best_color_sim = csim

            if best_color_sim < color_pf:
                continue  # patch colour is too different from all seeds

            result['patches_passed'] += 1

            # --------------------------------------------------------------
            # Stage 2: Full descriptor match (reuse already-computed HSV hist)
            # --------------------------------------------------------------
            gray = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
            gray_eq = clahe.apply(gray)

            # Check for flatness (mean gradient magnitude)
            gx = cv2.Sobel(gray_eq, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(gray_eq, cv2.CV_32F, 0, 1, ksize=3)
            magnitude = cv2.magnitude(gx, gy)
            if magnitude.mean() < self.config.min_gradient_magnitude:
                continue

            orientation = cv2.phase(gx, gy, angleInDegrees=True)
            lbp_hist = TextureDescriptor.compute_lbp_histogram(gray_eq)
            grad_hist = TextureDescriptor.compute_gradient_histogram(
                gray_eq, magnitude=magnitude, orientation=orientation
            )
            patch_desc = TextureDescriptor(patch_hsv, lbp_hist, grad_hist)

            # Compare against every seed
            for i, sd in enumerate(self.seed_descriptors):
                total, csim, tsim, gsim = match_score(patch_desc, sd, weights)

                if total >= conf_thr:
                    candidates.append({
                        'seed_name': self.seeds[i].name,
                        'patch_xy': (px, py),
                        'patch_size': ps,
                        'confidence': total,
                        'color_sim': csim,
                        'texture_sim': tsim,
                        'gradient_sim': gsim,
                        'hd_bbox': (px, py, ps, ps),
                    })

        # ------------------------------------------------------------------
        # Step 2 -- NMS
        # ------------------------------------------------------------------
        accepted = self._nms(candidates, self.config.nms_iou_threshold)

        # ------------------------------------------------------------------
        # Step 3 -- Record detections & save HD frame
        # ------------------------------------------------------------------
        detections = []
        for c in accepted:
            det = TextureResult(
                seed_name=c['seed_name'],
                patch_xy=c['patch_xy'],
                patch_size=c['patch_size'],
                confidence=c['confidence'],
                color_sim=c['color_sim'],
                texture_sim=c['texture_sim'],
                gradient_sim=c['gradient_sim'],
                frame_index=self.frame_count,
                hd_bbox=c['hd_bbox'],
            )
            detections.append(det)

        result['all_detections'] = detections

        if detections:
            best = detections[0]  # highest confidence (NMS preserves order)
            result['latest_detection'] = best
            result['detected'] = True
            self.confirmed = best

            is_first = self.first_confirmed is None
            if is_first:
                self.first_confirmed = best

            print(
                f"[texture] {'*** FIRST DETECTION ***' if is_first else 'DETECTED'}  "
                f"seed={best.seed_name}  conf={best.confidence:.3f}  "
                f"[c:{best.color_sim:.2f} t:{best.texture_sim:.2f} g:{best.gradient_sim:.2f}]  "
                f"patch=({best.patch_xy[0]},{best.patch_xy[1]})  scale={best.patch_size}px  "
                f"frame={self.frame_count}  "
                f"({len(detections)} det after NMS, "
                f"{result['patches_passed']}/{result['patches_total']} passed color)"
            )

            self._save_hd_frame(frame_for_save, detections, self.frame_count)

        return result

    # ------------------------------------------------------------------
    # Visualisation (--display mode, laptop only)
    # ------------------------------------------------------------------

    def draw_overlay(self, hd_frame, result):
        """Draw bounding boxes and status on a copy of the frame."""
        canvas = hd_frame.copy()
        h, w = canvas.shape[:2]

        all_dets = result.get('all_detections', [])
        latest = result.get('latest_detection')

        # Draw all current detections
        for det in all_dets:
            x, y, bw, bh = det.hd_bbox
            is_best = (det is latest)
            color = (0, 255, 0) if is_best else (0, 200, 200)
            thickness = 3 if is_best else 2
            cv2.rectangle(canvas, (x, y), (x + bw, y + bh), color, thickness)
            label = f"{det.seed_name} {det.confidence:.2f}"
            cv2.putText(canvas, label, (x, max(22, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # Show last-seen box if no current detection
        if not all_dets and self.confirmed is not None:
            det = self.confirmed
            x, y, bw, bh = det.hd_bbox
            cv2.rectangle(canvas, (x, y), (x + bw, y + bh), (255, 130, 0), 2)
            cv2.putText(canvas, "last seen", (x, max(20, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 130, 0), 1)

        # Status line
        patches_total = result.get('patches_total', 0)
        patches_passed = result.get('patches_passed', 0)
        n_dets = len(all_dets)
        status = (
            f"FPS:{result['fps']:.1f}  "
            f"frame:{result['frame_index']}  "
            f"patches:{patches_passed}/{patches_total}  "
            f"dets:{n_dets}  "
            f"{'CONFIRMED' if self.confirmed else 'searching...'}"
        )
        if result.get('blurry'):
            status += "  | BLURRY"

        cv2.putText(canvas, status, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if self.confirmed:
            cv2.putText(canvas, "TARGET CONFIRMED",
                        (w // 2 - 180, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3)

        return canvas
