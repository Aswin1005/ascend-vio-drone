import cv2
import numpy as np
import os
import time

from .config import TrackingConfig
from .seeds import SeedTemplate


class SurveyResult:
    """Single detection event."""
    def __init__(self, seed_name, patch_xy, patch_size, inliers, score, frame_index,
                 hd_quad=None, hd_bbox=None):
        self.seed_name = seed_name        # name of the best-matching seed
        self.patch_xy = patch_xy          # (x, y) top-left of patch in HD frame
        self.patch_size = patch_size      # which scale caught this (256 or 512)
        self.inliers = inliers            # RANSAC inliers
        self.score = score                # inliers / good_matches
        self.frame_index = frame_index
        self.hd_quad = hd_quad            # np.ndarray (4, 2) corners in HD pixels, or None
        self.hd_bbox = hd_bbox            # (x, y, w, h) axis-aligned bbox in HD, or None


class SurveyPipeline:
    """Survey scanner — multi-seed ORB matching, multi-scale patches, HD bounding boxes.

    For each incoming frame:
      1. Extract patches at multiple scales from the HD frame.
      2. Resize each patch to 128×128 and run ORB once per patch (not per seed).
      3. Match the ORB descriptors against every loaded seed.
      4. Pick the best candidate by inlier count.
      5. Save the original 1280×720 HD frame whenever a detection fires.
    """

    def __init__(self, seeds, config: TrackingConfig):
        self.seeds = seeds
        self.config = config

        self.orb = cv2.ORB_create(
            nfeatures=config.max_features,
            fastThreshold=config.fast_threshold,
            scoreType=cv2.ORB_HARRIS_SCORE,
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Last confirmed detection (updated every frame a match is found).
        self.confirmed = None
        # First detection (frozen — used for logging).
        self.first_confirmed = None

        self.frame_count = 0
        self.frames_processed = 0
        self.last_tick = time.perf_counter()
        self._fps_times = []

        if config.hd_save_dir:
            os.makedirs(config.hd_save_dir, exist_ok=True)

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
        """Return sliding-window start positions, including the edge-aligned patch."""
        if patch_size > length:
            return []
        positions = list(range(0, length - patch_size + 1, stride))
        last = length - patch_size
        if not positions or positions[-1] != last:
            positions.append(last)
        return sorted(set(positions))

    def _extract_patches_at_scale(self, frame_bgr, patch_size):
        """Slide a window of patch_size×patch_size across the HD frame.
        Each crop is resized to lr_size×lr_size (128×128) for ORB matching.
        Returns list of (x, y, lr_bgr).
        """
        h, w = frame_bgr.shape[:2]
        overlap = self.config.patch_overlap
        stride = max(int(patch_size * (1.0 - overlap)), 1)
        lr = self.config.lr_size
        patches = []
        for y in self._axis_positions(h, patch_size, stride):
            for x in self._axis_positions(w, patch_size, stride):
                crop = frame_bgr[y: y + patch_size, x: x + patch_size]
                lr_bgr = cv2.resize(crop, (lr, lr), interpolation=cv2.INTER_AREA)
                patches.append((x, y, lr_bgr))
        return patches

    def _match_seed_to_features(self, seed, kps, descs):
        """Match one seed against pre-computed 128×128 patch ORB features.

        Returns (inliers, score, homography) if match passes thresholds, else None.
        """
        if descs is None or len(kps) < 4:
            return None

        try:
            knn_matches = self.matcher.knnMatch(seed.descriptors, descs, k=2)
        except cv2.error:
            return None

        good = [m for pair in knn_matches
                if len(pair) == 2
                for m, n in [pair]
                if m.distance < self.config.match_ratio * n.distance]

        if len(good) < self.config.min_match_count:
            return None

        src_pts = np.float32([seed.keypoints[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kps[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 4.0)
        if homography is None or mask is None:
            return None

        inliers = int(mask.ravel().sum())
        if inliers < self.config.min_inliers:
            return None

        score = inliers / max(len(good), 1)
        if score < self.config.min_inlier_ratio:
            return None

        projected = self._project_seed_lr(seed, homography)
        if not self._valid_lr_projection(projected):
            return None

        return (inliers, score, homography)

    def _project_seed_lr(self, seed, homography):
        """Project seed corners into 128×128 LR patch coordinates."""
        seed_corners = np.array(
            [[0, 0], [seed.width - 1, 0],
             [seed.width - 1, seed.height - 1], [0, seed.height - 1]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        try:
            return cv2.perspectiveTransform(seed_corners, homography).reshape(-1, 2)
        except Exception:
            return None

    def _valid_lr_projection(self, projected):
        """Reject homographies that warp the seed completely outside the patch."""
        if projected is None or projected.shape != (4, 2):
            return False
        if not np.isfinite(projected).all():
            return False

        contour = projected.astype(np.float32).reshape(-1, 1, 2)
        if not cv2.isContourConvex(contour):
            return False

        lr = float(self.config.lr_size)
        margin = lr * self.config.projection_margin
        x1, y1 = projected[:, 0].min(), projected[:, 1].min()
        x2, y2 = projected[:, 0].max(), projected[:, 1].max()
        bw, bh = x2 - x1, y2 - y1

        if x1 < -margin or y1 < -margin or x2 > lr + margin or y2 > lr + margin:
            return False
        if bw < 6.0 or bh < 6.0:
            return False

        area_ratio = abs(cv2.contourArea(contour)) / max(lr * lr, 1.0)
        if area_ratio < self.config.min_projected_area_ratio:
            return False
        if area_ratio > self.config.max_projected_area_ratio:
            return False

        aspect = bw / max(bh, 1.0)
        if aspect < self.config.min_projected_aspect or aspect > self.config.max_projected_aspect:
            return False

        return True

    def _compute_hd_quad(self, homography, seed, px, py, patch_size):
        """Project seed corners through homography into original HD frame coordinates."""
        scale = patch_size / self.config.lr_size  # e.g. 256 / 128 = 2.0
        seed_corners = np.array(
            [[0, 0], [seed.width - 1, 0],
             [seed.width - 1, seed.height - 1], [0, seed.height - 1]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        try:
            proj_lr = cv2.perspectiveTransform(seed_corners, homography).reshape(-1, 2)
            proj_hd = proj_lr * scale
            proj_hd[:, 0] += px
            proj_hd[:, 1] += py
            return proj_hd  # shape (4, 2)
        except Exception:
            return None

    def _quad_to_bbox(self, quad, frame_shape):
        """Axis-aligned bounding rect of a quadrilateral, clamped to the frame."""
        if quad is None:
            return None
        fh, fw = frame_shape[:2]
        x1 = max(0, int(quad[:, 0].min()))
        y1 = max(0, int(quad[:, 1].min()))
        x2 = min(fw - 1, int(quad[:, 0].max()))
        y2 = min(fh - 1, int(quad[:, 1].max()))
        w, h = x2 - x1, y2 - y1
        if w < 8 or h < 8:
            return None

        expansion = max(1.0, float(self.config.bbox_expansion))
        if expansion > 1.0:
            cx, cy = x1 + w / 2.0, y1 + h / 2.0
            x1 = max(0, int(round(cx - w * expansion / 2.0)))
            y1 = max(0, int(round(cy - h * expansion / 2.0)))
            x2 = min(fw - 1, int(round(cx + w * expansion / 2.0)))
            y2 = min(fh - 1, int(round(cy + h * expansion / 2.0)))
            w, h = x2 - x1, y2 - y1

        return (x1, y1, w, h)

    def _save_hd_frame(self, hd_frame_original, result):
        """Save the original 1280×720 HD frame with and without the bbox drawn."""
        if not self.config.hd_save_dir:
            return
        stem = f"target_frame{result.frame_index:06d}"
        raw_path = os.path.join(self.config.hd_save_dir, f"{stem}.jpg")
        boxed_path = os.path.join(self.config.hd_save_dir, f"{stem}_boxed.jpg")

        # Raw HD frame (no annotations)
        cv2.imwrite(raw_path, hd_frame_original, [cv2.IMWRITE_JPEG_QUALITY, 95])

        # Annotated copy
        boxed = hd_frame_original.copy()
        if result.hd_quad is not None:
            pts = result.hd_quad.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(boxed, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        if result.hd_bbox is not None:
            x, y, w, h = result.hd_bbox
            cv2.rectangle(boxed, (x, y), (x + w, y + h), (0, 220, 80), 2)
            label = f"{result.seed_name}  inl:{result.inliers}  score:{result.score:.2f}"
            cv2.putText(boxed, label, (x, max(24, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imwrite(boxed_path, boxed, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"[survey] HD frame saved → {raw_path}  ({hd_frame_original.shape[1]}×{hd_frame_original.shape[0]})")
        print(f"[survey] boxed frame  → {boxed_path}")

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------

    def process(self, hd_frame, hd_frame_original=None):
        """Process one frame.

        Args:
            hd_frame:          Working resolution frame (may be resized from HD).
            hd_frame_original: The original 1280×720 frame to save on detection.
                               If None, hd_frame is used for saving (fallback).

        Returns a dict with keys:
            latest_detection — SurveyResult or None
            detected        — bool: a match was found this frame
            blurry          — bool: frame was skipped for blur
            fps             — float
            frame_index     — int
        """
        self.frame_count += 1
        fps = self._compute_fps()

        result = {
            'latest_detection': None,
            'detected': False,
            'blurry': False,
            'fps': fps,
            'frame_index': self.frame_count,
        }

        # Blur check on working-resolution grayscale (cheap)
        gray_full = cv2.cvtColor(hd_frame, cv2.COLOR_BGR2GRAY)
        if self._is_blurry(gray_full):
            result['blurry'] = True
            return result

        result_frame_for_save = hd_frame_original if hd_frame_original is not None else hd_frame

        self.frames_processed += 1

        # ----------------------------------------------------------------
        # Step 1 — Extract patches and compute ORB once per patch.
        #   ORB always runs on 128×128 (lr_size). The patch_size controls
        #   how large a region of the HD frame is cropped before downsampling.
        # ----------------------------------------------------------------
        patch_features = {}
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        for ps in self.config.patch_sizes:
            for px, py, lr_bgr in self._extract_patches_at_scale(hd_frame, ps):
                lr_gray = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2GRAY)
                lr_gray_clahe = clahe.apply(lr_gray)
                kps, descs = self.orb.detectAndCompute(lr_gray_clahe, None)
                if descs is not None and len(kps) >= 4:
                    patch_features[(px, py, ps)] = (kps, descs)

        # ----------------------------------------------------------------
        # Step 2 — Match every seed against every patch. Keep best by inliers.
        # ----------------------------------------------------------------
        best_candidate = None

        for seed in self.seeds:
            for (px, py, ps), (kps, descs) in patch_features.items():
                match = self._match_seed_to_features(seed, kps, descs)
                if match is None:
                    continue
                inliers, score, homography = match

                if best_candidate is not None and inliers <= best_candidate['inliers']:
                    continue  # skip if not better

                hd_quad = self._compute_hd_quad(homography, seed, px, py, ps)
                hd_bbox = self._quad_to_bbox(hd_quad, hd_frame.shape)
                if hd_bbox is None:
                    continue

                best_candidate = {
                    'seed': seed,
                    'patch_xy': (px, py),
                    'patch_size': ps,
                    'inliers': inliers,
                    'score': score,
                    'hd_quad': hd_quad,
                    'hd_bbox': hd_bbox,
                }

        # ----------------------------------------------------------------
        # Step 3 — Record and save.
        # ----------------------------------------------------------------
        if best_candidate is not None:
            px, py = best_candidate['patch_xy']
            ps = best_candidate['patch_size']
            det = SurveyResult(
                seed_name=best_candidate['seed'].name,
                patch_xy=(px, py),
                patch_size=ps,
                inliers=best_candidate['inliers'],
                score=best_candidate['score'],
                frame_index=self.frame_count,
                hd_quad=best_candidate['hd_quad'],
                hd_bbox=best_candidate['hd_bbox'],
            )

            result['latest_detection'] = det
            result['detected'] = True
            self.confirmed = det

            is_first = self.first_confirmed is None
            if is_first:
                self.first_confirmed = det

            print(
                f"[survey] {'*** FIRST DETECTION ***' if is_first else 'DETECTED'}  "
                f"seed={det.seed_name}  inliers={det.inliers}  score={det.score:.2f}  "
                f"patch=({px},{py})  scale={ps}px  frame={self.frame_count}"
            )

            # Save the original 1280×720 HD frame
            self._save_hd_frame(result_frame_for_save, det)

        return result

    # ------------------------------------------------------------------
    # Visualisation (for --display mode on laptop only)
    # ------------------------------------------------------------------

    def draw_overlay(self, hd_frame, result):
        """Draw bounding box and status text on a copy of the frame."""
        canvas = hd_frame.copy()
        h, w = canvas.shape[:2]

        latest = result.get('latest_detection')

        if latest is not None:
            if latest.hd_quad is not None:
                pts = latest.hd_quad.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(canvas, [pts], isClosed=True, color=(0, 255, 0), thickness=3)

            if latest.hd_bbox is not None:
                x, y, bw, bh = latest.hd_bbox
                cv2.rectangle(canvas, (x, y), (x + bw, y + bh), (0, 220, 80), 2)
                label = (
                    f"{latest.seed_name.upper()}  "
                    f"inl:{latest.inliers}  scr:{latest.score:.2f}  "
                    f"scale:{latest.patch_size}px"
                )
                cv2.putText(canvas, label, (x, max(22, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            else:
                px, py = latest.patch_xy
                ps = latest.patch_size
                cv2.rectangle(canvas, (px, py), (px + ps, py + ps), (0, 255, 0), 3)

        elif self.confirmed is not None:
            det = self.confirmed
            if det.hd_bbox is not None:
                x, y, bw, bh = det.hd_bbox
                cv2.rectangle(canvas, (x, y), (x + bw, y + bh), (255, 130, 0), 2)
                cv2.putText(canvas, "last seen", (x, max(20, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 130, 0), 1)

        status = (
            f"FPS:{result['fps']:.1f}  "
            f"frame:{result['frame_index']}  "
            f"{'CONFIRMED' if self.confirmed else 'searching...'}"
        )
        if result.get('blurry'):
            status += "  | BLURRY"

        cv2.putText(canvas, status, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        if self.confirmed:
            cv2.putText(canvas, "TARGET CONFIRMED",
                        (w // 2 - 180, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3)

        return canvas
