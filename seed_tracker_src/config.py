class TrackingConfig:
    def __init__(self, **kwargs):
        # --- Resolution ---
        self.input_width = kwargs.get('input_width', 1280)
        self.input_height = kwargs.get('input_height', 720)

        # --- ORB feature extraction ---
        self.max_features = kwargs.get('max_features', 1000)
        self.fast_threshold = kwargs.get('fast_threshold', 8)

        # --- Matching thresholds ---
        self.match_ratio = kwargs.get('match_ratio', 0.85)
        self.min_match_count = kwargs.get('min_match_count', 10)
        self.min_inliers = kwargs.get('min_inliers', 8)
        self.min_inlier_ratio = kwargs.get('min_inlier_ratio', 0.30)

        # --- LR patch config ---
        # lr_size: all ORB matching happens at this resolution (must be 128 per rulebook)
        self.lr_size = kwargs.get('lr_size', 128)

        # patch_sizes: HD frame crop sizes that get downsampled to lr_size×lr_size for ORB.
        # 256 → good for objects that fill ~1/5 of the frame width.
        # 512 → good for objects that fill ~1/3 of the frame width.
        self.patch_sizes = kwargs.get('patch_sizes', [256, 512])
        self.patch_overlap = kwargs.get('patch_overlap', 0.5)

        # --- Frame pipeline ---
        # Blur rejection. Lower = keep more frames. 30.0 is safe for drone-in-motion.
        self.blur_threshold = kwargs.get('blur_threshold', 30.0)

        # Target processing rate (Hz). The capture thread sleeps between reads.
        # Keep low (1-3) to leave CPU headroom for OpenVINS on Jetson Nano.
        self.target_fps = kwargs.get('target_fps', 1.0)

        # --- Output ---
        # Directory to save HD frames on detection (1280×720).
        self.hd_save_dir = kwargs.get('hd_save_dir', './detections')
        self.stop_on_any_match = kwargs.get('stop_on_any_match', False)
        self.display = kwargs.get('display', False)
        # For ROS live mode: always drop stale frames; for video files: keep all.
        self.drop_stale_frames = kwargs.get('drop_stale_frames', True)

        # --- Homography sanity checks (in 128×128 LR patch coordinates) ---
        self.projection_margin = kwargs.get('projection_margin', 0.18)
        self.min_projected_area_ratio = kwargs.get('min_projected_area_ratio', 0.02)
        self.max_projected_area_ratio = kwargs.get('max_projected_area_ratio', 1.20)
        self.min_projected_aspect = kwargs.get('min_projected_aspect', 0.12)
        self.max_projected_aspect = kwargs.get('max_projected_aspect', 8.0)
        self.vote_iou_threshold = kwargs.get('vote_iou_threshold', 0.20)
        self.bbox_expansion = kwargs.get('bbox_expansion', 1.0)
