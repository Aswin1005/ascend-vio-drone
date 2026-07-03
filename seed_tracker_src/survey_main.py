"""CLI entry point for IRoC survey mode.

Usage:
    # Standard run on Jetson alongside OpenVINS (1 fps, ROS color topic)
    python3 -m src.survey_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw --fps 1

    # Faster processing for testing
    python3 -m src.survey_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw --fps 3

    # Video file (offline testing)
    python3 -m src.survey_main \\
        --seeds ./seeds/color_mars --video ./test.mp4 --display

    # Show processed 128x128 seed images and exit
    python3 -m src.survey_main --seeds ./seeds/color_mars --show-seeds

    # Looser thresholds for challenging scene
    python3 -m src.survey_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw \\
        --min-match-count 8 --min-inliers 6
"""

import argparse
import cv2
import os
import sys

from .capture import OpenCVCapture, RealSenseCapture, ROSCapture
from .config import TrackingConfig
from .seeds import load_seeds, show_seeds, save_lr_seeds
from .survey import SurveyPipeline
from .survey_runner import SurveyRunner


def main():
    parser = argparse.ArgumentParser(
        description="IRoC Survey Scanner — seed ORB matching, multi-scale patches, HD frame save"
    )

    # ------------------------------------------------------------------ Input
    input_group = parser.add_mutually_exclusive_group()
    input_group.add_argument("--camera", type=int, default=None,
                             help="camera index (e.g. 0 for laptop webcam)")
    input_group.add_argument("--video", default=None,
                             help="path to video file")
    input_group.add_argument("--camera-gst", default=None,
                             help="GStreamer pipeline string (Jetson CSI)")
    input_group.add_argument("--realsense", action="store_true",
                             help="use Intel RealSense D435i camera natively")
    input_group.add_argument("--ros-topic", default=None,
                             help="ROS image topic (e.g. /camera/color/image_raw). "
                                  "Use this when the realsense2_camera ROS node is running.")

    parser.add_argument("--realsense-fps", type=int, default=6,
                        choices=[6, 15, 30],
                        help="RealSense stream rate (default: 6)")

    # ------------------------------------------------------------------ Seeds
    parser.add_argument("--seeds", required=True,
                        help="directory with seed images")
    parser.add_argument("--seed-center-crop", type=float, default=1.0,
                        help="center crop fraction of each seed (default: 1.0). "
                             "Try 0.7-0.9 if seeds have large backgrounds.")

    # --------------------------------------------------------------- Resolution
    parser.add_argument("--width", type=int, default=1280,
                        help="working frame width (default: 1280)")
    parser.add_argument("--height", type=int, default=720,
                        help="working frame height (default: 720)")

    # --------------------------------------------------------------- Patching
    parser.add_argument(
        "--patch-sizes", type=int, nargs="+", default=[256, 512],
        metavar="PX",
        help=(
            "HD crop sizes downsampled to 128×128 for ORB matching "
            "(default: 256 512). 256px finds closer/larger objects; "
            "512px finds farther/smaller objects. Use one size to save compute."
        ),
    )
    parser.add_argument("--patch-overlap", type=float, default=0.5,
                        help="sliding window overlap (0.0–0.9, default: 0.5). "
                             "Lower = fewer patches = faster.")
    parser.add_argument("--lr-size", type=int, default=128,
                        help="LR patch size for ORB matching (default: 128, rulebook requirement)")

    # ---------------------------------------------------------------- Matching
    parser.add_argument("--max-features", type=int, default=1000,
                        help="ORB max features per 128×128 patch (default: 1000)")
    parser.add_argument("--fast-threshold", type=int, default=8,
                        help="ORB FAST threshold (default: 8; lower = more weak features)")
    parser.add_argument("--match-ratio", type=float, default=0.85,
                        help="Lowe ratio test threshold (default: 0.85). "
                             "Lower = stricter. Higher = more matches but more false positives.")
    parser.add_argument("--min-match-count", type=int, default=10,
                        help="min good matches before RANSAC (default: 10)")
    parser.add_argument("--min-inliers", type=int, default=8,
                        help="min RANSAC inliers to accept a detection (default: 8)")
    parser.add_argument("--min-inlier-ratio", type=float, default=0.30,
                        help="min inliers/matches ratio (default: 0.30)")
    parser.add_argument("--projection-margin", type=float, default=0.18,
                        help="allowed projected bbox spill outside 128×128 patch (default: 0.18)")
    parser.add_argument("--bbox-expansion", type=float, default=1.0,
                        help="expand reported bbox by this factor (default: 1.0 = no expand)")

    # --------------------------------------------------------------- Frame I/O
    parser.add_argument("--fps", type=float, default=1.0,
                        help="target processing frame rate in Hz (default: 1.0). "
                             "Keep low (1-3) alongside OpenVINS on Jetson Nano.")
    parser.add_argument("--blur-threshold", type=float, default=30.0,
                        help="Laplacian variance below this = blurry, skip frame (default: 30.0)")

    # ------------------------------------------------------------------ Output
    parser.add_argument("--save-dir", default="./detections",
                        help="directory to save 1280×720 HD frames on detection (default: ./detections)")
    parser.add_argument("--display", action="store_true",
                        help="show live annotated display window (laptop only)")
    parser.add_argument("--stop-on-any-match", action="store_true",
                        help="stop immediately when the first detection fires")

    # -------------------------------------------------------- Debugging
    parser.add_argument("--show-seeds", action="store_true",
                        help="display preprocessed 128×128 seed images and exit")

    args = parser.parse_args()

    # ---------------------------------------------------------------- Config
    config = TrackingConfig(
        input_width=args.width,
        input_height=args.height,
        max_features=args.max_features,
        match_ratio=args.match_ratio,
        min_match_count=args.min_match_count,
        min_inliers=args.min_inliers,
        min_inlier_ratio=args.min_inlier_ratio,
        fast_threshold=args.fast_threshold,
        lr_size=args.lr_size,
        patch_sizes=args.patch_sizes,
        patch_overlap=args.patch_overlap,
        blur_threshold=args.blur_threshold,
        target_fps=args.fps,
        hd_save_dir=args.save_dir,
        display=args.display,
        stop_on_any_match=args.stop_on_any_match,
        drop_stale_frames=args.video is None,
        projection_margin=args.projection_margin,
        bbox_expansion=args.bbox_expansion,
    )

    # --------------------------------------------------------------- Load seeds
    print(f"[survey] loading seeds from: {args.seeds}")
    print(f"[survey] LR size: {config.lr_size}×{config.lr_size}  (rulebook requirement)")
    print(f"[survey] patch scales: {config.patch_sizes}px → each resized to {config.lr_size}×{config.lr_size} for ORB")
    print(f"[survey] seed center crop: {args.seed_center_crop}")
    print(f"[survey] target fps: {config.target_fps:.1f}")

    seed_orb = cv2.ORB_create(
        nfeatures=config.max_features,
        fastThreshold=config.fast_threshold,
        scoreType=cv2.ORB_HARRIS_SCORE,
    )
    seeds = load_seeds(
        args.seeds,
        seed_orb,
        lr_size=config.lr_size,
        seed_center_crop=args.seed_center_crop,
    )

    if not seeds:
        print("[survey] ERROR: no valid seeds loaded. check your seed directory.")
        sys.exit(1)

    print(f"[survey] loaded {len(seeds)} seed(s):")
    for s in seeds:
        print(f"  - {s.name}: {len(s.keypoints)} keypoints at {s.width}×{s.height}")

    # Auto-save generated 128×128 seed images for validation
    lr_seeds_dir = os.path.join(args.save_dir, "lr_seeds")
    save_lr_seeds(seeds, lr_seeds_dir)

    if args.show_seeds:
        print("[survey] showing 128×128 seed images (press any key to continue)...")
        show_seeds(seeds, lr=True, wait=True)
        if not any([args.camera is not None, args.video, args.camera_gst, args.realsense, args.ros_topic]):
            print("[survey] no input source specified — exiting after seed preview.")
            sys.exit(0)

    # -------------------------------------------------------- Validate input
    if not any([args.camera is not None, args.video, args.camera_gst, args.realsense, args.ros_topic]):
        print("[survey] ERROR: specify --camera, --video, --camera-gst, --realsense, or --ros-topic")
        sys.exit(1)

    # ------------------------------------------------------------- Open capture
    print("[survey] opening capture: ", end="")
    if args.video:
        print(f"video={args.video}")
        capture = OpenCVCapture(video_path=args.video)
    elif args.realsense:
        print(f"realsense {args.width}×{args.height} @ {args.realsense_fps}fps")
        capture = RealSenseCapture(width=args.width, height=args.height, fps=args.realsense_fps)
    elif args.ros_topic:
        print(f"ros-topic={args.ros_topic}")
        capture = ROSCapture(topic=args.ros_topic)
    elif args.camera is not None:
        print(f"camera={args.camera}")
        capture = OpenCVCapture(camera_index=args.camera,
                                width=config.input_width, height=config.input_height)
    else:
        print(f"gst={args.camera_gst}")
        capture = OpenCVCapture(camera_gst=args.camera_gst,
                                width=config.input_width, height=config.input_height)

    # ------------------------------------------ Build pipeline + runner and go
    pipeline = SurveyPipeline(seeds, config)
    runner = SurveyRunner(capture, pipeline, config)
    runner.run()


if __name__ == "__main__":
    main()
