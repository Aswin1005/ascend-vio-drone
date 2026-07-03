"""CLI entry point for texture-based IRoC survey mode.

Usage:
    # Standard run on Jetson alongside OpenVINS (1 fps, ROS colour topic)
    python3 -m src.texture_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw --fps 1

    # Faster processing for testing
    python3 -m src.texture_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw --fps 3

    # Video file (offline testing)
    python3 -m src.texture_main \\
        --seeds ./seeds/color_mars --video ./test.mp4 --display

    # Show preprocessed 128x128 seed images and exit
    python3 -m src.texture_main --seeds ./seeds/color_mars --show-seeds

    # Tune matching thresholds
    python3 -m src.texture_main \\
        --seeds ./seeds/color_mars --ros-topic /camera/color/image_raw \\
        --confidence-threshold 0.50 --w-color 0.5 --w-texture 0.3 --w-gradient 0.2

This pipeline replaces ORB feature matching with multi-descriptor histogram
matching (HSV colour + LBP texture + gradient orientation).  It reuses the
existing capture, seeds, and runner modules.
"""

import argparse
import cv2
import numpy as np
import os
import sys

from .capture import OpenCVCapture, RealSenseCapture, ROSCapture
from .seeds import load_seeds, show_seeds
from .texture_match import TextureDescriptor
from .texture_pipeline import TexturePipeline, TextureConfig
from .survey_runner import SurveyRunner


def main():
    parser = argparse.ArgumentParser(
        description="IRoC Texture Survey -- histogram-based matching "
                    "(HSV colour + LBP texture + gradient structure)"
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
                             help="ROS image topic (e.g. /camera/color/image_raw)")

    parser.add_argument("--realsense-fps", type=int, default=6,
                        choices=[6, 15, 30],
                        help="RealSense stream rate (default: 6)")

    # ------------------------------------------------------------------ Seeds
    parser.add_argument("--seeds", required=True,
                        help="directory with seed images")
    parser.add_argument("--seed-center-crop", type=float, default=1.0,
                        help="center crop fraction of each seed (default: 1.0)")

    # --------------------------------------------------------------- Resolution
    parser.add_argument("--width", type=int, default=1280,
                        help="working frame width (default: 1280)")
    parser.add_argument("--height", type=int, default=720,
                        help="working frame height (default: 720)")

    # --------------------------------------------------------------- Patching
    parser.add_argument(
        "--patch-sizes", type=int, nargs="+", default=[256, 512],
        metavar="PX",
        help="HD crop sizes downsampled to 128x128 for matching "
             "(default: 256 512)")
    parser.add_argument("--patch-overlap", type=float, default=0.5,
                        help="sliding window overlap (0.0-0.9, default: 0.5)")
    parser.add_argument("--lr-size", type=int, default=128,
                        help="LR patch size for matching (default: 128, rulebook)")

    # ------------------------------------------------------------- Texture matching
    parser.add_argument("--color-prefilter", type=float, default=0.35,
                        help="min HSV colour similarity to pass Stage 1 pre-filter "
                             "(default: 0.35). Lower = more patches proceed.")
    parser.add_argument("--confidence-threshold", type=float, default=0.50,
                        help="min weighted score to accept a detection "
                             "(default: 0.50). Lower = more detections.")
    parser.add_argument("--w-color", type=float, default=0.45,
                        help="weight for HSV colour similarity (default: 0.45)")
    parser.add_argument("--w-texture", type=float, default=0.35,
                        help="weight for LBP texture similarity (default: 0.35)")
    parser.add_argument("--w-gradient", type=float, default=0.20,
                        help="weight for gradient similarity (default: 0.20)")
    parser.add_argument("--nms-iou", type=float, default=0.30,
                        help="NMS IoU threshold for overlapping detections "
                             "(default: 0.30)")

    # --------------------------------------------------------------- Frame I/O
    parser.add_argument("--fps", type=float, default=1.0,
                        help="target processing rate in Hz (default: 1.0)")
    parser.add_argument("--blur-threshold", type=float, default=30.0,
                        help="Laplacian variance below this = blurry (default: 30.0)")

    # ------------------------------------------------------------------ Output
    parser.add_argument("--save-dir", default="./detections",
                        help="directory to save HD frames on detection "
                             "(default: ./detections)")
    parser.add_argument("--display", action="store_true",
                        help="show live annotated display window (laptop only)")
    parser.add_argument("--stop-on-any-match", action="store_true",
                        help="stop after the first detection")

    # -------------------------------------------------------- Debugging
    parser.add_argument("--show-seeds", action="store_true",
                        help="display preprocessed 128x128 seed images and exit")

    args = parser.parse_args()

    # ---------------------------------------------------------------- Config
    config = TextureConfig(
        input_width=args.width,
        input_height=args.height,
        lr_size=args.lr_size,
        patch_sizes=args.patch_sizes,
        patch_overlap=args.patch_overlap,
        blur_threshold=args.blur_threshold,
        target_fps=args.fps,
        hd_save_dir=args.save_dir,
        display=args.display,
        stop_on_any_match=args.stop_on_any_match,
        drop_stale_frames=args.video is None,
        # Texture-specific
        color_prefilter=args.color_prefilter,
        confidence_threshold=args.confidence_threshold,
        w_color=args.w_color,
        w_texture=args.w_texture,
        w_gradient=args.w_gradient,
        nms_iou_threshold=args.nms_iou,
    )

    # --------------------------------------------------------------- Load seeds
    print(f"[texture] loading seeds from: {args.seeds}")
    print(f"[texture] LR size: {config.lr_size}x{config.lr_size} (rulebook)")
    print(f"[texture] patch scales: {config.patch_sizes}px -> resized to "
          f"{config.lr_size}x{config.lr_size}")
    print(f"[texture] seed center crop: {args.seed_center_crop}")
    print(f"[texture] target fps: {config.target_fps:.1f}")
    print(f"[texture] weights: color={config.w_color} texture={config.w_texture} "
          f"gradient={config.w_gradient}")
    print(f"[texture] confidence threshold: {config.confidence_threshold}")
    print(f"[texture] colour prefilter: {config.color_prefilter}")

    # Load seeds in texture mode (no ORB computation)
    seeds = load_seeds(
        args.seeds,
        orb=None,
        lr_size=config.lr_size,
        seed_center_crop=args.seed_center_crop,
        mode='texture',
    )

    if not seeds:
        print("[texture] ERROR: no valid seeds loaded. check your seed directory.")
        sys.exit(1)

    print(f"[texture] loaded {len(seeds)} seed(s):")
    for s in seeds:
        print(f"  - {s.name}: {s.width}x{s.height}")

    if args.show_seeds:
        print("[texture] showing 128x128 seed images (press any key to continue)...")
        show_seeds(seeds, lr=True, wait=True)
        if not any([args.camera is not None, args.video, args.camera_gst,
                     args.realsense, args.ros_topic]):
            print("[texture] no input source specified -- exiting after seed preview.")
            sys.exit(0)

    # ---------------------------------------------------------- Precompute descriptors
    print("[texture] computing seed texture descriptors...")
    seed_descriptors = []
    for s in seeds:
        # Create mask to exclude black padding from pad-to-square
        gray_mask = cv2.cvtColor(s.image_lr_bgr, cv2.COLOR_BGR2GRAY)
        mask = (gray_mask > 2).astype(np.uint8) * 255
        valid_pct = np.count_nonzero(mask) / mask.size
        if valid_pct > 0.95:
            mask = None  # negligible padding

        desc = TextureDescriptor.from_image(s.image_lr_bgr, mask=mask)
        seed_descriptors.append(desc)

        hsv_nz = int(np.count_nonzero(desc.hsv_hist))
        lbp_nz = int(np.count_nonzero(desc.lbp_hist))
        grad_nz = int(np.count_nonzero(desc.grad_hist))
        mask_str = f"  (mask: {valid_pct*100:.0f}% valid)" if mask is not None else ""
        print(f"  - {s.name}: HSV {hsv_nz}/512 active bins, "
              f"LBP {lbp_nz}/59, GRAD {grad_nz}/9{mask_str}")

    # -------------------------------------------------------- Validate input
    if not any([args.camera is not None, args.video, args.camera_gst,
                 args.realsense, args.ros_topic]):
        print("[texture] ERROR: specify --camera, --video, --camera-gst, "
              "--realsense, or --ros-topic")
        sys.exit(1)

    # ------------------------------------------------------------- Open capture
    print("[texture] opening capture: ", end="")
    if args.video:
        print(f"video={args.video}")
        capture = OpenCVCapture(video_path=args.video)
    elif args.realsense:
        print(f"realsense {args.width}x{args.height} @ {args.realsense_fps}fps")
        capture = RealSenseCapture(width=args.width, height=args.height,
                                    fps=args.realsense_fps)
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
    pipeline = TexturePipeline(seeds, seed_descriptors, config)
    runner = SurveyRunner(capture, pipeline, config)
    runner.run()


if __name__ == "__main__":
    main()
