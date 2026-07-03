import cv2
import numpy as np
import os
import glob


class SeedTemplate:
    def __init__(self, name, image_bgr, image_gray, keypoints, descriptors, width, height,
                 image_lr_bgr=None, image_lr_gray=None):
        self.name = name
        self.image_bgr = image_bgr
        self.image_gray = image_gray
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.width = width
        self.height = height
        # LR versions (only populated when lr_size is set)
        self.image_lr_bgr = image_lr_bgr
        self.image_lr_gray = image_lr_gray

    @property
    def corners(self):
        return np.array(
            [[0, 0], [self.width - 1, 0], [self.width - 1, self.height - 1], [0, self.height - 1]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)


def _pad_to_square(img):
    """Pad image to square with black border, preserving aspect ratio."""
    h, w = img.shape[:2]
    if h == w:
        return img
    side = max(h, w)
    if len(img.shape) == 3:
        canvas = np.zeros((side, side, img.shape[2]), dtype=img.dtype)
    else:
        canvas = np.zeros((side, side), dtype=img.dtype)
    y_off = (side - h) // 2
    x_off = (side - w) // 2
    canvas[y_off:y_off + h, x_off:x_off + w] = img
    return canvas


def _center_crop(img, fraction):
    """Crop the centered fraction of an image."""
    if fraction <= 0.0 or fraction > 1.0:
        raise ValueError("seed_center_crop must be in the range (0, 1]")
    if fraction >= 0.999:
        return img

    h, w = img.shape[:2]
    crop_w = max(8, int(round(w * fraction)))
    crop_h = max(8, int(round(h * fraction)))
    x = max(0, (w - crop_w) // 2)
    y = max(0, (h - crop_h) // 2)
    return img[y:y + crop_h, x:x + crop_w]


def load_seeds(seed_dir, orb=None, lr_size=0, seed_center_crop=1.0, mode='orb'):
    """Load seed images and optionally compute descriptors.

    Args:
        seed_dir: path to directory containing seed images.
        orb: cv2.ORB instance for feature detection.
        lr_size: if > 0, preprocess seeds to this square size for LR matching.
                 Seeds already at lr_size×lr_size are passed through without
                 re-processing. If 0, seeds are used at original resolution
                 (backwards-compatible with existing tracker).
        seed_center_crop: centered fraction of each seed to learn. Use values
                 like 0.65-0.85 when seed photos include background around the
                 object.
        mode: 'orb' (default — compute ORB keypoints/descriptors) or 'texture'
                 (load images only, skip ORB — for texture-based matching).
    """
    if not os.path.exists(seed_dir):
        raise Exception("bruh seed dir doesnt exist")

    if mode == 'orb' and orb is None:
        raise ValueError("ORB detector required when mode='orb'")

    seeds = []

    # just grab all images
    files = []
    for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        files.extend(glob.glob(os.path.join(seed_dir, ext)))

    for f in sorted(files):
        img = cv2.imread(f, cv2.IMREAD_COLOR)
        if img is None:
            continue

        h, w = img.shape[:2]
        name, _ = os.path.splitext(os.path.basename(f))
        seed_img = _center_crop(img, seed_center_crop)

        if mode == 'texture':
            # --- Texture mode: image only, no ORB features needed ---
            effective_lr = lr_size if lr_size > 0 else 128
            seed_h, seed_w = seed_img.shape[:2]
            if seed_h == effective_lr and seed_w == effective_lr:
                lr_bgr = seed_img.copy()
            else:
                squared = _pad_to_square(seed_img)
                lr_bgr = cv2.resize(squared, (effective_lr, effective_lr),
                                    interpolation=cv2.INTER_AREA)
            lr_gray = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2GRAY)
            seeds.append(
                SeedTemplate(
                    name=name,
                    image_bgr=img,
                    image_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                    keypoints=[],
                    descriptors=None,
                    width=effective_lr,
                    height=effective_lr,
                    image_lr_bgr=lr_bgr,
                    image_lr_gray=lr_gray,
                )
            )
            continue

        if lr_size > 0:
            # --- LR mode (survey pipeline) ---
            # Check if seed is already exactly lr_size×lr_size
            seed_h, seed_w = seed_img.shape[:2]
            if seed_h == lr_size and seed_w == lr_size:
                lr_bgr = seed_img.copy()
            else:
                # Pad to square first to preserve aspect ratio, then resize
                squared = _pad_to_square(seed_img)
                lr_bgr = cv2.resize(squared, (lr_size, lr_size),
                                    interpolation=cv2.INTER_AREA)

            lr_gray = cv2.cvtColor(lr_bgr, cv2.COLOR_BGR2GRAY)
            
            # Create a mask to ignore pure black background (from padding or background removal)
            _, mask = cv2.threshold(lr_gray, 0, 255, cv2.THRESH_BINARY)
            
            # Apply CLAHE to enhance subtle textures at low resolutions
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            lr_gray_clahe = clahe.apply(lr_gray)
            
            kps, descs = orb.detectAndCompute(lr_gray_clahe, mask)
            if descs is None or len(kps) < 4:
                print(f"[seeds] warning: {name} has too few features at LR ({len(kps) if kps else 0}), skipping")
                continue

            seeds.append(
                SeedTemplate(
                    name=name,
                    image_bgr=img,
                    image_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                    keypoints=kps,
                    descriptors=descs,
                    width=lr_size,
                    height=lr_size,
                    image_lr_bgr=lr_bgr,
                    image_lr_gray=lr_gray,
                )
            )
        else:
            # --- original mode (tracker pipeline, unchanged behaviour) ---
            gray = cv2.cvtColor(seed_img, cv2.COLOR_BGR2GRAY)
            kps, descs = orb.detectAndCompute(gray, None)
            if descs is None or len(kps) < 8:
                continue

            crop_h, crop_w = seed_img.shape[:2]
            seeds.append(
                SeedTemplate(
                    name=name,
                    image_bgr=img,
                    image_gray=gray,
                    keypoints=kps,
                    descriptors=descs,
                    width=crop_w,
                    height=crop_h,
                )
            )

    if len(seeds) == 0:
        print("no seeds found man")

    return seeds


def show_seeds(seeds, lr=True, wait=True):
    """Display loaded seed images in a grid window. For laptop testing only.

    Args:
        seeds: list of SeedTemplate.
        lr: if True, show the LR (128×128) versions. If False, show originals.
        wait: if True, block until user presses a key. If False, return immediately.
    """
    if not seeds:
        print("[show_seeds] no seeds to display")
        return

    images = []
    labels = []
    for s in seeds:
        if lr and s.image_lr_bgr is not None:
            images.append(s.image_lr_bgr)
            labels.append(f"{s.name} (LR {s.image_lr_bgr.shape[1]}x{s.image_lr_bgr.shape[0]})")
        else:
            # Scale original down for display if it's too large
            disp = s.image_bgr
            if max(disp.shape[:2]) > 300:
                scale = 300.0 / max(disp.shape[:2])
                disp = cv2.resize(disp, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            images.append(disp)
            labels.append(f"{s.name} (orig)")

    # Pad all to the same height for horizontal concat
    max_h = max(im.shape[0] for im in images)
    padded = []
    for im, label in zip(images, labels):
        h, w = im.shape[:2]
        if h < max_h:
            pad = np.zeros((max_h - h, w, 3), dtype=np.uint8)
            im = np.vstack([im, pad])
        # Draw label
        cv2.putText(im, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        # Draw keypoint count
        seed_obj = None
        for s in seeds:
            if s.name in label:
                seed_obj = s
                break
        if seed_obj:
            kp_text = f"kps: {len(seed_obj.keypoints)}"
            cv2.putText(im, kp_text, (4, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
        padded.append(im)

    grid = np.hstack(padded)
    cv2.imshow("Seed Images (press any key to close)", grid)
    if wait:
        cv2.waitKey(0)
        cv2.destroyWindow("Seed Images (press any key to close)")
    else:
        cv2.waitKey(1)


def save_lr_seeds(seeds, output_dir):
    """Save the preprocessed 128×128 seed images to the output directory.

    Args:
        seeds: list of SeedTemplate.
        output_dir: directory path to save images to.
    """
    if not seeds:
        return
    os.makedirs(output_dir, exist_ok=True)
    for s in seeds:
        if s.image_lr_bgr is not None:
            path = os.path.join(output_dir, f"{s.name}_lr.png")
            cv2.imwrite(path, s.image_lr_bgr)
            print(f"[seeds] Saved LR seed image to: {path}")
