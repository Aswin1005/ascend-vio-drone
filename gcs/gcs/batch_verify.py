#!/usr/bin/env python3
"""
batch_verify.py  —  ASCEND Seed Verification (v4)
===================================================
Simple, correct approach:
  For EACH SEED independently → find the frame most visually similar to it.
  Keep that frame if similarity > QUALITY_THRESHOLD.
  Deduplicate by GPS location (keep stronger match).

No winner-take-all. Each seed competes independently.
Scoring: HSV colour (35%) + SSIM centre-crop (30%) + template match (25%) + ORB (10%).
"""
import os, sys, csv, cv2, shutil, math, tempfile, zipfile, datetime
import numpy as np
from skimage.metrics import structural_similarity as ssim

# ── Config ─────────────────────────────────────────────────────────────────
SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
SEEDS_DIR      = (os.path.join(SCRIPT_DIR, "seeds (2)")
                  if os.path.exists(os.path.join(SCRIPT_DIR, "seeds (2)"))
                  else os.path.join(SCRIPT_DIR, "seeds"))
MATCH_LOGS_DIR = os.path.join(SCRIPT_DIR, "match_logs")

# Minimum score to consider a seed "detected" (0-1 scale).
# Lower = more detections (risk of false positives).
# Higher = fewer but more confident detections.
QUALITY_THRESHOLD = 0.38

# Spatial dedup: two matches within this distance (m) → keep stronger one.
DEDUP_DISTANCE_M = 1.5

# Working resolution
ORB_W, ORB_H = 640, 360

# Template-matching scales: seed_native_size × scale
TEMPLATE_SCALES = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
orb   = cv2.ORB_create(nfeatures=4000)
bf    = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)


# ── Feature helpers ─────────────────────────────────────────────────────────
def get_feature_group(filename):
    return "feature_" + os.path.splitext(filename)[0]


def hsv_sim(img1_bgr, img2_bgr):
    """
    Hue + Saturation histogram similarity (HISTCMP_CORREL, 0-1).
    HSV is the most discriminative signal for different geological colours
    (dark-red Olympus Mons vs grey rocks vs brown terrain).
    Operates on the central 70% of each image to avoid border noise.
    """
    def crop_center(img, frac=0.70):
        h, w = img.shape[:2]
        dh, dw = int(h*(1-frac)/2), int(w*(1-frac)/2)
        return img[dh:h-dh, dw:w-dw] if dh > 0 and dw > 0 else img

    a = cv2.cvtColor(crop_center(img1_bgr), cv2.COLOR_BGR2HSV)
    b = cv2.cvtColor(crop_center(img2_bgr), cv2.COLOR_BGR2HSV)
    score = 0.0
    for ch, bins, rng in [(0, 32, [0, 180]),   # Hue
                           (1, 32, [0, 256]),   # Saturation
                           (2, 32, [0, 256])]:  # Value
        h1 = cv2.calcHist([a], [ch], None, [bins], rng)
        h2 = cv2.calcHist([b], [ch], None, [bins], rng)
        cv2.normalize(h1, h1, 0, 1, cv2.NORM_MINMAX)
        cv2.normalize(h2, h2, 0, 1, cv2.NORM_MINMAX)
        score += max(0.0, float(cv2.compareHist(h1, h2, cv2.HISTCMP_CORREL)))
    return score / 3.0


def centre_ssim(gray_frame, gray_seed):
    """SSIM on centre 70% of frame vs seed resized to match."""
    h, w = gray_frame.shape
    dh, dw = int(h*0.15), int(w*0.15)
    crop = gray_frame[dh:h-dh, dw:w-dw]
    seed_r = cv2.resize(gray_seed, (crop.shape[1], crop.shape[0]))
    try:
        score, _ = ssim(crop, seed_r, full=True)
        return float(max(0.0, score))
    except Exception:
        return 0.0


def template_match(gray_frame, seed_gray_native):
    """Multi-scale template match. Returns best TM_CCOEFF_NORMED score."""
    best = 0.0
    fh, fw = gray_frame.shape
    for scale in TEMPLATE_SCALES:
        sh, sw = seed_gray_native.shape
        nh, nw = max(8, int(sh * scale)), max(8, int(sw * scale))
        if nh > fh or nw > fw:
            continue
        tmpl = cv2.resize(seed_gray_native, (nw, nh))
        res  = cv2.matchTemplate(gray_frame, tmpl, cv2.TM_CCOEFF_NORMED)
        _, mx, _, _ = cv2.minMaxLoc(res)
        if mx > best:
            best = mx
    return best


def orb_score(gray_frame, gray_seed_wk):
    """ORB + Lowe's ratio + RANSAC. Returns normalised score 0-1."""
    kp1, d1 = orb.detectAndCompute(clahe.apply(gray_frame),    None)
    kp2, d2 = orb.detectAndCompute(clahe.apply(gray_seed_wk),  None)
    if d1 is None or d2 is None or len(d1) < 4 or len(d2) < 4:
        return 0.0
    pairs = bf.knnMatch(d1, d2, k=2)
    good  = [m for p in pairs if len(p)==2
             for m, n in [p] if m.distance < 0.82 * n.distance]
    if len(good) < 4:
        return 0.0
    src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)
    _, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    ransac = int(np.sum(mask)) if mask is not None else 0
    return min(ransac, 30) / 30.0   # normalise to 0-1


def fg_object_sim(frame_bgr, seed_bgr):
    """
    Extract the main foreground object from the frame using Otsu threshold,
    then compare that crop with the seed using HSV histogram similarity.

    Why this helps for rocks:
      - Rocks are tiny dark objects (~5% of frame) on a bright white floor.
      - HSV/SSIM of the full frame is dominated by the white floor (useless).
      - This crops just the dark region and compares that directly.
    Also helps for posters: extracts the dark poster rectangle.
    Returns 0.0 if no clear foreground object is found.
    """
    h, w = frame_bgr.shape[:2]
    frame_area = h * w

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Clean noise
    k = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0

    # Find largest contour that is between 0.5% and 80% of frame area
    valid = sorted([c for c in cnts
                    if 0.005 * frame_area < cv2.contourArea(c) < 0.80 * frame_area],
                   key=cv2.contourArea, reverse=True)
    if not valid:
        return 0.0

    x, y, bw, bh = cv2.boundingRect(valid[0])
    crop = frame_bgr[y:y+bh, x:x+bw]
    if crop.size == 0:
        return 0.0

    # Compare cropped FG with seed using HSV (most colour-discriminative)
    crop_r = cv2.resize(crop,    (64, 64))
    seed_r = cv2.resize(seed_bgr, (64, 64))
    return hsv_sim(crop_r, seed_r)


def combined_score(hsv, sim, tmpl, orb_n, fg=0.0):
    # HSV 0.25 + SSIM 0.25 + TMPL 0.20 + ORB 0.10 + FG 0.20 = 1.0
    return 0.25*hsv + 0.25*sim + 0.20*tmpl + 0.10*orb_n + 0.20*fg



# ── Load seeds ──────────────────────────────────────────────────────────────
def load_seeds():
    seeds = {}
    if not os.path.exists(SEEDS_DIR):
        print(f"Error: Seeds dir not found: {SEEDS_DIR}"); return seeds
    for root, dirs, files in os.walk(SEEDS_DIR):
        if "lr_seeds" in root.split(os.sep):
            continue
        for fn in files:
            if not fn.lower().endswith(('.png', '.jpg', '.jpeg')):
                continue
            img = cv2.imread(os.path.join(root, fn))
            if img is None: continue
            gray_native = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_wk      = cv2.resize(img, (ORB_W, ORB_H))
            gray_wk     = cv2.cvtColor(img_wk, cv2.COLOR_BGR2GRAY)
            seeds[fn] = {
                'img':        img,
                'img_wk':     img_wk,
                'gray_native': gray_native,
                'gray_wk':    gray_wk,
                'fg':         get_feature_group(fn)
            }
    print(f"Loaded {len(seeds)} seed(s): {sorted(seeds.keys())}")
    return seeds


# ── Load telemetry ──────────────────────────────────────────────────────────
def load_telemetry(data_dir):
    for fname, cols in [
        ("positions.csv", ('frame_file','pos_x_east_m','pos_y_north_m','pos_z_up_m')),
        ("telemetry.csv", ('filename','x','y','z'))
    ]:
        p = os.path.join(data_dir, fname)
        if not os.path.exists(p): continue
        result = {}
        with open(p) as f:
            for row in csv.DictReader(f):
                fn = row.get(cols[0])
                if fn:
                    result[fn] = (row.get(cols[1],'0'), row.get(cols[2],'0'), row.get(cols[3],'0'))
        return result
    print(f"Warning: no CSV telemetry in {data_dir}")
    return {}


# ── Spatial dedup ────────────────────────────────────────────────────────────
def parse_coords(s):
    try:
        p = s.replace("X:","").replace("Y:","").replace("Z:","").split(",")
        return float(p[0]), float(p[1]), float(p[2])
    except: return None

def dist3(a, b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def deduplicate(detections):
    """Remove detections at the same GPS location, keep the higher-scoring one."""
    keys   = list(detections.keys())
    remove = set()
    for i in range(len(keys)):
        if keys[i] in remove: continue
        for j in range(i+1, len(keys)):
            if keys[j] in remove: continue
            c1 = parse_coords(detections[keys[i]]['coords'])
            c2 = parse_coords(detections[keys[j]]['coords'])
            if c1 and c2 and dist3(c1, c2) < DEDUP_DISTANCE_M:
                weaker = keys[i] if detections[keys[i]]['combined'] < detections[keys[j]]['combined'] else keys[j]
                print(f"  [DEDUP] {weaker} removed (same GPS location, weaker match)")
                remove.add(weaker)
    for k in remove: del detections[k]
    return detections


# ── Main ─────────────────────────────────────────────────────────────────────
def process_batch(data_dir):
    seeds = load_seeds()
    if not seeds: return
    telemetry     = load_telemetry(data_dir)
    feature_groups = set(s['fg'] for s in seeds.values())

    print(f"\nVerifying: {data_dir}")
    print(f"Seeds: {len(seeds)}  |  Quality threshold: {QUALITY_THRESHOLD}")
    print("=" * 70)

    # Collect all frame paths (sorted, skip _boxed debug frames)
    frame_files = sorted([
        f for f in os.listdir(data_dir)
        if f.lower().endswith(('.jpg','.jpeg','.png'))
        and '_boxed' not in f.lower()
    ])
    total = len(frame_files)
    print(f"Frames to process: {total}\n")

    # Pre-load + pre-process all frames for speed
    print("Pre-loading frames...")
    frames = {}
    for i, fn in enumerate(frame_files):
        img = cv2.imread(os.path.join(data_dir, fn))
        if img is None: continue
        img_wk   = cv2.resize(img, (ORB_W, ORB_H))
        gray_wk  = cv2.cvtColor(img_wk, cv2.COLOR_BGR2GRAY)
        frames[fn] = {'img': img, 'img_wk': img_wk, 'gray_wk': gray_wk,
                      'orig_bgr': img}  # keep orig for fg extraction
        if (i+1) % 100 == 0:
            print(f"  Loaded {i+1}/{total}...")
    print(f"  Done ({len(frames)} frames loaded).\n")

    # Per-seed independent search
    detections = {}   # fg -> best match dict

    for seed_name, sd in seeds.items():
        fg = sd['fg']
        print(f"--- Matching seed: {seed_name} ({fg}) ---")
        best_score  = -1.0
        best_frame  = None
        best_data   = None

        for fn, fdata in frames.items():
            gray_f  = fdata['gray_wk']
            img_wk  = fdata['img_wk']

            hsv   = hsv_sim(img_wk, sd['img_wk'])
            sim   = centre_ssim(gray_f, sd['gray_wk'])
            tmpl  = template_match(gray_f, sd['gray_native'])
            orb_n = orb_score(gray_f, sd['gray_wk'])
            fg_s  = fg_object_sim(fdata['orig_bgr'], sd['img'])
            comb  = combined_score(hsv, sim, tmpl, orb_n, fg_s)

            if comb > best_score:
                best_score = comb
                best_frame = fn
                best_data  = {'hsv': hsv, 'ssim': sim, 'tmpl': tmpl, 'orb': orb_n, 'fg': fg_s}

        if best_frame and best_score >= QUALITY_THRESHOLD:
            coords    = telemetry.get(best_frame, ("unknown","unknown","unknown"))
            coord_str = f"X: {coords[0]}, Y: {coords[1]}, Z: {coords[2]}"
            detections[fg] = {
                'image': best_frame, 'seed': seed_name, 'feature': fg,
                'coords': coord_str,
                'combined': best_score,
                'hsv':  best_data['hsv'],
                'ssim': best_data['ssim'],
                'tmpl': best_data['tmpl'],
                'orb':  best_data['orb'],
                'img_data': frames[best_frame]['img']
            }
            print(f"  MATCH: {best_frame} | comb={best_score:.3f} "
                  f"HSV={best_data['hsv']:.3f} SSIM={best_data['ssim']:.3f} "
                  f"TMPL={best_data['tmpl']:.3f} FG={best_data.get('fg',0):.3f} ORB={best_data['orb']:.3f}")
        else:
            top = f"{best_frame} comb={best_score:.3f}" if best_frame else "none"
            print(f"  NO MATCH (best: {top}, below threshold {QUALITY_THRESHOLD})")
        print()

    # Dedup + output
    print("=" * 70)
    print("Spatial deduplication...")
    detections = deduplicate(detections)

    if os.path.exists(MATCH_LOGS_DIR): shutil.rmtree(MATCH_LOGS_DIR)
    os.makedirs(MATCH_LOGS_DIR, exist_ok=True)

    final = sorted(detections.values(), key=lambda m: m['feature'])
    for m in final:
        print(f"[VERIFIED] {m['feature']}: {m['image']} | "
              f"COMB={m['combined']:.3f} HSV={m['hsv']:.3f} "
              f"SSIM={m['ssim']:.3f} TMPL={m['tmpl']:.3f}")
        cv2.imwrite(os.path.join(MATCH_LOGS_DIR, f"{m['feature']}_{m['image']}"), m['img_data'])

    report = os.path.join(MATCH_LOGS_DIR, "verification_report.txt")
    with open(report, 'w') as f:
        f.write("ASCEND Seed Verification Report\n")
        f.write(f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("="*50 + "\n")
        f.write(f"Unique features detected: {len(final)} / {len(feature_groups)}\n\n")
        for m in final:
            f.write(f"Feature:       {m['feature']}\n")
            f.write(f"Best Frame:    {m['image']}\n")
            f.write(f"Matched Seed:  {m['seed']}\n")
            f.write(f"Coordinates:   {m['coords']} (wrt base station)\n")
            f.write(f"Match Scores:  HSV={m['hsv']:.3f} SSIM={m['ssim']:.3f} "
                    f"TMPL={m['tmpl']:.3f} ORB={m['orb']:.3f} "
                    f"COMBINED={m['combined']:.3f}\n")
            f.write("-"*30 + "\n")

    print(f"\nReport: {report}")
    print(f"Unique features found: {len(final)} / {len(feature_groups)}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python batch_verify.py <directory_or_zip>"); sys.exit(1)
    target = sys.argv[1]
    if target.endswith('.zip') and os.path.isfile(target):
        tmp = tempfile.mkdtemp()
        try:
            with zipfile.ZipFile(target) as z: z.extractall(tmp)
            proc = tmp
            for root, dirs, files in os.walk(tmp):
                if 'positions.csv' in files: proc = root; break
            process_batch(proc)
        finally: shutil.rmtree(tmp)
    elif os.path.isdir(target):
        process_batch(target)
    else:
        print(f"Error: '{target}' is not a valid directory or zip."); sys.exit(1)
