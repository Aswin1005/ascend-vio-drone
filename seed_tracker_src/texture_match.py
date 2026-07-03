"""Multi-cue texture descriptor matching for IRoC survey detection.

Computes three complementary descriptors per 128x128 image patch:
  1. HSV Color Histogram  -- captures colour distribution (rocks, red-oxide, terrain)
  2. Uniform LBP Histogram -- captures micro-texture patterns (rock grain, printed imagery)
  3. Gradient Orientation Histogram -- captures edge structure (crater rims, layering)

Matching uses Bhattacharyya coefficient (colour, gradient) and chi-squared
similarity (texture), fused with configurable weights.

All operations are pure OpenCV + NumPy.  No neural network, no GPU required.
"""

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Precompute uniform LBP lookup table (module-level, computed once on import)
# ---------------------------------------------------------------------------

def _build_uniform_lbp_map():
    """Build mapping from 8-bit LBP code to uniform pattern bin index.

    Uniform patterns have at most 2 bitwise transitions in the circular
    bit string.  For 8 neighbours there are 58 uniform patterns (bins 0-57)
    plus 1 catch-all bin for non-uniform patterns (bin 58).  Total = 59 bins.
    """
    table = np.zeros(256, dtype=np.uint8)
    uid = 0
    for v in range(256):
        transitions = 0
        for i in range(8):
            b1 = (v >> i) & 1
            b2 = (v >> ((i + 1) % 8)) & 1
            transitions += int(b1 != b2)
        if transitions <= 2:
            table[v] = uid
            uid += 1
        else:
            table[v] = 58  # non-uniform bin
    return table


_UNIFORM_LBP_MAP = _build_uniform_lbp_map()
_N_LBP_BINS = 59  # 58 uniform + 1 non-uniform


# ---------------------------------------------------------------------------
# Descriptor class
# ---------------------------------------------------------------------------

class TextureDescriptor:
    """Multi-cue feature descriptor for a 128x128 image region.

    Attributes
    ----------
    hsv_hist  : np.ndarray (512,)  -- normalised HSV colour histogram
    lbp_hist  : np.ndarray (59,)   -- normalised uniform-LBP histogram
    grad_hist : np.ndarray (9,)    -- normalised gradient-orientation histogram
    """

    __slots__ = ('hsv_hist', 'lbp_hist', 'grad_hist')

    def __init__(self, hsv_hist, lbp_hist, grad_hist):
        self.hsv_hist = hsv_hist
        self.lbp_hist = lbp_hist
        self.grad_hist = grad_hist

    # ----- Factory methods -------------------------------------------------

    @classmethod
    def from_image(cls, bgr_img, mask=None):
        """Compute full descriptor from a BGR image (should be 128x128).

        Args:
            bgr_img: BGR image.
            mask: optional uint8 mask (255 = valid pixel, 0 = ignore).
                  Used to exclude black padding from pad-to-square seeds.
        """
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

        # CLAHE enhances texture/gradient contrast under varying lighting.
        # Applied only to grayscale (LBP & gradient); raw HSV keeps colour fidelity.
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        gray_eq = clahe.apply(gray)

        hsv_hist = cls.compute_hsv_histogram(hsv, mask)
        lbp_hist = cls.compute_lbp_histogram(gray_eq, mask)
        grad_hist = cls.compute_gradient_histogram(gray_eq, mask)

        return cls(hsv_hist, lbp_hist, grad_hist)

    # ----- Static histogram computation (public for staged pipeline use) ---

    @staticmethod
    def compute_hsv_histogram(hsv_img, mask=None):
        """Normalised 3-D HSV histogram  (H=16, S=8, V=4  ->  512 bins).

        V uses only 4 bins so brightness variations are partially tolerated
        while H and S capture the discriminative colour signature.
        """
        hist = cv2.calcHist(
            [hsv_img], [0, 1, 2], mask,
            [16, 8, 4],
            [0, 180, 0, 256, 0, 256],
        )
        hist = hist.flatten().astype(np.float32)
        total = hist.sum()
        if total > 0:
            hist /= total
        return hist

    @staticmethod
    def compute_lbp_histogram(gray_img, mask=None):
        """Normalised uniform-LBP histogram (8 neighbours, radius 1  ->  59 bins).

        Implemented with vectorised NumPy shifts -- no scikit-image dependency.
        """
        h, w = gray_img.shape
        if h < 3 or w < 3:
            return np.zeros(_N_LBP_BINS, dtype=np.float32)

        center = gray_img[1:-1, 1:-1].astype(np.int16)

        # 8 neighbours clockwise from top-left
        neighbours = [
            gray_img[0:-2, 0:-2],   # NW
            gray_img[0:-2, 1:-1],   # N
            gray_img[0:-2, 2:],     # NE
            gray_img[1:-1, 2:],     # E
            gray_img[2:,   2:],     # SE
            gray_img[2:,   1:-1],   # S
            gray_img[2:,   0:-2],   # SW
            gray_img[1:-1, 0:-2],   # W
        ]

        lbp = np.zeros_like(center, dtype=np.uint8)
        for i, n in enumerate(neighbours):
            lbp |= ((n.astype(np.int16) >= center).astype(np.uint8)) << i

        # Map raw 8-bit codes -> uniform bins
        lbp_u = _UNIFORM_LBP_MAP[lbp]

        if mask is not None:
            mask_inner = mask[1:-1, 1:-1]
            lbp_values = lbp_u[mask_inner > 0]
        else:
            lbp_values = lbp_u.ravel()

        hist = np.bincount(lbp_values, minlength=_N_LBP_BINS).astype(np.float32)
        total = hist.sum()
        if total > 0:
            hist /= total
        return hist

    @staticmethod
    def compute_gradient_histogram(gray_img, mask=None):
        """Normalised gradient-orientation histogram (9 bins, 0-180deg, magnitude-weighted).

        Similar to a single-cell simplified HOG.
        """
        gx = cv2.Sobel(gray_img, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray_img, cv2.CV_32F, 0, 1, ksize=3)

        magnitude = cv2.magnitude(gx, gy)
        orientation = cv2.phase(gx, gy, angleInDegrees=True)  # 0-360
        orientation = orientation % 180.0  # unsigned -> 0-180

        n_bins = 9
        bin_width = 180.0 / n_bins
        bins = np.clip((orientation / bin_width).astype(np.int32), 0, n_bins - 1)

        if mask is not None:
            valid = mask > 0
            hist = np.bincount(
                bins[valid].ravel(),
                weights=magnitude[valid].ravel(),
                minlength=n_bins,
            ).astype(np.float32)
        else:
            hist = np.bincount(
                bins.ravel(),
                weights=magnitude.ravel(),
                minlength=n_bins,
            ).astype(np.float32)

        total = hist.sum()
        if total > 0:
            hist /= total
        return hist


# ---------------------------------------------------------------------------
# Similarity / distance functions
# ---------------------------------------------------------------------------

def bhattacharyya_coeff(h1, h2):
    """Bhattacharyya coefficient between two normalised histograms.

    Returns a value in [0, 1] where 1 = identical distributions.
    """
    return float(np.sum(np.sqrt(np.maximum(h1 * h2, 0.0))))


def chi2_similarity(h1, h2):
    """Chi-squared similarity between two normalised histograms.

    Returns a value in (0, 1] where 1 = identical.  Uses  1 / (1 + chi2).
    """
    eps = 1e-10
    chi2 = float(np.sum((h1 - h2) ** 2 / (h1 + h2 + eps)))
    return 1.0 / (1.0 + chi2)


def match_score(patch_desc, seed_desc, weights=(0.45, 0.35, 0.20)):
    """Weighted multi-cue similarity between a patch and a seed descriptor.

    Args:
        patch_desc: TextureDescriptor for the candidate patch.
        seed_desc:  TextureDescriptor for the reference seed.
        weights:    (w_color, w_texture, w_gradient).

    Returns:
        (total_score, color_sim, texture_sim, gradient_sim)
        Each value in [0, 1].
    """
    w_c, w_t, w_g = weights

    color_sim = bhattacharyya_coeff(patch_desc.hsv_hist, seed_desc.hsv_hist)
    texture_sim = chi2_similarity(patch_desc.lbp_hist, seed_desc.lbp_hist)
    gradient_sim = bhattacharyya_coeff(patch_desc.grad_hist, seed_desc.grad_hist)

    total = w_c * color_sim + w_t * texture_sim + w_g * gradient_sim
    return (total, color_sim, texture_sim, gradient_sim)
