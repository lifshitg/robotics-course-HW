"""Pure perception functions (no ROS): marker detection, depth selection, back-projection.

Kept free of rclpy so they can be unit-tested offline with synthetic or saved frames.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class DetectionParams:
    # HSV thresholds (OpenCV hue range is 0..179). Red wraps around 0, so two bands.
    # The upper bound of the low band stays below the yellow arm links (hue ~12).
    hue_low_max: int = 8
    hue_high_min: int = 170
    sat_min: int = 120
    val_min: int = 60
    min_area_px: int = 12          # smaller blobs are noise or a mostly hidden marker
    max_area_px: int = 3000        # much larger than the marker can appear in a 320x240 frame
    min_fill_ratio: float = 0.55   # area / bbox area; a full disk gives pi/4 ~ 0.785
    max_aspect_ratio: float = 1.6  # bbox long/short side; a disk seen from above is ~1
    max_second_ratio: float = 0.3  # reject if a 2nd red blob is this large relative to the 1st
    border_margin_px: int = 1      # a blob touching the image edge is cut off -> biased centroid


@dataclass
class Detection:
    u: float            # centroid column (pixel index, OpenCV convention)
    v: float            # centroid row
    area: int
    mask: np.ndarray    # bool HxW, pixels of the selected blob


@dataclass
class DepthParams:
    min_depth_m: float = 0.05      # camera near clip
    max_depth_m: float = 3.0       # camera far clip
    min_valid_px: int = 5
    min_valid_fraction: float = 0.5
    max_spread_m: float = 0.02     # median absolute deviation limit; larger means mixed surfaces


def red_mask(bgr, p=DetectionParams()):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red_hue = (h <= p.hue_low_max) | (h >= p.hue_high_min)
    return red_hue & (s >= p.sat_min) & (v >= p.val_min)


def detect_marker(bgr, p=DetectionParams()):
    """Return (Detection, None) or (None, reason)."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        return None, 'image is not a 3-channel colour image'
    mask = red_mask(bgr, p).astype(np.uint8)
    # Remove isolated noisy pixels; a 3x3 opening keeps a ~10 px wide disk intact.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None, 'no red pixels (marker absent or fully occluded)'
    areas = stats[1:, cv2.CC_STAT_AREA]
    order = np.argsort(areas)[::-1]
    best = order[0] + 1
    area = int(stats[best, cv2.CC_STAT_AREA])
    if area < p.min_area_px:
        return None, f'red blob too small ({area} px)'
    if area > p.max_area_px:
        return None, f'red blob too large ({area} px)'
    if len(order) > 1 and areas[order[1]] > p.max_second_ratio * area:
        return None, 'several comparable red blobs; ambiguous target'
    x, y, w, hgt = (int(stats[best, k]) for k in
                    (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    rows, cols = mask.shape
    m = p.border_margin_px
    if x < m or y < m or x + w > cols - m or y + hgt > rows - m:
        return None, 'blob touches the image border (marker cut off)'
    if area / float(w * hgt) < p.min_fill_ratio:
        return None, 'blob shape irregular (partly occluded?)'
    if max(w, hgt) / float(min(w, hgt)) > p.max_aspect_ratio:
        return None, 'blob elongated (partly occluded?)'
    u, v = centroids[best]
    return Detection(float(u), float(v), area, labels == best), None


def select_depth(depth, mask, p=DepthParams()):
    """Robust marker depth (metres along optical Z). Return (z, None) or (None, reason).

    Edge pixels can mix marker and platform depth, so the mask is eroded first when
    enough interior pixels remain; then invalid values are discarded and the median
    of the rest is used. A large spread means the pixels do not lie on one surface.
    """
    if depth.shape != mask.shape:
        return None, f'depth shape {depth.shape} != image shape {mask.shape}'
    inner = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    use = inner if inner.sum() >= p.min_valid_px else mask
    values = depth[use].astype(np.float64)
    valid = values[np.isfinite(values) & (values > p.min_depth_m) & (values < p.max_depth_m)]
    if valid.size < p.min_valid_px or valid.size < p.min_valid_fraction * values.size:
        return None, f'too few valid depth pixels ({valid.size}/{values.size})'
    z = float(np.median(valid))
    spread = float(np.median(np.abs(valid - z)))
    if spread > p.max_spread_m:
        return None, f'depth spread too large ({spread:.3f} m)'
    return z, None


def intrinsics(k):
    """(fx, fy, cx, cy) from a row-major 3x3 camera matrix (CameraInfo.k)."""
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(a) for a in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError('invalid camera intrinsics')
    return fx, fy, cx, cy


def backproject(u, v, z, k):
    """Pinhole back-projection into the optical frame (+X right, +Y down, +Z forward)."""
    fx, fy, cx, cy = intrinsics(k)
    return (u - cx) * z / fx, (v - cy) * z / fy, z


def depth_to_metres(array, encoding):
    """Convert a depth image to float metres according to its ROS encoding."""
    if encoding == '32FC1':
        return array.astype(np.float32, copy=False)
    if encoding == '16UC1':  # millimetres, 0 = no reading
        out = array.astype(np.float32) * 0.001
        out[array == 0] = np.nan
        return out
    raise ValueError(f'unsupported depth encoding {encoding!r}')
