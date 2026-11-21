"""Image-only marker estimation; no simulator poses or preset information."""
import math
from collections.abc import Sequence

import cv2
import numpy as np


class ObservationError(ValueError):
    """An observation is unsuitable for commanding motion."""


def check_timing(rgb_stamp: float, depth_stamp: float, now: float,
                 wall_age: float) -> None:
    if not all(math.isfinite(value) for value in (rgb_stamp, depth_stamp, now, wall_age)):
        raise ObservationError('Nonfinite observation time')
    if rgb_stamp <= 0 or depth_stamp <= 0:
        raise ObservationError('Image timestamps must be nonzero simulation times')
    if abs(rgb_stamp - depth_stamp) > 0.05 + 1e-9:
        raise ObservationError('RGB/depth timestamps differ by more than 0.05 seconds')
    if not (0 <= now - rgb_stamp <= 0.5 and 0 <= now - depth_stamp <= 0.5):
        raise ObservationError('RGB/depth observation is stale or in the future')
    if not 0 <= wall_age < 2.0:
        raise ObservationError('No fresh camera pair within 2 wall-clock seconds')


def marker_point(bgr: np.ndarray, depth: np.ndarray,
                 intrinsic: Sequence[float]) -> tuple[float, float, float]:
    """Back-project a reliable visible red region using metres of optical depth."""
    if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ObservationError('Expected an 8-bit, three-channel BGR image')
    if depth.ndim != 2 or depth.shape != bgr.shape[:2] or depth.dtype != np.float32:
        raise ObservationError('Expected aligned 32FC1 depth matching the RGB dimensions')
    if len(intrinsic) != 9 or not all(math.isfinite(value) for value in intrinsic):
        raise ObservationError('Camera intrinsic matrix must contain nine finite values')
    fx, fy, cx, cy = intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
    if fx <= 0 or fy <= 0:
        raise ObservationError('Camera focal lengths must be positive')

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Red wraps around OpenCV's hue range.
    mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
    mask |= cv2.inRange(hsv, (170, 120, 70), (179, 255, 255))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 30]
    if len(candidates) != 1:
        raise ObservationError('Expected exactly one reliable red marker region')
    label = candidates[0]
    x, y, width, height, _ = stats[label]
    if (x == 0 or y == 0 or x + width == mask.shape[1] or y + height == mask.shape[0]
            or min(width, height) < 5):
        raise ObservationError('Red region is clipped or too thin')

    # Avoid mixed marker/background depth at the color boundary.
    interior = cv2.erode((labels == label).astype(np.uint8),
                         np.ones((3, 3), dtype=np.uint8)).astype(bool)
    minimum_samples = max(10, 0.6 * np.count_nonzero(interior))
    valid = interior & np.isfinite(depth) & (depth >= 0.05) & (depth <= 5.0)
    if np.count_nonzero(valid) < minimum_samples:
        raise ObservationError('Insufficient valid marker depth in the 0.05-5.0 m range')
    median = float(np.median(depth[valid]))
    inliers = valid & (np.abs(depth - median) <= 0.01)
    if np.count_nonzero(inliers) < minimum_samples:
        raise ObservationError('Marker depth is inconsistent across the visible region')
    rows, cols = np.nonzero(inliers)
    u = float(np.mean(cols))
    v = float(np.mean(rows))
    z = float(np.median(depth[inliers]))
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    point = (x, y, z)
    if not all(math.isfinite(value) for value in point):
        raise ObservationError('Back-projection produced a nonfinite point')
    return point
