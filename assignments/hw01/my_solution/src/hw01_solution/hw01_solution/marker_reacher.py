"""Reconstruct the red marker in the camera frame and compute a hover goal."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import PointStamped
import numpy as np
from sensor_msgs.msg import CameraInfo


BASE_FRAME = 'base_link'
HOVER_OFFSET_METRES = 0.08


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters from CameraInfo.k (row-major 3x3 matrix)."""

    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0


def intrinsics_from_camera_info(info: CameraInfo) -> Intrinsics:
    """Read and validate the intrinsic matrix (no hardcoded focal length)."""
    k = [float(value) for value in info.k]
    if len(k) != 9 or not all(math.isfinite(value) for value in k):
        raise ValueError('Camera matrix must contain nine finite values.')
    if k[0] <= 0.0 or k[4] <= 0.0:
        raise ValueError('Camera focal lengths must be positive.')
    return Intrinsics(fx=k[0], fy=k[4], cx=k[2], cy=k[5], skew=k[1])


def pixel_to_optical_point(
    u: float, v: float, depth: float, intrinsics: Intrinsics
) -> tuple[float, float, float]:
    """
    Back-project pixel (u, v) with optical-Z depth into camera_optical_frame.

    Inverts u = fx*X/Z + skew*Y/Z + cx and v = fy*Y/Z + cy, with Z = depth.
    The depth was already validated by measure_marker_depth.
    """
    y = (v - intrinsics.cy) * depth / intrinsics.fy
    x = (u - intrinsics.cx - intrinsics.skew * y / depth) * depth / intrinsics.fx
    return x, y, depth


def hover_goal(target: PointStamped) -> PointStamped:
    """Return the base_link goal HOVER_OFFSET_METRES above the marker."""
    if target.header.frame_id != BASE_FRAME:
        raise ValueError('Hover goals must be computed in base_link.')
    goal = PointStamped(header=target.header)
    goal.point.x = target.point.x
    goal.point.y = target.point.y
    goal.point.z = target.point.z + HOVER_OFFSET_METRES
    return goal


def _xyz(point) -> np.ndarray:
    """Return a Point (or PointStamped) as a numpy array."""
    point = getattr(point, 'point', point)
    return np.array([point.x, point.y, point.z], dtype=float)
