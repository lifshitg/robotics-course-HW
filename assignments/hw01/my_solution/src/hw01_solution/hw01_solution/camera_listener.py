"""Find the red marker in RGB images."""

from dataclasses import dataclass
import math

import cv2
import numpy as np
from numpy.typing import NDArray


MIN_REGION_PIXELS = 20


def make_red_mask(rgb_pixels: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return 255 where a pixel is clearly red (R >= 150, G <= 100, B <= 100)."""
    red, green, blue = (rgb_pixels[:, :, i] for i in range(3))
    is_red = (red >= 150) & (green <= 100) & (blue <= 100)
    return is_red.astype(np.uint8) * np.uint8(255)


@dataclass(frozen=True)
class MarkerDetection:
    """Subpixel center (u=column, v=row) and the mask of the chosen region."""

    pixel: tuple[float, float]
    mask: NDArray[np.uint8]


def _bounding_box_rejection(
    stats_row: NDArray[np.int32], image_height: int, image_width: int
) -> str | None:
    """Return why the region's bounding box looks wrong, or None if it is fine."""
    x, y, width, height, area = (int(value) for value in stats_row[:5])
    if x == 0 or y == 0 or x + width == image_width or y + height == image_height:
        return 'Red region touches the image border.'
    if min(width, height) / max(width, height) < 0.75:
        return 'Red region is elongated (possible occlusion).'
    if not 0.55 <= area / (width * height) <= 0.90:
        return 'Red region does not have a filled round shape.'
    return None


def _outline_rejection(region_mask: NDArray[np.uint8]) -> str | None:
    """Return why the region's outline is not round, or None if it is fine."""
    contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 1:
        return 'Red region outline is unreliable.'
    perimeter = cv2.arcLength(contours[0], True)
    if perimeter <= 0:
        return 'Red region outline is degenerate.'
    if 4.0 * math.pi * cv2.contourArea(contours[0]) / perimeter**2 < 0.70:
        return 'Red region is not round enough (possible occlusion).'
    return None


def detect_marker(mask: NDArray[np.uint8]) -> tuple[MarkerDetection | None, str]:
    """
    Pick the single round red region in a 0/255 mask.

    Returns (detection, reason). detection is None when there is no red region,
    only tiny specks, more than one sizeable region (ambiguous, we never guess by
    size), or the one region fails the shape checks (border, elongated, not
    filled, not round).
    """
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count == 1:
        return None, 'No red region.'
    regions = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= MIN_REGION_PIXELS]
    if not regions:
        return None, 'Red regions are too small.'
    if len(regions) != 1:
        return None, 'Multiple sizeable red regions; marker is ambiguous.'
    region = regions[0]
    reason = _bounding_box_rejection(stats[region], *mask.shape)
    if reason is not None:
        return None, reason
    region_mask = (labels == region).astype(np.uint8) * np.uint8(255)
    reason = _outline_rejection(region_mask)
    if reason is not None:
        return None, reason
    u, v = centers[region]
    return MarkerDetection((float(u), float(v)), region_mask), 'Marker region accepted.'
