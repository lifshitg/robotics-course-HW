"""Find the red marker in RGB images."""

from dataclasses import dataclass

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


def detect_marker(mask: NDArray[np.uint8]) -> tuple[MarkerDetection | None, str]:
    """
    Pick the single red region in a 0/255 mask.

    Returns (detection, reason). detection is None when there is no red region,
    only tiny specks, or more than one sizeable region (ambiguous, we never
    guess by size).
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
    region_mask = (labels == region).astype(np.uint8) * np.uint8(255)
    u, v = centers[region]
    return MarkerDetection((float(u), float(v)), region_mask), 'Marker region accepted.'
