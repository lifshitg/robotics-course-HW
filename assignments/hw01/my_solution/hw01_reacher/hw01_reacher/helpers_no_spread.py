import cv2
import numpy as np
from numpy.typing import NDArray

# Measured on presets 1-3: a fully visible disc covers 87-91 red pixels.
# 60 is about two thirds of that: a disc missing a third shifts its centre by ~1.5 px (~5 mm),
# whether the missing part is hidden by the arm or cut off by the image border.
MIN_RED_PIXELS = 60

# The median can be trusted only when most of its samples are valid.
MIN_VALID_FRACTION = 0.5


def red_mask(rgb_img: NDArray[np.uint8]) -> NDArray[np.bool_]:
    """ image: (H, W, 3) uint8, channel order RGB. Returns the (H, W) bool mask of red pixels. """
    if rgb_img.ndim != 3 or rgb_img.shape[2] != 3:
        raise ValueError(f'expected an (H, W, 3) image, got shape {rgb_img.shape}')
    R, G, B = (rgb_img[..., i].astype(int) for i in range(3))
    return (R > 100) & (R - G > 50) & (R - B > 50)


def recognize_disc(rgb_img: NDArray[np.uint8]
                   ) -> tuple[tuple[float, float], NDArray[np.bool_]] | None:
    """ image: (H, W, 3) uint8, channel order RGB.
    Returns (disk_center, mask): disk_center = (u, v) in pixels, mask = (H, W) bool
    of the largest red blob. Returns None if unreliable. """
    # Only the largest red blob counts, so stray red pixels elsewhere cannot shift the centre
    n_blobs, labels, stats, _ = cv2.connectedComponentsWithStats(red_mask(rgb_img).astype(np.uint8))
    if n_blobs < 2:
        return None             # label 0 is the background: no red at all
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = labels == largest
    n_red_pixels = int(mask.sum())
    if n_red_pixels < MIN_RED_PIXELS:
        return None             # missing, mostly hidden, or only noise

    ys, xs = np.nonzero(mask)
    disk_center = (float(xs.mean()), float(ys.mean()))
    return disk_center, mask


def choose_depth(depth_img: NDArray[np.float32], mask: NDArray[np.bool_]) -> float | None:
    """ depth_img: (H, W) float32 in metres, aligned with the RGB image.
    mask: (H, W) bool, the disc pixels from recognize_disc.
    Keeps finite, positive depths (the camera marks readings outside its range as
    +inf or -inf, so the finite check rejects them), needs at least half of the disc
    pixels to be valid, and returns their median. Returns None if unreliable. """
    if depth_img.shape != mask.shape:
        raise ValueError(f'depth shape {depth_img.shape} does not match mask shape {mask.shape}')

    disc_depths = depth_img[mask]
    valid_depths = disc_depths[np.isfinite(disc_depths) & (disc_depths > 0)]
    if valid_depths.size == 0 or valid_depths.size < MIN_VALID_FRACTION * disc_depths.size:
        return None             # too few valid depth readings on the disc

    return float(np.median(valid_depths))


def locate_disc(rgb_img: NDArray[np.uint8],
                depth_img: NDArray[np.float32]) -> tuple[float, float, float] | None:
    """ Returns (u, v, depth): the disc centre in pixels and its depth in metres,
    or None if either the detection or the depth is unreliable. """
    detection = recognize_disc(rgb_img)
    if detection is None:
        return None
    disk_center, mask = detection

    depth = choose_depth(depth_img, mask)
    if depth is None:
        return None
    return disk_center[0], disk_center[1], depth
