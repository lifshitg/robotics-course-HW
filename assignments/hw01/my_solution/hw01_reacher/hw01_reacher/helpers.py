import numpy as np
from numpy.typing import NDArray

# Measured on presets 1-3: a fully visible disc covers 87-91 red pixels.
# 60 is about two thirds of that: a disc missing a third shifts its centre by ~1.5 px (~5 mm),
# whether the missing part is hidden by the arm or cut off by the image border.
MIN_RED_PIXELS = 60

# The median can be trusted only when most of its samples are valid.
MIN_VALID_FRACTION = 0.5

# Metres. Measured spread: ~1 mm inside the disc, up to ~1 cm at its edge pixels.
# The arm above the disc would read ~8 cm closer, well outside this margin.
MAX_DEPTH_SPREAD = 0.03
# Share of valid depths that must lie within MAX_DEPTH_SPREAD of the median.
MIN_CONSISTENT_FRACTION = 0.8


def recognize_disc(rgb_img: NDArray[np.uint8]
                   ) -> tuple[tuple[float, float], NDArray[np.bool_]] | None:
    """ image: (H, W, 3) uint8, channel order RGB.
    Returns (disk_center, mask): disk_center = (u, v) in pixels, mask = (H, W) bool
    of the red pixels. Returns None if unreliable. """
    if rgb_img.ndim != 3 or rgb_img.shape[2] != 3:
        raise ValueError(f'expected an (H, W, 3) image, got shape {rgb_img.shape}')

    R, G, B = (rgb_img[..., i].astype(int) for i in range(3))
    mask = (R > 100) & (R - G > 50) & (R - B > 50)
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
    pixels to be valid, and returns their median. Most valid depths must also lie
    close to that median. Returns None if unreliable. """
    if depth_img.shape != mask.shape:
        raise ValueError(f'depth shape {depth_img.shape} does not match mask shape {mask.shape}')

    disc_depths = depth_img[mask]
    valid_depths = disc_depths[np.isfinite(disc_depths) & (disc_depths > 0)]
    if valid_depths.size == 0 or valid_depths.size < MIN_VALID_FRACTION * disc_depths.size:
        return None             # too few valid depth readings on the disc

    depth = float(np.median(valid_depths))
    if np.mean(np.abs(valid_depths - depth) <= MAX_DEPTH_SPREAD) < MIN_CONSISTENT_FRACTION:
        return None             # depths too spread out: the mask mixes surfaces
    return depth


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
