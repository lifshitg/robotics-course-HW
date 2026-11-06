"""Find the red marker in RGB images."""

import numpy as np
from numpy.typing import NDArray


def make_red_mask(rgb_pixels: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return 255 where a pixel is clearly red (R >= 150, G <= 100, B <= 100)."""
    red, green, blue = (rgb_pixels[:, :, i] for i in range(3))
    is_red = (red >= 150) & (green <= 100) & (blue <= 100)
    return is_red.astype(np.uint8) * np.uint8(255)
