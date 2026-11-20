"""Offline tests for detection, depth selection and back-projection (no ROS needed)."""
import math

import cv2
import numpy as np
import pytest

from hw01_solution.perception import (backproject, depth_to_metres, detect_marker,
                                      select_depth)

W, H = 320, 240
FX = FY = (W / 2) / math.tan(1.0471975512 / 2)   # 60 deg horizontal FOV, like the sim camera
K = [FX, 0, W / 2, 0, FY, H / 2, 0, 0, 1]
PLATFORM_DEPTH, MARKER_DEPTH, RADIUS_M = 0.95, 0.94, 0.018


def scene(u=200.0, v=90.0, colour=(0, 0, 210), occlude=None, extra_blob=None):
    """Grey platform, yellow arm bar and a red disk at pixel (u, v)."""
    bgr = np.full((H, W, 3), (200, 200, 200), np.uint8)
    depth = np.full((H, W), PLATFORM_DEPTH, np.float32)
    cv2.rectangle(bgr, (20, 150), (140, 170), (25, 178, 242), -1)       # yellow link
    r = RADIUS_M * FX / MARKER_DEPTH
    shift = 4  # sub-pixel drawing
    centre = (int(round(u * 2**shift)), int(round(v * 2**shift)))
    disk = np.zeros((H, W), np.uint8)
    cv2.circle(disk, centre, int(round(r * 2**shift)), 255, -1, cv2.LINE_8, shift)
    bgr[disk > 0] = colour
    depth[disk > 0] = MARKER_DEPTH
    if occlude is not None:
        x0, y0, x1, y1 = occlude
        bgr[y0:y1, x0:x1] = (230, 90, 40)   # blue link
        depth[y0:y1, x0:x1] = 0.80
    if extra_blob is not None:
        cv2.circle(bgr, extra_blob, 5, colour, -1)
    return bgr, depth


def test_detects_marker_centroid():
    bgr, _ = scene(200.3, 90.7)
    det, reason = detect_marker(bgr)
    assert det is not None, reason
    assert abs(det.u - 200.3) < 0.5 and abs(det.v - 90.7) < 0.5


@pytest.mark.parametrize('colour', [(0, 0, 255), (0, 0, 120), (20, 20, 180)])
def test_red_shades_detected(colour):
    det, reason = detect_marker(scene(colour=colour)[0])
    assert det is not None, reason


def test_yellow_arm_alone_is_not_red():
    bgr, _ = scene()
    bgr[:] = (200, 200, 200)
    cv2.rectangle(bgr, (20, 150), (140, 170), (25, 178, 242), -1)
    det, reason = detect_marker(bgr)
    assert det is None and 'no red' in reason


def test_rejects_heavily_occluded_marker():
    u, v = 200, 90
    det, reason = detect_marker(scene(u, v, occlude=(u - 10, v - 10, u + 3, v + 10))[0])
    assert det is None, 'a sliver of the marker must not be trusted'


def test_rejects_ambiguous_blobs():
    det, reason = detect_marker(scene(extra_blob=(60, 60))[0])
    assert det is None and 'ambiguous' in reason


def test_rejects_marker_on_border():
    det, reason = detect_marker(scene(u=2, v=100)[0])
    assert det is None


def test_depth_median_ignores_invalid_and_edges():
    bgr, depth = scene()
    det, _ = detect_marker(bgr)
    noisy = depth.copy()
    ys, xs = np.nonzero(det.mask)
    noisy[ys[0], xs[0]] = np.nan
    noisy[ys[1], xs[1]] = np.inf
    noisy[ys[2], xs[2]] = 0.0
    noisy[ys[3], xs[3]] = -1.0
    z, reason = select_depth(noisy, det.mask)
    assert z == pytest.approx(MARKER_DEPTH, abs=1e-6), reason


def test_depth_all_invalid_rejected():
    bgr, depth = scene()
    det, _ = detect_marker(bgr)
    depth[det.mask] = np.nan
    z, reason = select_depth(depth, det.mask)
    assert z is None and 'valid' in reason


def test_depth_out_of_range_rejected():
    bgr, depth = scene()
    det, _ = detect_marker(bgr)
    depth[det.mask] = 5.0
    assert select_depth(depth, det.mask)[0] is None


def test_backproject_principal_point_and_offset():
    assert backproject(W / 2, H / 2, 1.0, K) == pytest.approx((0, 0, 1.0))
    x, y, z = backproject(W / 2 + FX * 0.1, H / 2 - FY * 0.2, 2.0, K)
    assert (x, y, z) == pytest.approx((0.2, -0.4, 2.0))


def test_backproject_rejects_bad_intrinsics():
    with pytest.raises(ValueError):
        backproject(1, 1, 1, [0, 0, 0, 0, 0, 0, 0, 0, 1])


def test_end_to_end_reconstruction_error_small():
    true = (0.05, -0.12, MARKER_DEPTH)              # point in the optical frame
    u = FX * true[0] / true[2] + W / 2
    v = FY * true[1] / true[2] + H / 2
    bgr, depth = scene(u, v)
    det, reason = detect_marker(bgr)
    assert det is not None, reason
    z, reason = select_depth(depth, det.mask)
    est = backproject(det.u, det.v, z, K)
    assert math.dist(est, true) < 0.003


def test_depth_encodings():
    mm = np.array([[0, 940]], np.uint16)
    out = depth_to_metres(mm, '16UC1')
    assert np.isnan(out[0, 0]) and out[0, 1] == pytest.approx(0.94)
    with pytest.raises(ValueError):
        depth_to_metres(mm, 'mono8')
