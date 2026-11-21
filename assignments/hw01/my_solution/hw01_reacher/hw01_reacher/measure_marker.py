"""Measure how the red marker looks to the camera, to justify the detection thresholds.

Run while the simulation is running (one preset at a time):
    ros2 run hw01_reacher measure_marker              # print the measurements
    ros2 run hw01_reacher measure_marker preset1      # also save preset1_rgb.png and preset1_mask.png
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .helpers_no_spread import recognize_disc, red_mask


def receive_latest(node, seconds=3.0):
    """Collect the latest RGB, depth and camera_info messages."""
    latest = {}
    for key, msg_type, topic in (('rgb', Image, '/camera/image_raw'),
                                 ('depth', Image, '/camera/depth/image_raw'),
                                 ('info', CameraInfo, '/camera/camera_info')):
        node.create_subscription(msg_type, topic, lambda m, k=key: latest.__setitem__(k, m),
                                 qos_profile_sensor_data)
    deadline = time.monotonic() + 30
    while len(latest) < 3 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    settle = time.monotonic() + seconds          # skip the first frames after startup
    while time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)
    return latest


def main():
    rclpy.init()
    node = Node('measure_marker')
    msgs = receive_latest(node)
    node.destroy_node()
    rclpy.shutdown()
    if len(msgs) < 3:
        print(f'missing topics, received only: {sorted(msgs)}')
        return 1

    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(msgs['rgb'], desired_encoding='rgb8')
    depth = bridge.imgmsg_to_cv2(msgs['depth'], desired_encoding='passthrough')
    k = msgs['info'].k
    print(f'encodings: rgb={msgs["rgb"].encoding}, depth={msgs["depth"].encoding}, size={rgb.shape[1]}x{rgb.shape[0]}')
    print(f'calibration: fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.1f} cy={k[5]:.1f}')

    detection = recognize_disc(rgb)
    if detection is None:
        print('marker not detected')
        return 1
    (u, v), mask = detection
    n_blobs = cv2.connectedComponents(red_mask(rgb).astype(np.uint8))[0] - 1
    print(f'marker: {int(mask.sum())} pixels, {n_blobs} red blob(s) in the image, '
          f'centre (u, v) = ({u:.1f}, {v:.1f})')

    marker_depths = depth[mask]
    valid = marker_depths[np.isfinite(marker_depths) & (marker_depths > 0)]
    print(f'depth on marker: {valid.size}/{marker_depths.size} valid, '
          f'min/median/max = {valid.min():.3f} / {np.median(valid):.3f} / {valid.max():.3f} m')

    R, G, B = (rgb[..., i].astype(int) for i in range(3))
    for name, region in (('marker', mask), ('rest of image', ~mask)):
        print(f'{name:14s} R: {R[region].min():3d}-{R[region].max():3d}   '
              f'R-G: {(R - G)[region].min():4d}-{(R - G)[region].max():4d}   '
              f'R-B: {(R - B)[region].min():4d}-{(R - B)[region].max():4d}')

    if len(sys.argv) > 1:
        cv2.imwrite(f'{sys.argv[1]}_rgb.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'{sys.argv[1]}_mask.png', mask.astype(np.uint8) * 255)
        print(f'saved {sys.argv[1]}_rgb.png and {sys.argv[1]}_mask.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
