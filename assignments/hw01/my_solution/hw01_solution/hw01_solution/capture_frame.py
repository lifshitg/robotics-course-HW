#!/usr/bin/env python3
"""Debug tool: save one synchronized RGB/depth pair, calibration and the camera TF.

Usage: ros2 run hw01_solution capture_frame [output_dir]
Writes rgb.png, depth.npy, camera_info.json, tf_base_optical.json, detection.png.
"""
import json
import os
import sys

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .perception import depth_to_metres, detect_marker, select_depth


class Capture(Node):
    def __init__(self, out_dir):
        super().__init__('hw01_capture', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = out_dir
        self.bridge = CvBridge()
        self.info = None
        self.done = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        rgb = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        dep = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 10, 0.05)
        self.sync.registerCallback(self.on_pair)

    def on_pair(self, rgb_msg, depth_msg):
        if self.done or self.info is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('base_link', rgb_msg.header.frame_id, Time())
        except TransformException:
            return
        os.makedirs(self.out, exist_ok=True)
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = depth_to_metres(self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'),
                                depth_msg.encoding)
        cv2.imwrite(os.path.join(self.out, 'rgb.png'), bgr)
        np.save(os.path.join(self.out, 'depth.npy'), depth)
        info = {'frame_id': self.info.header.frame_id, 'width': self.info.width,
                'height': self.info.height, 'k': list(self.info.k), 'd': list(self.info.d),
                'distortion_model': self.info.distortion_model,
                'rgb_encoding': rgb_msg.encoding, 'depth_encoding': depth_msg.encoding,
                'rgb_stamp': [rgb_msg.header.stamp.sec, rgb_msg.header.stamp.nanosec],
                'depth_stamp': [depth_msg.header.stamp.sec, depth_msg.header.stamp.nanosec]}
        with open(os.path.join(self.out, 'camera_info.json'), 'w') as f:
            json.dump(info, f, indent=2)
        t, r = tf.transform.translation, tf.transform.rotation
        with open(os.path.join(self.out, 'tf_base_optical.json'), 'w') as f:
            json.dump({'parent': 'base_link', 'child': rgb_msg.header.frame_id,
                       'translation': [t.x, t.y, t.z], 'rotation_xyzw': [r.x, r.y, r.z, r.w]},
                      f, indent=2)
        det, reason = detect_marker(bgr)
        vis = bgr.copy()
        if det is not None:
            vis[det.mask] = (0, 255, 0)
            z, why = select_depth(depth, det.mask)
            print(f'Detection u={det.u:.2f} v={det.v:.2f} area={det.area} depth={z} {why or ""}')
        else:
            print(f'No detection: {reason}')
        cv2.imwrite(os.path.join(self.out, 'detection.png'), vis)
        finite = depth[np.isfinite(depth)]
        print(f'Saved frame to {self.out}: {bgr.shape[1]}x{bgr.shape[0]} {rgb_msg.encoding}, '
              f'depth {depth_msg.encoding} range [{finite.min():.3f}, {finite.max():.3f}] m')
        self.done = True


def main():
    rclpy.init()
    out = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 'hw01_frame'
    node = Capture(os.path.abspath(out))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
