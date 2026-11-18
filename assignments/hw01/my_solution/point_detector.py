#!/usr/bin/env python3
"""HW1 point detector: red marker (RGB + depth) -> 3D point in base_link.

Pipeline per synchronized RGB/depth pair:
  1. Segment red pixels in HSV, keep the largest blob (reject if too small).
  2. Pick a robust depth: median of valid depth values inside the blob mask.
  3. Back-project the blob centroid with the CameraInfo intrinsics into
     camera_optical_frame (metres).
  4. Transform with TF2 at the *image timestamp* into base_link.
  5. Publish /hw01/target_camera and /hw01/target_base (before the hover offset).

Nothing is published when the detection, depth, TF or data freshness is bad, and
no earlier target is ever reused.

Run (after sourcing ROS + the hw01 environment, simulation running):
    python3 point_detector.py
"""
import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

BASE_FRAME = 'base_link'


class PointDetector(Node):
    """ROS 2 node that turns RGB + depth images into a marker point in ``base_link``.

    ROS parameters:
        sync_slop (float): Max RGB/depth timestamp difference to pair them, in seconds.
        max_age (float): Max image age in simulated seconds before it is considered stale.
        min_area_px (int): Minimum blob area in pixels to accept a detection.
        min_valid_depth_frac (float): Minimum fraction of blob pixels with valid depth.
        depth_min (float): Smallest accepted depth, in metres (exclusive).
        depth_max (float): Largest accepted depth, in metres (exclusive).

    Subscriptions:
        /camera/image_raw (sensor_msgs/Image),
        /camera/depth/image_raw (sensor_msgs/Image),
        /camera/camera_info (sensor_msgs/CameraInfo), plus /tf and /tf_static via TF2.

    Publications:
        /hw01/target_camera (geometry_msgs/PointStamped): marker in the optical frame.
        /hw01/target_base (geometry_msgs/PointStamped): marker in ``base_link``.
    """

    def __init__(self) -> None:
        """Declare parameters, set up TF2, publishers, and the synchronized subscribers."""
        super().__init__('point_detector', parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.declare_parameter('sync_slop', 0.05)         # s, RGB/depth match
        self.declare_parameter('max_age', 0.5)            # s sim time, staleness
        self.declare_parameter('min_area_px', 25)         # reject tiny blobs
        self.declare_parameter('min_valid_depth_frac', 0.3)
        self.declare_parameter('depth_min', 0.05)         # m
        self.declare_parameter('depth_max', 5.0)          # m

        self.camera_info: CameraInfo | None = None

        self.tf_buffer: tf2_ros.Buffer = tf2_ros.Buffer()
        self.tf_listener: tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_cam = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.pub_base = self.create_publisher(PointStamped, '/hw01/target_base', 10)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)

        rgb_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=self.get_parameter('sync_slop').value)
        self.sync.registerCallback(self.on_images)


    def on_info(self, msg: CameraInfo) -> None:
        """Store the latest camera calibration.

        Args:
            msg: Camera calibration message; ``msg.k`` holds the 3x3 intrinsic matrix.
        """
        self.camera_info = msg


    @staticmethod
    def decode_rgb(msg: Image) -> np.ndarray:
        """Decode an RGB image message into an OpenCV-style BGR array.

        Args:
            msg: Image with encoding ``rgb8`` or ``bgr8``. Row padding (``step``) is honored.

        Returns:
            Array of shape (height, width, 3), dtype uint8, in BGR channel order.

        Raises:
            ValueError: If the encoding is not ``rgb8`` or ``bgr8``.
        """
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'unsupported RGB encoding {msg.encoding!r}')
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img


    @staticmethod
    def decode_depth(msg: Image) -> np.ndarray:
        """Decode a depth image message into metres.

        Args:
            msg: Depth image with encoding ``32FC1``.

        Returns:
            array of shape (height, width), dtype float32, depth in metres. May
            contain NaN/inf/zero values, which the caller must filter.
        """
        rows = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)
        return rows[:, :msg.width]


    def detect_red(self, bgr: np.ndarray) -> tuple[np.ndarray, tuple[float, float]] | None:
        """Find the largest red blob in an image.

        Args:
            bgr: Image of shape (height, width, 3), dtype uint8, BGR channel order.

        Returns:
            ``(blob_mask, (u, v))`` where ``blob_mask`` is a boolean (height, width) array
            that is True on the blob's pixels and ``(u, v)`` is its centroid in pixels
            (u = column, v = row). Returns None if no red pixels exist or the largest
            blob is smaller than ``min_area_px``.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Red wraps around the hue axis so logical OR is needed
        mask = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255)) | cv2.inRange(hsv, (170, 100, 60), (179, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        if count <= 1:
            return None
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[best, cv2.CC_STAT_AREA] < self.get_parameter('min_area_px').value:
            return None
        return labels == best, tuple(centroids[best])


    def pick_depth(self, depth: np.ndarray, blob: np.ndarray) -> float | None:
        """Choose a robust depth for the marker.

        Args:
            depth: Depth image in metres, shape (height, width), dtype float32.
            blob: Boolean mask of shape (height, width) selecting the marker's pixels.

        Returns:
            Median of the valid depth values inside ``blob``, in metres. Values that are
            non-finite or outside (``depth_min``, ``depth_max``) are discarded. Returns
            None if fewer than ``min_valid_depth_frac`` of the blob's pixels are valid.
        """
        values = depth[blob]
        valid = values[np.isfinite(values)
                       & (values > self.get_parameter('depth_min').value)
                       & (values < self.get_parameter('depth_max').value)]
        if valid.size == 0 or \
                valid.size < self.get_parameter('min_valid_depth_frac').value * values.size:
            return None
        return float(np.median(valid))


    def on_images(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Process one synchronized RGB/depth pair and publish the marker position.

        Skips the frame (publishing nothing) if calibration is missing, the image is stale,
        decoding fails, no reliable marker or valid depth is found, or TF is unavailable.

        Args:
            rgb_msg: RGB image; its header stamp and frame_id are used for the output point.
            depth_msg: Aligned depth image paired with ``rgb_msg`` by timestamp.
        """
        log =self.get_logger()

        if self.camera_info is None:
            log.warn('No CameraInfo yet.', throttle_duration_sec=2.0)
            return

        age = (self.get_clock().now() - Time.from_msg(rgb_msg.header.stamp)).nanoseconds * 1e-9
        if age > self.get_parameter('max_age').value:
            log.warn(f'Stale image ({age:.2f} s old); skipping.', throttle_duration_sec=2.0)
            return

        try:
            bgr = self.decode_rgb(rgb_msg)
            depth = self.decode_depth(depth_msg)
        except ValueError as exc:
            log.error(str(exc), throttle_duration_sec=5.0)
            return

        found = self.detect_red(bgr)
        if found is None:
            log.info('No reliable red marker detected.', throttle_duration_sec=2.0)
            return
        blob, (u, v) = found

        z = self.pick_depth(depth, blob)
        if z is None:
            log.warn('Marker found but depth is invalid; skipping.', throttle_duration_sec=2.0)
            return

        # Back-project with intrinsics
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        cam_pt = PointStamped()
        cam_pt.header.stamp = rgb_msg.header.stamp          # image time, not now()
        cam_pt.header.frame_id = rgb_msg.header.frame_id    # camera_optical_frame
        cam_pt.point.x = (u - cx) * z / fx
        cam_pt.point.y = (v - cy) * z / fy
        cam_pt.point.z = z

        try:
            base_pt = self.tf_buffer.transform(cam_pt, BASE_FRAME, timeout=Duration(seconds=0.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            log.warn(f'TF unavailable at image time: {exc}', throttle_duration_sec=2.0)
            return

        self.pub_cam.publish(cam_pt)
        self.pub_base.publish(base_pt)
        log.info(
            f'pixel=({u:.1f},{v:.1f}) depth={z:.3f} m  '
            f'base=({base_pt.point.x:.3f}, {base_pt.point.y:.3f}, {base_pt.point.z:.3f})',
            throttle_duration_sec=1.0)


def main():
    """Initialize rclpy, spin the detector until interrupted, then shut down cleanly."""
    rclpy.init()
    node = PointDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()