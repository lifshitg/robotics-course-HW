#!/usr/bin/env python3
"""HW1: find the red marker with the RGB-D camera and move the arm tip above it."""
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  (lets tf2 transform PointStamped messages)

from hw01_sim.arm_client import ArmClient

MIN_RED_PIXELS = 30              # full marker is ~90 px; fewer means mostly hidden, centre unreliable
MIN_DEPTH, MAX_DEPTH = 0.05, 3.0  # camera clip range, metres
HOVER = 0.08                     # tip goes 8 cm above the marker top
MAX_AGE = 0.5                    # observations older than this (sim seconds) are stale


def find_red_pixels(rgb):
    """Return the row and column indices of strongly red pixels."""
    r, g, b = (rgb[:, :, i].astype(int) for i in range(3))
    return np.nonzero((r > 100) & (g < 80) & (b < 80))


class ReachMarker(Node):
    def __init__(self):
        super().__init__('hw01_reach')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.bridge = CvBridge()
        self.arm = ArmClient(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.motion = None
        self.last_data = time.monotonic()
        self.pub_camera = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.pub_base = self.create_publisher(PointStamped, '/hw01/target_base', 10)

        # Camera topics are best-effort, so the subscribers must use sensor-data QoS.
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        rgb_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw',
                               qos_profile=qos_profile_sensor_data)
        # Calls on_images only with an RGB and a depth image whose stamps differ by <= 0.05 s.
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_images)

    def warn(self, text):
        self.get_logger().warning(text, throttle_duration_sec=2.0)

    def on_info(self, info):
        self.camera_info = info

    def on_images(self, rgb_msg, depth_msg):
        self.last_data = time.monotonic()
        age = (self.get_clock().now() - Time.from_msg(rgb_msg.header.stamp)).nanoseconds / 1e9
        if abs(age) > MAX_AGE:
            self.warn(f'Stale observation ({age:.2f} s old), skipping')
            return
        point_camera = self.locate_marker(rgb_msg, depth_msg)
        if point_camera is None:
            return
        point_base = self.to_base(point_camera)
        if point_base is None:
            return
        self.pub_camera.publish(point_camera)
        self.pub_base.publish(point_base)
        if self.motion is None:
            self.send_goal(point_base)

    def locate_marker(self, rgb_msg, depth_msg):
        """Return the marker centre as a PointStamped in the camera optical frame, or None."""
        if self.camera_info is None or self.camera_info.k[0] <= 0:
            self.warn('No camera calibration yet')
            return None
        if depth_msg.encoding != '32FC1':
            self.warn(f'Unexpected depth encoding {depth_msg.encoding}')
            return None
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')  # handles bgr8 too
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if rgb.shape[:2] != depth.shape:
            self.warn('RGB and depth sizes differ')
            return None

        rows, cols = find_red_pixels(rgb)
        if len(rows) < MIN_RED_PIXELS:
            self.warn(f'Marker not found or occluded ({len(rows)} red pixels)')
            return None
        depths = depth[rows, cols]
        depths = depths[np.isfinite(depths) & (depths > MIN_DEPTH) & (depths < MAX_DEPTH)]
        if len(depths) < MIN_RED_PIXELS:
            self.warn('Not enough valid depth values on the marker')
            return None

        # Pixel (u, v) = centre of the red pixels; Z = median depth (robust to edge pixels).
        u, v = cols.mean(), rows.mean()
        z = float(np.median(depths))
        # Pinhole model: u = fx * X / Z + cx  ->  X = (u - cx) * Z / fx  (same for Y with v).
        fx, cx = self.camera_info.k[0], self.camera_info.k[2]
        fy, cy = self.camera_info.k[4], self.camera_info.k[5]
        point = PointStamped()
        point.header = rgb_msg.header  # camera_optical_frame + image timestamp
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def to_base(self, point_camera):
        """Transform the point into base_link with TF2, at the image timestamp."""
        try:
            return self.tf_buffer.transform(point_camera, 'base_link')
        except TransformException as error:
            self.warn(f'Transform not available yet: {error}')
            return None

    def send_goal(self, point_base):
        if not self.arm.ready():
            self.warn('Motion helper not ready yet')
            return
        goal = PointStamped()
        goal.header = point_base.header
        goal.point.x = point_base.point.x
        goal.point.y = point_base.point.y
        goal.point.z = point_base.point.z + HOVER
        p = goal.point
        self.get_logger().info(f'Marker found, moving tip to ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) in base_link')
        self.motion = self.arm.move_to_point(goal)


def main():
    # Handle Ctrl+C ourselves so ROS stays alive long enough to send the cancel request.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ReachMarker()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - node.last_data > 2.0:
                node.warn('No fresh camera data for 2 seconds')
            if node.motion is not None and node.motion.done():
                result = node.motion.result()
                print(f'{"SUCCESS" if result.success else "FAILED"}: {result.message}')
                return 0 if result.success else 1
        return 1
    except KeyboardInterrupt:
        print('Interrupted, cancelling motion')
        return 1
    finally:
        node.arm.cancel()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
