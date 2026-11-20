"""Locate the red marker in synchronized RGB-D data and reach above it."""

import math
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from hw01_sim.arm_client import ArmClient


MAX_PAIR_DELTA = 0.05  # seconds of simulation time
MAX_OBSERVATION_AGE = 0.5
MAX_WALL_AGE = 2.0
MIN_DEPTH = 0.05  # metres, camera clipping range
MAX_DEPTH = 3.0


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MarkerReach(Node):
    def __init__(self):
        super().__init__(
            'hw01_marker_reach',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = ArmClient(self)
        self.camera_info = None
        self.last_rgb_wall = None
        self.last_depth_wall = None
        self.last_info_wall = None
        self.rgb_receipts = {}
        self.depth_receipts = {}
        self.motion = None
        self.completed = False
        self.last_goal_stamp = -1.0
        self.retry_after_wall = 0.0
        self.last_notice = {}

        self.camera_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data
        )
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data
        )
        # Register receipt tracking before the synchronizer invokes on_pair.
        self.rgb_sub.registerCallback(self.on_rgb_received)
        self.depth_sub.registerCallback(self.on_depth_received)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=MAX_PAIR_DELTA
        )
        self.synchronizer.registerCallback(self.on_pair)
        self.get_logger().info('Waiting for fresh RGB-D observations and the motion helper')

    def notice(self, message):
        now = time.monotonic()
        if now - self.last_notice.get(message, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.last_notice[message] = now

    def on_info(self, message):
        self.camera_info = message
        self.last_info_wall = time.monotonic()

    def record_receipt(self, receipts, message):
        now = time.monotonic()
        for stamp, received in list(receipts.items()):
            if now - received > MAX_WALL_AGE:
                del receipts[stamp]
        stamp = message.header.stamp
        receipts[(stamp.sec, stamp.nanosec)] = now
        return now

    def on_rgb_received(self, message):
        self.last_rgb_wall = self.record_receipt(self.rgb_receipts, message)

    def on_depth_received(self, message):
        self.last_depth_wall = self.record_receipt(self.depth_receipts, message)

    def fresh(self, stamps):
        wall_now = time.monotonic()
        if any(t is None or wall_now - t > MAX_WALL_AGE for t in
               (self.last_rgb_wall, self.last_depth_wall, self.last_info_wall)):
            return False
        for stamp, receipts in zip(stamps[:2], (self.rgb_receipts, self.depth_receipts)):
            received = receipts.get((stamp.sec, stamp.nanosec))
            if received is None or wall_now - received > MAX_WALL_AGE:
                return False
        sim_now = self.get_clock().now().nanoseconds * 1e-9
        for stamp in stamps:
            observed = stamp_seconds(stamp)
            age = sim_now - observed
            if observed <= 0.0 or age < 0.0 or age > MAX_OBSERVATION_AGE:
                return False
        return True

    def estimate(self, rgb, depth, info):
        if (rgb.header.frame_id != 'camera_optical_frame' or
                depth.header.frame_id != rgb.header.frame_id or
                info.header.frame_id != rgb.header.frame_id):
            raise ValueError('Camera frames do not agree with camera_optical_frame')
        if (rgb.width != depth.width or rgb.height != depth.height or
                info.width != rgb.width or info.height != rgb.height):
            raise ValueError('RGB, depth, and calibration dimensions do not match')
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            raise ValueError('Camera intrinsics are invalid')
        if depth.encoding != '32FC1':
            raise ValueError('Expected depth encoding 32FC1 in metres')

        image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Red crosses the hue wraparound in OpenCV's 0..179 hue scale.
        red = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 120, 60), (10, 255, 255)),
            cv2.inRange(hsv, (170, 120, 60), (179, 255, 255)),
        )
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            x, y, width, height = cv2.boundingRect(contour)
            if (area < 12 or perimeter <= 0 or x <= 0 or y <= 0 or
                    x + width >= rgb.width or y + height >= rgb.height):
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            aspect = min(width, height) / max(width, height)
            if circularity >= 0.45 and aspect >= 0.5:
                candidates.append((area, contour))
        candidates.sort(key=lambda item: item[0], reverse=True)
        if not candidates or (len(candidates) > 1 and candidates[1][0] >= 0.5 * candidates[0][0]):
            raise ValueError('Marker is missing, occluded, or ambiguous')

        region = np.zeros(red.shape, dtype=np.uint8)
        cv2.drawContours(region, [candidates[0][1]], -1, 255, thickness=cv2.FILLED)
        # Use detected red pixels inside the contour, eroded to avoid mixed edges.
        region = cv2.bitwise_and(region, red)
        region = cv2.erode(region, np.ones((3, 3), np.uint8), iterations=1) > 0
        valid = region & np.isfinite(depth_image) & (depth_image > MIN_DEPTH) & (depth_image < MAX_DEPTH)
        if np.count_nonzero(valid) < max(5, math.ceil(0.6 * np.count_nonzero(region))):
            raise ValueError('Insufficient valid marker depth')
        values = depth_image[valid].astype(np.float64)
        median_depth = float(np.median(values))
        mad = float(np.median(np.abs(values - median_depth)))
        if mad > 0.02:
            raise ValueError('Marker depth is inconsistent')
        # Discard remaining depth outliers, then use the corresponding pixel centroid.
        inliers = valid & (np.abs(depth_image - median_depth) <= max(0.01, 3.0 * mad))
        rows, cols = np.nonzero(inliers)
        if len(rows) < 5 or len(rows) < 0.6 * np.count_nonzero(region):
            raise ValueError('Insufficient consistent marker depth')
        z = float(np.median(depth_image[inliers]))
        u, v = float(np.mean(cols)), float(np.mean(rows))
        point = PointStamped()
        point.header = rgb.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def on_pair(self, rgb, depth):
        if self.completed or self.motion is not None or time.monotonic() < self.retry_after_wall:
            return
        info = self.camera_info
        if info is None:
            self.notice('Waiting for camera calibration')
            return
        if abs(stamp_seconds(rgb.header.stamp) - stamp_seconds(depth.header.stamp)) > MAX_PAIR_DELTA:
            return
        stamps = (rgb.header.stamp, depth.header.stamp, info.header.stamp)
        if not self.fresh(stamps) or stamp_seconds(rgb.header.stamp) <= self.last_goal_stamp:
            self.notice('Waiting for fresh synchronized camera data')
            return
        try:
            camera_point = self.estimate(rgb, depth, info)
            self.camera_pub.publish(camera_point)
            transform = self.tf_buffer.lookup_transform(
                'base_link', camera_point.header.frame_id,
                Time.from_msg(camera_point.header.stamp), timeout=Duration(seconds=0.0),
            )
            base_point = do_transform_point(camera_point, transform)
            # Keep the observation timestamp, even for a static transform.
            base_point.header.stamp = camera_point.header.stamp
            self.base_pub.publish(base_point)
            if not all(math.isfinite(v) for v in
                       (base_point.point.x, base_point.point.y, base_point.point.z)):
                raise ValueError('Transformed marker position is not finite')
            if not self.fresh(stamps):
                raise ValueError('Observation became stale before sending the goal')
            if not self.arm.ready():
                self.notice('Waiting for the motion helper')
                return
            goal = PointStamped()
            goal.header = base_point.header
            goal.point.x = base_point.point.x
            goal.point.y = base_point.point.y
            goal.point.z = base_point.point.z + 0.08
            self.last_goal_stamp = stamp_seconds(rgb.header.stamp)
            self.motion = self.arm.move_to_point(goal)
            self.motion.add_done_callback(self.on_motion_done)
            self.get_logger().info(
                f'Sent hover goal in base_link: ({goal.point.x:.3f}, '
                f'{goal.point.y:.3f}, {goal.point.z:.3f}) m'
            )
        except (ValueError, CvBridgeError, cv2.error, TransformException) as error:
            self.notice(f'No reliable target: {error}')

    def on_motion_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.completed = True
                self.get_logger().info(f'Motion succeeded: {result.message}')
            else:
                self.retry_after_wall = time.monotonic() + 2.0
                self.get_logger().error(f'Motion failed: {result.message}; waiting for a fresh observation')
        except Exception as error:
            self.retry_after_wall = time.monotonic() + 2.0
            self.get_logger().error(f'Motion failed: {error}; waiting for a fresh observation')
        finally:
            self.motion = None


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MarkerReach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.cancel()
        # Allow cancellation of an accepted or still-pending action to be sent.
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and node.motion is not None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
