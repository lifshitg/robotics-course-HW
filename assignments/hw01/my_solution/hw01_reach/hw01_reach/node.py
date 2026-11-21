"""Synchronize RGB-D, transform a marker estimate, and send one hover goal."""
from copy import deepcopy
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from hw01_sim.arm_client import ArmClient
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .perception import ObservationError, check_timing, marker_point


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ReachMarker(Node):
    def __init__(self):
        super().__init__('hw01_reach_marker')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.bridge = CvBridge()
        self.tf = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf, self)
        self.arm = ArmClient(self)
        self.info = None
        self.motion = None
        self.exit_code = None
        self.started = time.monotonic()
        self.camera_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.rgb_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile_sensor_data)
        self.depth_sub = Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_images)
        self.get_logger().info('Waiting for calibrated RGB-D and timestamped TF; one goal per run.')

    def on_info(self, message):
        self.info = message

    def on_images(self, rgb, depth):
        if self.motion is not None or self.exit_code is not None:
            return
        received = time.monotonic()
        try:
            self.check_times(rgb, depth, received)
            info = self.info
            if info is None:
                raise ObservationError('Waiting for camera calibration')
            if not (rgb.header.frame_id == depth.header.frame_id == info.header.frame_id
                    == 'camera_optical_frame'):
                raise ObservationError('RGB, depth, and calibration must use camera_optical_frame')
            if (rgb.width, rgb.height) != (info.width, info.height):
                raise ObservationError('Camera calibration dimensions do not match RGB')
            if rgb.encoding not in ('rgb8', 'bgr8') or depth.encoding != '32FC1':
                raise ObservationError('Expected rgb8/bgr8 RGB and 32FC1 depth encodings')
            if info.distortion_model != 'plumb_bob' or any(info.d):
                raise ObservationError('This pinhole implementation requires undistorted images')
            bgr_image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            camera = PointStamped()
            camera.header = rgb.header
            camera.point.x, camera.point.y, camera.point.z = marker_point(
                bgr_image, depth_image, info.k)
            transform = self.tf.lookup_transform(
                'base_link', camera.header.frame_id, Time.from_msg(camera.header.stamp))
            base = do_transform_point(camera, transform)
            base.header.stamp = camera.header.stamp
            self.check_times(rgb, depth, received)
            self.camera_pub.publish(camera)
            self.base_pub.publish(base)
            if not self.arm.ready():
                raise ObservationError('Waiting for the motion helper action server')
            hover = deepcopy(base)
            hover.point.z += 0.08
            self.get_logger().info(
                f'Marker in base_link: ({base.point.x:.4f}, {base.point.y:.4f}, '
                f'{base.point.z:.4f}) m; sending hover goal 0.08 m above it.')
            self.motion = self.arm.move_to_point(hover)
            self.motion.add_done_callback(self.on_motion_done)
        except (ObservationError, CvBridgeError, cv2.error, TransformException) as error:
            self.get_logger().warning(f'Not sending a goal: {error}', throttle_duration_sec=5.0)

    def check_times(self, rgb, depth, received):
        check_timing(
            stamp_seconds(rgb.header.stamp), stamp_seconds(depth.header.stamp),
            self.get_clock().now().nanoseconds * 1e-9, time.monotonic() - received)

    def on_motion_done(self, future):
        result = future.result()
        self.exit_code = 0 if result.success else 1
        if result.success:
            self.get_logger().info(f'SUCCESS: {result.message}')
        else:
            self.get_logger().error(f'FAILURE: {result.message}')

    def check_timeout(self):
        if self.motion is not None:
            return
        if time.monotonic() - self.started >= 120.0:
            self.get_logger().error('No safe goal could be sent within 120 wall-clock seconds.')
            self.exit_code = 1


def main(args=None):
    # Keep the ROS context alive during Ctrl+C so cancellation can be transmitted.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ReachMarker()
    interrupted = None

    def request_stop(signum, _frame):
        nonlocal interrupted
        interrupted = signum

    previous_int = signal.signal(signal.SIGINT, request_stop)
    previous_term = signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and node.exit_code is None and interrupted is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_timeout()
        if interrupted is not None:
            node.get_logger().info('Interrupted; cancelling pending motion.')
            return 128 + interrupted
        return node.exit_code if node.exit_code is not None else 1
    finally:
        if node.arm.busy and rclpy.ok():
            node.arm.cancel()
            deadline = time.monotonic() + 2.0
            while node.arm.busy and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.arm.busy:
                node.get_logger().warning('Cancellation sent but not confirmed before shutdown.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
