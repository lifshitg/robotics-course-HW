import copy
import math
import time

import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2, needed even though it looks unused)
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from hw01_sim.arm_client import ArmClient

MIN_DEPTH = 0.1             # metres; anything closer is not a plausible surface
MAX_DEPTH = 3.0             # metres; the camera is ~1 m above the platform
MIN_VALID_DEPTH_FRACTION = 0.5
HOVER_HEIGHT = 0.08         # metres above the marker's top surface
MAX_OBSERVATION_AGE = 0.5   # simulated seconds
MAX_MOTION_ATTEMPTS = 3
TIP_TOLERANCE = 0.01        # metres; check_reach allows 0.02
TIP_SETTLE_TIME = 0.5       # simulated seconds to wait after a motion before measuring the tip

def main():
    # I handle Ctrl+C myself, so the ROS context is still alive to cancel an active motion.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("reach_node",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    logger = node.get_logger()
    bridge = cv_bridge.CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841  (must stay alive to fill the buffer)
    target_camera_publisher = node.create_publisher(PointStamped, "/hw01/target_camera", 10)
    target_base_publisher = node.create_publisher(PointStamped, "/hw01/target_base", 10)
    arm = ArmClient(node)

    latest_camera_info: CameraInfo = None
    latest_measured_tip: PoseStamped = None
    motion = None                   
    motion_attempts = 0
    hover_target: PointStamped = None
    verify_timer = None
    done = False

    def handle_motion_result(future):
        nonlocal motion, verify_timer
        result = future.result()
        logger.info(f"motion finished: success={result.success}, message={result.message}")
        if result.success:
            # check if the tip reached it's place after it's settled
            verify_timer = node.create_timer(TIP_SETTLE_TIME, verify_tip_position)
        else:
            motion = None 

    def verify_tip_position():
        nonlocal motion, done, verify_timer
        node.destroy_timer(verify_timer)  # one-shot
        verify_timer = None
        motion = None
        if latest_measured_tip is None:
            logger.warning("no /hw01/measured_tip data; cannot verify the motion, will retry")
            return
        measurement_age = (node.get_clock().now() - Time.from_msg(latest_measured_tip.header.stamp)).nanoseconds / 1e9
        if measurement_age > MAX_OBSERVATION_AGE:
            logger.warning(f"/hw01/measured_tip is {measurement_age:.2f} s old; cannot verify the motion, will retry")
            return
        tip = latest_measured_tip.pose.position
        error = math.dist((tip.x, tip.y, tip.z),
                          (hover_target.point.x, hover_target.point.y, hover_target.point.z))
        if error <= TIP_TOLERANCE:
            done = True
            logger.info(f"tip verified at ({tip.x:.3f}, {tip.y:.3f}, {tip.z:.3f}), {error * 1000:.1f} mm from the target")
        else:
            logger.warning(f"tip is {error * 1000:.1f} mm from the target; will retry from a fresh observation")

    def handle_measured_tip(measured_tip_message: PoseStamped):
        nonlocal latest_measured_tip
        latest_measured_tip = measured_tip_message

    def handle_rgb_depth_pair(rgb_message: Image, depth_message: Image):
        nonlocal motion, motion_attempts, hover_target
        if done:
            return  # job complete; the tip now hides the marker from the camera anyway
        now = node.get_clock().now()
        if now.nanoseconds == 0:
            logger.info("waiting for /clock", throttle_duration_sec=1.0)
            return
        observation_age = (now - Time.from_msg(rgb_message.header.stamp)).nanoseconds / 1e9
        if observation_age > MAX_OBSERVATION_AGE:
            logger.warning(f"stale observation ({observation_age:.2f} s old), skipping", throttle_duration_sec=1.0)
            return

        if latest_camera_info is None:
            logger.info("no camera info yet", throttle_duration_sec=1.0)
            return

        marker = get_marker_location(rgb_message)
        if marker is None:
            logger.info("no reliable marker detection", throttle_duration_sec=1.0)
            return
        (marker_x, marker_y), marker_mask = marker

        marker_depth = get_marker_depth(depth_message, marker_mask)
        if marker_depth is None:
            logger.info("no reliable marker depth", throttle_duration_sec=1.0)
            return

        marker_camera_point = pixel_to_camera_point(marker_x, marker_y, marker_depth, latest_camera_info, rgb_message)
        target_camera_publisher.publish(marker_camera_point)

        try:
            # Uses the point's header: source frame camera_optical_frame, at the image timestamp.
            marker_base = tf_buffer.transform(marker_camera_point, "base_link")
        except TransformException as error:
            logger.info(f"transform unavailable: {error}", throttle_duration_sec=1.0)
            return
        target_base_publisher.publish(marker_base)

        if motion is not None:
            return  # only one motion at a time
        if motion_attempts >= MAX_MOTION_ATTEMPTS:
            logger.error(f"giving up after {MAX_MOTION_ATTEMPTS} failed motions", throttle_duration_sec=5.0)
            return
        if not arm.ready():
            logger.info("motion helper not ready yet", throttle_duration_sec=1.0)
            return

        hover_target = copy.deepcopy(marker_base)
        hover_target.point.z += HOVER_HEIGHT
        motion_attempts += 1
        logger.info(f"marker at ({marker_base.point.x:.3f}, {marker_base.point.y:.3f}, {marker_base.point.z:.3f}) in base_link, "
                    f"sending tip to ({hover_target.point.x:.3f}, {hover_target.point.y:.3f}, {hover_target.point.z:.3f}) "
                    f"(attempt {motion_attempts})")
        motion = arm.move_to_point(hover_target)
        motion.add_done_callback(handle_motion_result)  # may run immediately if the goal was rejected

    def get_marker_location(rgb_message):
        cv2_image = bridge.imgmsg_to_cv2(rgb_message, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
        red_mask: cv2.typing.MatLike = cv2.inRange(hsv_image, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv_image, (170, 120, 70), (179, 255, 255))
        component_count, component_labels, component_stats, component_centroids = cv2.connectedComponentsWithStats(red_mask)

        def is_valid_marker(stat) -> bool:
            _, _, width, height, area = stat
            if area < 40:
                return False # Too small
            if area > 400:
                return False # Too big
            if area / (width * height) < 0.6:
                return False # A circle should occupy about pi/4 or 78% of it's bounding square, if the ratio is too small, the circle is probably obstructed
            return True
        valid_labels = [label for label in range(1, component_count) if is_valid_marker(component_stats[label])]

        if len(valid_labels) == 1:
            # Return a dot only if it's the only detected marker in the image
            marker_label = valid_labels[0]
            return component_centroids[marker_label], component_labels == marker_label

        return None

    def get_marker_depth(depth_message: Image, marker_mask):
        depth_image = bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
        if depth_message.encoding == '32FC1':
            depth_image = depth_image.astype(np.float32) # already metres
        elif depth_message.encoding == '16UC1':
            depth_image = depth_image.astype(np.float32) / 1000.0  # millimetres -> metres
        else:
            logger.warning(f"unsupported depth encoding {depth_message.encoding}", throttle_duration_sec=5.0)
            return None
        if depth_image.shape != marker_mask.shape:
            logger.warning("depth and RGB images are not aligned", throttle_duration_sec=5.0)
            return None

        marker_depths = depth_image[marker_mask]
        valid_depths_mask = np.isfinite(marker_depths) & (marker_depths > MIN_DEPTH) & (marker_depths < MAX_DEPTH)
        if valid_depths_mask.sum() < MIN_VALID_DEPTH_FRACTION * marker_depths.size:
            return None
        return float(np.median(marker_depths[valid_depths_mask]))

    def pixel_to_camera_point(pixel_x, pixel_y, depth, camera_info: CameraInfo, rgb_message: Image) -> PointStamped:
        focal_length_x, _, principal_point_x, _, focal_length_y, principal_point_y, _, _, _ = camera_info.k
        point = PointStamped()
        point.header = rgb_message.header  # image timestamp + camera_optical_frame
        point.point.x = float((pixel_x - principal_point_x) * depth / focal_length_x)
        point.point.y = float((pixel_y - principal_point_y) * depth / focal_length_y)
        point.point.z = depth
        return point

    def handle_camera_info(camera_info_message: CameraInfo):
        nonlocal latest_camera_info
        latest_camera_info = camera_info_message

    rgb_sub = message_filters.Subscriber(node, Image, "/camera/image_raw", qos_profile_sensor_data)
    depth_sub = message_filters.Subscriber(node, Image, "/camera/depth/image_raw", qos_profile_sensor_data)
    sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.05)
    sync.registerCallback(handle_rgb_depth_pair)
    node.create_subscription(CameraInfo, "/camera/camera_info", handle_camera_info, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, "/hw01/measured_tip", handle_measured_tip, 10)

    try:
        while rclpy.ok():
            # Short waits instead of rclpy.spin(): Python only sees Ctrl+C between waits,
            # and with no messages arriving (e.g. no simulation) spin() would wait forever.
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if motion is not None:
            logger.info("interrupted: cancelling the active motion")
            arm.cancel()
            deadline = time.monotonic() + 2.0
            while motion is not None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)  # let the cancel request and its reply go through
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
