import copy
import time

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import Buffer, TransformListener, TransformException
from hw01_sim.arm_client import ArmClient


class ReachMarker(Node):

    def __init__(self):
        super().__init__('reach_marker')

        self.bridge = CvBridge()
        self.arm = ArmClient(self)

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb = None
        self.depth = None
        self.info = None
        self.future = None
        self.success = False
        self.stopping = False
        self.last_attempt = -float('inf')

        self.create_subscription(
            Image, '/camera/image_raw',
            self.rgb_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.info_callback, qos_profile_sensor_data
        )

        self.camera_pub = self.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )

        self.base_pub = self.create_publisher(
            PointStamped, '/hw01/target_base', 10
        )

        self.get_logger().info('Waiting for camera data')

    def rgb_callback(self, msg):
        self.rgb = (msg, time.monotonic())
        self.try_process()

    def depth_callback(self, msg):
        self.depth = (msg, time.monotonic())
        self.try_process()

    def info_callback(self, msg):
        self.info = msg

    @staticmethod
    def timestamp(msg):
        stamp = msg.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def fresh(self, rgb, depth, rgb_wall, depth_wall):
        now = self.get_clock().now().nanoseconds * 1e-9
        wall_now = time.monotonic()

        return (
            -0.05 <= now - self.timestamp(rgb) <= 0.5
            and -0.05 <= now - self.timestamp(depth) <= 0.5
            and wall_now - rgb_wall <= 2.0
            and wall_now - depth_wall <= 2.0
        )

    def try_process(self):
        if self.stopping or self.info is None:
            return

        if self.rgb is None or self.depth is None:
            return

        rgb, rgb_wall = self.rgb
        depth, depth_wall = self.depth

        # Match RGB and depth by simulation timestamp.
        if abs(self.timestamp(rgb) - self.timestamp(depth)) > 0.05:
            return

        self.rgb = None
        self.depth = None

        if not self.fresh(rgb, depth, rgb_wall, depth_wall):
            return

        if rgb.header.frame_id != self.info.header.frame_id:
            return

        if depth.header.frame_id != rgb.header.frame_id:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(
                depth, 'passthrough'
            )
        except CvBridgeError as error:
            self.get_logger().warning(str(error))
            return

        if depth.encoding != '32FC1':
            return

        if image.shape[:2] != depth_image.shape:
            return

        if image.shape[:2] != (self.info.height, self.info.width):
            return

        # Detect the red marker using HSV color segmentation.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, (0, 100, 60), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 60), (179, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)

        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        if count <= 1:
            return

        # Select the largest connected red region.
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest = int(np.argmax(areas)) + 1
        area = int(stats[largest, cv2.CC_STAT_AREA])

        if area < 8:
            return

        # Reject ambiguous detections.
        if len(areas) > 1:
            sorted_areas = np.sort(areas)
            if sorted_areas[-2] > 0.6 * area:
                return

        rows, cols = np.where(labels == largest)
        depths = depth_image[rows, cols]

        # Discard invalid or implausible depth measurements.
        valid = (
            np.isfinite(depths)
            & (depths > 0.05)
            & (depths < 3.0)
        )

        if np.count_nonzero(valid) < max(4, 0.6 * area):
            return

        rows = rows[valid]
        cols = cols[valid]
        depths = depths[valid]

        # Reject depth outliers around the median.
        median_depth = np.median(depths)
        inliers = np.abs(depths - median_depth) < 0.02

        if np.count_nonzero(inliers) < max(4, 0.6 * len(depths)):
            return

        u = float(np.median(cols[inliers]))
        v = float(np.median(rows[inliers]))
        z = float(np.median(depths[inliers]))

        # Camera intrinsic parameters.
        fx = self.info.k[0]
        fy = self.info.k[4]
        cx = self.info.k[2]
        cy = self.info.k[5]

        if not np.all(np.isfinite([fx, fy, cx, cy])):
            return

        if fx <= 0 or fy <= 0:
            return

        # Reconstruct a 3D point in the optical frame.
        camera_point = PointStamped()
        camera_point.header = copy.deepcopy(rgb.header)
        camera_point.point.x = (u - cx) * z / fx
        camera_point.point.y = (v - cy) * z / fy
        camera_point.point.z = z

        self.camera_pub.publish(camera_point)

        # Transform into the robot's coordinate system.
        try:
            base_point = self.tf_buffer.transform(
                camera_point, 'base_link'
            )
        except TransformException:
            return

        base_point.header.stamp = rgb.header.stamp

        coordinates = [
            base_point.point.x,
            base_point.point.y,
            base_point.point.z
        ]

        if not np.all(np.isfinite(coordinates)):
            return

        self.base_pub.publish(base_point)

        # Move only when no other movement is active.
        if self.success or self.future is not None:
            return

        if time.monotonic() - self.last_attempt < 1.0:
            return

        if not self.arm.ready():
            return

        if not self.fresh(rgb, depth, rgb_wall, depth_wall):
            return

        # Hover eight centimeters above the marker.
        goal = copy.deepcopy(base_point)
        goal.point.z += 0.08

        if abs(goal.point.z - 0.14) > 0.01:
            self.get_logger().warning('Invalid hover height')
            return

        self.get_logger().info(
            f'Marker: ({base_point.point.x:.3f}, '
            f'{base_point.point.y:.3f}, '
            f'{base_point.point.z:.3f})'
        )

        self.last_attempt = time.monotonic()
        self.future = self.arm.move_to_point(goal)
        self.future.add_done_callback(self.motion_finished)

    def motion_finished(self, future):
        try:
            result = future.result()
            self.success = bool(result.success)

            if self.success:
                self.get_logger().info(
                    f'Motion succeeded: {result.message}'
                )
            else:
                self.get_logger().warning(
                    f'Motion failed: {result.message}'
                )

        except Exception as error:
            self.get_logger().error(str(error))

        finally:
            self.future = None


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = ReachMarker()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        node.stopping = True

        if node.future is not None:
            node.arm.cancel()

            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
