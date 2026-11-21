import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from hw01_sim.arm_client import ArmClient
import tf2_geometry_msgs
import cv2
import numpy as np
import time
import signal


class MarkerReacher(Node):
    def __init__(self):
        super().__init__("marker_reacher")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()
        self.marker_u = None
        self.marker_v = None
        self.marker_mask = None
        self.rgb_stamp = None
        self.last_data_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = ArmClient(self)
        self.goal_sent = False
        self.attempts = 0
        self.stopping = False
        self.camera_pub = self.create_publisher(PointStamped, "/hw01/target_camera", 10)
        self.base_pub = self.create_publisher(PointStamped, "/hw01/target_base", 10)
        self.image_sub = Subscriber(
            self, Image, "/camera/image_raw", qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, "/camera/depth/image_raw", qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=20, slop=0.05
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.watchdog = self.create_timer(
            1.0,
            self.watchdog_callback,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        self.get_logger().info("Marker Reacher node started")
        self.sync.registerCallback(self.synced_callback)

    def image_callback(self, msg):

        self.rgb_stamp = msg.header.stamp
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        moments = cv2.moments(red_mask, binaryImage=True)
        red_pixels = cv2.countNonZero(red_mask)

        if moments["m00"] > 0 and red_pixels >= 45:
            center_u = int(moments["m10"] / moments["m00"])
            center_v = int(moments["m01"] / moments["m00"])

            self.marker_u = center_u
            self.marker_v = center_v
            self.marker_mask = red_mask
            self.get_logger().info(f"Marker center:({center_u},{center_v})")
        else:
            self.marker_u = None
            self.marker_v = None
            self.marker_mask = None
            self.get_logger().warn("No reliable marker detected")
        self.get_logger().info(f"Image shape: {image.shape}")
        self.get_logger().info(f"Red pixels detected: {red_pixels}")

    def depth_callback(self, msg):

        if self.rgb_stamp is None:
            return

        rgb_time = self.rgb_stamp.sec + self.rgb_stamp.nanosec * 1e-9

        depth_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        time_diff = abs(rgb_time - depth_time)

        self.get_logger().info(f"RGB-Depth time difference:{ time_diff:.4f} seconds")
        if time_diff > 0.05:
            return

        if msg.encoding != "32FC1":
            self.get_logger().warn(f"Wrong depth encoding: {msg.encoding}")
            return
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.marker_u is None or self.marker_v is None:
            return
        height, width = depth_image.shape[:2]
        if not (0 <= self.marker_u < width and 0 <= self.marker_v < height):
            return
        if self.marker_mask.shape != depth_image.shape[:2]:
            return
        values = depth_image[self.marker_mask > 0]
        values = values[np.isfinite(values) & (values > 0.05) & (values < 3.0)]
        if values.size == 0:
            self.get_logger().warn("No valid depth for the marker")
            return
        depth_value = float(np.median(values))

        if not np.isfinite(depth_value) or depth_value <= 0:
            return
        if not hasattr(self, "fx"):
            return
        X = (self.marker_u - self.cx) * depth_value / self.fx
        Y = (self.marker_v - self.cy) * depth_value / self.fy
        Z = depth_value

        point_camera = PointStamped()
        point_camera.header.stamp = msg.header.stamp
        point_camera.header.frame_id = msg.header.frame_id

        point_camera.point.x = float(X)
        point_camera.point.y = float(Y)
        point_camera.point.z = float(Z)

        try:
            point_base = self.tf_buffer.transform(point_camera, "base_link")
            self.base_pub.publish(point_base)
            self.get_logger().info(
                f"Marker Base:"
                f"X={point_base.point.x:.3f},"
                f"Y={point_base.point.y:.3f},"
                f"Z={point_base.point.z:.3f},"
            )
            self.move_arm(point_base)
        except TransformException as ex:
            self.get_logger().warn(f"TF transform failed : {ex}")
        self.camera_pub.publish(point_camera)
        self.get_logger().info(f"Marker 3D: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        self.get_logger().info(
            f"Marker depth at ({self.marker_u},{self.marker_v}):"
            f"{depth_value:.3f} meters"
        )
        self.get_logger().info(f"Depth array shape: {depth_image.shape}")
        self.get_logger().info(
            f"Image Depth: {msg.width} x {msg.height},encoding={msg.encoding}"
        )

    
    def move_arm(self, point_base):
        if self.goal_sent or self.stopping or self.attempts >= 3:
            return
        if self.last_data_time is None or time.monotonic() - self.last_data_time > 2.0:
            self.get_logger().warn("No fresh camera data, goal not sent")
            return

        if not self.arm.ready():
            self.get_logger().warn("ArmClient is not ready")
            return
        self.get_logger().info("ArmClient is ready!")

        target = PointStamped()
        target.header = point_base.header
        target.point.x = point_base.point.x
        target.point.y = point_base.point.y
        target.point.z = point_base.point.z + 0.08

        self.goal_sent = True
        self.attempts += 1

        self.get_logger().info(
            f"Moving arm to: "
            f"X={target.point.x:.3f}, "
            f"Y={target.point.y:.3f}, "
            f"Z={target.point.z:.3f}"
        )

        future = self.arm.move_to_point(target)
        future.add_done_callback(self.motion_done_callback)

    def motion_done_callback(self, future):
        try:
            result = future.result()

            if result.success:
                self.get_logger().info(
                    "Arm reached target successfully!"
                )
            else:
                self.get_logger().error(
                    f"Arm movement failed: {result.message}"
                )
                self.goal_sent = False

        except Exception as ex:
            self.get_logger().error(f"Motion error: {ex}")
            self.goal_sent = False
        if not self.goal_sent and self.attempts >= 3:
            self.get_logger().error("Too many failed attempts, giving up")

    def camera_info_callback(self, msg):
        if not (0 < msg.k[0] < np.inf and 0 < msg.k[4] < np.inf):
            self.get_logger().warn("Invalid camera calibration")
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )
        self.get_logger().info(f"Camera matrix K: {msg.k}")

    def synced_callback(self, rgb_msg, depth_msg):
        rgb_time = (
            rgb_msg.header.stamp.sec
            + rgb_msg.header.stamp.nanosec * 1e-9
        )
        depth_time = (
            depth_msg.header.stamp.sec
            + depth_msg.header.stamp.nanosec * 1e-9
        )

        time_diff = abs(rgb_time - depth_time)

        self.get_logger().info(
            f"Synchronized pair: {time_diff:.4f} seconds"
        )

        age = self.get_clock().now().nanoseconds * 1e-9 - rgb_time
        if abs(age) > 0.5:
            self.get_logger().warn(f"Stale or future image: {age:.2f} seconds")
            return
        self.last_data_time = time.monotonic()

        self.image_callback(rgb_msg)
        self.depth_callback(depth_msg)

    def watchdog_callback(self):
        if self.last_data_time is None:
            self.get_logger().warn("Waiting for camera data")
        elif time.monotonic() - self.last_data_time > 2.0:
            self.get_logger().warn("No fresh camera data for 2 seconds")

    def stop(self, signum, frame):
        self.stopping = True

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MarkerReacher()
    signal.signal(signal.SIGINT, node.stop)

    try:
        while not node.stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.stopping = True
            node.get_logger().info("Shutting down, canceling arm motion")
            node.arm.cancel()
            node.destroy_subscription(node.image_sub.sub)
            node.destroy_subscription(node.depth_sub.sub)
            end = time.monotonic() + 3.0
            while rclpy.ok() and node.arm.busy and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
