

from copy import deepcopy
import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from hw01_sim.arm_client import ArmClient
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class solution(Node):
    def __init__(self):
        super().__init__(
            'solution', parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        self.bridge = CvBridge()
        self.arm = ArmClient(self)
        self.movement_sent = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_target_publisher = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.base_target_publisher = self.create_publisher(PointStamped, '/hw01/target_base', 10)
        self.depth_min = self.declare_parameter('depth_min', 0.05).value
        self.depth_max = self.declare_parameter('depth_max', 3.0).value

        self.camera_info = None
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.rgb_subscription = Subscriber(
            self,
            Image,
            '/camera/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscription = Subscriber(
            self,
            Image,
            '/camera/depth/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.image_synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_subscription, self.depth_subscription],
            queue_size=10,
            slop=0.05,
        )
        self.image_synchronizer.registerCallback(self.synchronized_callback)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def synchronized_callback(self, rgb_msg, depth_msg):
        red_pixel = self.detect_red_pixel(rgb_msg)
        if red_pixel is None or self.camera_info is None:
            return
        age = (self.get_clock().now() - Time.from_msg(rgb_msg.header.stamp)).nanoseconds / 1e9
        if age > 0.5:
            return

        try:
            if depth_msg.encoding != '32FC1':
                self.get_logger().warning('Expected 32FC1 depth in metres', throttle_duration_sec=5.0)
                return
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().error(f'Cannot convert depth image: {exc}')
            return
        if (depth_msg.width, depth_msg.height) != (rgb_msg.width, rgb_msg.height):
            return
        u, v = red_pixel
        # The median of a small patch avoids relying on a single noisy depth pixel.
        patch = depth[max(0, v - 1):v + 2, max(0, u - 1):u + 2]
        valid = patch[np.isfinite(patch) & (patch >= self.depth_min) & (patch <= self.depth_max)]
        if valid.size == 0:
            return
        z = float(np.median(valid))
        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
            return
        if not self.camera_info.header.frame_id:
            return

        camera_point = PointStamped()
        camera_point.header.frame_id = self.camera_info.header.frame_id
        camera_point.header.stamp = rgb_msg.header.stamp
        camera_point.point.x = float((u - cx) * z / fx)
        camera_point.point.y = float((v - cy) * z / fy)
        camera_point.point.z = z
        self.camera_target_publisher.publish(camera_point)
        try:
            base_point = self.tf_buffer.transform(camera_point, 'base_link')
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for camera transform: {exc}', throttle_duration_sec=5.0)
            return
        self.base_target_publisher.publish(base_point)
        if not self.movement_sent and self.arm.ready() and not self.arm.busy:
            hover_target = deepcopy(base_point)
            hover_target.point.z += 0.08
            self.movement_sent = True
            future = self.arm.move_to_point(hover_target)
            future.add_done_callback(self.movement_finished)

    def movement_finished(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Movement failed: {exc}')
            return
        if result.success:
            self.get_logger().info(f'Movement succeeded: {result.message}')
        else:
            self.get_logger().error(f'Movement failed: {result.message}')

    def detect_red_pixel(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
            mask2 = cv2.inRange(hsv, (170, 100, 80), (179, 255, 255))
            mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) < 20: # make sure red pixels are big enough and not noise
                                    return None
                moments = cv2.moments(largest)
                if moments['m00'] > 0:
                    u = int(moments['m10'] / moments['m00'])
                    v = int(moments['m01'] / moments['m00'])
                    return u, v
        except (CvBridgeError, cv2.error) as exc:
            self.get_logger().error(f'Cannot process camera image: {exc}')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = solution()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()