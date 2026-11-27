import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from hw01_sim.arm_client import ArmClient

HOVER_OFFSET = 0.08
BASE_FRAME = 'base_link'
STAMP_MATCH = 0.05
SIM_STALE = 0.5
WALL_STALE = 2.0
DEPTH_MIN = 0.05
DEPTH_MAX = 5.0
MIN_AREA = 25.0
WINDOW = 2
MIN_SAMPLES = 4
RETRY_COOLDOWN = 1.0
LOWER_RED_1 = np.array([0, 110, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 110, 70])
UPPER_RED_2 = np.array([180, 255, 255])


class Reach(Node):
    def __init__(self):
        super().__init__('hw01_reach')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = ArmClient(self)
        self.intrinsics = None
        self.rgb = None
        self.depth_buffer = deque(maxlen=15)
        self.rgb_wall = 0.0
        self.depth_wall = 0.0
        self.motion = None
        self.reached = False
        self.last_attempt = 0.0
        self.target_camera_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.target_base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)

    def on_rgb(self, message):
        self.rgb = message
        self.rgb_wall = time.monotonic()

    def on_depth(self, message):
        self.depth_buffer.append(message)
        self.depth_wall = time.monotonic()

    def on_info(self, message):
        if self.intrinsics is None and message.k[0] > 0 and message.k[4] > 0:
            self.intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def tick(self):
        if self.motion is not None:
            if self.motion.done():
                result = self.motion.result()
                if result.success:
                    self.reached = True
                    self.get_logger().info(f'reached target: {result.message}')
                else:
                    self.get_logger().warn(f'motion failed: {result.message}')
                self.motion = None
            return
        if self.reached:
            return
        if time.monotonic() - self.last_attempt < RETRY_COOLDOWN:
            return
        target = self.observe()
        if target is None or not self.arm.ready():
            return
        self.last_attempt = time.monotonic()
        self.motion = self.arm.move_to_point(target)

    def observe(self):
        if self.intrinsics is None or self.rgb is None or not self.depth_buffer:
            return None
        now = time.monotonic()
        if now - self.rgb_wall > WALL_STALE or now - self.depth_wall > WALL_STALE:
            return None
        rgb = self.rgb
        t_rgb = Time.from_msg(rgb.header.stamp).nanoseconds / 1e9
        depth = min(self.depth_buffer,
                    key=lambda d: abs(Time.from_msg(d.header.stamp).nanoseconds / 1e9 - t_rgb))
        t_depth = Time.from_msg(depth.header.stamp).nanoseconds / 1e9
        if abs(t_rgb - t_depth) > STAMP_MATCH:
            return None
        sim_now = self.get_clock().now().nanoseconds / 1e9
        if sim_now - t_rgb > SIM_STALE:
            return None
        pixel = self.detect(rgb)
        if pixel is None:
            return None
        z = self.sample_depth(depth, pixel)
        if z is None:
            return None
        fx, fy, cx, cy = self.intrinsics
        u, v = pixel
        point = PointStamped()
        point.header = rgb.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        self.target_camera_pub.publish(point)
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, rgb.header.frame_id, Time.from_msg(rgb.header.stamp),
                Duration(seconds=0.1))
        except TransformException:
            return None
        base_point = do_transform_point(point, transform)
        self.target_base_pub.publish(base_point)
        goal = PointStamped()
        goal.header.frame_id = BASE_FRAME
        goal.header.stamp = rgb.header.stamp
        goal.point.x = base_point.point.x
        goal.point.y = base_point.point.y
        goal.point.z = base_point.point.z + HOVER_OFFSET
        return goal

    def detect(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1) | cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_AREA:
            return None
        moments = cv2.moments(largest)
        if moments['m00'] == 0:
            return None
        return moments['m10'] / moments['m00'], moments['m01'] / moments['m00']

    def sample_depth(self, message, pixel):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        frame = np.asarray(frame, dtype=np.float32)
        u, v = int(round(pixel[0])), int(round(pixel[1]))
        height, width = frame.shape[:2]
        rows = slice(max(0, v - WINDOW), min(height, v + WINDOW + 1))
        cols = slice(max(0, u - WINDOW), min(width, u + WINDOW + 1))
        patch = frame[rows, cols].reshape(-1)
        valid = patch[np.isfinite(patch) & (patch > DEPTH_MIN) & (patch < DEPTH_MAX)]
        if valid.size < MIN_SAMPLES:
            return None
        return float(np.median(valid))

    def shutdown(self):
        self.arm.cancel()
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Reach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
