import math
import signal

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from hw01_sim.arm_client import ArmClient
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .helpers_no_spread import locate_disc

HOVER_HEIGHT = 0.08     # m above the marker
MAX_AGE = 0.5           # s, simulation time
MARKER_HEIGHT = 0.06    # m, marker top in base_link (from the assignment)
HEIGHT_TOLERANCE = 0.01 # m, the helper's tolerance on the 0.14 m tip height
TIP_TOLERANCE = 0.02    # m, the checker's tolerance on the final tip position


class MarkerReacher(Node):
    def __init__(self):
        super().__init__('marker_reacher',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.K = None                   # (fx, fy, cx, cy)
        self.state = 'searching'        # searching -> moving -> done
        self.bridge = CvBridge()
        self.arm = ArmClient(self)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.rgb_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        # Pairs RGB and depth whose stamps are within 0.05 s; each pair is used once
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.on_pair)

        self.camera_publisher = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.base_publisher = self.create_publisher(PointStamped, '/hw01/target_base', 10)

    # ---------- callbacks ----------

    def on_info(self, msg):
        # k = [fx, 0, cx,  0, fy, cy,  0, 0, 1]; an all-zero k means no calibration
        fx, fy = msg.k[0], msg.k[4]
        if fx > 0 and fy > 0:
            self.K = (fx, fy, msg.k[2], msg.k[5])

    def on_pair(self, rgb_msg, depth_msg):
        # Every goal comes from the pair that just arrived, so no image for 2 s means no goal
        if self.state != 'searching' or self.K is None or not self.is_fresh(rgb_msg):
            return

        images = self.to_arrays(rgb_msg, depth_msg)
        if images is None:
            return
        disk_center_point = locate_disc(*images)
        if disk_center_point is None:
            return

        camera_point = self.to_camera_point(disk_center_point, rgb_msg.header)
        self.camera_publisher.publish(camera_point)
        base_point = self.to_base(camera_point)
        if base_point is None:
            return
        self.base_publisher.publish(base_point)

        # A wrong depth or transform shows up as a wrong height: the tip can only reach 0.14 m
        if abs(base_point.point.z - MARKER_HEIGHT) > HEIGHT_TOLERANCE:
            self.get_logger().warning(f'Marker height {base_point.point.z:.3f} m, expected '
                                      f'{MARKER_HEIGHT} m, skipping', throttle_duration_sec=5.0)
            return

        self.send_goal(base_point)

    def on_motion_done(self, future):
        result = future.result()
        if result.success:
            self.state = 'done'
            self.get_logger().info(f'Motion succeeded: {result.message}')
            self.check_tip()
        else:
            self.state = 'searching'
            self.get_logger().warning(f'Motion failed: {result.message}', throttle_duration_sec=2.0)

    # ---------- steps ----------

    def is_fresh(self, msg):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            self.get_logger().warning('Clock is not ready, skipping', throttle_duration_sec=5.0)
            return False
        age = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age > MAX_AGE:
            self.get_logger().warning(f'RGB image is {age:.3f} s old, skipping', throttle_duration_sec=5.0)
            return False
        return True

    def to_arrays(self, rgb_msg, depth_msg):
        if depth_msg.encoding != '32FC1':   # depth must be metres
            self.get_logger().warning(f'Unexpected depth encoding {depth_msg.encoding}',
                                      throttle_duration_sec=5.0)
            return None
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.get_logger().warning(f'Cannot convert images: {error}', throttle_duration_sec=5.0)
            return None
        return rgb, depth

    def to_camera_point(self, disk_center_point, header):
        # Pinhole model
        u, v, z = disk_center_point
        fx, fy, cx, cy = self.K
        point = PointStamped()
        point.header = header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def to_base(self, camera_point):
        # Transform at the time the image was taken (target frame first, then source)
        try:
            transform = self.buffer.lookup_transform(
                'base_link', camera_point.header.frame_id, Time.from_msg(camera_point.header.stamp))
        except TransformException as error:
            self.get_logger().warning(f'Waiting for camera-to-base transform: {error}',
                                      throttle_duration_sec=5.0)
            return None
        base_point = do_transform_point(camera_point, transform)
        base_point.header.stamp = camera_point.header.stamp     # keep the image time
        return base_point

    def send_goal(self, base_point):
        if not self.arm.ready():
            self.get_logger().warning('Motion helper not ready, skipping', throttle_duration_sec=5.0)
            return
        goal = PointStamped()
        goal.header = base_point.header
        goal.point.x = base_point.point.x
        goal.point.y = base_point.point.y
        goal.point.z = base_point.point.z + HOVER_HEIGHT
        self.get_logger().info(f'Marker at ({base_point.point.x:.3f}, {base_point.point.y:.3f}, '
                               f'{base_point.point.z:.3f}) m, sending goal')
        self.goal = goal
        self.state = 'moving'           # before add_done_callback: it may run immediately
        self.arm.move_to_point(goal).add_done_callback(self.on_motion_done)

    def check_tip(self):
        # Action success is not proof: compare the measured tip (TF from joint states) with the goal
        try:
            tip = self.buffer.lookup_transform('base_link', 'tool0', Time()).transform.translation
        except TransformException as error:
            self.get_logger().warning(f'Cannot check the tip position: {error}')
            return
        goal = self.goal.point
        error = math.dist((tip.x, tip.y, tip.z), (goal.x, goal.y, goal.z))
        if error > TIP_TOLERANCE:
            self.get_logger().warning(f'Helper reported success, but the tip is {error * 1000:.1f} mm from the goal')
        else:
            self.get_logger().info(f'Tip is {error * 1000:.1f} mm from the goal')


def main():
    # Ctrl+C raises KeyboardInterrupt and ROS stays up, so the cancel can still be sent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = MarkerReacher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)      # short timeout: Ctrl+C is noticed quickly
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.arm.cancel()               # cancel a pending motion on Ctrl+C
        for _ in range(5):              # let the cancel request go out
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
