#!/usr/bin/env python3
"""HW1: detect the red marker, reconstruct it in 3D, transform to base_link, hover above it."""
from enum import Enum
import math
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401  registers PointStamped with tf2_ros.Buffer.transform
from tf2_ros import Buffer, TransformException, TransformListener

from hw01_sim.arm_client import ArmClient

from .logic import ConsistencyFilter, MAX_WALL_GAP_S, observation_age_problem
from .perception import (DepthParams, DetectionParams, backproject, depth_to_metres,
                         detect_marker, select_depth)

BASE_FRAME = 'base_link'
SYNC_SLOP_S = 0.05
TIP_HEIGHT = 0.14


class State(Enum):
    WAITING = 'waiting for a reliable target'
    MOVING = 'motion in progress'
    SUCCEEDED = 'motion succeeded'
    FAILED = 'gave up'


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ReachNode(Node):
    def __init__(self):
        super().__init__('hw01_reach', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.hover_offset = self.declare_parameter('hover_offset', 0.08).value
        required = self.declare_parameter('required_consistent', 3).value
        tolerance = self.declare_parameter('consistency_tolerance', 0.01).value
        self.max_attempts = self.declare_parameter('max_attempts', 3).value

        self.bridge = CvBridge()
        self.det_params = DetectionParams()
        self.depth_params = DepthParams()
        self.filter = ConsistencyFilter(required, tolerance)
        self.camera_info = None
        self.state = State.WAITING
        self.attempts = 0
        self.motion = None
        self.goal = None
        self.verify_after = None
        self.last_pair_wall = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = ArmClient(self)

        self.pub_camera = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.pub_base = self.create_publisher(PointStamped, '/hw01/target_base', 10)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        rgb = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, '/camera/depth/image_raw',
                           qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb, depth], queue_size=10, slop=SYNC_SLOP_S)
        self.sync.registerCallback(self.on_pair)
        self.create_subscription(PoseStamped, '/hw01/measured_tip', self.on_tip,
                                 qos_profile_sensor_data)
        # Wall-clock watchdog: independent of simulation time, which may freeze.
        self.create_timer(0.25, self.watchdog, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info('HW1 reach node started; ' + self.state.value)

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg):
        self.camera_info = msg

    def reject(self, reason):
        """A frame that cannot be trusted: forget earlier estimates and say why."""
        self.filter.reset()
        if self.state is State.WAITING:
            self.get_logger().warn(f'Rejected frame: {reason}', throttle_duration_sec=2.0)

    def watchdog(self):
        if self.last_pair_wall is None:
            return
        gap = time.monotonic() - self.last_pair_wall
        if gap > MAX_WALL_GAP_S:
            self.filter.reset()
            self.get_logger().warn(f'No fresh camera data for {gap:.1f} s (wall clock)',
                                   throttle_duration_sec=5.0)

    def on_pair(self, rgb_msg, depth_msg):
        self.last_pair_wall = time.monotonic()
        if self.state in (State.SUCCEEDED, State.FAILED):
            return
        point_cam, point_base = self.estimate(rgb_msg, depth_msg)
        if point_base is None:
            return
        if self.state is not State.WAITING:
            return  # only one motion at a time; keep publishing estimates for debugging
        target = self.filter.add((point_base.point.x, point_base.point.y, point_base.point.z))
        if target is not None:
            self.send_goal(target, rgb_msg.header.stamp)

    # -------------------------------------------------------------- perception
    def estimate(self, rgb_msg, depth_msg):
        """Return (camera_point, base_point) PointStamped, or (None, None) after reject()."""
        dt = abs(stamp_s(rgb_msg.header.stamp) - stamp_s(depth_msg.header.stamp))
        if dt > SYNC_SLOP_S:
            self.reject(f'RGB/depth stamps differ by {dt:.3f} s')
            return None, None
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            self.reject('no /clock yet')
            return None, None
        problem = observation_age_problem(stamp_s(rgb_msg.header.stamp), now.nanoseconds * 1e-9)
        if problem:
            self.reject(problem)
            return None, None
        info = self.camera_info
        if info is None:
            self.reject('no camera calibration yet')
            return None, None
        frame = rgb_msg.header.frame_id
        if info.header.frame_id != frame or depth_msg.header.frame_id != frame:
            self.reject(f'frame mismatch rgb={frame} depth={depth_msg.header.frame_id} '
                        f'info={info.header.frame_id}')
            return None, None
        if (info.width, info.height) != (rgb_msg.width, rgb_msg.height):
            self.reject('calibration size does not match image size')
            return None, None
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = depth_to_metres(self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'),
                                    depth_msg.encoding)
        except Exception as error:  # cv_bridge raises its own error types
            self.reject(f'image conversion failed: {error}')
            return None, None

        detection, reason = detect_marker(bgr, self.det_params)
        if detection is None:
            self.reject(reason)
            return None, None
        z, reason = select_depth(depth, detection.mask, self.depth_params)
        if z is None:
            self.reject(reason)
            return None, None
        try:
            x, y, z = backproject(detection.u, detection.v, z, info.k)
        except ValueError as error:
            self.reject(str(error))
            return None, None

        point_cam = PointStamped()
        point_cam.header = rgb_msg.header  # optical frame + image timestamp
        point_cam.point.x, point_cam.point.y, point_cam.point.z = x, y, z
        try:
            # Lookup at the image timestamp, camera frame -> base_link (non-blocking).
            point_base = self.tf_buffer.transform(point_cam, BASE_FRAME)
        except TransformException as error:
            self.reject(f'transform {frame} -> {BASE_FRAME} unavailable: {error}')
            return None, None
        self.pub_camera.publish(point_cam)
        self.pub_base.publish(point_base)
        return point_cam, point_base

    # ------------------------------------------------------------------ motion
    def send_goal(self, target, stamp):
        x, y, z = target
        goal = PointStamped()
        goal.header.frame_id = BASE_FRAME
        goal.header.stamp = stamp
        goal.point.x, goal.point.y, goal.point.z = x, y, z + self.hover_offset
        if abs(goal.point.z - TIP_HEIGHT) > 0.01:
            self.get_logger().error(
                f'Hover point z={goal.point.z:.3f} m is not near {TIP_HEIGHT} m; '
                'perception or TF is wrong, not sending')
            self.filter.reset()
            return
        if not self.arm.ready():
            self.get_logger().warn('Motion helper not ready yet', throttle_duration_sec=2.0)
            return
        self.attempts += 1
        self.goal = goal
        self.state = State.MOVING
        self.get_logger().info(
            f'Marker in {BASE_FRAME}: ({x:.3f}, {y:.3f}, {z:.3f}) m. Sending hover goal '
            f'({goal.point.x:.3f}, {goal.point.y:.3f}, {goal.point.z:.3f}) '
            f'[attempt {self.attempts}/{self.max_attempts}]')
        self.motion = self.arm.move_to_point(goal)
        self.motion.add_done_callback(self.on_motion_done)

    def on_motion_done(self, future):
        result = future.result()
        self.motion = None
        if result.success:
            self.state = State.SUCCEEDED
            self.verify_after = self.get_clock().now().nanoseconds * 1e-9 + 1.0
            self.get_logger().info(f'MOTION SUCCEEDED: {result.message}')
            return
        self.get_logger().error(f'MOTION FAILED: {result.message}')
        self.filter.reset()
        self.state = State.WAITING if self.attempts < self.max_attempts else State.FAILED
        if self.state is State.FAILED:
            self.get_logger().error('Giving up after repeated motion failures')

    def on_tip(self, msg):
        """Independent physical check: action success is not proof of the final position."""
        if self.verify_after is None or self.goal is None:
            return
        if self.get_clock().now().nanoseconds * 1e-9 < self.verify_after:
            return
        self.verify_after = None
        p, g = msg.pose.position, self.goal.point
        error = math.dist((p.x, p.y, p.z), (g.x, g.y, g.z))
        verdict = 'OK' if error <= 0.02 else 'WARNING: outside 0.02 m'
        self.get_logger().info(f'Measured tip error vs. commanded hover point: {error:.4f} m '
                               f'({verdict})')

    def cancel_motion(self):
        if self.motion is not None:
            self.get_logger().info('Interrupted: cancelling pending motion')
        self.arm.cancel()


def main(args=None):
    # Handle Ctrl+C ourselves so the context stays alive long enough to send the cancel.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ReachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_motion()
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
