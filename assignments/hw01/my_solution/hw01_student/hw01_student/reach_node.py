import math
import time

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2 transform())
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from hw01_sim.arm_client import ArmClient

# ---------- Detection ----------
# HSV thresholds for red (OpenCV hue range is 0-180; red wraps around both ends)
RED_LOW_1 = (0, 120, 70)
RED_HIGH_1 = (10, 255, 255)
RED_LOW_2 = (170, 120, 70)
RED_HIGH_2 = (180, 255, 255)
MIN_AREA = 20              # pixels; smaller blobs are treated as noise
SECOND_BLOB_RATIO = 0.25   # a second blob >= 1/4 of the largest makes detection ambiguous

# ---------- Depth ----------
SYNC_SLOP = 0.05           # max time difference (sim seconds) between RGB and depth
MIN_DEPTH = 0.05           # metres; valid depth range
MAX_DEPTH = 3.0
MIN_DEPTH_PIXELS = 10      # need at least this many valid depth pixels on the marker

# ---------- Target / motion ----------
BASE_FRAME = 'base_link'
HOVER_OFFSET = 0.08        # metres above the detected marker
MAX_OBS_AGE = 0.5          # sim seconds: older observations are stale
MAX_WALL_GAP = 2.0         # wall seconds without camera data -> stale
MAX_ATTEMPTS = 5           # motion attempts before giving up
RETRY_DELAY = 2.0          # wall seconds to wait before retrying a failed motion


class ReachNode(Node):
    def __init__(self):
        # use_sim_time: timestamps come from the simulation clock (/clock)
        super().__init__('reach_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()
        self.K = None                      # (fx, fy, cx, cy) from camera_info
        self.last_camera_wall = None       # wall time of the last camera pair
        self.start_wall = time.monotonic() # wall time the node started
        self.measured_tip = None           # latest physical tip pose (for checking the result)

        # Motion state: only one motion at a time
        self.arm = ArmClient(self)
        self.motion = None                 # Future of the active motion, or None
        self.goal = None                   # goal of the active motion (for the physical check)
        self.attempts = 0
        self.finished = False              # True after success, giving up, or a simulation restart
        self.next_attempt_wall = 0.0       # earliest wall time for the next motion attempt
        self.last_stamp = None             # image timestamp of the previous camera pair

        # TF: buffer of all transforms + listener that fills it from /tf and /tf_static
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera calibration (intrinsic matrix)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info,
                                 qos_profile_sensor_data)

        # RGB + depth, matched by timestamp (within SYNC_SLOP seconds)
        rgb_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw',
                               qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=SYNC_SLOP)
        self.sync.registerCallback(self.on_images)

        # Physical tip position measured by the simulator (independent of the action result)
        self.create_subscription(PoseStamped, '/hw01/measured_tip', self.on_measured_tip,
                                 qos_profile_sensor_data)

        # Debug outputs: estimated marker position (before the hover offset)
        self.target_camera_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.target_base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)

        # Periodic check: motion result + camera data freshness.
        # Uses the wall clock, so it keeps running even if the simulation (and /clock) stops.
        self.create_timer(0.1, self.on_timer, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    # ---------- Callbacks ----------

    def on_camera_info(self, msg):
        # K is a 3x3 matrix stored row by row: [fx 0 cx; 0 fy cy; 0 0 1]
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if fx > 0 and fy > 0:
            self.K = (fx, fy, cx, cy)

    def on_measured_tip(self, msg):
        self.measured_tip = msg

    def on_images(self, rgb_msg, depth_msg):
        self.last_camera_wall = time.monotonic()
        if self.finished:
            return

        # Simulation restarted while this node was running: time jumped backwards, so
        # TF, observations and the pending motion belong to the old run. Stop safely.
        stamp = Time.from_msg(rgb_msg.header.stamp)
        # (compare in nanoseconds: rclpy Time cannot hold negative values near t=0)
        if self.last_stamp is not None and stamp.nanoseconds < self.last_stamp.nanoseconds - 1e9:
            self.get_logger().error('simulation time jumped backwards (simulation restarted); '
                                    'cancelling motion and stopping - restart this node')
            self.arm.cancel()
            self.finished = True
            return
        self.last_stamp = stamp

        if self.K is None:
            self.get_logger().info('waiting for camera_info')
            return

        # Reject stale observations (simulation time)
        age = (self.get_clock().now() - Time.from_msg(rgb_msg.header.stamp)).nanoseconds / 1e9
        if age > MAX_OBS_AGE:
            self.get_logger().warn(f'stale observation ({age:.2f} s old), ignoring')
            return

        # Step 1: find the marker in the color image
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        result = self.find_marker(bgr)
        if result is None:
            self.get_logger().info('no marker', throttle_duration_sec=1.0)
            return
        u, v, mask = result

        # Step 2: marker depth (median of valid depth values on the marker)
        z = self.marker_depth(depth_msg, mask)
        if z is None:
            self.get_logger().warn('no valid depth on marker', throttle_duration_sec=1.0)
            return

        # Step 3: pixel + depth -> 3D point in camera_optical_frame
        point_cam = self.to_3d(u, v, z, rgb_msg.header)
        self.target_camera_pub.publish(point_cam)

        # Step 4: camera_optical_frame -> base_link, at the image timestamp
        try:
            point_base = self.tf_buffer.transform(point_cam, BASE_FRAME)
        except TransformException as error:
            self.get_logger().warn(f'transform unavailable: {error}', throttle_duration_sec=1.0)
            return
        self.target_base_pub.publish(point_base)

        c, b = point_cam.point, point_base.point
        self.get_logger().info(
            f'marker camera=({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) '
            f'base=({b.x:.3f}, {b.y:.3f}, {b.z:.3f})', throttle_duration_sec=1.0)

        # Step 5: send the hover goal (only from this fresh observation, one motion at a time)
        self.try_send_goal(point_base)

    def on_timer(self):
        # Wall-clock freshness of camera data
        if self.last_camera_wall is None:
            # Allow a short startup period (ROS discovery) before reporting missing data
            if time.monotonic() - self.start_wall > MAX_WALL_GAP:
                self.get_logger().info('waiting for camera data', throttle_duration_sec=2.0)
        elif time.monotonic() - self.last_camera_wall > MAX_WALL_GAP:
            self.get_logger().warn('no fresh camera data for 2 s', throttle_duration_sec=2.0)

        # Check the result of the active motion
        if self.motion is None or not self.motion.done():
            return
        result = self.motion.result()
        self.motion = None
        if result.success:
            self.get_logger().info(f'MOTION SUCCEEDED: {result.message}')
            self.report_tip_error()
            self.finished = True
        else:
            self.get_logger().error(f'MOTION FAILED (attempt {self.attempts}/{MAX_ATTEMPTS}): '
                                    f'{result.message}')
            if self.attempts >= MAX_ATTEMPTS:
                self.get_logger().error('giving up')
                self.finished = True
            else:
                # Wait before retrying (e.g. controllers still starting); the retry
                # uses a new, fresh observation - never the old target.
                self.next_attempt_wall = time.monotonic() + RETRY_DELAY

    # ---------- Motion ----------

    def try_send_goal(self, point_base):
        if self.finished or self.motion is not None:
            return                          # done, or a motion is already active
        if time.monotonic() < self.next_attempt_wall:
            return                          # waiting before a retry
        if not self.arm.ready():
            self.get_logger().info('motion helper not ready yet', throttle_duration_sec=1.0)
            return

        goal = PointStamped()
        goal.header.frame_id = BASE_FRAME
        goal.header.stamp = point_base.header.stamp
        goal.point.x = point_base.point.x
        goal.point.y = point_base.point.y
        goal.point.z = point_base.point.z + HOVER_OFFSET
        if not all(math.isfinite(val) for val in (goal.point.x, goal.point.y, goal.point.z)):
            self.get_logger().warn('non-finite goal, not sending')
            return

        self.attempts += 1
        self.goal = goal
        self.get_logger().info(
            f'sending goal ({goal.point.x:.3f}, {goal.point.y:.3f}, {goal.point.z:.3f}) '
            f'[attempt {self.attempts}]')
        self.motion = self.arm.move_to_point(goal)

    def report_tip_error(self):
        """Action success is not enough: compare with the physically measured tip."""
        if self.measured_tip is None or self.goal is None:
            self.get_logger().warn('no measured tip pose to verify the result')
            return
        t, g = self.measured_tip.pose.position, self.goal.point
        error = math.dist((t.x, t.y, t.z), (g.x, g.y, g.z))
        self.get_logger().info(f'measured tip=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), '
                               f'distance to goal={error * 100:.1f} cm')

    # ---------- Pipeline steps ----------

    def find_marker(self, bgr):
        """Return (u, v, marker_mask) of the red marker, or None if not reliably found."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1) | cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        biggest = contours[0]
        biggest_area = cv2.contourArea(biggest)

        if biggest_area < MIN_AREA:
            return None
        if len(contours) > 1 and cv2.contourArea(contours[1]) >= SECOND_BLOB_RATIO * biggest_area:
            self.get_logger().warn('ambiguous detection: more than one significant red blob',
                                   throttle_duration_sec=1.0)
            return None

        m = cv2.moments(biggest)
        if m['m00'] == 0:
            return None
        u = m['m10'] / m['m00']
        v = m['m01'] / m['m00']

        marker_mask = np.zeros_like(mask)
        cv2.drawContours(marker_mask, [biggest], -1, 255, thickness=-1)
        return u, v, marker_mask

    def marker_depth(self, depth_msg, mask):
        """Median of valid depth values inside the marker mask, or None."""
        if depth_msg.encoding != '32FC1':
            self.get_logger().warn(f'unexpected depth encoding {depth_msg.encoding}')
            return None
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth.shape[:2] != mask.shape[:2]:
            self.get_logger().warn('depth and RGB image sizes differ')
            return None

        values = depth[mask > 0]
        # Reject nonfinite (NaN/inf), zero, negative and out-of-range values
        values = values[np.isfinite(values) & (values > MIN_DEPTH) & (values < MAX_DEPTH)]
        if values.size < MIN_DEPTH_PIXELS:
            return None
        return float(np.median(values))

    def to_3d(self, u, v, z, header):
        """Pinhole back-projection: pixel (u, v) at depth z -> point in the optical frame."""
        fx, fy, cx, cy = self.K
        point = PointStamped()
        point.header = header          # same timestamp and frame (camera_optical_frame) as the image
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point


def main():
    rclpy.init()
    node = ReachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cancel any pending motion when interrupted
        node.arm.cancel()
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()