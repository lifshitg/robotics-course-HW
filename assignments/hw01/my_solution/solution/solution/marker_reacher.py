import numpy as np
import time
import math

import rclpy
from rclpy.qos       import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.duration  import Duration
from rclpy.node      import Node
from rclpy.time      import Time

import tf2_geometry_msgs
import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from hw01_sim.arm_client import ArmClient

MAX_STAMP_DIFF_NS = 50 * 1000000 # 50ms
MAX_OBS_SIM_AGE_NS = 500 * 1000000  # 500ms in ns
HOVER_TARGET_Z = 0.14
HOVER_Z_TOLERANCE = 0.01

def stamp_to_ns(stamp) -> int:
    return Time.from_msg(stamp).nanoseconds


class MarkerReacherNode(Node):
    def __init__(self):
        super().__init__('marker_reacher')
        self.get_logger().info('Starting MarkerReacherNode')

        # make sure 'use_sim_time' is True
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self._last_paired_key = None
        self._last_cam_wall_time = None

        self.create_subscriber(Image, '/camera/image_raw', self._on_rgb_update)
        self.create_subscriber(Image, '/camera/depth/image_raw', self._on_depth_update)
        self.create_subscriber(CameraInfo, '/camera/camera_info', self._on_camera_info_update)

        self.target_cam_pub = self.create_publisher(PointStamped, '/hw01/target_camera', 10)
        self.target_base_pub = self.create_publisher(PointStamped, '/hw01/target_base', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm = ArmClient(self)
        self.motion_future = None
        self._last_sent_goal = None

        # create a timer that runs every 50ms
        self.timer = self.create_timer(0.05, self.run_logic)
        self.get_logger().info('MarkerReacherNode started!')

    def run_logic(self):
        # Get valid pair of imgs
        current = self.get_current_imgs()
        if current is None:
            return
        rgb_msg, depth_msg, diff_ns = current

        # Get camera info
        camera_info = self.latest_camera_info
        if camera_info is None:
            self.get_logger().debug('No camera info yet.')
            return

        # Get img from msg
        rgb_img = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape((rgb_msg.height, rgb_msg.width, 3))
        depth_img = np.frombuffer(depth_msg.data, dtype='<f4').reshape((depth_msg.height, depth_msg.width))

        # Find the red marker
        marker = self._get_red_marker(rgb_img)
        if marker is None:
            self.get_logger().debug("Failed to find Red Marker")
            return
        u, v = marker
        self.get_logger().debug(f"Red marker Center is at: [{u}, {v}]")

        # Convert pixels to 3D 'camera_optical_frame' cors
        red_marker_point = self._get_3d_camera_optical_frame(u, v, depth_img, camera_info)
        if red_marker_point is None:
            return
        self.get_logger().debug(f"[camera_link] Red Marker: {red_marker_point}")
        red_marker_camera_cors = self.create_point_stamped(red_marker_point, rgb_msg.header)
        self.publish_camera_point(red_marker_camera_cors)

        # Convert 'camera_optical_frame' to 'base_link' cors
        red_marker_base_link_cors = self.transform_camera_cors_to_base_link(red_marker_camera_cors)
        if red_marker_base_link_cors is None:
            return
        self.publish_base_point(red_marker_base_link_cors)
        red_marker_base_link_cors.point.z += 0.08

        self.send_goal_to_arm_if_should(red_marker_base_link_cors)

    def get_current_imgs(self):
        matched = self._get_matched_imgs()
        if matched is None:
            self.get_logger().debug('No RGB/depth pair within 50 ms yet.')
            return
        rgb_msg, depth_msg, diff_ns = matched

        # Check if data fresh (< 2s)
        now_wall = time.monotonic()
        if self._last_cam_wall_time is None or (now_wall - self._last_cam_wall_time) > 2.0:
            self.get_logger().warn("No fresh camera data within 2s, skipping..")
            return None

        # Check if pair is too old ( > 0.5s)
        rgb_ns = stamp_to_ns(rgb_msg.header.stamp)
        depth_ns = stamp_to_ns(depth_msg.header.stamp)
        obs_sim_ns = max(rgb_ns, depth_ns)
        now_sim_ns = self.get_clock().now().nanoseconds
        if now_sim_ns <= 0:
            self.get_logger().warn("Simulation clock not active yet.")
            return None

        sim_age_ns = now_sim_ns - obs_sim_ns
        if sim_age_ns > MAX_OBS_SIM_AGE_NS:
            self.get_logger().debug("Imgs pair too old (>0.5s), skipping...")
            return None

        # Check that we don't handle the same pair twice
        pair_key = (rgb_ns, depth_ns)
        if pair_key == self._last_paired_key:
            return None

        self._last_paired_key = pair_key
        return rgb_msg, depth_msg, diff_ns


    def send_goal_to_arm_if_should(self, goal):
        if not self._is_valid_hover_goal(goal):
            return
        if self._already_there(goal):
            return

        if self.arm.ready() and self.motion_future is None:
           self.get_logger().info(f"Moving arm to: {str(goal)}")
           self.motion_future = self.arm.move_to_point(goal)
           if self.motion_future is None:
               return
           self._last_sent_goal = (float(goal.point.x), float(goal.point.y), float(goal.point.z))
           self.motion_future.add_done_callback(self._on_motion_done)
        else:
            self.get_logger().debug("Arm is not ready or notion already in progress, skipping...")

    def destroy_node(self):
        self.arm.cancel()
        super().destroy_node()

    def create_point_stamped(self, point, header):
        pointStamped = PointStamped()
        pointStamped.header.stamp = header.stamp
        pointStamped.header.frame_id = header.frame_id
        pointStamped.point.x = float(point[0])
        pointStamped.point.y = float(point[1])
        pointStamped.point.z = float(point[2])
        return pointStamped

    def publish_camera_point(self, pointStamped):
        self.target_cam_pub.publish(pointStamped)

    def publish_base_point(self, pointStamped):
        self.target_base_pub.publish(pointStamped)

    def transform_camera_cors_to_base_link(self, pointStamped):
        point_as_base_link_cors = None
        try:
            point_as_base_link_cors = self.tf_buffer.transform(pointStamped, 'base_link', timeout=Duration(seconds=0.2))
        except TransformException as ex:
            self.get_logger().warn(f'Failed to transform {pointStamped.header.frame_id} -> base_link: {ex}',)
        return point_as_base_link_cors

    def create_subscriber(self, update_type, topic, on_update_func):
        self.create_subscription(update_type, topic, on_update_func, qos_profile_sensor_data)
        self.get_logger().info(f'Subscribed to {topic}')

    def _on_rgb_update(self, msg: Image):
        self.get_logger().debug(f'RGB frame: {msg.width}x{msg.height}, encoding={msg.encoding}')
        self._on_camera_img_update()
        self.latest_rgb = msg

    def _on_depth_update(self, msg: Image):
        self.get_logger().debug(f'Depth frame: {msg.width}x{msg.height}, encoding={msg.encoding}')
        self._on_camera_img_update()
        self.latest_depth = msg

    def _on_camera_img_update(self):
        self._last_cam_wall_time = time.monotonic()

    def _on_camera_info_update(self, msg: CameraInfo):
        self.latest_camera_info = msg
        self.get_logger().debug(f'CameraInfo updated: {msg.width}x{msg.height} {str(msg.k)}')

    def _get_matched_imgs(self):
        # Store values because they can get changed during update
        latest_rgb = self.latest_rgb
        latest_depth = self.latest_depth

        if latest_rgb is None or latest_depth is None:
            return None

        rgb_ns = stamp_to_ns(latest_rgb.header.stamp)
        depth_ns = stamp_to_ns(latest_depth.header.stamp)
        diff_ns = abs(rgb_ns - depth_ns)

        if diff_ns <= MAX_STAMP_DIFF_NS:
            return latest_rgb, latest_depth, diff_ns
        return None

    def _get_red_marker(self, rgb_img):
        red_color_mask = (rgb_img[:, :, 0] > 150) & (rgb_img[:, :, 1] < 10) & (rgb_img[:, :, 2] < 10)
        ys, xs = np.where(red_color_mask)
        if xs.size == 0 or ys.size == 0:
            return None
        x = int(round(float(xs.mean())))
        y = int(round(float(ys.mean())))
        return x, y

    def _get_pixel_depth(self, u, v, depth_img, radius=2):
        h, w = depth_img.shape[:2]
        u0 = max(0, u - radius)
        v0 = max(0, v - radius)
        u1 = min(w, u + radius + 1)
        v1 = min(h, v + radius + 1)
        around_pixel = depth_img[v0:v1, u0:u1]
        filterred = around_pixel[np.isfinite(around_pixel) & (around_pixel > 0.0)]
        if filterred.size == 0:
            self.get_logger().debug(f"Failed to find pixels with valid values around point: [{u}, {v}]")
            return None
        return float(np.median(filterred))

    def _get_3d_camera_optical_frame(self, u, v, depth_img, camera_info):
        z = self._get_pixel_depth(u, v, depth_img)
        if z is None:
            return None

        fx = float(camera_info.k[0])
        cx = float(camera_info.k[2])
        fy = float(camera_info.k[4])
        cy = float(camera_info.k[5])

        x = ((float(u) - cx) * z) / fx
        y = ((float(v) - cy) * z) / fy
        return (x, y, float(z))

    def _on_motion_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Motion future raised an exception: {exc}')
            self._last_sent_goal = None
            self.motion_future = None
            return

        if not result.success:
            self._last_sent_goal = None

        self.motion_future = None
        self.get_logger().info(f'Motion finished with result: {result}')

    def _is_valid_hover_goal(self, point_stamped: PointStamped) -> bool:
        if point_stamped.header.frame_id != 'base_link':
            self.get_logger().warn(f"Invalid goal frame_id '{point_stamped.header.frame_id}', expected 'base_link'.")
            return False

        x = float(point_stamped.point.x)
        y = float(point_stamped.point.y)
        z = float(point_stamped.point.z)

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            self.get_logger().warn(f"Non-finite goal coordinates: ({x}, {y}, {z}).")
            return False

        if abs(z - HOVER_TARGET_Z) > HOVER_Z_TOLERANCE:
            self.get_logger().warn(f"Goal z={z}m outside of allowed range: [{HOVER_TARGET_Z - HOVER_Z_TOLERANCE:.2f}, {HOVER_TARGET_Z + HOVER_Z_TOLERANCE:.2f}]m.")
            return False

        return True

    def _already_there(self, goal, min_delta = 0.02):
        if self._last_sent_goal is None:
            self.get_logger().debug("last goal is none!")
            return False

        pos = (float(goal.point.x), float(goal.point.y), float(goal.point.z))
        dist = math.dist(pos, self._last_sent_goal)
        if dist < min_delta:
            self.get_logger().debug(f"Goal unchanged (delta={dist:.5f}m < {min_delta}m), skipping goal update.")
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = MarkerReacherNode()
    try:
        rclpy.spin(node)
    except:
        node.destroy_node()

if __name__ == '__main__':
    main()
