"""HW1 items 4, 5, 6: TF2 transform, hover goal execution, and failure handling."""
import math
import time
from typing import Optional

from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401 - required to register PointStamped with TF2

from hw01_sim.arm_client import ArmClient

HOVER_OFFSET_Z = 0.08
EXPECTED_TIP_Z = 0.14
TIP_Z_TOLERANCE = 0.01
MAX_SIM_AGE_SEC = 0.5
MAX_WALL_AGE_SEC = 2.0
TF_TIMEOUT_SEC = 0.2


class TargetExecutor:
    """Transforms detected camera points to base_link and safely commands the arm."""

    def __init__(self, node: Node):
        self.node = node
        self.arm = ArmClient(node)
        self.active_motion = None

        # Item 4: TF2 buffer/listener and debug publishers (pre-hover offset)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.pub_target_camera = node.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )
        self.pub_target_base = node.create_publisher(
            PointStamped, '/hw01/target_base', 10
        )

    def is_observation_fresh(
        self, point_camera: PointStamped, last_camera_wall_time: Optional[float]
    ) -> bool:
        """Check wall-clock (<= 2.0s) and sim-time (<= 0.5s) staleness limits."""
        if last_camera_wall_time is None:
            return False

        wall_age = time.monotonic() - last_camera_wall_time
        if wall_age > MAX_WALL_AGE_SEC:
            self.node.get_logger().warn(
                f'Camera stream stalled ({wall_age:.2f}s wall-clock); skipping.'
            )
            return False

        now_sim = self.node.get_clock().now()
        obs_sim = Time.from_msg(point_camera.header.stamp)
        sim_age = (now_sim - obs_sim).nanoseconds / 1e9

        if sim_age > MAX_SIM_AGE_SEC:
            self.node.get_logger().warn(
                f'Observation is stale ({sim_age:.2f}s sim time); skipping.'
            )
            return False
        if sim_age < 0.0:
            self.node.get_logger().warn(
                'Observation timestamp is ahead of sim clock; skipping.'
            )
            return False

        return True

    def transform_to_base(
        self, point_camera: PointStamped
    ) -> Optional[PointStamped]:
        """Item 4: Publish camera point, transform to base_link via TF2, publish base point."""
        self.pub_target_camera.publish(point_camera)

        try:
            point_base = self.tf_buffer.transform(
                point_camera,
                'base_link',
                timeout=Duration(seconds=TF_TIMEOUT_SEC),
            )
        except TransformException as ex:
            self.node.get_logger().warn(f'TF2 transform failed: {ex}')
            return None

        self.pub_target_base.publish(point_base)
        return point_base

    def build_hover_goal(
        self, point_base: PointStamped
    ) -> Optional[PointStamped]:
        """Item 5: Add 0.08 m hover offset in base_link and validate coordinates."""
        goal = PointStamped()
        goal.header = point_base.header
        goal.header.frame_id = 'base_link'
        goal.point.x = point_base.point.x
        goal.point.y = point_base.point.y
        goal.point.z = point_base.point.z + HOVER_OFFSET_Z

        coords = (goal.point.x, goal.point.y, goal.point.z)
        if not all(math.isfinite(v) for v in coords):
            self.node.get_logger().warn('Goal contains non-finite coordinates; skipping.')
            return None

        if abs(goal.point.z - EXPECTED_TIP_Z) > TIP_Z_TOLERANCE:
            self.node.get_logger().warn(
                f'Goal Z ({goal.point.z:.3f} m) outside allowed range around {EXPECTED_TIP_Z} m.'
            )
            return None

        return goal

    def process_observation(
        self,
        point_camera: Optional[PointStamped],
        last_camera_wall_time: Optional[float],
    ) -> bool:
        """Validate, transform, and send a new goal if no motion is currently active.

        Returns True if a new motion was started, False otherwise.
        """
        # Item 6: Only one motion may be active at a time
        if self.active_motion is not None:
            return False

        # Item 6: Never reuse an earlier target if the current observation is missing
        if point_camera is None:
            return False

        if not self.is_observation_fresh(point_camera, last_camera_wall_time):
            return False

        point_base = self.transform_to_base(point_camera)
        if point_base is None:
            return False

        goal = self.build_hover_goal(point_base)
        if goal is None:
            return False

        if not self.arm.ready():
            self.node.get_logger().warn('ArmClient is not ready yet.')
            return False

        self.node.get_logger().info(
            f'Sending hover goal: ({goal.point.x:.3f}, {goal.point.y:.3f}, {goal.point.z:.3f})'
        )
        self.active_motion = self.arm.move_to_point(goal)
        return True

    def check_active_motion(self) -> Optional[bool]:
        """Item 6: Poll active motion future and report outcome when done.

        Returns True/False once finished, or None if idle/still running.
        """
        if self.active_motion is None or not self.active_motion.done():
            return None

        result = self.active_motion.result()
        self.active_motion = None

        if result.success:
            self.node.get_logger().info(f'Motion SUCCEEDED: {result.message}')
        else:
            self.node.get_logger().error(f'Motion FAILED: {result.message}')

        return result.success

    def cancel_motion(self) -> None:
        """Item 6: Cancel any pending motion on node shutdown/interrupt."""
        if self.active_motion is not None and not self.active_motion.done():
            self.node.get_logger().info('Canceling pending motion...')
        self.arm.cancel()
        self.active_motion = None