"""Reconstruct the red marker, transform it with TF2 and hover the arm tip above it."""

from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .camera_listener import CameraListener


BASE_FRAME = 'base_link'
HOVER_OFFSET_METRES = 0.08
MAX_OBSERVATION_AGE_SECONDS = 0.5
MAX_CAMERA_SILENCE_WALL_SECONDS = 2.0


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters from CameraInfo.k (row-major 3x3 matrix)."""

    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0


def intrinsics_from_camera_info(info: CameraInfo) -> Intrinsics:
    """Read and validate the intrinsic matrix (no hardcoded focal length)."""
    k = [float(value) for value in info.k]
    if len(k) != 9 or not all(math.isfinite(value) for value in k):
        raise ValueError('Camera matrix must contain nine finite values.')
    if k[0] <= 0.0 or k[4] <= 0.0:
        raise ValueError('Camera focal lengths must be positive.')
    return Intrinsics(fx=k[0], fy=k[4], cx=k[2], cy=k[5], skew=k[1])


def pixel_to_optical_point(
    u: float, v: float, depth: float, intrinsics: Intrinsics
) -> tuple[float, float, float]:
    """
    Back-project pixel (u, v) with optical-Z depth into camera_optical_frame.

    Inverts u = fx*X/Z + skew*Y/Z + cx and v = fy*Y/Z + cy, with Z = depth.
    The depth was already validated by measure_marker_depth.
    """
    y = (v - intrinsics.cy) * depth / intrinsics.fy
    x = (u - intrinsics.cx - intrinsics.skew * y / depth) * depth / intrinsics.fx
    return x, y, depth


def hover_goal(target: PointStamped) -> PointStamped:
    """Return the base_link goal HOVER_OFFSET_METRES above the marker."""
    if target.header.frame_id != BASE_FRAME:
        raise ValueError('Hover goals must be computed in base_link.')
    goal = PointStamped(header=target.header)
    goal.point.x = target.point.x
    goal.point.y = target.point.y
    goal.point.z = target.point.z + HOVER_OFFSET_METRES
    return goal


def _xyz(point) -> np.ndarray:
    """Return a Point (or PointStamped) as a numpy array."""
    point = getattr(point, 'point', point)
    return np.array([point.x, point.y, point.z], dtype=float)


class Rejected(Exception):
    """This observation cannot be used; the message says why."""


class MarkerReacher(CameraListener):
    """Turn each camera pair into a marker estimate in base_link."""

    def __init__(self) -> None:
        """Add TF2 and the target publishers."""
        super().__init__(
            'marker_reacher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )
        self.base_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_base', 10
        )

        self.latest_target_base: PointStamped | None = None
        self.last_camera_wall: float | None = None
        self._last_reason: str | None = None
        self._camera_silent = False

        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(0.5, self.check_camera_silence, clock=wall_clock)

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Estimate the marker from this pair."""
        super().on_synchronized_images(rgb, depth)
        self.last_camera_wall = time.monotonic()
        if self._camera_silent:
            self._camera_silent = False
            self.get_logger().info('Fresh camera data resumed.')
        try:
            target = self.estimate_target(rgb)
        except Rejected as reason:
            self.forget_observations()
            self.log_once(f'No target: {reason}', warning=True)
            return
        self.latest_target_base = target

    def check_camera_silence(self) -> None:
        """Wall-clock timer: forget observations after 2 s without camera data."""
        if self.last_camera_wall is None:
            return
        if time.monotonic() - self.last_camera_wall <= MAX_CAMERA_SILENCE_WALL_SECONDS:
            return
        self.forget_observations()
        if not self._camera_silent:
            self._camera_silent = True
            self.get_logger().warning('No camera data for 2 wall-clock seconds; not moving.')

    def forget_observations(self) -> None:
        """Drop the current target so it can never be reused."""
        self.latest_target_base = None

    def log_once(self, message: str, warning: bool = False) -> None:
        """Log only when the message changes, to avoid one line per frame."""
        if message == self._last_reason:
            return
        self._last_reason = message
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def estimate_target(self, rgb: Image) -> PointStamped:
        """Return the marker in base_link for this pair, or raise Rejected."""
        self.check_fresh(rgb)
        camera_point = self.reconstruct(rgb)
        self.camera_target_publisher.publish(camera_point)
        base_point = self.to_base_frame(camera_point)
        self.base_target_publisher.publish(base_point)
        cam, base = _xyz(camera_point).round(3), _xyz(base_point).round(3)
        self.log_once(f'Marker camera={tuple(cam.tolist())} base={tuple(base.tolist())}')
        return base_point

    def check_fresh(self, rgb: Image) -> None:
        """Reject observations older than 0.5 simulated seconds."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            raise Rejected('waiting for simulation time on /clock.')
        age = (now - Time.from_msg(rgb.header.stamp)).nanoseconds / 1e9
        if age > MAX_OBSERVATION_AGE_SECONDS:
            raise Rejected(f'observation is stale ({age:.2f} simulated seconds old).')
        if age < -MAX_OBSERVATION_AGE_SECONDS:
            raise Rejected('observation timestamp is in the future (clock reset?).')

    def reconstruct(self, rgb: Image) -> PointStamped:
        """Back-project the marker pixel + depth into the image's optical frame."""
        if self.latest_marker is None or self.latest_marker_depth is None:
            raise Rejected('marker or its depth is unavailable.')
        info = self.camera_info
        if info is None:
            raise Rejected('no camera calibration received yet.')
        if info.header.frame_id != rgb.header.frame_id:
            raise Rejected('camera calibration frame does not match the image frame.')
        u, v = self.latest_marker.pixel
        try:
            xyz = pixel_to_optical_point(
                u, v, self.latest_marker_depth, intrinsics_from_camera_info(info)
            )
        except ValueError as error:
            raise Rejected(f'cannot reconstruct marker: {error}') from error
        point = PointStamped(header=rgb.header)
        point.point.x, point.point.y, point.point.z = xyz
        return point

    def to_base_frame(self, camera_point: PointStamped) -> PointStamped:
        """Transform the point into base_link with TF2 at the image timestamp."""
        source = camera_point.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, source, Time.from_msg(camera_point.header.stamp)
            )
        except TransformException as error:
            raise Rejected(f'TF {source} -> {BASE_FRAME} unavailable: {error}') from error
        base_point = do_transform_point(camera_point, transform)
        base_point.header = Header(stamp=camera_point.header.stamp, frame_id=BASE_FRAME)
        if not np.all(np.isfinite(_xyz(base_point))):
            raise Rejected('transformed marker point is not finite.')
        return base_point


def main(args=None) -> None:
    """Run the marker reacher until interrupted."""
    rclpy.init(args=args)
    node = MarkerReacher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
