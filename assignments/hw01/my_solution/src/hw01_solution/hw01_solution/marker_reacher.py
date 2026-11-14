"""Reconstruct the red marker in the camera frame and compute a hover goal."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from .camera_listener import CameraListener


BASE_FRAME = 'base_link'
HOVER_OFFSET_METRES = 0.08


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
    """Turn each camera pair into a marker estimate in the camera frame."""

    def __init__(self) -> None:
        """Add the camera target publisher."""
        super().__init__(
            'marker_reacher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.camera_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )
        self._last_reason: str | None = None

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Estimate the marker from this pair."""
        super().on_synchronized_images(rgb, depth)
        try:
            self.estimate_target(rgb)
        except Rejected as reason:
            self.log_once(f'No target: {reason}', warning=True)

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
        """Return the marker in the camera optical frame, or raise Rejected."""
        camera_point = self.reconstruct(rgb)
        self.camera_target_publisher.publish(camera_point)
        cam = _xyz(camera_point).round(3)
        self.log_once(f'Marker camera={tuple(cam.tolist())}')
        return camera_point

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
