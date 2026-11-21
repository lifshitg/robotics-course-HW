#!/usr/bin/env python3
"""HW1 arm motion node: send the detected marker position (plus hover offset) to the arm.

Listens to /hw01/target_base (marker position in base_link, from point_detector.py),
adds the hover offset to z, and sends it to the motion helper (ArmClient). A target is
only used if it is fresh, and it is cleared once sent, so an old target is never reused.

Run (after sourcing ROS + the hw01 environment, simulation and point_detector running):
    ros2 run hw01_solution arm_motion
"""
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from hw01_sim.arm_client import ArmClient

HOVER_OFFSET = 0.08     # m, added to the marker's base-frame z
MAX_AGE_SIM = 0.5       # s of simulated time before a target is stale
MAX_AGE_WALL = 2.0      # s of wall-clock time since the target arrived before it is stale
RUN_TIMEOUT = 100.0     # s of wall-clock time to wait for a successful run


def main():
    """Wait for a fresh marker position, command one arm motion, and report the result.

    Returns:
        0 if the motion succeeded, 1 otherwise (failure, timeout, or interruption).
    """
    rclpy.init()
    node = Node('arm_motion', parameter_overrides=[Parameter('use_sim_time', value=True)])
    log = node.get_logger()
    arm = ArmClient(node)

    marker_position_received: PointStamped | None = None
    received_wall: float = 0.0   # wall-clock time when the target arrived

    def on_marker_receive(marker_position: PointStamped) -> None:
        """Store the newest marker position and remember when it arrived."""
        nonlocal marker_position_received, received_wall
        log.info(f'Marker position received: ({marker_position.point.x:.3f}, '
                 f'{marker_position.point.y:.3f}, {marker_position.point.z:.3f})',
                 throttle_duration_sec=1.0)
        marker_position_received = marker_position
        received_wall = time.monotonic()

    node.create_subscription(PointStamped, '/hw01/target_base', on_marker_receive, 10)

    def target_is_fresh() -> bool:
        """True if the stored target is recent in both simulated and wall-clock time."""
        if marker_position_received is None:
            return False
        sim_age = (node.get_clock().now() - Time.from_msg(marker_position_received.header.stamp)).nanoseconds * 1e-9
        wall_age = time.monotonic() - received_wall
        return sim_age <= MAX_AGE_SIM and wall_age <= MAX_AGE_WALL

    motion = None
    deadline = time.monotonic() + RUN_TIMEOUT
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            if motion is None and marker_position_received is not None:
                if not target_is_fresh():
                    log.warn('Marker position is stale; discarding it.', throttle_duration_sec=2.0)
                    marker_position_received = None
                elif arm.ready():
                    goal = deepcopy(marker_position_received)   # do not modify the stored message
                    goal.point.z += HOVER_OFFSET
                    marker_position_received = None     # never reuse an old target
                    motion = arm.move_to_point(goal)

            if motion is not None and motion.done():
                break

        motion_ok = motion is not None and motion.done() and motion.result().success
        log.info(f'{"PASS" if motion_ok else "FAIL"}: arm motion to marker')
        if motion is not None and motion.done():
            log.info(motion.result().message)
        return 0 if motion_ok else 1
    except KeyboardInterrupt:
        return 1
    finally:
        arm.cancel()
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())