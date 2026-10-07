"""Manual single-destination command."""
import argparse
import time
import rclpy
from .robot_client import RobotClient


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True); parser.add_argument('--y', type=float, required=True)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args); node = rclpy.create_node('hw02_go_to_cli'); robot = RobotClient(node)
    future = None
    try:
        deadline = time.monotonic()+10
        while not robot.ready() and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
        future = robot.go_to(options.x, options.y)
        while rclpy.ok() and not future.done(): rclpy.spin_once(node, timeout_sec=.1)
        result = future.result(); print(('PASS: ' if result.success else 'FAIL: ')+result.message)
        return 0 if result.success else 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): return 1
    finally:
        robot.cancel()
        deadline = time.monotonic()+2
        while rclpy.ok() and future is not None and not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
