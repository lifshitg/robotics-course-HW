#!/usr/bin/env python3
"""Run after launching HW1: receive RGB images and move to one fixed point."""
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hw01_sim.arm_client import ArmClient


def main():
    rclpy.init()
    node = Node('hw01_check_setup')
    arm = ArmClient(node)
    image_received = None

    # Camera data uses best-effort QoS. No image processing is needed for this check.
    def on_image(image):
        nonlocal image_received
        if image_received is None:
            print(f'Image: {image.width} x {image.height}, encoding={image.encoding}', flush=True)
        if image.width > 0 and image.height > 0 and len(image.data) >= image.step * image.height:
            image_received = time.monotonic()
    node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)

    motion = None
    deadline = time.monotonic() + 100.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if motion is None and image_received is not None and arm.ready():
                # Fixed demo point, unrelated to assignment targets. Units are metres.
                point = PointStamped()
                point.header.frame_id = 'base_link'
                point.point.x, point.point.y, point.point.z = 0.20, -0.20, 0.14
                motion = arm.move_to_point(point)
            if motion is not None and motion.done():
                break
        image_ok = image_received is not None and time.monotonic() - image_received < 2.0
        motion_ok = motion is not None and motion.done() and motion.result().success
        print(f'{"PASS" if image_ok else "FAIL"}: fresh camera image reception')
        print(f'{"PASS" if motion_ok else "FAIL"}: fixed-point arm movement')
        if motion is not None and motion.done():
            print(motion.result().message)
        return 0 if image_ok and motion_ok else 1
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
