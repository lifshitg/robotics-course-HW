#!/usr/bin/env python3
"""HW1 integrated node: perception (partner) + TargetExecutor (items 4-6)."""
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from target_executor import TargetExecutor


def find_marker(cvb: CvBridge, image: Image):
    cvimage = cvb.imgmsg_to_cv2(image, desired_encoding='bgr8')
    lower_red = np.array([0, 0, 150])
    upper_red = np.array([60, 60, 255])

    mask = cv2.inRange(cvimage, lower_red, upper_red)

    def is_contour_circular(con) -> bool:
        p = cv2.arcLength(con, True)
        a = cv2.contourArea(con)
        if p == 0: return False
        circularity = 4*np.pi*(a/(p**2))
        return 0.6 < circularity < 1.2
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circular_cons = [c for c in contours if is_contour_circular(c)]
    if len(circular_cons) == 0: return None

    mcon = max(circular_cons, key=cv2.contourArea)

    M = cv2.moments(mcon)
    if M['m00'] > 0:
        return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
    return None


def image_to_3d(K, cix: int, ciy: int, cz: float) -> tuple[float, float]:
    cx = cz * float(cix - K[0][2]) / K[0][0]
    cy = cz * float(ciy - K[1][2]) / K[1][1]
    return (cx, cy)


def main():
    rclpy.init()
    node = Node(
        'hw01_solution',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    executor = TargetExecutor(node)
    cvb = CvBridge()

    image_received = None
    depth_received = None

    raw_image = None
    depth_image = None
    camera_info = None

    def on_image(image: Image):
        nonlocal image_received, raw_image
        if image_received is None:
            print(f'Image: {image.width} x {image.height}, encoding={image.encoding}', flush=True)
        if image.width > 0 and image.height > 0 and len(image.data) >= image.step * image.height:
            image_received = time.monotonic()
            raw_image = image

    def on_depth(depth: Image):
        nonlocal depth_received, depth_image
        if depth_received is None:
            print(f'Depth: {depth.width} x {depth.height}, encoding={depth.encoding}', flush=True)
        if depth.width > 0 and depth.height > 0 and len(depth.data) >= depth.step * depth.height:
            depth_received = time.monotonic()
            depth_image = depth

    def on_info(info: CameraInfo):
        nonlocal camera_info
        if camera_info is None:
            print('Info received', flush=True)
        camera_info = info

    node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw', on_depth, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/camera_info', on_info, qos_profile_sensor_data)

    motion_result = None
    deadline = time.monotonic() + 60.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if an active motion has finished (item 6)
            motion_result = executor.check_active_motion()
            if motion_result is not None:
                break

            # Process fresh observation only when no motion is active
            if (
                executor.active_motion is None
                and raw_image is not None
                and depth_image is not None
                and camera_info is not None
            ):
                cvdepth = cvb.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
                marker = find_marker(cvb, raw_image)
                if marker is None:
                    print("Marker is nonexistant/partially occluded")
                    return 1

                cix, ciy = marker
                cz = float(cvdepth[ciy][cix])
                if not math.isfinite(cz) or cz <= 0.0:
                    continue

                K = np.reshape(np.array(camera_info.k), (3, 3))
                cx, cy = image_to_3d(K, cix, ciy, cz)

                point_camera = PointStamped()
                point_camera.header = raw_image.header
                point_camera.point.x = float(cx)
                point_camera.point.y = float(cy)
                point_camera.point.z = cz

                last_wall = min(image_received, depth_received)
                executor.process_observation(point_camera, last_wall)

        print(f'{"PASS" if motion_result else "FAIL"}: target reach execution')
        return 0 if motion_result else 1

    except KeyboardInterrupt:
        return 1

    finally:
        executor.cancel_motion()
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
