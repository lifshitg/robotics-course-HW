
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.parameter import Parameter #to use sim time
from geometry_msgs.msg import PointStamped

from tf2_ros.buffer import Buffer #for tf2 transform to base_link frame
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from hw01_sim.arm_client import ArmClient

import cv2
import numpy as np
from cv_bridge import CvBridge #bridges ros2 Image data to numpy array
#filters library that combines messages


import time

#converting time to nanoseconds
def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec

def main():
    rclpy.init()
    node = Node('markerDetector')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    bridge = CvBridge() #converter ros2 to cv for image pixel data

    camera_data = None
    #create publisher
    printed = False
    target_pub = node.create_publisher(PointStamped, '/hw01/target_camera', 10)     #publisher for the marker position in camera coordinates
    
    tf_buffer = Buffer() #creates a tf2 object to store transforms
    target_base_pub = node.create_publisher(PointStamped, '/hw01/target_base', 10) #publisher as required for marker relative to base
    tf_listener = TransformListener(tf_buffer, node)

    arm = ArmClient(node)
    motion_future = None #check

    last_pair_stamps = None
    last_fresh_wall_time = None

    def report(future):
        nonlocal motion_future
        try:
            result = future.result()
            if result.success:
                node.get_logger().info(f'Motion succeeded: {result.message}')
            else:
                node.get_logger().warning(f'Motion failed: {result.message}')
                motion_future = None
        except Exception as e:
            node.get_logger().warning(f'Motion failed: {e}')
            motion_future = None
            return
        return
        
        

    def on_calib(msg):
        nonlocal camera_data
        #validating camera
        if (not np.all(np.isfinite(msg.k))
            or msg.k[0] <= 0 or msg.k[4] <= 0
            or not msg.header.frame_id):
            node.get_logger().warning('Invalid camera calibration received')
            return

        if camera_data is None:
            node.get_logger().info(
                f'Camera calibration received: '
                f'{msg.width}x{msg.height}, '
                f'frame={msg.header.frame_id}, '
                f'fx={msg.k[0]:.2f}, '
                f'fy={msg.k[4]:.2f}'
            )
        camera_data = msg
    
    def on_images(rgb, depth):
    
        #getting global printer
        nonlocal printed, motion_future, last_pair_stamps, last_fresh_wall_time

        #calculating time for sync and rejecting difference of over 0.05 seconds
        rgb_time = stamp_to_ns(rgb.header.stamp)
        depth_time = stamp_to_ns(depth.header.stamp)
        delta_ns = abs(rgb_time - depth_time)
        if delta_ns > 50000000:
            node.get_logger().warning('Rejected: timestamp difference > 0.05s')
            return

        now_ns = node.get_clock().now().nanoseconds #checking if the images are stale (older than 0.5 seconds)
        oldest_stamp_ns = min(rgb_time, depth_time)
        if now_ns - oldest_stamp_ns > 500_000_000:
            node.get_logger().warning('Skipping goal: camera observation is stale')
            return
        
        pair_stamps = (rgb_time, depth_time)

        if pair_stamps == last_pair_stamps:
            return

        #check whether fresh camera data was missing for 2 wall-clock seconds
        current_wall_time = time.monotonic()

        camera_gap = (last_fresh_wall_time is not None and current_wall_time - last_fresh_wall_time >= 2.0)

        last_pair_stamps = pair_stamps
        last_fresh_wall_time = current_wall_time

        if camera_gap:
            node.get_logger().warning('No fresh camera data for 2 seconds - skipping this observation')
            return

        #safety checks - camera calib, images dim and depth format
        if camera_data is None: return
        if (rgb.width != depth.width
            or rgb.height != depth.height
            or rgb.width != camera_data.width
            or rgb.height != camera_data.height):
            node.get_logger().warning('Image dimensions do not match')
            return
        #frame_id — the coordinate system associated with the image.
        if (not rgb.header.frame_id
            or rgb.header.frame_id != depth.header.frame_id
            or rgb.header.frame_id != camera_data.header.frame_id):
            node.get_logger().warning('Frames do not match')
            return
        if depth.encoding.lower() != '32fc1':
            node.get_logger().warning(f'Wrong depth encoding: {depth.encoding}')
            return
        
        #added try and except
        try:
            image = bridge.imgmsg_to_cv2(rgb, "rgb8") #expected encoding
            depth_data = bridge.imgmsg_to_cv2(depth)
        except Exception as e:
            node.get_logger().error(f'Conversion failed: {e}')
            return

        #finds red
        #using relative values instead of fixed values (to find darker reds)
        r = image[:, :, 0].astype(np.int16)
        g = image[:, :, 1].astype(np.int16)
        b = image[:, :, 2].astype(np.int16)
        red = ((r > 150) & (r > 3 * g) & (r > 3 * b))
        
        #converting boolean to uint8 for OpenCV
        red = red.astype(np.uint8) * 255

        #detecting groups of red pixels
        # count - num of groups
        # labels - matrix that replaces 255 in red with the num of the group
        # stats - measurements for each connected component
        # centers - average position of all pixels in the group
        count, labels, stats, centers = cv2.connectedComponentsWithStats(red, connectivity=8)

        #keeping large red groups only and returning if there are non
        regions = []
        for i in range(1, count): #0 is the background
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= 5:
                regions.append((area, i))
        if len(regions) == 0: return

        #sort regions by area size and use the largest
        regions.sort(reverse=True)
        largest_group_id = regions[0][1]

        #approximate the center of the red region
        center_x = int(round(centers[largest_group_id][0]))
        center_y = int(round(centers[largest_group_id][1]))

        #single out the chosen group and retrieve valid depths
        chosen_group_mask = (labels == largest_group_id)
        depths = depth_data[chosen_group_mask]
        depths = depths[np.isfinite(depths) & (depths > 0) & (depths <= 10.0)]
                
        #for valid depths calculate median else return
        if len(depths) < 5: return
        redDepth = float(np.median(depths))

        #reconstruct 3D position using camera calibration
        fx = camera_data.k[0]
        fy = camera_data.k[4]
        cx = camera_data.k[2]
        cy = camera_data.k[5]
        x = (center_x - cx) * redDepth / fx
        y = (center_y - cy) * redDepth / fy
        z = redDepth

        #publish marker position in camera_optical_frame
        point = PointStamped()
        point.header = rgb.header
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        target_pub.publish(point)

        #transorms into base frame
        try:
            point_base = tf_buffer.transform(point, 'base_link')
        except TransformException as e:
            node.get_logger().warning(f'TF2 failed: {e}')
            return

        point_base.header.stamp = point.header.stamp
        target_base_pub.publish(point_base)

        if motion_future is None and arm.ready():
            #check again that the observation is still fresh before moving
            now_ns = node.get_clock().now().nanoseconds
            age_ns = now_ns - oldest_stamp_ns
            if age_ns > 500_000_000:
                node.get_logger().warning('Skipping motion: camera observation became stale')
                return

            #check wall-clock freshness
            if time.monotonic() - last_fresh_wall_time >= 2.0:
                node.get_logger().warning('No fresh camera data for 2 seconds')
                return
            
            #adding offset of 0.08
            point_base.point.z += 0.08
            motion_future = arm.move_to_point(point_base)
            motion_future.add_done_callback(report)

        #print the first successful detection
        if not printed:
            node.get_logger().info(
                f'Marker found at pixel ({center_x}, {center_y}), '
                f'depth={redDepth:.3f}m, '
                f'3D position=({x:.3f}, {y:.3f}, {z:.3f})'
            )
            printed = True

    rgb = Subscriber(node, Image, '/camera/image_raw',qos_profile_sensor_data)
    depth = Subscriber(node, Image, '/camera/depth/image_raw', qos_profile_sensor_data) #passing to synchronizer (subscriber vs node.create_subscription)
    calibration = node.create_subscription(CameraInfo, '/camera/camera_info',on_calib, qos_profile_sensor_data)

    # Approximate time synchronization explain what it is a tool made for this specifically by ros
    sync = ApproximateTimeSynchronizer([rgb, depth], 10, 0.05)
    sync.registerCallback(on_images)

    try:
        rclpy.spin(node)
    #added except
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down marker detector')
    finally:
        if motion_future is not None and not motion_future.done():
            arm.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()