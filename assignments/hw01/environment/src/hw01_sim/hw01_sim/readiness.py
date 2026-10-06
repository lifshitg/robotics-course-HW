"""Detailed installation check. Contains no target detection or reaching solution."""
import time
import math

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge, CvBridgeError
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .arm_client import ArmClient
from .kinematics import DEMO, HOME, JOINTS, TRAJECTORY_ACTION
from .motion import trajectory_goal


class Readiness(Node):
    def __init__(self):
        super().__init__('hw01_readiness')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.data = {}
        self.first_clock = None
        self.clock_progress = False
        self.bridge = CvBridge()
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        for key, topic, kind in [
            ('rgb', '/camera/image_raw', Image), ('depth', '/camera/depth/image_raw', Image),
            ('info', '/camera/camera_info', CameraInfo), ('joints', '/joint_states', JointState),
        ]:
            self.create_subscription(kind, topic, lambda msg, k=key: self.store(k, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.arm = ArmClient(self)
        self.controller = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)
        self.list_controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')

    def store(self, key, message):
        self.data[key] = (message, time.monotonic())

    def on_clock(self, message):
        now = message.clock.sec + message.clock.nanosec * 1e-9
        if self.first_clock is None:
            self.first_clock = now
        self.clock_progress = self.clock_progress or now - self.first_clock > 0.5
        self.store('clock', message)

    def wait(self, condition, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
        return False

    def fresh(self, key):
        return key in self.data and time.monotonic() - self.data[key][1] < 2.0

    def checks(self):
        flags = {'advancing simulation clock': self.fresh('clock') and self.clock_progress,
                 'RGB camera': False, 'aligned depth camera': False, 'camera calibration': False,
                 'joint feedback': False, 'TF connections': False,
                 'trajectory action': self.controller.server_is_ready(),
                 'point helper': self.arm.ready()}
        try:
            rgb = self.data['rgb'][0]
            depth = self.data['depth'][0]
            info = self.data['info'][0]
            flags['RGB camera'] = (self.fresh('rgb') and rgb.width == 320 and rgb.height == 240
                                   and rgb.encoding in ['rgb8', 'bgr8']
                                   and len(rgb.data) == rgb.step * rgb.height
                                   and rgb.header.frame_id == 'camera_optical_frame')
            array = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            flags['aligned depth camera'] = (self.fresh('depth') and depth.encoding == '32FC1'
                and (depth.width, depth.height) == (rgb.width, rgb.height)
                and depth.header.frame_id == rgb.header.frame_id
                and bool(np.any(np.isfinite(array) & (array > 0))))
            flags['camera calibration'] = (self.fresh('info') and info.k[0] > 0 and info.k[4] > 0
                and info.width == rgb.width and info.height == rgb.height
                and info.header.frame_id == rgb.header.frame_id)
        except (KeyError, ValueError, TypeError, CvBridgeError):
            pass
        if self.fresh('joints'):
            values = dict(zip(self.data['joints'][0].name, self.data['joints'][0].position))
            flags['joint feedback'] = all(name in values and math.isfinite(values[name]) for name in JOINTS)
        try:
            self.tf.lookup_transform('base_link', 'camera_optical_frame', Time())
            self.tf.lookup_transform('base_link', 'tool0', Time())
            flags['TF connections'] = True
        except TransformException:
            pass
        return flags

    def controllers_active(self):
        # Actions can be advertised during configuration, before activation finishes.
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and rclpy.ok():
            if self.list_controllers.service_is_ready():
                future = self.list_controllers.call_async(ListControllers.Request())
                if self.wait(future.done, min(5.0, deadline-time.monotonic())):
                    active = {c.name for c in future.result().controller if c.state == 'active'}
                    if {'joint_state_broadcaster', 'arm_controller'} <= active:
                        return True
            self.wait(lambda: False, 0.2)
        return False

    def move_joints(self, target):
        values = dict(zip(self.data['joints'][0].name, self.data['joints'][0].position))
        sent = self.controller.send_goal_async(trajectory_goal([values[name] for name in JOINTS], target))
        if not self.wait(sent.done, 5.0) or not sent.result().accepted:
            return False
        handle = sent.result()
        done = handle.get_result_async()
        if not self.wait(done.done, 60.0):
            handle.cancel_goal_async()
            self.wait(lambda: False, 0.5)
            return False
        if done.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False
        def settled():
            if not self.fresh('joints'):
                return False
            values = dict(zip(self.data['joints'][0].name, self.data['joints'][0].position))
            return all(abs(values[name] - q) < 0.02 for name, q in zip(JOINTS, target))
        return self.wait(settled, 5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Readiness()
    try:
        print('Waiting up to 120 seconds for camera, clock, TF and controllers...', flush=True)
        node.wait(lambda: all(node.checks().values()), 120.0)
        flags = node.checks()
        flags['active controllers'] = all(flags.values()) and node.controllers_active()
        for label, ok in flags.items():
            print(f'{"PASS" if ok else "FAIL"}: {label}', flush=True)
        if not all(flags.values()):
            print('Not ready. Follow the troubleshooting table in environment/README.md.')
            return 1
        home_ok = node.move_joints(HOME)
        demo_ok = home_ok and node.move_joints(DEMO)
        return_ok = node.move_joints(HOME)
        for label, ok in [('home position', home_ok), ('measured demonstration movement', demo_ok),
                          ('return home', return_ok)]:
            print(f'{"PASS" if ok else "FAIL"}: {label}', flush=True)
        return 0 if home_ok and demo_ok and return_ok else 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
