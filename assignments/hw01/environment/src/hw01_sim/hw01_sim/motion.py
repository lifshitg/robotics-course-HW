"""Provided IK and trajectory execution, independent of target perception."""
import threading
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from hw01_interfaces.action import MoveToPoint
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import ACTION, JOINTS, TRAJECTORY_ACTION, inverse, trajectory_samples


def trajectory_goal(start, end):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    for seconds, positions, velocities, accelerations in trajectory_samples(start, end):
        ns = round(seconds * 1_000_000_000)
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=positions, velocities=velocities, accelerations=accelerations,
            time_from_start=Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)))
    return goal


class MotionHelper(Node):
    def __init__(self):
        super().__init__('hw01_motion_helper')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.joints = None
        self.joint_time = 0.0
        self.wall = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.backend = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION,
                                    callback_group=self.group)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10,
                                 callback_group=self.group)
        self.server = ActionServer(self, MoveToPoint, ACTION, self.execute,
                                   goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.get_logger().info('HW1 point helper started; waiting for joint states/controller')

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            with self.lock:
                self.joints = tuple(values[name] for name in JOINTS)
                self.joint_time = time.monotonic()

    def accept(self, goal):
        p = goal.target.point
        try:
            inverse(goal.target.header.frame_id, (p.x, p.y, p.z))
        except ValueError as error:
            self.get_logger().warning(f'Rejected goal: {error}')
            return GoalResponse.REJECT
        with self.lock:
            if (self.busy or self.joints is None or time.monotonic() - self.joint_time > 2.0
                    or not self.backend.server_is_ready()):
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    async def bounded(self, future, seconds):
        race = Future()
        guard = threading.Lock()
        def complete(done=None):
            with guard:
                if race.done():
                    return
                try:
                    race.set_result(done.result() if done is not None else None)
                except Exception:
                    race.set_result(None)
        future.add_done_callback(complete)
        timer = self.create_timer(seconds, complete, clock=self.wall, callback_group=self.group)
        try:
            return await race
        finally:
            self.destroy_timer(timer)

    async def execute(self, handle):
        upstream = None
        monitor = None
        result = MoveToPoint.Result(success=False, message='Motion failed')
        try:
            point = handle.request.target.point
            target = inverse(handle.request.target.header.frame_id, (point.x, point.y, point.z))
            with self.lock:
                start = self.joints
            if handle.is_cancel_requested:
                handle.canceled()
                result.message = 'Canceled before execution'
                return result
            handle.publish_feedback(MoveToPoint.Feedback(phase='executing joint trajectory'))
            sent = self.backend.send_goal_async(trajectory_goal(start, target))
            upstream = await self.bounded(sent, 5.0)
            if upstream is None:
                # A late acceptance must not move the arm after the point action failed.
                def cancel_late(future):
                    accepted = future.result()
                    if accepted is not None and accepted.accepted:
                        accepted.cancel_goal_async()
                sent.add_done_callback(cancel_late)
                handle.abort()
                result.message = 'Controller did not respond'
                return result
            if not upstream.accepted:
                handle.abort()
                result.message = 'Trajectory controller rejected the goal'
                return result
            cancel_sent = False
            def monitor_cancel():
                nonlocal cancel_sent
                if handle.is_cancel_requested and not cancel_sent:
                    cancel_sent = True
                    upstream.cancel_goal_async()
            monitor = self.create_timer(0.1, monitor_cancel, clock=self.wall, callback_group=self.group)
            reply = await self.bounded(upstream.get_result_async(), 60.0)
            if reply is None:
                upstream.cancel_goal_async()
                handle.abort()
                result.message = 'Trajectory timed out; check whether Gazebo is paused'
            elif handle.is_cancel_requested:
                handle.canceled()
                result.message = 'Motion canceled'
            elif (reply.status == GoalStatus.STATUS_SUCCEEDED and
                  reply.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                handle.succeed()
                result.success = True
                result.message = 'Trajectory completed'
            else:
                handle.abort()
                result.message = f'Controller error {reply.result.error_code} (status {reply.status}): {reply.result.error_string}'
            return result
        except Exception as error:
            if upstream is not None and upstream.accepted:
                upstream.cancel_goal_async()
            handle.abort()
            result.message = str(error)
            return result
        finally:
            if monitor is not None:
                self.destroy_timer(monitor)
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionHelper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
