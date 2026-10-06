"""ROS action integration tests with a fake backend; no perception or Gazebo."""
import threading
import time

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from hw01_interfaces.action import MoveToPoint
from hw01_sim.kinematics import ACTION, HOME, JOINTS, TRAJECTORY_ACTION
from hw01_sim.motion import MotionHelper
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


def wait(future, timeout=8):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'ROS operation exceeded wall-time deadline'
    return future.result()


class Backend(Node):
    def __init__(self, enable_server=True):
        super().__init__('hw01_test_backend')
        self.group = ReentrantCallbackGroup()
        self.error = 0
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish, callback_group=self.group)
        self.server = None
        if enable_server:
            self.server = ActionServer(self, FollowJointTrajectory, TRAJECTORY_ACTION,
                                       self.execute, callback_group=self.group,
                                       cancel_callback=lambda _: CancelResponse.ACCEPT)

    def publish(self):
        self.publisher.publish(JointState(name=JOINTS, position=list(HOME)))

    def execute(self, goal):
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            if goal.is_cancel_requested:
                goal.canceled()
                return FollowJointTrajectory.Result(error_code=-1, error_string='canceled')
            time.sleep(0.02)
        if self.error:
            goal.abort()
        else:
            goal.succeed()
        return FollowJointTrajectory.Result(error_code=self.error, error_string='test controller failure')


@pytest.fixture
def setup(request):
    rclpy.init()
    enabled = getattr(request, 'param', True)
    backend, helper, client_node = Backend(enabled), MotionHelper(), Node('hw01_test_client')
    client = ActionClient(client_node, MoveToPoint, ACTION)
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (backend, helper, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_server(timeout_sec=5)
    deadline = time.monotonic() + 5
    while (helper.joints is None or (enabled and not helper.backend.server_is_ready())) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert helper.joints is not None
    assert helper.backend.server_is_ready() == enabled
    yield client, backend, helper
    executor.shutdown(timeout_sec=8)
    thread.join(timeout=8)
    client.destroy()
    helper.server.destroy()
    helper.backend.destroy()
    if backend.server is not None:
        backend.server.destroy()
    for node in (backend, helper, client_node):
        node.destroy_node()
    rclpy.shutdown()


def goal(x=0.2, y=-0.2, z=0.14, frame='base_link'):
    target = PointStamped()
    target.header.frame_id = frame
    target.point.x, target.point.y, target.point.z = float(x), float(y), float(z)
    return MoveToPoint.Goal(target=target)


@pytest.mark.parametrize('point_goal', [goal(x=2), goal(z=0.3), goal(frame='camera_optical_frame'), goal(x=0.55, y=0)])
def test_invalid_goal_rejected(setup, point_goal):
    client, _, _ = setup
    assert not wait(client.send_goal_async(point_goal)).accepted


def test_success_and_concurrent_rejection(setup):
    client, _, _ = setup
    first = wait(client.send_goal_async(goal()))
    assert first.accepted
    assert not wait(client.send_goal_async(goal())).accepted
    assert wait(first.get_result_async()).result.success


def test_cancel(setup):
    client, _, _ = setup
    handle = wait(client.send_goal_async(goal()))
    assert handle.accepted
    assert wait(handle.cancel_goal_async()).goals_canceling
    reply = wait(handle.get_result_async())
    assert not reply.result.success
    assert 'cancel' in reply.result.message.lower()


def test_controller_failure(setup):
    client, backend, _ = setup
    backend.error = -4
    handle = wait(client.send_goal_async(goal()))
    assert handle.accepted
    result = wait(handle.get_result_async()).result
    assert not result.success and 'Controller error' in result.message


@pytest.mark.parametrize('setup', [False], indirect=True)
def test_missing_controller(setup):
    client, _, helper = setup
    assert not helper.backend.server_is_ready()
    assert not wait(client.send_goal_async(goal())).accepted
