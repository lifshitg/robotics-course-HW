"""Single-destination direct steering; no path storage, planning or avoidance."""
import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from hw02_interfaces.action import GoTo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool
from .scenario import steering, validate_target

ACTION = '/hw02/go_to'


def xy_yaw(message):
    p, q = message.pose.position, message.pose.orientation
    return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class MotionHelper(Node):
    def __init__(self):
        super().__init__('hw02_motion_helper')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.pose = None
        self.pose_time = self.contact_time = 0.0
        self.collision = False
        self.busy = False
        self.active = None
        self.finished = None
        self.target = None
        self.started = 0.0
        self.clock_seen = None
        self.clock_changed = time.monotonic()
        self.settled = None
        self.command = self.create_publisher(Twist, '/hw02/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_pose, 10, callback_group=self.group)
        self.create_subscription(Bool, '/hw02/collision', self.on_contact, 10, callback_group=self.group)
        self.server = ActionServer(self, GoTo, ACTION, self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.timer = self.create_timer(.05, self.tick, clock=self.wall, callback_group=self.group)

    def on_pose(self, message):
        values = xy_yaw(message)
        if message.header.frame_id == 'map' and all(math.isfinite(v) for v in values):
            with self.lock:
                self.pose, self.pose_time = values, time.monotonic()

    def on_contact(self, message):
        with self.lock:
            self.contact_time = time.monotonic()
            self.collision |= message.data

    def unavailable(self):
        now = time.monotonic()
        if self.collision: return 'Obstacle contact latched; restart the simulation'
        if self.pose is None or now-self.pose_time > 1.0: return 'Robot localization unavailable or stale'
        if now-self.contact_time > 1.0: return 'Contact monitoring unavailable or stale'
        return None

    def accept(self, goal):
        p = goal.target.point
        try: validate_target(goal.target.header.frame_id, p.x, p.y, p.z)
        except ValueError as error:
            self.get_logger().warning(str(error)); return GoalResponse.REJECT
        with self.lock:
            if self.busy or self.unavailable(): return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        with self.lock:
            self.active = handle
            self.finished = Future()
            self.target = (handle.request.target.point.x, handle.request.target.point.y)
            self.started = self.clock_changed = time.monotonic()
            self.clock_seen = self.get_clock().now().nanoseconds
            self.settled = None
            result_future = self.finished
        return await result_future

    def stop(self):
        self.command.publish(Twist())

    def finish(self, success, message, canceled=False):
        self.stop()
        handle, future = self.active, self.finished
        if canceled: handle.canceled()
        elif success: handle.succeed()
        else: handle.abort()
        self.active = self.finished = None
        self.busy = False
        future.set_result(GoTo.Result(success=success, message=message))

    def tick(self):
        with self.lock:
            if self.active is None: return
            now = time.monotonic()
            if self.active.is_cancel_requested:
                self.finish(False, 'Movement canceled', canceled=True); return
            problem = self.unavailable()
            if problem:
                self.finish(False, problem); return
            clock = self.get_clock().now().nanoseconds
            if clock != self.clock_seen:
                self.clock_seen, self.clock_changed = clock, now
            if now-self.clock_changed > 2.0:
                self.finish(False, 'Simulation clock stalled'); return
            if now-self.started > 120.0:
                self.finish(False, 'Destination timed out after 120 wall seconds'); return
            linear, angular, distance = steering(self.pose, self.target)
            self.active.publish_feedback(GoTo.Feedback(distance_remaining=distance))
            command = Twist(); command.linear.x, command.angular.z = linear, angular
            self.command.publish(command)
            if distance <= .02:
                if self.settled is None: self.settled = clock
                if (clock-self.settled)/1e9 >= .2:
                    self.finish(True, 'Destination reached; robot stopped')
            else: self.settled = None


def main(args=None):
    rclpy.init(args=args); node = MotionHelper(); executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        # Launch/terminal shutdown can deliver SIGINT twice. Let cleanup finish.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok(): node.stop()
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
