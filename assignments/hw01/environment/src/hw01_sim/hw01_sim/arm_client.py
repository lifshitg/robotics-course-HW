"""Small provided interface for commanding the simulated arm."""
from copy import deepcopy
import time

from hw01_interfaces.action import MoveToPoint
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.task import Future

from .kinematics import ACTION, inverse


class ArmClient:
    """Use from a spinning ROS node. Futures return success/message result objects."""
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, MoveToPoint, ACTION)
        self.busy = False
        self.handle = None
        self.cancel_requested = False

    def ready(self):
        return self.client.server_is_ready()

    def cancel(self):
        self.cancel_requested = True
        if self.handle is not None:
            self.handle.cancel_goal_async()

    def move_to_point(self, point):
        """Return Future[MoveToPoint.Result]; at most one request per client."""
        result = Future()
        def failed(message):
            result.set_result(MoveToPoint.Result(success=False, message=message))
        if self.busy:
            failed('This client already has an active goal')
            return result
        try:
            inverse(point.header.frame_id, (point.point.x, point.point.y, point.point.z))
        except ValueError as error:
            failed(str(error))
            return result
        if not self.ready():
            failed('Motion helper unavailable; launch the environment first')
            return result
        self.busy = True
        self.cancel_requested = False
        deadline = time.monotonic() + 75.0
        timer = None

        def finish(reply):
            nonlocal timer
            if result.done():
                return
            self.busy = False
            self.handle = None
            if timer is not None:
                self.node.destroy_timer(timer)
                timer = None
            result.set_result(reply)

        def watchdog():
            if time.monotonic() >= deadline:
                self.cancel()
                finish(MoveToPoint.Result(success=False, message='Motion helper timed out'))
        timer = self.node.create_timer(0.2, watchdog, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

        def completed(future):
            try:
                finish(future.result().result)
            except Exception as error:
                finish(MoveToPoint.Result(success=False, message=str(error)))

        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    finish(MoveToPoint.Result(success=False,
                           message='Goal rejected: helper busy, joint data stale, or controller unavailable'))
                    return
                if result.done():
                    handle.cancel_goal_async()
                    return
                self.handle = handle
                if self.cancel_requested:
                    handle.cancel_goal_async()
                handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                finish(MoveToPoint.Result(success=False, message=str(error)))
        self.client.send_goal_async(MoveToPoint.Goal(target=deepcopy(point))).add_done_callback(accepted)
        return result
