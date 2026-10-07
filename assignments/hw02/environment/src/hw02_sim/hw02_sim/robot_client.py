"""Student-facing asynchronous interface to a single direct go-to movement."""
import threading
import time
from geometry_msgs.msg import PointStamped
from hw02_interfaces.action import GoTo
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.task import Future
from .scenario import validate_target
from .motion import ACTION


class RobotClient:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, GoTo, ACTION)
        self.lock = threading.RLock()
        self.pending = None
        self.handle = None
        self.cancel_requested = False

    def ready(self):
        return self.client.server_is_ready()

    def cancel(self):
        with self.lock:
            self.cancel_requested = True
            if self.handle is not None: self.handle.cancel_goal_async()

    def go_to(self, x, y):
        result = Future()
        with self.lock:
            try: validate_target('map', x, y)
            except (ValueError, TypeError) as error:
                result.set_result(GoTo.Result(success=False, message=str(error))); return result
            if self.pending is not None:
                result.set_result(GoTo.Result(success=False, message='This client has an active request')); return result
            if not self.ready():
                result.set_result(GoTo.Result(success=False, message='Go-to helper unavailable')); return result
            self.pending = result
            self.cancel_requested = False
        point = PointStamped(); point.header.frame_id = 'map'
        point.point.x, point.point.y = float(x), float(y)
        deadline = time.monotonic()+130.0
        timer = None
        def finish(reply):
            with self.lock:
                if result.done(): return
                self.pending = self.handle = None
                self.node.destroy_timer(timer)
                result.set_result(reply)
        def watchdog():
            if time.monotonic() >= deadline:
                self.cancel(); finish(GoTo.Result(success=False, message='Go-to helper response timed out'))
        timer = self.node.create_timer(.1, watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        def completed(future):
            try: finish(future.result().result)
            except Exception as error: finish(GoTo.Result(success=False, message=str(error)))
        def accepted(future):
            try:
                handle = future.result()
                with self.lock:
                    if not handle.accepted:
                        finish(GoTo.Result(success=False, message='Goal rejected: busy, invalid, stale data or contact')); return
                    if result.done():
                        handle.cancel_goal_async(); return
                    self.handle = handle
                    if self.cancel_requested: handle.cancel_goal_async()
                    handle.get_result_async().add_done_callback(completed)
            except Exception as error: finish(GoTo.Result(success=False, message=str(error)))
        try: self.client.send_goal_async(GoTo.Goal(target=point)).add_done_callback(accepted)
        except Exception as error: finish(GoTo.Result(success=False, message=str(error)))
        return result
