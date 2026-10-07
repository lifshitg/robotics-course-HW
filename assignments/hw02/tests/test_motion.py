"""Isolated ROS tests of the provided helper/client; no student planning code."""
import math
import threading
import time
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rosgraph_msgs.msg import Clock
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from hw02_sim.motion import MotionHelper
from hw02_sim.robot_client import RobotClient


def wait(future,seconds=8):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline:time.sleep(.01)
    assert future.done(),'wall-time deadline exceeded'
    return future.result()


class Plant(Node):
    def __init__(self):
        super().__init__('hw02_test_plant')
        self.x,self.y,self.yaw=1.,3.,0.
        self.command=Twist();self.localized=True;self.paused=False;self.hit=False;self.sim=0.
        self.pose=self.create_publisher(PoseStamped,'/hw02/robot_pose',10)
        self.contact=self.create_publisher(Bool,'/hw02/collision',10)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.create_subscription(Twist,'/hw02/cmd_vel',self.on_command,10)
        self.last=time.monotonic()
        self.create_timer(.02,self.tick,clock=RclClock(clock_type=ClockType.STEADY_TIME))
    def on_command(self,msg):self.command=msg
    def tick(self):
        now=time.monotonic();dt=now-self.last;self.last=now
        if not self.paused:
            self.sim+=dt
            self.yaw+=self.command.angular.z*dt
            self.x+=self.command.linear.x*math.cos(self.yaw)*dt
            self.y+=self.command.linear.x*math.sin(self.yaw)*dt
        clock=Clock();clock.clock.sec=int(self.sim);clock.clock.nanosec=int((self.sim%1)*1e9)
        self.clock.publish(clock)
        if self.localized:
            pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=clock.clock
            pose.pose.position.x,pose.pose.position.y=self.x,self.y
            pose.pose.orientation.z,pose.pose.orientation.w=math.sin(self.yaw/2),math.cos(self.yaw/2)
            self.pose.publish(pose)
        self.contact.publish(Bool(data=self.hit))


@pytest.fixture
def rig():
    rclpy.init();plant=Plant();helper=MotionHelper();node=Node('hw02_test_client')
    helper.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    robot=RobotClient(node)
    executor=MultiThreadedExecutor(num_threads=6)
    for item in (plant,helper,node):executor.add_node(item)
    stopping=threading.Event()
    def spin():
        while not stopping.is_set():executor.spin_once(timeout_sec=.02)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    deadline=time.monotonic()+5
    while (not robot.ready() or helper.unavailable()) and time.monotonic()<deadline:time.sleep(.02)
    assert robot.ready() and not helper.unavailable()
    yield robot,plant,helper
    robot.cancel();time.sleep(.3)
    for item in (plant,helper,node):
        for timer in item.timers:timer.cancel()
    time.sleep(.2)  # Drain callbacks before destroying the executor's guard condition.
    stopping.set();executor.wake();thread.join(timeout=5)
    assert not thread.is_alive()
    executor.shutdown(timeout_sec=5)
    robot.client.destroy();helper.server.destroy()
    for item in (plant,helper,node):item.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('x,y',[(float('nan'),3),(float('inf'),3),(0,3),(9,3)])
def test_invalid_targets(rig,x,y):
    robot,_,_=rig
    assert not wait(robot.go_to(x,y)).success


def test_success_and_stop(rig):
    robot,plant,_=rig
    assert wait(robot.go_to(1.04,3)).success
    assert abs(plant.x-1.04)<=.02
    assert plant.command.linear.x==0 and plant.command.angular.z==0


def test_client_concurrency_and_cancel(rig):
    robot,plant,_=rig
    first=robot.go_to(2,3)
    assert not wait(robot.go_to(1.1,3)).success
    time.sleep(.2);robot.cancel()
    result=wait(first)
    assert not result.success and 'cancel' in result.message.lower()
    time.sleep(.1);assert plant.command.linear.x==0


def test_server_concurrency(rig):
    robot,_,helper=rig
    from hw02_sim.scenario import validate_target
    from hw02_interfaces.action import GoTo
    from geometry_msgs.msg import PointStamped
    first=robot.go_to(2,3);time.sleep(.2)
    goal=GoTo.Goal(target=PointStamped())
    goal.target.header.frame_id='map';goal.target.point.x=1.1;goal.target.point.y=3.
    assert not wait(robot.client.send_goal_async(goal)).accepted
    robot.cancel();assert not wait(first).success


def test_localization_loss_stops_motion(rig):
    robot,plant,_=rig
    first=robot.go_to(2,3);time.sleep(.2);plant.localized=False
    result=wait(first)
    assert not result.success and 'localization' in result.message.lower()
    assert plant.command.linear.x==0
    assert not wait(robot.go_to(1.1,3)).success


def test_contact_stops_and_latches(rig):
    robot,plant,_=rig
    first=robot.go_to(2,3);time.sleep(.2);plant.hit=True
    result=wait(first)
    assert not result.success and 'contact' in result.message.lower()
    plant.hit=False;time.sleep(.1)
    assert not wait(robot.go_to(1.1,3)).success


def test_clock_stall_stops_motion(rig):
    robot,plant,_=rig
    first=robot.go_to(2,3);time.sleep(.2);plant.paused=True
    result=wait(first)
    assert not result.success and 'stalled' in result.message.lower()
    assert plant.command.linear.x==0


def test_cancel_before_acceptance(rig):
    robot,_,_=rig
    first=robot.go_to(2,3);robot.cancel()
    assert not wait(first).success


def test_wall_deadline_stops_motion(rig):
    robot,plant,helper=rig
    first=robot.go_to(2,3)
    time.sleep(.2)
    with helper.lock:helper.started=time.monotonic()-121
    result=wait(first)
    assert not result.success and 'timed out' in result.message.lower()
    time.sleep(.1)
    assert plant.command.linear.x==0
