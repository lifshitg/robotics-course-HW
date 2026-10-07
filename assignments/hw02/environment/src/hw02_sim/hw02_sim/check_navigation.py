"""Public independent observer; does not plan or command the robot."""
import argparse
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from .map_server import MAP_QOS
from .scenario import START, GOAL


def main(args=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--layout',choices=['wide','mixed','blocked'],required=True)
    parser.add_argument('--timeout',type=float,default=300)
    options,ros_args=parser.parse_known_args(args)
    rclpy.init(args=ros_args); node=rclpy.create_node('hw02_navigation_check')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    data={}; collision=False; moved=False
    def store(key,message): data[key]=(message,time.monotonic())
    def on_pose(message):
        nonlocal moved
        store('pose',message)
        p=message.pose.position
        moved |= math.hypot(p.x-START[0],p.y-START[1]) > .03
    def on_contacts(message):
        nonlocal collision
        store('contacts',message); collision |= bool(message.contacts)
    node.create_subscription(PoseStamped,'/hw02/robot_pose',on_pose,10)
    node.create_subscription(Twist,'/hw02/robot_velocity',lambda m:store('speed',m),10)
    node.create_subscription(Contacts,'/hw02/contacts',on_contacts,10)
    node.create_subscription(String,'/hw02/planner_status',lambda m:store('status',m),MAP_QOS)
    held=None; error=float('inf')
    try:
        deadline=time.monotonic()+options.timeout
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if collision:
                print('FAIL: Gazebo observed wall/pillar contact'); return 1
            if options.layout=='blocked' and moved:
                print('FAIL: robot moved in the blocked-room demonstration'); return 1
            if not all(k in data and time.monotonic()-data[k][1]<1 for k in ['pose','speed','contacts']):
                held=None; continue
            pose,speed=data['pose'][0],data['speed'][0]
            now=node.get_clock().now().nanoseconds/1e9
            stamp=pose.header.stamp.sec+pose.header.stamp.nanosec/1e9
            p=pose.pose.position
            error=math.hypot(p.x-GOAL[0],p.y-GOAL[1])
            stopped=math.hypot(speed.linear.x,speed.linear.y)<.02 and abs(speed.angular.z)<.05
            valid=(pose.header.frame_id=='map' and 0<=now-stamp<.2 and stopped)
            if options.layout=='blocked':
                valid &= 'status' in data and data['status'][0].data=='no_path_found' and not moved
            else: valid &= error<=.10
            if valid:
                if held is None: held=now
                if now-held>=2:
                    print(f'PASS: {options.layout}; '+('no path reported, no movement' if options.layout=='blocked' else f'goal error={error:.4f} m')+'; stopped 2 simulated seconds; contact stream clear')
                    return 0
            else: held=None
        print(f'FAIL: independent navigation verification timed out; last goal error={error:.3f} m')
        return 1
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
