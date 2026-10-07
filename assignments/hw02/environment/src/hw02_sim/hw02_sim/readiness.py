"""Shared infrastructure check without configuration-space or planning logic."""
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.time import Time
from .map_server import MAP_QOS
from .robot_client import RobotClient


def main(args=None):
    rclpy.init(args=args); node = rclpy.create_node('hw02_readiness')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    data = {}; clocks = []; robot = RobotClient(node)
    tf = Buffer(node=node); listener = TransformListener(tf,node)
    def store(label, msg): data[label] = (msg,time.monotonic())
    for label, topic, kind, qos in [
        ('map','/map',OccupancyGrid,MAP_QOS), ('start','/hw02/start',PoseStamped,MAP_QOS),
        ('goal','/hw02/goal',PoseStamped,MAP_QOS), ('pose','/hw02/robot_pose',PoseStamped,10),
        ('contacts','/hw02/collision',Bool,10)]:
        node.create_subscription(kind,topic,lambda msg,key=label:store(key,msg),qos)
    def on_clock(msg):
        stamp=msg.clock.sec+msg.clock.nanosec/1e9
        if not clocks: clocks.append(stamp)
        if len(clocks)==1: clocks.append(stamp)
        else: clocks[-1]=stamp
        store('clock',msg)
    node.create_subscription(Clock,'/clock',on_clock,10)
    def checks():
        fresh=lambda key:key in data and time.monotonic()-data[key][1]<1
        flags={'advancing clock':fresh('clock') and len(clocks)>1 and clocks[-1]-clocks[0]>.5,
               'raw map':False,'start/goal':all(key in data for key in ['start','goal']),
               'fresh robot pose':fresh('pose'),'live contact monitoring':fresh('contacts') and not data['contacts'][0].data,
               'map/base TF':False,'go-to helper':robot.ready()}
        if 'map' in data:
            msg=data['map'][0]; cells=np.asarray(msg.data)
            flags['raw map']=(msg.header.frame_id=='map' and msg.info.width==320 and msg.info.height==240 and
                              abs(msg.info.resolution-.025)<1e-6 and len(cells)==320*240 and
                              set(np.unique(cells))<={0,100} and np.any(cells==100))
        try:
            tf.lookup_transform('map','base_link',Time()); flags['map/base TF']=True
        except TransformException: pass
        return flags
    try:
        deadline=time.monotonic()+45
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if all(checks().values()):break
        flags=checks()
        for label,ok in flags.items():print(f'{"PASS" if ok else "FAIL"}: {label}',flush=True)
        return 0 if all(flags.values()) else 1
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
