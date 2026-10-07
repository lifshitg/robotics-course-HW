"""Publish the supplied raw map/static targets and truth TF, without planning."""
import rclpy
import signal
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from .scenario import raw_map, START, GOAL, RESOLUTION, ROBOT_RADIUS

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MapServer(Node):
    def __init__(self):
        super().__init__('hw02_map_server')
        self.declare_parameter('layout', 'wide')
        grid = raw_map(self.get_parameter('layout').value)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.resolution = RESOLUTION
        self.map.info.height, self.map.info.width = grid.shape
        self.map.info.origin.orientation.w = 1.0
        self.map.data = grid.ravel().tolist()
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.targets = []
        for label, xy in [('start', START), ('goal', GOAL)]:
            pose = PoseStamped(); pose.header.frame_id = 'map'; pose.pose.orientation.w = 1.0
            pose.pose.position.x, pose.pose.position.y = xy
            pub = self.create_publisher(PoseStamped, '/hw02/'+label, MAP_QOS)
            self.targets.append((pub, pose))
        self.tf = TransformBroadcaster(self)
        self.markers = self.create_publisher(MarkerArray, '/hw02/markers', MAP_QOS)
        self.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_pose, 10)
        self.publish_map()
        self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        for pub, pose in self.targets:
            pub.publish(pose)

    def on_pose(self, pose):
        transform = TransformStamped(header=pose.header, child_frame_id='base_link')
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf.sendTransform(transform)
        marker = Marker(header=pose.header, ns='robot', id=0, type=Marker.CYLINDER, action=Marker.ADD)
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = 2*ROBOT_RADIUS
        marker.scale.z = 0.1
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .15, .4, .85, 1.0
        self.markers.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args); node = MapServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
