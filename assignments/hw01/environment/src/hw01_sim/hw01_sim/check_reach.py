"""Public checker: observe actual tip pose and contact streams, without moving."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Contacts


class ReachCheck(Node):
    def __init__(self):
        super().__init__('hw01_reach_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.contact_count = {'platform': 0, 'marker': 0}
        self.contact_time = {'platform': 0.0, 'marker': 0.0}
        self.collision = False
        self.tip = None
        self.tip_received = 0.0
        self.create_subscription(PoseStamped, '/hw01/measured_tip', self.on_tip, 10)
        for label in self.contact_count:
            self.create_subscription(Contacts, f'/hw01/{label}_contacts',
                                     lambda message, key=label: self.contacts(message, key), 10)

    def on_tip(self, message):
        self.tip = message
        self.tip_received = time.monotonic()

    def contacts(self, message, key):
        self.contact_count[key] += 1
        self.contact_time[key] = time.monotonic()
        for contact in message.contacts:
            names = contact.collision1.name + ' ' + contact.collision2.name
            if 'hw01_arm' in names or 'link1' in names or 'link2' in names:
                self.collision = True

    def run(self, preset, timeout):
        xy = {1: (0.35, -0.15), 2: (0.40, 0.0), 3: (0.35, 0.15)}[preset]
        expected = (*xy, 0.14)
        deadline = time.monotonic() + timeout
        held_since = None
        error = float('inf')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.collision:
                print('FAIL: robot contact with platform or marker')
                return False
            if self.tip is None or self.tip.header.frame_id != 'base_link':
                continue
            stamp = Time.from_msg(self.tip.header.stamp).nanoseconds / 1e9
            now = self.get_clock().now().nanoseconds / 1e9
            p = self.tip.pose.position
            error = math.dist((p.x, p.y, p.z), expected)
            contacts_live = all(self.contact_count[key] > 1 and
                                time.monotonic() - self.contact_time[key] < 2.0
                                for key in self.contact_count)
            if (error <= 0.02 and 0 <= now-stamp < 0.5 and
                    time.monotonic()-self.tip_received < 2.0 and contacts_live):
                if held_since is None:
                    held_since = now
                if now - held_since >= 2.0:
                    print(f'PASS: tip error {error:.4f} m, held 2 simulated seconds; live contact streams clear')
                    return True
            else:
                held_since = None
        print(f'FAIL: target not verified; last error={error:.4f} m, contact samples={self.contact_count}')
        return False


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--preset', type=int, choices=[1, 2, 3], required=True)
    parser.add_argument('--timeout', type=float, default=90.0)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = ReachCheck()
    try:
        return_code = 0 if node.run(options.preset, options.timeout) else 1
    except KeyboardInterrupt:
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)
