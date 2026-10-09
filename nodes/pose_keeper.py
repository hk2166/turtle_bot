#!/usr/bin/env python3
# Written 2026-10-09: remember where the robot is, so it knows again after a reboot.
"""Saves AMCL's pose to ~/.campus_bot/run/last_pose_<map>.yaml while navigating.

    started by run/remap.sh with navigation:  python3 nodes/pose_keeper.py --ros-args -p map:=<area>

run/remap.sh reads the file at the next start, checks it against the lidar (tools/fit_pose.py) and
sets AMCL there when the walls line up. Writes are rare and atomic (SD card, see CLAUDE.md): only
after a move of 5 cm / 3°, at most every 10 s, plus once on exit.
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

RUN_DIR = os.path.expanduser('~/.campus_bot/run')


class PoseKeeper(Node):
    def __init__(self):
        super().__init__('pose_keeper')
        area = self.declare_parameter('map', '').value
        if not area:
            raise SystemExit('map parameter is required')
        self.path = os.path.join(RUN_DIR, f'last_pose_{area}.yaml')
        self.latest = None       # (x, y, yaw)
        self.saved = None
        self.saved_t = 0.0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.0, self.maybe_save)
        self.get_logger().info(f'keeping the pose in {self.path}')

    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.latest = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def maybe_save(self, force=False):
        if self.latest is None:
            return
        if not force:
            if time.monotonic() - self.saved_t < 10.0:
                return
            if self.saved is not None:
                dx, dy = self.latest[0] - self.saved[0], self.latest[1] - self.saved[1]
                dyaw = abs((self.latest[2] - self.saved[2] + math.pi) % (2 * math.pi) - math.pi)
                if math.hypot(dx, dy) < 0.05 and dyaw < math.radians(3):
                    return
        x, y, yaw = self.latest
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(f'# last AMCL pose, {time.strftime("%Y-%m-%d %H:%M:%S")} (nodes/pose_keeper.py)\n'
                    f'x: {x:.3f}\ny: {y:.3f}\nyaw: {yaw:.4f}\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        self.saved, self.saved_t = self.latest, time.monotonic()


def main():
    rclpy.init()
    node = PoseKeeper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.maybe_save(force=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
