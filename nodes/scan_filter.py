#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
"""Mask the 4 column slivers out of the lidar scan.

The 2020 columns cross the scan plane (~215 mm), so the C1 sees them as 4 short-range blobs that
would be mapped as obstacles moving with the robot. Beams inside the sectors in config/scan_mask.yaml
are set to +inf (the same "no return" value sllidar itself uses) and the result is republished.

    /scan  (sensor_msgs/LaserScan)  →  scan_filter  →  /scan_filtered

The last session's all-green state: 52.0 beams masked, 0 leaks. Calibrate with tools/calibrate_mask.py.
The mask file is re-read when it changes, so recalibrating doesn't need a restart.
"""
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from campus_common import MASK_FILE, load_mask, mask_indices  # noqa: E402


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.in_topic = self.declare_parameter('in_topic', '/scan').value
        self.out_topic = self.declare_parameter('out_topic', '/scan_filtered').value
        self.mask_file = self.declare_parameter('mask_file', MASK_FILE).value
        self.pub = self.create_publisher(LaserScan, self.out_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.in_topic, self.on_scan, qos_profile_sensor_data)
        self._mtime = None
        self._sectors = []
        self._cache_key = None
        self._idx = []
        self.reload_mask()
        self.create_timer(2.0, self.reload_mask)

    def reload_mask(self):
        try:
            mtime = os.path.getmtime(self.mask_file)
        except OSError:
            mtime = None
        if mtime == self._mtime:
            return
        self._mtime = mtime
        self._sectors, calibrated = load_mask(self.mask_file)
        self._cache_key = None
        if not self._sectors:
            self.get_logger().warn('No mask sectors: the columns will show up in the map. '
                                   'Run tools/calibrate_mask.py with the robot in open space.')
        else:
            self.get_logger().info(f'Mask: {len(self._sectors)} sectors from {self.mask_file}'
                                   + ('' if calibrated else ' (NOT calibrated)'))

    def on_scan(self, msg):
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if key != self._cache_key:
            self._idx = mask_indices(msg.angle_min, msg.angle_increment, len(msg.ranges), self._sectors)
            self._cache_key = key
            self.get_logger().info(f'{len(self._idx)} of {len(msg.ranges)} beams masked')
        if self._idx:
            ranges = list(msg.ranges)
            for i in self._idx:
                ranges[i] = float('inf')
            msg.ranges = ranges
            if msg.intensities:
                inten = list(msg.intensities)
                for i in self._idx:
                    inten[i] = 0.0
                msg.intensities = inten
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
