#!/usr/bin/env python3
# Written 2026-09-30 (new tool: LASER_YAW was not in the recovered notes).
"""Check which way the lidar faces. Put a box ~0.5–1 m STRAIGHT AHEAD of the robot (the drive
direction), nothing else within 1.5 m, then:
    python3 tools/lidar_front.py
It prints the bearing of the nearest object in the base_link frame, using LASER_YAW from
config/robot.env. Straight ahead must read ~0°, and a box moved to the robot's LEFT must read ~+90°.
If ahead doesn't read 0°, it prints the LASER_YAW to put in robot.env. If left reads -90°, the lidar
is mounted upside down: set inverted:=true in launch/bringup.launch.py.
"""
import math
import os
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import load_env  # noqa: E402

env = load_env()
YAW = float(env.get('LASER_YAW', 0.0))
SELF = float(env.get('SELF_RADIUS', 0.40))


def on_scan(msg):
    best = None
    for i, r in enumerate(msg.ranges):
        if math.isfinite(r) and SELF < r < 1.5 and (best is None or r < best[0]):
            best = (r, msg.angle_min + i * msg.angle_increment)
    if best is None:
        print('\r  nothing between %.2f and 1.5 m                                   ' % SELF, end='', flush=True)
        return
    r, a = best
    bearing = math.degrees(math.atan2(math.sin(a + YAW), math.cos(a + YAW)))
    fix = math.atan2(math.sin(YAW - math.radians(bearing)), math.cos(YAW - math.radians(bearing)))
    print(f'\r  nearest: {r:.2f} m at {bearing:+6.1f}° (base_link)   '
          f'→ if that box is straight ahead, LASER_YAW={fix:.4f}   ', end='', flush=True)


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_front')
    node.create_subscription(LaserScan, env.get('SCAN_FILTERED_TOPIC', '/scan_filtered'), on_scan,
                             qos_profile_sensor_data)
    print(f'LASER_YAW currently {YAW:.4f} rad. Ctrl-C to quit.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
