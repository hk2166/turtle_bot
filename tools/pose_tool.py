#!/usr/bin/env python3
# Written 2026-09-30 — helper for run/remap.sh.
"""Small TF/pose helper used by run/remap.sh.

    pose_tool.py probe PARENT CHILD [--timeout S]   exit 0 if the transform resolves
    pose_tool.py get                                print "x y yaw" of base_link in map
    pose_tool.py set X Y YAW                        publish /initialpose until AMCL confirms it (exit 1 if not)
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def lookup(node, parent, child, timeout):
    buf = Buffer()
    TransformListener(buf, node)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0))
        except Exception:
            pass
    return None


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('probe')
    p.add_argument('parent')
    p.add_argument('child')
    p.add_argument('--timeout', type=float, default=3.0)
    g = sub.add_parser('get')
    g.add_argument('--timeout', type=float, default=3.0)
    s = sub.add_parser('set')
    s.add_argument('x', type=float)
    s.add_argument('y', type=float)
    s.add_argument('yaw', type=float)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('pose_tool')
    code = 0
    if args.cmd == 'probe':
        code = 0 if lookup(node, args.parent, args.child, args.timeout) else 1
    elif args.cmd == 'get':
        t = lookup(node, 'map', 'base_link', args.timeout)
        if t is None:
            code = 1
        else:
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            print(f'{t.transform.translation.x:.3f} {t.transform.translation.y:.3f} {yaw:.4f}')
    else:
        pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = args.x, args.y
        msg.pose.pose.orientation.z = math.sin(args.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(args.yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(15) ** 2
        # Re-send until AMCL reports a pose near this one. Right after boot (load ~15) a few blind sends
        # were lost before AMCL's reader had matched, and nothing noticed (2026-10-09).
        got = []
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', got.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def confirmed():
            for m in got:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                if (math.hypot(p.x - args.x, p.y - args.y) < 0.3
                        and abs((yaw - args.yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(10)):
                    return True
            return False

        end = time.monotonic() + 20
        code = 1
        while time.monotonic() < end:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            got.clear()
            t = time.monotonic() + 2.0
            while time.monotonic() < t and not confirmed():
                rclpy.spin_once(node, timeout_sec=0.1)
            if confirmed():
                code = 0
                break
        if code:
            print('AMCL did not confirm the pose within 20 s', file=sys.stderr)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
