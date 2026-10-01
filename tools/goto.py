#!/usr/bin/env python3
# Written 2026-10-01 for the first autonomous tests.
"""Send the robot somewhere with Nav2 and watch it get there (navigation must be running: run/nav.sh).

    python3 tools/goto.py --forward 1.0        1 m straight ahead of where it is now (negative = behind)
    python3 tools/goto.py --turn 90            turn on the spot, +90° = left
    python3 tools/goto.py X Y [YAW_DEG]        a map position (as RViz shows it)
    python3 tools/goto.py --back               back to where this tool last started a goal from

Checks the target is open floor in the map (≥ 0.3 m from walls) before sending it. Prints progress;
Ctrl-C (or --timeout) cancels the goal and stops the robot.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LAST = os.path.expanduser('~/.campus_bot/run/goto_last_start.json')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xyyaw', nargs='*', type=float, help='X Y [YAW_DEG] in the map')
    ap.add_argument('--forward', type=float)
    ap.add_argument('--turn', type=float, help='degrees, + = left')
    ap.add_argument('--back', action='store_true')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--clearance', type=float, default=0.3, help='min distance from map walls (m)')
    ap.add_argument('--dry-run', action='store_true', help="check the target, don't send it")
    a = ap.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('goto')
    buf = Buffer()
    TransformListener(buf, node)
    maps = []
    node.create_subscription(OccupancyGrid, '/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stop_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    here = None
    end = time.monotonic() + 8
    while time.monotonic() < end and (here is None or not maps):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0))
            here = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
        except Exception:
            pass
    if here is None:
        sys.exit("no robot pose in the map: is navigation running, and the pose set (run/reset_pose.sh)?")
    x, y, yaw = here
    if a.forward is not None:
        gx, gy, gyaw = x + a.forward * math.cos(yaw), y + a.forward * math.sin(yaw), yaw
    elif a.turn is not None:
        gx, gy, gyaw = x, y, yaw + math.radians(a.turn)
    elif a.back:
        try:
            with open(LAST) as f:
                gx, gy, gyaw = json.load(f)
        except (OSError, ValueError):
            sys.exit('no previous start position saved')
    elif len(a.xyyaw) in (2, 3):
        gx, gy = a.xyyaw[:2]
        gyaw = math.radians(a.xyyaw[2]) if len(a.xyyaw) == 3 else math.atan2(gy - y, gx - x)
    else:
        sys.exit('say where: --forward D, --turn DEG, --back, or X Y [YAW_DEG]')
    gyaw = (gyaw + math.pi) % (2 * math.pi) - math.pi

    if maps and a.turn is None:          # a turn stays on the spot: nothing to check
        m = maps[-1]
        res, w, h = m.info.resolution, m.info.width, m.info.height
        grid = np.asarray(m.data, np.int16).reshape(h, w)
        cx = int((gx - m.info.origin.position.x) / res)
        cy = int((gy - m.info.origin.position.y) / res)
        if not (0 <= cx < w and 0 <= cy < h) or grid[cy, cx] != 0:
            sys.exit(f'target ({gx:.2f}, {gy:.2f}) is not open floor in the map: pick another')
        r = int(math.ceil(a.clearance / res))
        win = grid[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        if (win >= 65).any():
            sys.exit(f'target ({gx:.2f}, {gy:.2f}) is within {a.clearance} m of a wall: pick another')

    if a.dry_run:
        print(f'target x {gx:+.2f}  y {gy:+.2f}  {math.degrees(gyaw):+.0f}°: open floor, ≥ {a.clearance} m from walls')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    os.makedirs(os.path.dirname(LAST), exist_ok=True)
    with open(LAST, 'w') as f:
        json.dump([x, y, yaw], f)
    print(f'from  x {x:+.2f}  y {y:+.2f}  {math.degrees(yaw):+.0f}°')
    print(f'to    x {gx:+.2f}  y {gy:+.2f}  {math.degrees(gyaw):+.0f}°   (Ctrl-C cancels and stops)')
    if not client.wait_for_server(timeout_sec=10):
        sys.exit('Nav2 is not running (no navigate_to_pose action)')
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    fb = {}
    send = client.send_goal_async(goal, feedback_callback=lambda f: fb.update(last=f.feedback))
    handle = None
    t0 = time.monotonic()
    shown = 0.0
    status = None
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if handle is None and send.done():
                handle = send.result()
                if not handle.accepted:
                    sys.exit('Nav2 rejected the goal')
                result = handle.get_result_async()
            if handle is not None and result.done():
                status = result.result().status
                break
            if time.monotonic() - t0 > a.timeout:
                print(f'\ntimeout ({a.timeout:.0f} s)')
                raise KeyboardInterrupt
            if 'last' in fb and time.monotonic() - shown > 1.0:
                shown = time.monotonic()
                f = fb['last']
                p = f.current_pose.pose
                sys.stdout.write(f'\r  {f.distance_remaining:5.2f} m to go   at x {p.position.x:+.2f} y {p.position.y:+.2f} '
                                 f'{math.degrees(yaw_of(p.orientation)):+4.0f}°   recoveries {f.number_of_recoveries}   ')
                sys.stdout.flush()
    except KeyboardInterrupt:
        if handle is not None:
            c = handle.cancel_goal_async()
            end = time.monotonic() + 3
            while time.monotonic() < end and not c.done():
                rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            stop_pub.publish(Twist())
            time.sleep(0.1)
        print('\ncancelled: robot stopped')
        status = None
    print()
    try:
        t = buf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=1))
        ex = math.hypot(t.transform.translation.x - gx, t.transform.translation.y - gy)
        ey = math.degrees(abs((yaw_of(t.transform.rotation) - gyaw + math.pi) % (2 * math.pi) - math.pi))
        print(f'ended {ex * 100:.0f} cm and {ey:.0f}° from the target')
    except Exception:
        pass
    names = {GoalStatus.STATUS_SUCCEEDED: 'REACHED', GoalStatus.STATUS_ABORTED: 'FAILED (Nav2 gave up: see ~/.campus_bot/logs/nav.log)',
             GoalStatus.STATUS_CANCELED: 'cancelled'}
    if status is not None:
        print(names.get(status, f'status {status}'))
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if status == GoalStatus.STATUS_SUCCEEDED else 1)


if __name__ == '__main__':
    main()
