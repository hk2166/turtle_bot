#!/usr/bin/env python3
# Written 2026-10-13: get out of tight spots Nav2's recoveries refuse to move in.
"""Lidar-guided escape when Nav2 is wedged: a goal is active, the robot isn't moving, and the planner
keeps failing to start / spin and back-up keep refusing ("Collision Ahead").

    started by run/remap.sh with navigation (UNSTICK=1 in config/robot.env; 0 turns it off)
    python3 nodes/unstick.py --dry      plan once from the live scan, print the move, don't drive

Why: 2026-10-13 the robot stopped 6 cm from a desk side (lidar 0.25 m, half-width 0.19 m). Its cell
was in the costmap's "too close" band, so NavFn couldn't start, and the behaviours' 5 cm costmap check
called every motion a collision - even backing straight out along the desk, with 1.5 m free behind.

Escape: candidate moves (straight back / forward, arcs, short turns) are simulated against the raw
lidar points with the true footprint plus a 3 cm margin; only moves that stay clear the whole way are
kept, and the one ending furthest from every obstacle wins. It is driven at 8 cm/s (<= ~25 cm) with the
live scan re-checked at 10 Hz, then Nav2 resumes the same goal on its own (the planner can start
again). At most 3 escapes per goal, 20 s apart. Firmware bumper and 300 ms watchdog stay in charge.
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
HALF_L, HALF_W = 0.22, 0.19          # config/nav2_params.yaml footprint 0.44 x 0.38 m
MARGIN = 0.03                        # planning margin around the footprint
LIVE_MARGIN = 0.02                   # stop margin while driving
V, W = 0.08, 0.35                    # m/s, rad/s - slow
CANDIDATES = [(-V, 0.0, 3.0), (V, 0.0, 3.0), (V, W, 3.0), (V, -W, 3.0), (-V, W, 3.0), (-V, -W, 3.0),
              (0.0, W, 2.0), (0.0, -W, 2.0)]


def clearance(pts, x, y, th):
    """Distance from the footprint at (x, y, th) to the nearest point; negative = inside."""
    if not len(pts):
        return 9.9
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy               # points in the moved robot's frame
    ox, oy = np.abs(lx) - HALF_L, np.abs(ly) - HALF_W
    outside = np.hypot(np.maximum(ox, 0), np.maximum(oy, 0))
    inside = np.minimum(np.maximum(ox, oy), 0)
    return float(np.min(outside + inside))


def plan(pts, table=None):
    """(vx, wz, seconds, end clearance, now clearance) of the best escape, or None.
    table: optional list that receives (vx, wz, feasible seconds, end clearance or None) per candidate."""
    now = clearance(pts, 0, 0, 0)
    # Already inside the margin (that is how robots get wedged): a move may never get CLOSER than now,
    # so sliding out along the obstacle or turning away is allowed, approaching it is not.
    limit = min(MARGIN, now - 0.004)
    best = None
    for vx, wz, tmax in CANDIDATES:
        x = y = th = 0.0
        t, ok_t, end = 0.0, 0.0, None
        while t < tmax - 1e-9:
            t += 0.1
            th += wz * 0.1
            x += vx * 0.1 * math.cos(th)
            y += vx * 0.1 * math.sin(th)
            if clearance(pts, x, y, th) < limit:
                break
            ok_t, end = t, (x, y, th)
        if end is None or (math.hypot(end[0], end[1]) < 0.06 and abs(end[2]) < math.radians(8)):
            if table is not None:
                table.append((vx, wz, ok_t, None))
            continue
        cl = clearance(pts, *end)
        if table is not None:
            table.append((vx, wz, ok_t, cl))
        # prefer moves that end far from everything; a little credit for forward (no rear bumper)
        score = cl + (0.01 if vx > 0 else 0.0)
        if best is None or score > best[0]:
            best = (score, vx, wz, ok_t, cl)
    if best is None or best[4] < now + 0.04:
        return None
    return best[1], best[2], best[3], best[4], now, limit


class Unstick(Node):
    def __init__(self, dry=False):
        super().__init__('unstick')
        self.dry = dry
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=True)
        self.scan, self.T = None, None
        self.speed, self.still_since = 0.0, None
        self.goal_id, self.goal_active, self.goal_since = None, False, 0.0
        self.fails = []                 # monotonic times of planner/recovery aborts
        self.escapes, self.last_escape = 0, 0.0
        self.seen = {}                  # (topic, goal id) -> last status, to count each abort once
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav, 10)
        for a in ('compute_path_to_pose', 'spin', 'backup'):
            self.create_subscription(GoalStatusArray, f'/{a}/_action/status',
                                     lambda m, a=a: self.on_fail_status(a, m), 10)
        if not dry:
            self.create_timer(0.5, self.tick)
        self.get_logger().info('watching for wedged goals (lidar escape)' + (' [dry]' if dry else ''))

    # ---- inputs
    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        t = m.twist.twist
        self.speed = abs(t.linear.x) + 0.3 * abs(t.angular.z)
        now = time.monotonic()
        if self.speed > 0.02:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def on_nav(self, m):
        act = [s for s in m.status_list if s.status in ACTIVE]
        gid = bytes(act[-1].goal_info.goal_id.uuid) if act else None
        if gid != self.goal_id:
            self.goal_id, self.escapes, self.fails = gid, 0, []
            self.goal_since = time.monotonic()
        self.goal_active = gid is not None

    def on_fail_status(self, action, m):
        for s in m.status_list:
            key = (action, bytes(s.goal_info.goal_id.uuid))
            if s.status == GoalStatus.STATUS_ABORTED and self.seen.get(key) != s.status:
                self.fails.append(time.monotonic())
            self.seen[key] = s.status
        if len(self.seen) > 500:
            self.seen.clear()

    def points(self):
        """Lidar points in base_link (N x 2)."""
        if self.scan is None:
            return None
        if self.T is None:
            try:
                tr = self.tf.lookup_transform('base_link', self.scan.header.frame_id, Time(),
                                              timeout=Duration(seconds=0.5)).transform
            except Exception:
                return None
            q = tr.rotation
            self.T = (tr.translation.x, tr.translation.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        s = self.scan
        r = np.asarray(s.ranges, float)
        a = s.angle_min + s.angle_increment * np.arange(len(r)) + self.T[2]
        ok = np.isfinite(r) & (r > 0.05) & (r < 4.0)
        return np.column_stack([self.T[0] + r[ok] * np.cos(a[ok]), self.T[1] + r[ok] * np.sin(a[ok])])

    # ---- decision
    def tick(self):
        if os.path.exists(os.path.expanduser('~/.campus_bot/run/handbrake')):   # 🅿 parked: never drive
            return
        now = time.monotonic()
        self.fails = [t for t in self.fails if now - t < 30.0]
        wedged = (self.goal_active and now - self.goal_since > 5.0 and self.still_since is not None
                  and now - self.still_since > 2.0 and len(self.fails) >= 3)
        if not wedged or self.escapes >= 3 or now - self.last_escape < 20.0:
            return
        pts = self.points()
        if pts is None:
            return
        p = plan(pts)
        self.last_escape = now
        if p is None:
            self.get_logger().warn('wedged, but no lidar-clear escape move: waiting for Nav2 / a person')
            return
        self.escapes += 1
        vx, wz, secs, cl_end, cl_now, limit = p
        self.get_logger().warn(f'wedged ({len(self.fails)} planner/recovery aborts): escape {self.escapes}/3 '
                               f'vx {vx:+.2f} wz {wz:+.2f} for {secs:.1f} s, clearance {cl_now:.2f} -> {cl_end:.2f} m')
        self.drive(vx, wz, secs, min(LIVE_MARGIN, limit))
        self.fails = []

    def drive(self, vx, wz, secs, stop_at=LIVE_MARGIN):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        end = time.monotonic() + max(0.0, secs - 0.2)
        while time.monotonic() < end and rclpy.ok():
            pts = self.points()
            # live check: the footprint a moment ahead along the motion must stay clear
            if pts is not None and clearance(pts, vx * 0.3, 0.0, wz * 0.3) < stop_at:
                self.get_logger().warn('escape stopped early: something came close')
                break
            self.pub.publish(cmd)
            t = time.monotonic() + 0.1
            while time.monotonic() < t:
                rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(3):
            self.pub.publish(Twist())
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dry', action='store_true', help='plan once from the live scan and print it')
    a = ap.parse_args()
    rclpy.init()
    node = Unstick(dry=a.dry)
    try:
        if a.dry:
            end = time.monotonic() + 10
            pts = None
            while time.monotonic() < end and pts is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                pts = node.points()
            if pts is None:
                print('no scan / laser TF')
                return
            print(f'{len(pts)} lidar points, footprint clearance now {clearance(pts, 0, 0, 0):.3f} m')
            table = []
            p = plan(pts, table)
            names = {(-V, 0.0): 'straight back', (V, 0.0): 'straight forward', (V, W): 'forward arc left',
                     (V, -W): 'forward arc right', (-V, W): 'back arc (tail right)', (-V, -W): 'back arc (tail left)',
                     (0.0, W): 'turn left in place', (0.0, -W): 'turn right in place'}
            for vx, wz, t, cl in table:
                print(f'  {names[(vx, wz)]:22s} clear for {t:.1f} s' +
                      (f' -> ends {cl:.2f} m from everything' if cl is not None else ' -> blocked at once'))
            print('chosen escape:', 'none (nothing better)' if p is None else
                  f'vx {p[0]:+.2f} m/s, wz {p[1]:+.2f} rad/s for {p[2]:.1f} s -> clearance {p[4]:.3f} -> {p[3]:.2f} m')
            import os
            import sys
            sys.stdout.flush()
            os._exit(0)      # skip rclpy/tf teardown noise in a one-shot dry run
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
