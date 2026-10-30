#!/usr/bin/env python3
# Written 2026-10-30: catch the robot pushing against something (or its wheels slipping) BEFORE its pose
# is lost. New file.
"""Slip / stall watch: the lidar as a reality check on the step-counted odometry.

The firmware measures distance by counting the steps it SENT to the motors (heading comes from the gyro,
which is real). When the robot pushes against something the lidar can't see (a chair's star base or a
caster below the 21.5 cm scan plane) or its wheels slip on carpet, the steps keep counting: odometry says
"still driving". That false motion fools AMCL (the pose slides forward through the obstacle and is lost),
Nav2's progress checker (it sees "progress", so no recovery runs) and nodes/unstick.py (it waits for the
robot to be still). Seen on the robot, 2026-10-29/30.

Check, ~10 Hz while a Nav2 goal is active: over the last WINDOW s, if the steps claim >= MIN_CLAIM of
travel, the lidar ranges in the direction of travel (a +-25 deg cone ahead, or behind when reversing) must
shrink by about that much. If they confirm less than RATIO of it for HOLD s, the robot is stuck. Only
judged with enough returns in the cone (an empty corridor ahead can't confirm or deny motion, so it is
skipped, never flagged); rotation within the window is compensated by shifting the old scan.

When stuck:
- POST /api/stuck to the dashboard: cancel the Nav2 goal (a queue trip counts as blocked and retries,
  with a camera look at what's in the way), then re-fit the pose to the map around the LAST GOOD pose
  (before the push began), searching as far as the steps falsely claimed.
- Mark the spot in front (or behind) as an obstacle for STUCK_MARK_S s on /stuck_obstacles (PointCloud2,
  map frame, 8 cm up: below the lidar, so its sweeps don't erase it), a source in both costmaps
  (config/nav2_params.yaml): the retry routes around it instead of pushing into it again. On expiry an
  empty cloud is published and the local costmap cleared.

    started by run/remap.sh with navigation (SLIP_WATCH=1 in config/robot.env; 0 turns it off)
    log: ~/.campus_bot/logs/slip_watch.log
"""
import json
import math
import os
import threading
import time
import urllib.request
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

WINDOW = float(os.environ.get('SLIP_WINDOW', '1.0'))      # s of claimed motion compared with the lidar
MIN_CLAIM = float(os.environ.get('SLIP_MIN_CLAIM', '0.06'))  # m the steps must claim before we judge
RATIO = float(os.environ.get('SLIP_RATIO', '0.3'))        # lidar must confirm at least this fraction
HOLD = float(os.environ.get('SLIP_HOLD', '0.8'))          # s the mismatch must persist
CONE = math.radians(25.0)
MIN_BEAMS = 8
RMIN, RMAX = 0.15, 6.0
MAX_TURN = 0.6                                             # rad within the window; more = don't judge
COOLDOWN = 15.0
MARK_S = float(os.environ.get('STUCK_MARK_S', '180'))
HALF_LEN, HALF_WID = 0.22, 0.19                             # footprint 0.44 x 0.38 m (nav2_params.yaml)
DASH = os.environ.get('DASH_URL', 'http://127.0.0.1:8088')


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SlipWatch(Node):
    def __init__(self):
        super().__init__('slip_watch')
        self.tf = Buffer(cache_time=Duration(seconds=15.0))
        self.tfl = TransformListener(self.tf, self)
        self.odom = deque()       # (t, x, y, yaw)
        self.scans = deque()      # (t, ranges, angle_min, angle_inc)
        self.goal_active = False
        self.laser_yaw = None
        self.bad_since = None
        self.last_fire = 0.0
        self.marks = []           # [(expiry, Nx3 points in map)]
        self.had_marks = False
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav, 10)
        self.pub_marks = self.create_publisher(PointCloud2, '/stuck_obstacles', 10)
        from nav2_msgs.srv import ClearEntireCostmap
        self._Clear = ClearEntireCostmap
        self.clear_local = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.create_timer(1.0, self.publish_marks)
        self.get_logger().info(f'watching for wheel slip / pushing (window {WINDOW} s, lidar must confirm '
                               f'>= {RATIO:.0%} of >= {MIN_CLAIM * 100:.0f} cm for {HOLD} s)')

    @staticmethod
    def now():
        return time.time()

    # ---------------------------------------------------------------- inputs
    def on_odom(self, m):
        p = m.pose.pose
        self.odom.append((self.now(), p.position.x, p.position.y, yaw_of(p.orientation)))
        while self.odom and self.odom[0][0] < self.now() - 4.0:
            self.odom.popleft()

    def on_nav(self, m):
        self.goal_active = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                               for s in m.status_list)
        if not self.goal_active:
            self.bad_since = None

    def odom_at(self, t):
        if not self.odom:
            return None
        return min(self.odom, key=lambda o: abs(o[0] - t))

    def laser_offset(self):
        if self.laser_yaw is None:
            try:
                tr = self.tf.lookup_transform('base_link', 'laser', Time())
                self.laser_yaw = yaw_of(tr.transform.rotation)
            except Exception:
                return None
        return self.laser_yaw

    # ---------------------------------------------------------------- the check
    def on_scan(self, m):
        t = self.now()
        self.scans.append((t, np.asarray(m.ranges, dtype=np.float32), m.angle_min, m.angle_increment))
        while self.scans and self.scans[0][0] < t - 3.0:
            self.scans.popleft()
        if not self.goal_active:
            return
        verdict = self.judge(t)
        if verdict is None:                  # can't judge (not moving, no structure, turning): no streak
            self.bad_since = None
            return
        ratio, claimed, direction = verdict
        if ratio >= RATIO:
            self.bad_since = None
            return
        if self.bad_since is None:
            self.bad_since = t
        if t - self.bad_since >= HOLD and t - self.last_fire > COOLDOWN:
            self.last_fire = t
            self.fire(ratio, direction)

    def judge(self, t):
        """(confirmed ratio, claimed m, +1/-1) for the last WINDOW s, or None when it can't be judged."""
        old = min(self.scans, key=lambda s: abs(s[0] - (t - WINDOW)))
        if abs(old[0] - (t - WINDOW)) > 0.25:
            return None
        o0, o1 = self.odom_at(old[0]), self.odom_at(t)
        lyaw = self.laser_offset()
        if o0 is None or o1 is None or lyaw is None:
            return None
        dx, dy = o1[1] - o0[1], o1[2] - o0[2]
        d = dx * math.cos(o0[3]) + dy * math.sin(o0[3])           # along the old heading (+ = forward)
        dyaw = float(wrap(o1[3] - o0[3]))
        if abs(d) < MIN_CLAIM or abs(dyaw) > MAX_TURN:
            return None
        _, r1, amin, ainc = self.scans[-1]
        r0 = old[1]
        if len(r0) != len(r1) or ainc == 0:
            return None
        # rotation: a direction seen at old beam i is seen at new beam i - dyaw/ainc
        r0 = np.roll(r0, -int(round(dyaw / ainc)))
        ang = wrap(amin + np.arange(len(r1)) * ainc + lyaw)        # beam directions in base_link
        rel = wrap(ang - (0.0 if d > 0 else np.pi))
        ok = (np.abs(rel) <= CONE) & np.isfinite(r0) & np.isfinite(r1) & \
             (r0 > RMIN) & (r0 < RMAX) & (r1 > RMIN) & (r1 < RMAX)
        if int(ok.sum()) < MIN_BEAMS:
            return None
        expected = abs(d) * np.cos(rel[ok])
        observed = r0[ok] - r1[ok]                                 # > 0: things got closer
        return float(np.median(observed / expected)), abs(d), (1 if d > 0 else -1)

    # ---------------------------------------------------------------- response
    def fire(self, ratio, direction):
        t_good = self.bad_since - WINDOW                           # before the steps started lying
        o_good, o_now = self.odom_at(t_good), self.odom_at(self.now())
        claimed = math.hypot(o_now[1] - o_good[1], o_now[2] - o_good[2]) if o_good and o_now else 0.3
        try:
            tr = self.tf.lookup_transform('map', 'base_link', Time(nanoseconds=int(t_good * 1e9)),
                                          timeout=Duration(seconds=0.2))
        except Exception:
            try:
                tr = self.tf.lookup_transform('map', 'base_link', Time())
            except Exception as e:
                self.get_logger().warning(f'stuck, but no map pose to fall back to: {e}')
                return
        x, y = tr.transform.translation.x, tr.transform.translation.y
        yaw = yaw_of(tr.transform.rotation)
        side = 'ahead' if direction > 0 else 'behind'
        what = f'pushing against something {side} that the lidar can\'t see (or wheel slip)'
        self.get_logger().warning(f'STUCK: steps claimed {claimed:.2f} m, lidar confirmed {max(ratio, 0):.0%}; '
                                  f'last good pose {x:.2f}, {y:.2f} @ {math.degrees(yaw):.0f} deg')
        self.mark(x, y, yaw, direction)
        threading.Thread(target=self.tell_dashboard, daemon=True,
                         args=({'x': x, 'y': y, 'yaw': yaw, 'claimed': claimed, 'what': what},)).start()

    def tell_dashboard(self, body):
        try:
            req = urllib.request.Request(f'{DASH}/api/stuck', data=json.dumps(body).encode(), method='POST',
                                         headers={'Content-Type': 'application/json'})
            urllib.request.urlopen(req, timeout=5).read()
        except Exception as e:
            self.get_logger().warning(f'could not reach the dashboard ({e}): cancel the goal by hand')

    def mark(self, x, y, yaw, direction):
        """A band of points just past the robot's front (or back) edge, 8 cm up, in the map frame."""
        xs = np.arange(HALF_LEN + 0.04, HALF_LEN + 0.15, 0.05) * direction
        ys = np.arange(-HALF_WID, HALF_WID + 0.001, 0.05)
        gx, gy = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + gx.ravel() * c - gy.ravel() * s
        py = y + gx.ravel() * s + gy.ravel() * c
        pts = np.stack([px, py, np.full_like(px, 0.08)], axis=1).astype(np.float32)
        self.marks.append((self.now() + MARK_S, pts))
        self.get_logger().info(f'marked {len(pts)} points as an obstacle for {MARK_S:.0f} s')

    def publish_marks(self):
        now = self.now()
        self.marks = [mk for mk in self.marks if mk[0] > now]
        pts = np.concatenate([p for _, p in self.marks]) if self.marks else np.zeros((0, 3), np.float32)
        if len(pts) or self.had_marks:
            self.pub_marks.publish(self.cloud(pts))
        if self.had_marks and not len(pts):            # expired: forget the low marks in the 3D costmap too
            if self.clear_local.service_is_ready():
                self.clear_local.call_async(self._Clear.Request())
            self.get_logger().info('obstacle marks expired')
        self.had_marks = bool(len(pts))

    def cloud(self, pts):
        m = PointCloud2()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * len(pts)
        m.data = pts.astype('<f4').tobytes() if len(pts) else b''
        return m


def main():
    rclpy.init()
    node = SlipWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
