#!/usr/bin/env python3
# Written 2026-10-06: idle "breathing room" behaviour.
"""When the robot is IDLE and BOXED IN, move it to the nearest open patch of floor.

    started by run/remap.sh with navigation (AUTO_CLEAR=1 in config/robot.env; 0 turns it off)

Idle     = navigation running, no goal active, robot not moving, for idle_sec (default 60 s).
Boxed in = sustained for that whole time: something nearly touching the body (< touch_dist of the
           lidar) in any direction, or fewer than free_sectors_needed of 8 directions with at least
           free_dist of room.
Then     : pick the nearest map cell within max_travel that has target_clearance of room on the
           static map, and send one Nav2 goal there (the user's own goals always preempt it).
           After a move (or a failed one): cooldown_sec pause. Two failures = give up until the
           surroundings change.

Deliberately conservative: it never moves while a goal is active or recent, logs every decision,
and a person standing briefly next to the robot does not trigger it (the condition must hold for
the full minute).
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    import cv2
except ImportError:
    cv2 = None

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


def sector_mins(msg):
    """Minimum finite range in 8 equal angular sectors (robot-relative; orientation irrelevant)."""
    r = np.asarray(msg.ranges, float)
    ok = np.isfinite(r) & (r > 0.05)
    if not ok.any():
        return np.full(8, np.inf)
    idx = (np.arange(len(r)) * 8) // len(r)
    out = np.full(8, np.inf)
    for s in range(8):
        m = ok & (idx == s)
        if m.any():
            out[s] = float(r[m].min())
    return out


class AutoClear(Node):
    def __init__(self):
        super().__init__('auto_clear')
        self.idle_sec = float(self.declare_parameter('idle_sec', 60.0).value)
        self.touch_dist = float(self.declare_parameter('touch_dist', 0.40).value)        # from the lidar
        self.free_dist = float(self.declare_parameter('free_dist', 0.80).value)
        self.free_sectors_needed = int(self.declare_parameter('free_sectors_needed', 3).value)
        self.target_clearance = float(self.declare_parameter('target_clearance', 0.65).value)
        self.max_travel = float(self.declare_parameter('max_travel', 4.0).value)
        self.cooldown_sec = float(self.declare_parameter('cooldown_sec', 180.0).value)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, None, spin_thread=True)
        self.map_msg = None
        self.scan = None
        self.speed = 1.0
        self.goal_active_until = time.monotonic() + 10.0   # treat start-up as "recently active"
        self.ok_since = None          # when the boxed-in condition STOPPED holding
        self.bad_since = None         # when it STARTED holding
        self.cooldown_until = 0.0
        self.failures = 0
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map_msg', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan_filtered', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(2.0, self.tick)
        self.get_logger().info(f'watching: idle {self.idle_sec:.0f}s + boxed in → relocate ≤ {self.max_travel} m')

    def on_odom(self, m):
        t = m.twist.twist
        self.speed = math.hypot(t.linear.x, t.linear.y) + abs(t.angular.z) * 0.3
        if self.speed > 0.03:
            self.goal_active_until = max(self.goal_active_until, time.monotonic() + 5.0)

    def on_status(self, m):
        if any(s.status in ACTIVE for s in m.status_list):
            self.goal_active_until = time.monotonic() + 5.0

    def boxed_in(self):
        if self.scan is None:
            return False, ''
        mins = sector_mins(self.scan)
        free = int((mins > self.free_dist).sum())
        touch = float(mins.min())
        if touch < self.touch_dist:
            return True, f'something {touch:.2f} m from the lidar (nearly touching)'
        if free < self.free_sectors_needed:
            return True, f'only {free}/8 directions have {self.free_dist} m of room'
        return False, ''

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.2))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def find_spot(self, rx, ry):
        """Nearest map point within max_travel whose STATIC clearance ≥ target_clearance."""
        m = self.map_msg
        res = m.info.resolution
        grid = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
        # unknown counts as blocked: never relocate into space the map doesn't vouch for
        blocked = ((grid >= 50) | (grid < 0)).astype(np.uint8)
        dt = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 3) * res
        ys, xs = np.where(dt >= self.target_clearance)
        if len(xs) == 0:
            return None
        px = m.info.origin.position.x + (xs + 0.5) * res
        py = m.info.origin.position.y + (ys + 0.5) * res
        d = np.hypot(px - rx, py - ry)
        order = np.argsort(d)
        for i in order:
            if 0.5 <= d[i] <= self.max_travel:
                return float(px[i]), float(py[i]), float(d[i]), float(dt[ys[i], xs[i]])
        return None

    def tick(self):
        now = time.monotonic()
        busy = now < self.goal_active_until or self.speed > 0.03 or now < self.cooldown_until
        bad, why = self.boxed_in()
        if not bad:
            self.bad_since = None
            self.failures = 0
            return
        if busy:
            self.bad_since = None
            return
        if self.bad_since is None:
            self.bad_since = now
            self.get_logger().info(f'boxed in ({why}): will relocate if this holds {self.idle_sec:.0f} s')
            return
        if now - self.bad_since < self.idle_sec or self.failures >= 2:
            return
        if cv2 is None or self.map_msg is None:
            return
        pos = self.robot_xy()
        if pos is None:
            return
        spot = self.find_spot(*pos)
        if spot is None:
            self.get_logger().warn('boxed in, but no open spot within reach on the map')
            self.cooldown_until = now + self.cooldown_sec
            return
        x, y, d, clr = spot
        self.get_logger().warn(f'idle and boxed in for {self.idle_sec:.0f} s ({why}): '
                               f'moving {d:.1f} m to open floor at ({x:.2f}, {y:.2f}), {clr:.2f} m of room there')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        yaw = math.atan2(y - pos[1], x - pos[0])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.client.wait_for_server(timeout_sec=2.0):
            return
        send = self.client.send_goal_async(goal)
        send.add_done_callback(self.on_sent)
        self.bad_since = None
        self.cooldown_until = now + self.cooldown_sec

    def on_sent(self, fut):
        try:
            handle = fut.result()
        except Exception:
            self.failures += 1
            return
        if not handle.accepted:
            self.failures += 1
            return
        res = handle.get_result_async()

        def done(f):
            try:
                ok = f.result().status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                ok = False
            self.failures = 0 if ok else self.failures + 1
            self.get_logger().info('relocated to open floor' if ok else
                                   f'relocation failed ({self.failures}/2' + (': giving up until things change)' if self.failures >= 2 else ')'))
        res.add_done_callback(done)


def main():
    rclpy.init()
    node = AutoClear()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
