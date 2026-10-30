#!/usr/bin/env python3
# Written 2026-10-01: camera-aided localization with AprilTags. Not yet run on the robot end to end.
"""Builds a map of the AprilTags on the walls while mapping, then uses it to localize.

    record:    python3 nodes/tag_map.py --ros-args -p mode:=record -p tags_file:=<out.yaml>
    localize:  python3 nodes/tag_map.py --ros-args -p mode:=localize -p tags_file:=maps/<area>_tags.yaml

Input: /tags from nodes/camera_node.py (tag poses in camera_optical) and TF.

record    For every tag seen within max_range while the robot moves slowly, the tag's pose in the map
          frame (slam_toolbox's, live) is collected. Its robust mean is written to tags_file every 5 s and
          on exit. run/remap.sh saves it next to the map as maps/<area>_tags.yaml.
localize  Each sighting of a known tag gives a full robot pose in the map, independent of the lidar.
          When AMCL has no pose yet (after boot), or a consistent camera fix disagrees with AMCL by more
          than jump_dist / jump_yaw (AMCL lost, or the robot was carried), it publishes /initialpose to
          (re)start AMCL there. Small differences are left to AMCL, which is more precise once converged.
          Every fix also goes out on /tag_pose for RViz.
Both modes publish /tag_markers (the tags in the map) for RViz.

Camera tilt self-check (localize mode, 2026-10-10): a bumped camera made every fix "implausible".
While the robot stands still with a known tag 0.8-3.5 m away, the tag centre's elevation in the image
is compared with the one its recorded height and the camera height predict (no AMCL pose needed).
When >= 30 such sightings over >= 3 s agree within 0.5 deg and differ from the current trim by
> 0.4 deg (0.3 deg already nearly fails the fit), the camera pitch used here is trimmed by that much (capped at +-8 deg: beyond that the
mount needs fixing by hand). The trim is saved to ~/.campus_bot/run/camera_pitch_trim.yaml and loaded
by both modes at start; config/robot.env is never changed.
"""
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

import tags_common as tc

TRIM_FILE = os.path.expanduser('~/.campus_bot/run/camera_pitch_trim.yaml')


class TagMap(Node):
    def __init__(self):
        super().__init__('tag_map')
        self.mode = self.declare_parameter('mode', 'record').value
        self.tags_file = os.path.expanduser(self.declare_parameter('tags_file', '').value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        # record: only close, steady views go into the tag map
        self.rec_range = float(self.declare_parameter('record_max_range', 3.0).value)
        self.max_lin = float(self.declare_parameter('record_max_lin', 0.3).value)     # m/s
        self.max_ang = float(self.declare_parameter('record_max_ang', 0.3).value)     # rad/s
        # localize
        self.loc_range = float(self.declare_parameter('localize_max_range', 4.0).value)
        self.jump_dist = float(self.declare_parameter('jump_dist', 0.4).value)
        self.jump_yaw = float(self.declare_parameter('jump_yaw', 0.25).value)
        self.need_fixes = int(self.declare_parameter('fixes_to_confirm', 3).value)
        self.max_rms = float(self.declare_parameter('max_rms', 0.004).value)
        if self.mode not in ('record', 'localize'):
            raise SystemExit(f'mode must be record or localize, not {self.mode!r}')
        if not self.tags_file:
            raise SystemExit('tags_file is required')

        self.tf = Buffer(cache_time=Duration(seconds=30))
        # Its own node and thread: the lookups below wait, and TF must keep arriving meanwhile.
        self.tf_listener = TransformListener(self.tf, None, spin_thread=True)
        self.T_base_cam = None          # base_link ← camera_optical, with the pitch trim applied
        self.T_base_cam_tf = None       # the same, as the TF tree (config/robot.env) has it
        self.trim_deg = self.load_trim()
        self.trim_obs = deque(maxlen=120)   # (monotonic time, implied extra pitch deg)
        self.twist = None               # latest (v, w, time)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(Detection3DArray, '/tags', self.on_tags, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_markers = self.create_publisher(MarkerArray, '/tag_markers', latched)
        self.counts = {'seen': 0, 'used': 0, 'no_tf': 0, 'moving': 0, 'far': 0, 'odd': 0}

        if self.mode == 'record':
            self.obs = {}               # id → deque of (T_map_tag, weight)
            self.size = None
            self.dirty = False
            self.create_timer(5.0, self.save)
        else:
            self.tags = self.load(self.tags_file)
            self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.pub_fix = self.create_publisher(PoseWithCovarianceStamped, '/tag_pose', 10)
            self.fixes = deque(maxlen=10)   # (monotonic time, T_map_odom implied by the camera, sd_xy)
            self.last_init = 0.0
            self.publish_markers()
        self.create_timer(30.0, self.report)
        self.get_logger().info(f'mode {self.mode}, tags file {self.tags_file}')

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg):
        t = msg.twist.twist
        self.twist = (math.hypot(t.linear.x, t.linear.y), abs(t.angular.z), time.monotonic())

    def lookup(self, parent, child, stamp):
        try:
            return tc.tf_to_mat(self.tf.lookup_transform(parent, child, stamp,
                                                         timeout=Duration(seconds=0.15)).transform)
        except Exception:
            return None

    def on_tags(self, msg):
        if not msg.detections:
            return
        if self.T_base_cam is None:
            self.T_base_cam_tf = self.lookup(self.base_frame, msg.header.frame_id, Time())
            if self.T_base_cam_tf is None:
                self.counts['no_tf'] += len(msg.detections)
                return
            self.T_base_cam = self.trimmed(self.T_base_cam_tf, self.trim_deg)
            if self.trim_deg:
                self.get_logger().info(f'camera pitch trim {self.trim_deg:+.2f} deg (from {TRIM_FILE})')
        stamp = Time.from_msg(msg.header.stamp)
        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            try:
                tag_id = int(d.results[0].hypothesis.class_id)
            except ValueError:
                continue
            rays = [r.pose.pose.position for r in d.results[1:] if r.hypothesis.class_id == 'corner']
            seen = np.array([[r.x, r.y] for r in rays]) if len(rays) == 4 else None
            dets.append((tag_id, tc.pose_to_mat(d.results[0].pose.pose), d.bbox.size.x,
                         d.results[0].hypothesis.score < 0, seen))
        self.counts['seen'] += len(dets)
        if self.mode == 'record':
            self.record(dets, stamp)
        else:
            self.localize(dets, stamp)

    # ------------------------------------------------------------------ record
    def record(self, dets, stamp):
        tw = self.twist
        if tw is None or time.monotonic() - tw[2] > 1.0 or tw[0] > self.max_lin or tw[1] > self.max_ang:
            self.counts['moving'] += len(dets)        # motion blur and stamp error grow with speed
            return
        T_map_base = self.lookup(self.map_frame, self.base_frame, stamp)
        if T_map_base is None:
            self.counts['no_tf'] += len(dets)
            return
        T_map_cam = T_map_base @ self.T_base_cam
        for tag_id, T_cam_tag, size, ambiguous, _ in dets:
            rng = float(np.linalg.norm(T_cam_tag[:3, 3]))
            if rng > self.rec_range:
                self.counts['far'] += 1
                continue
            if ambiguous:                             # orientation may be flipped
                self.counts['odd'] += 1
                continue
            self.size = size
            self.obs.setdefault(tag_id, deque(maxlen=300)).append((T_map_cam @ T_cam_tag, 1.0 / max(rng, 0.5) ** 2))
            self.counts['used'] += 1
            self.dirty = True

    def summary(self, tag_id):
        """Robust weighted mean pose of one tag: drop sightings > 0.25 m from the median, average the rest."""
        obs = list(self.obs[tag_id])
        P = np.array([T[:3, 3] for T, _ in obs])
        W = np.array([w for _, w in obs])
        med = np.median(P, axis=0)
        keep = np.linalg.norm(P - med, axis=1) < 0.25
        if keep.sum() < max(3, 0.3 * len(obs)):
            keep = np.ones(len(obs), bool)
        p = np.average(P[keep], axis=0, weights=W[keep])
        R = tc.average_rotation([obs[i][0][:3, :3] for i in np.flatnonzero(keep)], W[keep])
        spread = float(np.sqrt(np.average(np.sum((P[keep] - p) ** 2, axis=1), weights=W[keep])))
        upright = self.upright(R)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = (upright if upright is not None else R), p
        return T, int(keep.sum()), spread, upright is not None

    @staticmethod
    def upright(R):
        """Tags are printed to hang upright on walls: when the average says so (face within 25° of
        horizontal, TOP within 25° of up), make it exactly so. Removes most of the orientation noise."""
        n, up = R[:, 2], R[:, 1]
        if abs(n[2]) > math.sin(math.radians(25)) or up[2] < math.cos(math.radians(25)):
            return None
        n = np.array([n[0], n[1], 0.0]) / math.hypot(n[0], n[1])
        z = np.array([0.0, 0.0, 1.0])
        return np.column_stack([np.cross(z, n), z, n])

    def save(self, markers=True):
        if self.mode != 'record' or not self.dirty:
            return
        tags = {}
        for tag_id in sorted(self.obs):
            if len(self.obs[tag_id]) < 5:
                continue                              # a passing glimpse: not trusted yet
            T, n, spread, upright = self.summary(tag_id)
            qx, qy, qz, qw = tc.mat_to_quat(T[:3, :3])
            facing = math.degrees(math.atan2(T[1, 2], T[0, 2]))    # direction the tag's face points
            tags[int(tag_id)] = {'x': round(float(T[0, 3]), 4), 'y': round(float(T[1, 3]), 4),
                                 'z': round(float(T[2, 3]), 4),
                                 'qx': round(float(qx), 6), 'qy': round(float(qy), 6),
                                 'qz': round(float(qz), 6), 'qw': round(float(qw), 6),
                                 'facing_deg': round(facing, 1), 'upright': upright, 'sightings': n,
                                 'spread_m': round(spread, 3)}
        data = {'frame': self.map_frame, 'family': tc.FAMILY, 'tag_size': self.size,
                'recorded': time.strftime('%Y-%m-%d %H:%M:%S'), 'tags': tags}
        tmp = self.tags_file + '.tmp'
        os.makedirs(os.path.dirname(os.path.abspath(self.tags_file)), exist_ok=True)
        with open(tmp, 'w') as f:
            f.write('# Written by nodes/tag_map.py (record mode): AprilTag poses in the map frame.\n')
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(tmp, self.tags_file)
        self.dirty = False
        self.tags = {i: self.entry_to_mat(e) for i, e in tags.items()}
        if markers:
            self.publish_markers()

    # ------------------------------------------------------------------ localize
    @staticmethod
    def entry_to_mat(e):
        T = np.eye(4)
        T[:3, :3] = tc.quat_to_mat(e['qx'], e['qy'], e['qz'], e['qw'])
        T[:3, 3] = (e['x'], e['y'], e['z'])
        return T

    def load(self, path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        tags = {int(i): self.entry_to_mat(e) for i, e in (data.get('tags') or {}).items()}
        self.size = data.get('tag_size')
        if not tags:
            raise SystemExit(f'{path} has no tags')
        self.get_logger().info(f'{len(tags)} tags loaded: {sorted(tags)}')
        return tags

    # ------------------------------------------------------------ camera tilt self-check
    @staticmethod
    def trimmed(T_base_cam, deg):
        """Camera pitched further down by deg: a rotation about the BODY's y axis, applied in the body
        frame (pre-multiplied). It used to be T @ Rx_optical(-deg), which assumed the optical -x axis is
        the body's y axis: true for an upright camera, reversed for the upside-down mount (2026-10-30),
        where it tilted the camera the wrong way (checked: +1 deg trim gave -1.00 deg; now +1.00 deg either
        way up). Upright, the pitch change is the same as before (+1.00 deg); the old version rotated about
        the camera's own sideways axis, ~2 deg (its yaw) off the body's, so the matrices differ slightly."""
        T = T_base_cam.copy()
        T[:3, :3] = tc.rpy_to_mat(0.0, math.radians(deg), 0.0) @ T_base_cam[:3, :3]
        return T

    def load_trim(self):
        try:
            with open(TRIM_FILE) as f:
                return float((yaml.safe_load(f) or {}).get('pitch_trim_deg', 0.0))
        except (OSError, ValueError, TypeError, AttributeError):
            return 0.0

    def check_tilt(self, dets):
        tw = self.twist
        if tw is None or tw[0] > 0.02 or tw[1] > 0.02 or time.monotonic() - tw[2] > 1.0:
            return                                       # only while standing still
        R_body = self.T_base_cam_tf[:3, :3] @ tc.R_BODY_OPTICAL.T
        p0 = math.degrees(tc.mat_to_rpy(R_body)[1])      # pitch the TF tree has (+ = nose down)
        cam_z = self.T_base_cam_tf[2, 3]
        now = time.monotonic()
        for tag_id, T_cam_tag, _, _, _ in dets:
            if tag_id not in self.tags:
                continue
            x, y, z = T_cam_tag[:3, 3]
            rng = math.sqrt(x * x + y * y + z * z)
            if not 0.8 <= rng <= 3.5:
                continue
            up = math.degrees(math.atan2(-y, math.hypot(x, z)))     # above the optical axis
            dz = self.tags[tag_id][2, 3] - cam_z
            elev = math.degrees(math.atan2(dz, math.sqrt(max(rng * rng - dz * dz, 1e-6))))
            self.trim_obs.append((now, up - elev - p0))
        recent = [d for t, d in self.trim_obs if now - t < 30.0]
        span = now - min((t for t, _ in self.trim_obs if now - t < 30.0), default=now)
        if len(recent) < 30 or span < 3.0:
            return
        med = float(np.median(recent))
        mad = float(np.median(np.abs(np.array(recent) - med)))
        if mad > 0.5 or abs(med - self.trim_deg) <= 0.4:
            return
        self.trim_obs.clear()
        if abs(med) > 8.0:
            self.get_logger().error(f'camera looks {med:+.1f} deg off its calibrated tilt: too much to trim, '
                                    'check the camera mount (re-measure: README, Camera localization)')
            return
        old, self.trim_deg = self.trim_deg, med
        self.T_base_cam = self.trimmed(self.T_base_cam_tf, med)
        os.makedirs(os.path.dirname(TRIM_FILE), exist_ok=True)
        with open(TRIM_FILE + '.tmp', 'w') as f:
            f.write(f'# written by nodes/tag_map.py {time.strftime("%Y-%m-%d %H:%M:%S")}: camera pitch beyond '
                    f'config/robot.env (+ = more nose-down), from {len(recent)} still sightings\n'
                    f'pitch_trim_deg: {med:.3f}\n')
        os.replace(TRIM_FILE + '.tmp', TRIM_FILE)
        self.get_logger().warning(f'camera tilt changed: pitch trim {old:+.2f} -> {med:+.2f} deg '
                                  f'({len(recent)} sightings, spread {mad:.2f} deg), saved to {TRIM_FILE}')

    def localize(self, dets, stamp):
        self.check_tilt(dets)
        T_odom_base = self.lookup(self.odom_frame, self.base_frame, stamp)
        if T_odom_base is None:
            self.counts['no_tf'] += len(dets)
            return
        T_base_cam_inv = tc.inv(self.T_base_cam)
        obs, guesses, nearest = [], [], 1e9
        for tag_id, T_cam_tag, size, ambiguous, seen in dets:
            if tag_id not in self.tags or seen is None:
                continue
            rng = float(np.linalg.norm(T_cam_tag[:3, 3]))
            if rng > self.loc_range:
                self.counts['far'] += 1
                continue
            obs.append((tc.tag_corners(self.tags[tag_id], self.size or size), seen))
            guesses.append(tc.planar(self.tags[tag_id] @ tc.inv(T_cam_tag) @ T_base_cam_inv)[:3])
            nearest = min(nearest, rng)
        if not obs:
            return
        T_amcl = self.lookup(self.map_frame, self.odom_frame, Time())
        if T_amcl is not None:
            guesses.append(tc.planar(T_amcl @ T_odom_base)[:3])
        # x, y, yaw only: the camera's height and tilt are known (config/robot.env, tools/camera_pose.py)
        best = None
        for g in guesses:
            f = tc.planar_fix(obs, self.T_base_cam, g)
            if f is not None and (best is None or f[3] < best[3]):
                best = f
        # rms in normalized units: ~0.004 is 5 px. More means a moved tag, a wrong camera pose or a bad view.
        if best is None or best[3] > self.max_rms:
            self.counts['odd'] += len(obs)
            return
        x, y, yaw, _ = best
        self.counts['used'] += len(obs)
        # One tag pins the heading weakly (5-10 cm per metre beyond 2 m in simulation); two in view, tightly.
        sd_xy, sd_yaw = (0.05, 0.02) if len(obs) > 1 else (max(0.05, 0.06 * nearest ** 2), 0.03 * nearest)
        self.publish_pose(self.pub_fix, x, y, yaw, stamp, sd_xy, sd_yaw)
        # Robot motion between fixes cancels out in map → odom, so fixes from different moments compare.
        T_map_odom = tc.planar_mat(x, y, yaw) @ tc.inv(tc.planar_mat(*tc.planar(T_odom_base)[:3]))
        now = time.monotonic()
        self.fixes.append((now, T_map_odom, sd_xy))
        recent = [(T, sd) for t, T, sd in self.fixes if now - t < 3.0]
        if len(recent) < self.need_fixes or now - self.last_init < 5.0:
            return
        # Confirmed when the recent fixes agree; their mean is the camera's answer.
        P = np.array([tc.planar(T @ T_odom_base)[:3] for T, _ in recent])
        mx, my = P[:, 0].mean(), P[:, 1].mean()
        myaw = math.atan2(np.sin(P[:, 2]).mean(), np.cos(P[:, 2]).mean())
        spread = math.sqrt(np.mean((P[:, 0] - mx) ** 2 + (P[:, 1] - my) ** 2))
        yaw_spread = math.sqrt(np.mean([tc.wrap(a - myaw) ** 2 for a in P[:, 2]]))
        if spread > 0.25 or yaw_spread > 0.12:
            return
        sd = max(sd for _, sd in recent)
        if T_amcl is None:
            why = 'AMCL has no pose yet'
        else:
            ax, ay, ayaw = tc.planar(T_amcl @ T_odom_base)[:3]
            dist, dyaw = math.hypot(ax - mx, ay - my), abs(tc.wrap(ayaw - myaw))
            # a single far tag is too loose to overrule AMCL on a small difference
            if dist <= max(self.jump_dist, 3 * sd) and dyaw <= max(self.jump_yaw, 3 * sd_yaw):
                return
            why = f'AMCL off by {dist:.2f} m / {math.degrees(dyaw):.0f}°'
        self.publish_pose(self.pub_init, mx, my, myaw, stamp, max(sd, 0.1), max(sd_yaw, 0.05))
        self.last_init = now
        self.fixes.clear()
        self.get_logger().warning(f'{why}: pose set from the camera: x {mx:.2f} y {my:.2f} '
                                  f'yaw {math.degrees(myaw):.0f}° ({len(obs)} tag(s) in view, nearest {nearest:.1f} m)')

    def publish_pose(self, pub, x, y, yaw, stamp, sd_xy, sd_yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp.to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = sd_xy ** 2
        cov[35] = sd_yaw ** 2
        m.pose.covariance = cov
        pub.publish(m)

    # ------------------------------------------------------------------ output
    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for tag_id, T in sorted(getattr(self, 'tags', {}).items()):
            qx, qy, qz, qw = tc.mat_to_quat(T[:3, :3])
            for k, (typ, scale) in enumerate(((Marker.CUBE, (0.16, 0.16, 0.01)), (Marker.TEXT_VIEW_FACING, None))):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns, m.id, m.type = 'tags', 2 * tag_id + k, typ
                m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in T[:3, 3])
                if typ == Marker.CUBE:
                    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
                    m.scale.x, m.scale.y, m.scale.z = scale
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.9
                else:
                    m.pose.position.z += 0.2
                    m.pose.orientation.w = 1.0
                    m.text = f'tag {tag_id}'
                    m.scale.z = 0.15
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                arr.markers.append(m)
        self.pub_markers.publish(arr)

    def report(self):
        c = self.counts
        extra = f', {len([i for i in self.obs if len(self.obs[i]) >= 5])} tags mapped' if self.mode == 'record' else ''
        trim = f', camera pitch trim {self.trim_deg:+.2f} deg' if self.trim_deg else ''
        self.get_logger().info(f'last 30 s: {c["seen"]} sightings, {c["used"]} used, {c["moving"]} while moving, '
                               f'{c["far"]} too far, {c["no_tf"]} without TF, {c["odd"]} implausible{extra}{trim}')
        self.counts = {k: 0 for k in c}


def main():
    rclpy.init()
    node = TagMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.mode == 'record':
            node.dirty = True
            node.save(markers=False)   # the ROS context may already be shut down
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
