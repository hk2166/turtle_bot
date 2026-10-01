#!/usr/bin/env python3
# Written 2026-10-01 after the first pantry map came out as rotated, sheared wall fragments.
"""Do the lidar and the wheel odometry agree? Finds LASER_YAW and odometry sign/scale problems.

    (bring-up running: run/remap.sh, or the sensors it leaves up)
    python3 tools/check_motion.py            then drive with run/drive.sh in another terminal:
                                             ~1 m forward, ~1 m back, a slow 90° turn left, one right.
                                             Ctrl-C (or --seconds) ends the recording.

The motion the lidar sees (scan matching between nearby scans, columns ignored) is compared with
/odom for the same moments:
  - turning:  the lidar must see the same rotation as the odometry, same sign. Opposite sign means
              the lidar is upside down (sllidar `inverted`) or the firmware's yaw sign is wrong.
  - driving:  the direction the lidar sees itself move, relative to the odometry's "forward", IS the
              lidar's mounting yaw: that's LASER_YAW in config/robot.env.
  - scale:    odometry distance / lidar distance and rotation ratios → wheel calibration hints.
SLAM can't map with LASER_YAW wrong: every straight move is applied in the wrong direction.
"""
import argparse
import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import load_env  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def scan_points(msg, min_r, max_r):
    r = np.asarray(msg.ranges, float)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > max(min_r, msg.range_min)) & (r < min(max_r, msg.range_max))
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def icp(src, dst, theta0, iters=40, max_pair=0.3):
    """Rigid 2D transform (theta, t) mapping src onto dst. Returns (theta, t, rms, inlier fraction)."""
    tree = cKDTree(dst)
    th, t = theta0, np.zeros(2)
    for k in range(iters):
        c, s = math.cos(th), math.sin(th)
        p = src @ np.array([[c, s], [-s, c]]) + t
        d, j = tree.query(p)
        gate = max(max_pair * (0.5 ** (k // 10)), 0.05)
        m = d < gate
        if m.sum() < 30:
            return None
        A, B = src[m], dst[j[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(th), math.sin(th)
        t = cb - ca @ np.array([[c, s], [-s, c]])
    c, s = math.cos(th), math.sin(th)
    d, _ = tree.query(src @ np.array([[c, s], [-s, c]]) + t)
    m = d < 0.05
    return th, t, float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else 1.0, float(m.mean())


def odom_at(hist, t):
    """(x, y, yaw) of the odometry at time t, interpolated."""
    ts = hist[:, 0]
    i = int(np.searchsorted(ts, t))
    if i <= 0 or i >= len(ts):
        return None
    a, b = hist[i - 1], hist[i]
    f = (t - a[0]) / max(b[0] - a[0], 1e-9)
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * wrap(b[3] - a[3])


def rel(p, q):
    """Motion from pose p to pose q, in p's frame: (dx, dy, dyaw)."""
    c, s = math.cos(p[2]), math.sin(p[2])
    dx, dy = q[0] - p[0], q[1] - p[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(q[2] - p[2])


def analyse(scans, hist, step_d=0.08, step_a=math.radians(6)):
    """scans: [(t, points)], hist: Nx4 array (t, x, y, yaw). Returns per-pair (odom_motion, laser_motion)."""
    keys = []
    for t, pts in scans:
        o = odom_at(hist, t)
        if o is None or len(pts) < 80:
            continue
        if not keys:
            keys.append((t, pts, o))
            continue
        d = rel(keys[-1][2], o)
        if math.hypot(d[0], d[1]) > step_d or abs(d[2]) > step_a:
            keys.append((t, pts, o))
    pairs = []
    for (t0, p0, o0), (t1, p1, o1) in zip(keys, keys[1:]):
        do = rel(o0, o1)
        if math.hypot(do[0], do[1]) > 0.4 or abs(do[2]) > math.radians(25):
            continue                      # a gap in the data: too far apart to match reliably
        best = None
        for th0 in {do[2], -do[2]}:        # don't assume the odometry's sign is right
            r = icp(p1, p0, th0)
            if r and (best is None or r[3] > best[3] + 0.02 or (abs(r[3] - best[3]) <= 0.02 and r[2] < best[2])):
                best = r
        if best is None or best[3] < 0.5 or best[2] > 0.04:
            continue
        th, t, _, _ = best
        pairs.append((do, (t[0], t[1], th)))
    return pairs


def report(pairs, laser_yaw_now):
    print(f'\n{len(pairs)} usable scan-match steps')
    rot = [(o[2], l[2]) for o, l in pairs if abs(o[2]) > math.radians(3) or abs(l[2]) > math.radians(3)]
    lin = [(o, l) for o, l in pairs if math.hypot(o[0], o[1]) > 0.04 and abs(o[2]) < math.radians(3)
           and math.hypot(l[0], l[1]) > 0.02]
    ok = True
    if len(rot) >= 3:
        ratio = np.median([l / o for o, l in rot if abs(o) > 1e-3])
        print(f'TURNING   lidar rotation / odometry rotation = {ratio:+.2f} (median of {len(rot)}; expect +1.00)')
        if ratio < 0:
            ok = False
            print('  ✗ OPPOSITE SIGNS: the lidar sees the robot turn the other way from /odom.')
            print('    Either the lidar is upside down (set inverted:=true for sllidar in launch/bringup.launch.py)')
            print('    or the firmware reports yaw backwards. A left turn (counter-clockwise from above) must')
            print('    give a POSITIVE angular.z in /odom: check with  ros2 topic echo /odom  while turning left.')
        elif abs(ratio - 1) > 0.1:
            print(f'  ! odometry turns {1 / ratio:.2f}× what the lidar sees: calibrate the wheel separation'
                  ' (tools/calibrate_odom.py spin)')
        else:
            print('  ✓ rotation agrees')
    else:
        print('TURNING   not enough turning recorded (turn ~90° slowly each way)')
    if len(lin) >= 3:
        # the laser's motion is the base's motion rotated by -LASER_YAW: the angle between them is the yaw
        ang = [wrap(math.atan2(o[1], o[0]) - math.atan2(l[1], l[0])) for o, l in lin]
        yaw = math.atan2(np.mean(np.sin(ang)), np.mean(np.cos(ang)))
        spread = math.degrees(math.sqrt(np.mean([wrap(a - yaw) ** 2 for a in ang])))
        scale = np.median([math.hypot(o[0], o[1]) / math.hypot(l[0], l[1]) for o, l in lin])
        print(f'DRIVING   lidar mounting yaw = {math.degrees(yaw):+.1f}° (±{spread:.0f}°, {len(lin)} steps); '
              f'robot.env LASER_YAW = {math.degrees(laser_yaw_now):+.1f}°')
        nice = round(math.degrees(yaw) / 90) * 90
        if abs(wrap(yaw - laser_yaw_now)) > math.radians(10):
            ok = False
            print(f'  ✗ LASER_YAW is wrong. Set  LASER_YAW={math.radians(nice) if abs(math.degrees(yaw) - nice) < 15 else yaw:.4f}'
                  f'   ({nice if abs(math.degrees(yaw) - nice) < 15 else round(math.degrees(yaw))}°) in config/robot.env')
        else:
            print('  ✓ LASER_YAW agrees')
        print(f'          odometry distance / lidar distance = {scale:.2f}'
              + ('' if abs(scale - 1) < 0.08 else '  ! calibrate the wheel diameter (tools/calibrate_odom.py straight)'))
    else:
        print('DRIVING   not enough straight driving recorded (drive ~1 m forward and back)')
    return ok


def main():
    env = load_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scan', default=env.get('SCAN_TOPIC', '/scan'), help='RAW scan (columns cut by --min-range)')
    ap.add_argument('--odom', default=env.get('ODOM_TOPIC', '/odom'))
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--min-range', type=float, default=0.45, help='ignore returns closer than this (the columns)')
    a = ap.parse_args()
    # Ctrl-C or SIGTERM (run/check_motion.sh) ends the recording and goes on to the analysis.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    node = rclpy.create_node('check_motion')
    scans, odo = [], []
    node.create_subscription(LaserScan, a.scan, lambda m: scans.append(
        (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, scan_points(m, a.min_range, 10.0))), qos_profile_sensor_data)

    def on_odom(m):
        q = m.pose.pose.orientation
        odo.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                    m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    node.create_subscription(Odometry, a.odom, on_odom, 20)
    print(f'Recording {a.scan} + {a.odom} for up to {a.seconds:.0f} s. Drive now (run/drive.sh): ~1 m forward,')
    print('back, then a slow 90° turn each way. Ctrl-C when done.')
    end = time.monotonic() + a.seconds
    shown = 0.0
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if sys.stdout.isatty() and time.monotonic() - shown > 0.5:
                shown = time.monotonic()
                sys.stdout.write(f'\r  {len(scans)} scans, {len(odo)} odometry messages ')
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    print()
    if len(scans) < 20 or len(odo) < 20:
        sys.exit('not enough data: is the bring-up running (lidar on /scan, ESP32 on /odom)?')
    hist = np.array(sorted(odo))
    moved = math.hypot(hist[-1, 1] - hist[0, 1], hist[-1, 2] - hist[0, 2])
    turned = np.abs(np.diff(np.unwrap(hist[:, 3]))).sum()
    print(f'odometry: {moved:.2f} m net, {math.degrees(turned):.0f}° turned in total')
    ok = report(analyse(scans, hist), float(env.get('LASER_YAW', 0.0)))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
