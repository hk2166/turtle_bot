#!/usr/bin/env python3
# Written 2026-10-01: find the robot's pose by fitting the current lidar scan to the map.
"""Fit the live scan to the map around a rough guess. Used by run/reset_pose.sh.

    python3 tools/fit_pose.py [X Y YAW_DEG] [--radius 1.0] [--angle 30]
        prints "x y yaw_rad score" for the best pose within --radius metres and --angle degrees of the
        guess (default guess: 0 0 0, the spot where mapping started). score = the fraction of laser
        points that land within 10 cm of a wall in the map (good fits score 0.8+).

Needs the map (/map from the map server, latched) and /scan_filtered, i.e. navigation running.
"""
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('guess', nargs='*', type=float, help='X Y YAW_DEG (default 0 0 0)')
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--angle', type=float, default=30.0)
    ap.add_argument('--scan', default='/scan_filtered')
    ap.add_argument('--map-file', help='fit against a saved map yaml instead of the live /map '
                                       '(no navigation needed, e.g. before remap.sh --continue)')
    a = ap.parse_args()
    if len(a.guess) not in (0, 3):
        sys.exit('give X Y YAW_DEG, or nothing')
    gx, gy, gyaw = (a.guess[0], a.guess[1], math.radians(a.guess[2])) if a.guess else (0.0, 0.0, 0.0)

    rclpy.init()
    node = rclpy.create_node('fit_pose')
    buf = Buffer()
    TransformListener(buf, node)
    maps, scans = [], []
    node.create_subscription(OccupancyGrid, '/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan, a.scan, scans.append, qos_profile_sensor_data)
    T_bl = None
    end = time.monotonic() + 10
    if a.map_file:
        import yaml
        meta = yaml.safe_load(open(a.map_file))
        pgm = os.path.join(os.path.dirname(os.path.abspath(a.map_file)), meta['image'])
        img = cv2.imread(pgm, -1)
        if img is None:
            sys.exit(f'cannot read {pgm}')
        maps.append(('file', np.where(img == 0, 100, np.where(img == 254, 0, -1))[::-1].astype(np.int8),
                     float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])))
    while time.monotonic() < end and (not maps or not scans or T_bl is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if scans and T_bl is None:
            try:
                T_bl = buf.lookup_transform('base_link', scans[-1].header.frame_id, Time(),
                                            timeout=Duration(seconds=0)).transform
            except Exception:
                pass
    node.destroy_node()
    rclpy.try_shutdown()
    if not maps or not scans or T_bl is None:
        sys.exit('no ' + ('map' if not maps else 'scan' if not scans else 'base_link → laser TF') +
                 ': is navigation running (run/nav.sh)?')

    # the map as a "distance to the nearest wall" image
    m = maps[-1]
    if isinstance(m, tuple):
        _, grid, res, ox, oy = m
        h, w = grid.shape
    else:
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.asarray(m.data, np.int8).reshape(h, w)
    dist = cv2.distanceTransform(np.where(grid >= 65, 0, 1).astype(np.uint8), cv2.DIST_L2, 3) * res

    # the scan in the robot's frame
    s = scans[-1]
    r = np.asarray(s.ranges, float)
    ang = s.angle_min + s.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.3) & (r < 10.0)
    q = T_bl.rotation
    lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    px = T_bl.translation.x + r[ok] * np.cos(ang[ok] + lyaw)
    py = T_bl.translation.y + r[ok] * np.sin(ang[ok] + lyaw)
    if len(px) < 50:
        sys.exit(f'only {len(px)} usable laser points: is something covering the lidar?')

    def score(x, y, yaw):
        c, sn = math.cos(yaw), math.sin(yaw)
        mx = ((x + c * px - sn * py - ox) / res).astype(int)
        my = ((y + sn * px + c * py - oy) / res).astype(int)
        inside = (mx >= 0) & (mx < w) & (my >= 0) & (my < h)
        d = np.full(len(mx), 9.9)
        d[inside] = dist[my[inside], mx[inside]]
        return float((d < 0.10).mean())

    def search(cx, cy, cyaw, rad, step, arad, astep):
        best = (-1.0, cx, cy, cyaw)
        for dyaw in np.arange(-arad, arad + 1e-9, astep):
            for dx in np.arange(-rad, rad + 1e-9, step):
                for dy in np.arange(-rad, rad + 1e-9, step):
                    if dx * dx + dy * dy > rad * rad + 1e-9:
                        continue
                    sc = score(cx + dx, cy + dy, cyaw + dyaw)
                    if sc > best[0]:
                        best = (sc, cx + dx, cy + dy, cyaw + dyaw)
        return best

    best = search(gx, gy, gyaw, a.radius, 0.1, math.radians(a.angle), math.radians(3))
    best = search(best[1], best[2], best[3], 0.12, 0.02, math.radians(4), math.radians(0.5))
    sc, x, y, yaw = best
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    print(f'{x:.3f} {y:.3f} {yaw:.4f} {sc:.2f}')


if __name__ == '__main__':
    main()
