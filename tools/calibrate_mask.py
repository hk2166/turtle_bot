#!/usr/bin/env python3
# Written 2026-09-30 (new tool: the original mask angles were lost, so they must be re-measured).
"""Measure the column slivers and write config/scan_mask.yaml.

Park the robot with nothing within ~1 m, lidar spinning (bring-up running), then:
    python3 tools/calibrate_mask.py
Beams that return closer than SELF_RADIUS in most scans are the robot's own columns. They're padded
by --pad beams each side (vibration) and grouped into sectors. Expect 4 sectors, ~52 beams in total.
scan_filter.py picks up the new file within 2 s; no restart needed.
"""
import argparse
import math
import os
import sys
import time
from datetime import date

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import MASK_FILE, beam_angles_deg, load_env  # noqa: E402


def main():
    env = load_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default=env.get('SCAN_TOPIC', '/scan'), help='the RAW scan')
    ap.add_argument('--scans', type=int, default=50)
    ap.add_argument('--self-radius', type=float, default=float(env.get('SELF_RADIUS', 0.40)))
    ap.add_argument('--pad', type=int, default=2, help='extra beams masked each side of a column')
    ap.add_argument('--dry-run', action='store_true', help="print, don't write the file")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('calibrate_mask')
    scans = []
    node.create_subscription(LaserScan, args.topic, scans.append, qos_profile_sensor_data)
    print(f'Collecting {args.scans} scans from {args.topic} … keep everything ≥ 1 m from the robot.')
    t0 = time.monotonic()
    while len(scans) < args.scans and time.monotonic() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if len(scans) < 10:
        sys.exit(f'Only got {len(scans)} scans in 30 s: is the lidar running?')

    ref = scans[-1]
    n = len(ref.ranges)
    scans = [s for s in scans if len(s.ranges) == n]
    hits = [0] * n
    near = 0
    for s in scans:
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min < r < args.self_radius:
                hits[i] += 1
            elif math.isfinite(r) and args.self_radius <= r < 0.8:
                near += 1
    self_beams = {i for i in range(n) if hits[i] > 0.5 * len(scans)}
    if not self_beams:
        sys.exit(f'No returns closer than {args.self_radius} m: are the columns really in the scan plane? '
                 'Nothing written.')
    padded = {(i + d) % n for i in self_beams for d in range(-args.pad, args.pad + 1)}

    # Group contiguous beams (wrapping around the end of the array) into sectors.
    order = sorted(padded)
    groups, cur = [], [order[0]]
    for i in order[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            groups.append(cur)
            cur = [i]
    groups.append(cur)
    if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == n - 1:
        groups[0] = groups.pop() + groups[0]

    angles = beam_angles_deg(ref.angle_min, ref.angle_increment, n)
    half = math.degrees(ref.angle_increment) / 2 * 0.99   # sector edges halfway between beams
    sectors = []
    for g in groups:
        a0 = (angles[g[0]] - half + 180) % 360 - 180
        a1 = (angles[g[-1]] + half + 180) % 360 - 180
        sectors.append((round(a0, 2), round(a1, 2)))
        print(f'  sector {a0:+8.2f}° … {a1:+8.2f}°   {len(g)} beams')
    print(f'Total {len(padded)} of {n} beams masked in {len(sectors)} sectors (last good: 52 in 4).')
    if len(sectors) != 4:
        print(f'WARNING: expected 4 columns, found {len(sectors)}. Is something else close to the robot?')
    if near > len(scans) * 3:
        print('WARNING: objects within 0.8 m: move the robot into open space and rerun for a clean mask.')

    if args.dry_run:
        return
    with open(MASK_FILE, 'w') as f:
        f.write(f'# Column mask for scan_filter.py, measured by tools/calibrate_mask.py on {date.today()}.\n'
                f'# {len(padded)} of {n} beams in {len(sectors)} sectors, self radius {args.self_radius} m, '
                f'pad {args.pad}. Degrees in the `laser` frame; start > end wraps through ±180.\n'
                'calibrated: true\nsectors:\n')
        for a0, a1 in sectors:
            f.write(f'  - [{a0}, {a1}]\n')
    print(f'Wrote {MASK_FILE}')


if __name__ == '__main__':
    main()
