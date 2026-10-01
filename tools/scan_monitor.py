#!/usr/bin/env python3
# Written 2026-10-01 (new tool, not a reconstruction).
"""Live view of what the lidar is sending, in the terminal (RViz shows the picture; this shows numbers).

  python3 tools/scan_monitor.py [--topic /scan]

Refreshes once a second: scan rate, beams per scan, how many returned a range, the nearest
obstacle (distance + bearing), the closest return in 8 sectors around the robot, and a small
top-down ASCII map (robot at the centre, front = up). Angles follow ROS: 0° = ahead,
+90° = left. Ctrl-C to quit.
"""
import argparse
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECTORS = [('front', 0), ('front-left', 45), ('left', 90), ('back-left', 135),
           ('back', 180), ('back-right', -135), ('right', -90), ('front-right', -45)]
GRID = 21            # ASCII map cells per side
GRID_RANGE = 3.0     # metres from centre to edge


def wrap_deg(a):
    return (a + 180) % 360 - 180


def render(scan):
    beams = [(scan.angle_min + i * scan.angle_increment, r) for i, r in enumerate(scan.ranges)]
    valid = [(a, r) for a, r in beams if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    out = [f'beams {len(beams)}   with range {len(valid)} ({100 * len(valid) / max(1, len(beams)):.0f}%)   '
           f'frame "{scan.header.frame_id}"   range limits {scan.range_min:.2f}–{scan.range_max:.1f} m']
    if not valid:
        return out + ['no valid returns: lens covered, or everything out of range?']
    a, r = min(valid, key=lambda b: b[1])
    out.append(f'nearest  {r:.2f} m at {wrap_deg(math.degrees(a)):+.0f}°')
    out.append('')
    for name, centre in SECTORS:
        sec = [r for a, r in valid if abs(wrap_deg(math.degrees(a) - centre)) <= 22.5]
        d = min(sec) if sec else None
        bar = '█' * int(min(d, 5.0) * 6) if d else ''
        out.append(f'  {name:>11} {f"{d:5.2f} m" if d else "   -   "}  {bar}')
    # top-down map: x ahead = up, y left = left
    cells = [[' '] * GRID for _ in range(GRID)]
    c = GRID // 2
    for a, r in valid:
        x, y = r * math.cos(a), r * math.sin(a)
        row, col = c - round(x / GRID_RANGE * c), c - round(y / GRID_RANGE * c)
        if 0 <= row < GRID and 0 <= col < GRID:
            cells[row][col] = '#'
    cells[c][c] = '^'
    out.append('')
    out.append(f'  top-down, ±{GRID_RANGE:.0f} m, front = up, ^ = robot')
    out += ['  |' + ' '.join(row) + '|' for row in cells]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/scan')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('scan_monitor')
    state = {'last': None, 'count': 0}

    def cb(msg):
        state['last'] = msg
        state['count'] += 1
    node.create_subscription(LaserScan, args.topic, cb, qos_profile_sensor_data)

    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - t0 < 1.0:
                continue
            hz = state['count'] / (time.time() - t0)
            t0, state['count'] = time.time(), 0
            lines = [f'{args.topic}: {hz:.1f} Hz' + ('   (NO DATA: is the lidar driver running?)' if hz == 0 else '')]
            if state['last'] is not None:
                lines += render(state['last'])
            print('\033[H\033[J' + '\n'.join(lines), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
