#!/usr/bin/env python3
# Written 2026-10-10: reuse recorded AprilTags on a new map of the same place.
"""Carry maps/<from>_tags.yaml over to another map of the same rooms, without re-driving the tags.

    python3 tools/transfer_tags.py <from_area> <to_area> [--force]   e.g. transfer_tags.py asdf asdf3

Each SLAM run has its own frame. The tool finds the rigid transform between the two clean maps with
tools/merge_maps.py's alignment (rotation search + FFT correlation, refined on the distance field),
then moves every tag's x/y and turns its orientation by the same angle (heights unchanged).
Alignment score = share of <from>'s walls landing on <to>'s walls:
    >= 0.75 trust it      0.60-0.75 written with a warning      < 0.60 refused (unless --force)
Writes maps/<to_area>_tags.yaml (an existing one is kept as .bak).
"""
import argparse
import math
import os
import shutil
import sys
import time

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PROJECT, 'nodes'))
import merge_maps as mm      # noqa: E402
import tags_common as tc     # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    src_tags = os.path.join(PROJECT, 'maps', f'{a.src}_tags.yaml')
    data = yaml.safe_load(open(src_tags))

    # transform src-map frame -> dst-map frame: merge_maps aligns its B onto its A
    triA, resA, oxA, oyA = mm.load(a.dst)
    triB, resB, oxB, oyB = mm.load(a.src)
    theta, t0, _ = mm.coarse_align(triA, resA, oxA, oyA, triB, resB, oxB, oyB)
    dtA = cv2.distanceTransform((triA != 2).astype(np.uint8), cv2.DIST_L2, 3) * resA
    theta, t0, score = mm.refine(mm.occ_points(triB, resB, oxB, oyB), dtA, resA, oxA, oyA, theta, t0)
    print(f'{a.src} -> {a.dst}: rotate {math.degrees(theta):+.1f}°, shift ({t0[0]:+.2f}, {t0[1]:+.2f}) m, '
          f'score {score:.2f}')
    if score < 0.60 and not a.force:
        sys.exit('REFUSED: the maps don\'t line up well enough (< 0.60). Re-record the tags on the new map.')
    if score < 0.75:
        print('! weak alignment: check the tags on the dashboard / with a live test before relying on them')

    Rz = tc.rpy_to_mat(0.0, 0.0, theta)
    out = {}
    for tid, e in (data.get('tags') or {}).items():
        x, y = mm.apply_T(np.array([[e['x'], e['y']]]), theta, t0)[0]
        R = Rz @ tc.quat_to_mat(e['qx'], e['qy'], e['qz'], e['qw'])
        qx, qy, qz, qw = tc.mat_to_quat(R)
        n = dict(e)
        n.update(x=round(float(x), 4), y=round(float(y), 4),
                 qx=round(float(qx), 6), qy=round(float(qy), 6), qz=round(float(qz), 6), qw=round(float(qw), 6),
                 facing_deg=round((e['facing_deg'] + math.degrees(theta) + 180) % 360 - 180, 1))
        out[tid] = n
    dst_tags = os.path.join(PROJECT, 'maps', f'{a.dst}_tags.yaml')
    if os.path.exists(dst_tags):
        shutil.copy(dst_tags, dst_tags + '.bak')
    data = dict(data, tags=out, recorded=f'{data.get("recorded")} (carried over from {a.src})')
    with open(dst_tags, 'w') as f:
        f.write(f'# AprilTags carried from maps/{a.src}_tags.yaml by tools/transfer_tags.py, '
                f'{time.strftime("%Y-%m-%d %H:%M")}\n# alignment: rotate {math.degrees(theta):+.2f} deg, '
                f'shift ({t0[0]:+.3f}, {t0[1]:+.3f}) m, score {score:.2f}\n')
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'wrote {dst_tags}: {len(out)} tags')


if __name__ == '__main__':
    main()
