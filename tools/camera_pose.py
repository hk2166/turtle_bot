#!/usr/bin/env python3
# Written 2026-10-01: measure where the mast camera is and where it points, from one tag.
"""Measure the camera's mounting (CAMERA_X/Y/Z, ROLL, PITCH, YAW in config/robot.env) with one printed tag.

    run/camera.sh off
    python3 tools/camera_pose.py --wall 1.50 1.25        # tag upright, facing the robot, centre 1.50 m
                                                         # ahead of the drive axle and 1.25 m above floor
    python3 tools/camera_pose.py --floor 2.00            # tag flat on the floor, TOP edge pointing away
    ... add --write to store the result in config/robot.env

Set-up: the tag centred on the robot's straight-ahead line (use a tape line from the axle centre; pass
--left 0.10 if it sits 10 cm to the robot's left). The axle (base_link) is at plate centre, X = 200 mm.
Use --wall when the camera looks level or up (tag at about camera height), --floor when it looks down.
Errors here become localization errors: 1° of yaw is 5 cm sideways at 3 m. Measure D and H to ±5 mm.
"""
import argparse
import math
import os
import re
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
import campus_common as cc  # noqa: E402
import tags_common as tc  # noqa: E402

ENV = os.path.join(tc.PROJECT_DIR, 'config', 'robot.env')


def tag_in_base(a):
    """T_base_tag for the set-up described on the command line."""
    T = np.eye(4)
    if a.wall:
        d, h = a.wall
        # tag x = robot right, tag y = up, tag z (out of the paper) = toward the robot
        T[:3, :3] = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
        T[:3, 3] = (d, a.left, h)
    else:
        # tag x = robot right, tag y (TOP) = forward, tag z = up
        T[:3, :3] = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
        T[:3, 3] = (a.floor, a.left, 0.0)
    return T


def write_env(vals):
    with open(ENV) as f:
        text = f.read()
    for key, v in vals.items():
        line = f'{key}={v:.4f}'
        if re.search(rf'^{key}=', text, re.M):
            text = re.sub(rf'^{key}=[^ \t#\n]*', line, text, flags=re.M)
        else:
            text = re.sub(r'^(CAMERA_PITCH=.*)$', rf'\1\n{line}', text, count=1, flags=re.M)
    with open(ENV, 'w') as f:
        f.write(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--wall', nargs=2, type=float, metavar=('D', 'H'), help='upright tag: distance, height (m)')
    g.add_argument('--floor', type=float, metavar='D', help='tag flat on the floor: distance (m)')
    ap.add_argument('--left', type=float, default=0.0, help="tag centre's offset to the robot's left (m)")
    ap.add_argument('--id', type=int, help='use only this tag id')
    ap.add_argument('--size-mm', type=float, help='measured black square (default: TAG_SIZE in robot.env)')
    ap.add_argument('--frames', type=int, default=40)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--write', action='store_true', help='store the result in config/robot.env')
    a = ap.parse_args()

    env = cc.load_env()
    size = a.size_mm / 1000.0 if a.size_mm else float(env.get('TAG_SIZE', 0.16))
    w, h = int(env.get('CAMERA_WIDTH', 1280)), int(env.get('CAMERA_HEIGHT', 720))
    busy = tc.camera_busy(a.device)
    if busy:
        sys.exit(f'{a.device} is in use (PID {busy}). Stop the camera node first: run/camera.sh off')
    K, D, calibrated = tc.load_intrinsics(w, h)
    if not calibrated:
        print('! camera not calibrated (tools/calibrate_camera.py): distances may be off by ~5%')
    cap = tc.open_camera(a.device, w, h)
    if cap is None:
        sys.exit(f'cannot open {a.device}')
    det = tc.TagDetector()
    T_base_tag = tag_in_base(a)
    T_body_opt = np.eye(4)
    T_body_opt[:3, :3] = tc.R_BODY_OPTICAL
    samples, seen, end = [], set(), time.monotonic() + 30
    print(f'tag size {size * 1000:.0f} mm; looking for the tag for up to 30 s ...')
    try:
        while len(samples) < a.frames and time.monotonic() < end:
            ok, img = cap.read()
            if not ok:
                sys.exit('camera read failed')
            for tag_id, corners in det.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)):
                seen.add(tag_id)
                if a.id is not None and tag_id != a.id:
                    continue
                T_opt_tag, _ = tc.tag_pose(corners, size, K, D)
                if T_opt_tag is not None:
                    samples.append(T_base_tag @ tc.inv(T_opt_tag) @ tc.inv(T_body_opt))
    finally:
        cap.release()
    if a.id is None and len(seen) > 1:
        sys.exit(f'several tags in view ({sorted(seen)}): pass --id')
    if len(samples) < 10:
        sys.exit(f'only {len(samples)} good views of the tag (saw ids {sorted(seen) or "none"}). '
                 'Closer, better lit, or the whole tag in the picture?')
    P = np.array([T[:3, 3] for T in samples])
    R = tc.average_rotation([T[:3, :3] for T in samples], np.ones(len(samples)))
    roll, pitch, yaw = tc.mat_to_rpy(R)
    x, y, z = P.mean(axis=0)
    sd = P.std(axis=0).max()
    print(f'\n{len(samples)} views (position scatter ±{sd * 1000:.0f} mm)')
    print(f'camera at  x {x:+.3f}  y {y:+.3f}  z {z:.3f} m   (from the axle centre on the floor)')
    print(f'points     pitch {math.degrees(pitch):+.1f}° ({"down" if pitch > 0 else "up"})   '
          f'yaw {math.degrees(yaw):+.1f}° ({"left" if yaw > 0 else "right"})   roll {math.degrees(roll):+.1f}°')
    old = {k: env.get(k) for k in ('CAMERA_X', 'CAMERA_Y', 'CAMERA_Z', 'CAMERA_ROLL', 'CAMERA_PITCH', 'CAMERA_YAW')}
    print('robot.env now: ' + '  '.join(f'{k}={v}' for k, v in old.items() if v is not None))
    vals = {'CAMERA_X': x, 'CAMERA_Y': y, 'CAMERA_Z': z, 'CAMERA_ROLL': roll, 'CAMERA_PITCH': pitch,
            'CAMERA_YAW': yaw}
    if abs(math.degrees(roll)) > 5:
        print('! roll over 5°: is the tag really level/upright, or the camera mounted crooked?')
    if a.write:
        write_env(vals)
        print(f'written to {ENV}. Restart the bring-up (run/stop.sh, then run/remap.sh) to apply it.')
    else:
        print('not written (add --write to store it in config/robot.env)')


if __name__ == '__main__':
    main()
