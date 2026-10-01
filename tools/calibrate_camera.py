#!/usr/bin/env python3
# Written 2026-10-01: camera intrinsics for tag localization.
"""Calibrate the mast camera with the printed checkerboard (python3 tools/make_tags.py --checkerboard).

    run/camera.sh off            # this tool needs the camera to itself
    python3 tools/calibrate_camera.py [--square-mm 25] [--cols 9 --rows 6]

Hold the board (taped flat on cardboard) in front of the camera at 0.5–2 m and move it around: all
corners of the picture, near and far, tilted left/right/up/down. A view is taken automatically when the
whole board is seen sharp and somewhere new; 25 views are enough. With a desktop (monitor, VNC or
remote desktop) a preview window shows the views so far; over plain SSH it prints progress.
Writes config/camera_intrinsics.yaml, which nodes/camera_node.py loads (restart it: run/camera.sh on).
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
import tags_common as tc  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column')
    ap.add_argument('--square-mm', type=float, default=25.0, help='MEASURED square size')
    ap.add_argument('--views', type=int, default=25)
    ap.add_argument('--out', default=tc.INTRINSICS_FILE)
    a = ap.parse_args()

    busy = tc.camera_busy(a.device)
    if busy:
        sys.exit(f'{a.device} is in use (PID {busy}). Stop the camera node first: run/camera.sh off')
    cap = tc.open_camera(a.device, a.width, a.height)
    if cap is None:
        sys.exit(f'cannot open {a.device}')
    show = bool(os.environ.get('DISPLAY'))
    pattern = (a.cols, a.rows)
    obj = np.zeros((a.cols * a.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square_mm / 1000.0
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    img_pts, signatures, last_take = [], [], 0.0
    coverage = np.zeros((a.height // 40 + 1, a.width // 40 + 1), np.uint8)
    print(f'Show the {a.cols}x{a.rows} board to the camera and move it around. q / Ctrl-C to stop early.')
    try:
        while len(img_pts) < a.views:
            ok, img = cap.read()
            if not ok:
                sys.exit('camera read failed')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, None, fx=0.5, fy=0.5)
            found, corners = cv2.findChessboardCorners(small, pattern, cv2.CALIB_CB_FAST_CHECK
                                                       + cv2.CALIB_CB_ADAPTIVE_THRESH)
            status = 'no board'
            if found:
                corners = cv2.cornerSubPix(gray, corners * 2.0, (7, 7), (-1, -1), crit)
                c = corners.reshape(-1, 2)
                # where it is, how big, how tilted: a view must differ from all taken ones
                width = np.linalg.norm(c[a.cols - 1] - c[0])
                height = np.linalg.norm(c[-1] - c[a.cols - 1])
                sig = np.array([c[:, 0].mean() / a.width, c[:, 1].mean() / a.height,
                                np.sqrt(cv2.contourArea(c[[0, a.cols - 1, -1, -a.cols]].astype(np.float32)))
                                / a.width, width / max(height, 1e-6) / 3.0])
                sharp = cv2.Laplacian(gray[int(c[:, 1].min()):int(c[:, 1].max()) + 1,
                                           int(c[:, 0].min()):int(c[:, 0].max()) + 1], cv2.CV_64F).var()
                new = all(np.linalg.norm(sig - s) > 0.08 for s in signatures)
                if sharp < 40:
                    status = 'board blurred: hold still'
                elif not new:
                    status = 'seen this view: move/tilt the board'
                elif time.monotonic() - last_take > 0.7:
                    img_pts.append(corners)
                    signatures.append(sig)
                    last_take = time.monotonic()
                    for x, y in c:
                        coverage[int(y) // 40, int(x) // 40] = 1
                    status = f'TAKEN view {len(img_pts)}/{a.views}'
                    print(f'  view {len(img_pts):2d}/{a.views}  (picture coverage {100 * coverage.mean():.0f}%)')
            if show:
                vis = img.copy()
                if found:
                    cv2.drawChessboardCorners(vis, pattern, corners, True)
                for p in img_pts:
                    for x, y in p.reshape(-1, 2)[::6]:
                        cv2.circle(vis, (int(x), int(y)), 2, (255, 0, 255), -1)
                cv2.putText(vis, f'{len(img_pts)}/{a.views}  {status}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 255, 255), 2)
                cv2.imshow('Campus Bot camera calibration (q = stop)', cv2.resize(vis, None, fx=0.6, fy=0.6))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()

    if len(img_pts) < 10:
        sys.exit(f'only {len(img_pts)} views: need at least 10. Nothing written.')
    print(f'calibrating from {len(img_pts)} views ...')
    rms, K, D, _, _ = cv2.calibrateCamera([obj] * len(img_pts), img_pts, (a.width, a.height), None, None)
    hfov = np.degrees(2 * np.arctan(a.width / 2 / K[0, 0]))
    print(f'  reprojection error {rms:.2f} px   fx {K[0, 0]:.0f} fy {K[1, 1]:.0f}   '
          f'centre ({K[0, 2]:.0f}, {K[1, 2]:.0f})   horizontal FOV {hfov:.1f}°')
    if rms > 1.0:
        print('  ! error above 1 px: board not flat, square size wrong, or blurry views. Consider redoing it.')
    if coverage.mean() < 0.5:
        print(f'  ! the board covered only {100 * coverage.mean():.0f}% of the picture: edges are less accurate.')
    tc.save_intrinsics(a.out, a.width, a.height, K, D, rms, len(img_pts))
    print(f'wrote {a.out}. Restart the camera node to use it: run/camera.sh off && run/camera.sh on')


if __name__ == '__main__':
    main()
