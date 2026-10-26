#!/usr/bin/env python3
# Written 2026-10-22: enrol someone standing at the robot, using the robot's own face camera (no phone,
# no ngrok). New file — same quality filters as enroll_service.py.
"""Enrol a person from the robot's face camera. Only embeddings are stored (encrypted, face_store.py);
frames are embedded in memory and dropped — no image is ever written.

    ~/.venvs/face/bin/python face/enroll_local.py --check                 # is a face in view? stores nothing
    ~/.venvs/face/bin/python face/enroll_local.py hemant --consent        # enrol (re-enrolling replaces)
    ... --device /dev/video2 --seconds 15

Stand ~0.5-1 m from the camera, face it, then slowly turn a little left, right, up and down while it
captures (default 15 s). Needs >= 5 usable faces that agree with each other, like the phone page.
Stop the face service first (run/face.sh off) if it is holding the camera.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from engine import Engine  # noqa: E402
from face_store import FaceStore, check_person_id  # noqa: E402
from verify import cam_rotation, upright  # noqa: E402

MIN_GOOD = 5
MIN_DET_SCORE = 0.6
MIN_CONSISTENCY = 0.35
MAX_KEEP = 30


def open_cam(device, w=1280, h=720):
    dev = int(device) if str(device).isdigit() else device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {device} (unplugged? face service holding it? run/face.sh off)')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()
    return cap


ROTATE = cam_rotation()            # the face camera's mount holds it upside-down (config/face_cam.yaml)


def fresh(cap):
    cap.grab()                      # drop the buffered frame so we embed what is in front of it NOW
    ok, f = cap.read()
    return upright(f, ROTATE) if ok else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('person_id', nargs='?')
    ap.add_argument('--consent', action='store_true', help='the person opts in (required to store)')
    ap.add_argument('--check', action='store_true', help='just report what the camera sees; store nothing')
    ap.add_argument('--device', default=os.environ.get('FACE_CAM', '/dev/face_cam'))
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--det-size', type=int, default=640, choices=(320, 640))
    a = ap.parse_args()
    if not a.check:
        if not a.person_id:
            ap.error('person_id required (or --check)')
        pid = check_person_id(a.person_id)
        if not a.consent:
            raise SystemExit('refusing to enrol without --consent (the person must opt in)')
    if not str(a.device).isdigit() and not os.path.exists(a.device):
        raise SystemExit(f'{a.device} not found (try --device /dev/video2, or sudo bash setup/10_face_cam.sh)')

    print(f'loading model (det_size {a.det_size}) ...', flush=True)
    eng = Engine(det_size=a.det_size)
    cap = open_cam(a.device)
    try:
        if a.check:
            for i in range(5):
                f = fresh(cap)
                emb, bbox, det, ms = eng.embed(f)
                size = '' if bbox is None else f'  face {int(bbox[2]-bbox[0])}x{int(bbox[3]-bbox[1])} px'
                print(f'frame {i+1}: {"FACE" if emb is not None else "no usable face"}  det {det:.2f}{size}  {ms:.0f} ms',
                      flush=True)
            return 0

        print(f'capturing {pid} for {a.seconds:.0f} s: face the camera, then slowly turn a little '
              'left, right, up, down', flush=True)
        embs, seen, rej = [], 0, {'no_face': 0, 'low_quality': 0}
        t0 = time.time()
        while time.time() - t0 < a.seconds and len(embs) < MAX_KEEP:
            f = fresh(cap)
            if f is None:
                continue
            seen += 1
            emb, _bbox, det, _ms = eng.embed(f)
            del f                                   # frames never outlive this loop
            if emb is None:
                rej['no_face'] += 1
            elif det < MIN_DET_SCORE:
                rej['low_quality'] += 1
            else:
                embs.append(emb)
        rej['inconsistent'] = 0
        if len(embs) >= 3:                          # drop frames that disagree (someone else in shot)
            e = np.stack(embs)
            agree = ((e @ e.T).sum(axis=1) - 1.0) / (len(e) - 1)
            keep = agree >= MIN_CONSISTENCY
            rej['inconsistent'] = int((~keep).sum())
            embs = list(e[keep])
        print(f'frames {seen}, usable {len(embs)}, rejected {rej}', flush=True)
        if len(embs) < MIN_GOOD:
            print(f'NOT enrolled: need {MIN_GOOD} usable faces. Better light on the face, face the camera, '
                  'stand closer.', flush=True)
            return 1
        n = FaceStore().enroll(pid, embs, consent=True)
        print(f'ENROLLED {pid}: {n} embeddings stored (encrypted). No images kept.', flush=True)
        return 0
    finally:
        cap.release()


if __name__ == '__main__':
    sys.exit(main())
