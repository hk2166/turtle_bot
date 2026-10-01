#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot.
"""1:1 face verification at handover: is the person at the crate the claimed recipient?

Best-of-N: grab up to N frames, score each face against the person's enrolled embeddings (max cosine),
accept as soon as one frame reaches the threshold. Never trust a single frame (one genuine frame
scored 0.296 in the last calibration). Total budget 2 s from the first frame; camera open is reported
separately.

    python face/verify.py hemant                       # Brio on /dev/video0
    python face/verify.py hemant --frames 5 --threshold 0.42 --det-size 320 --camera 0
    python face/verify.py hemant --images a.jpg b.jpg  # from files instead of the camera
Exit code: 0 accepted, 1 rejected, 2 error (not enrolled, no camera).

    from verify import verify
    r = verify('hemant', frames)   # frames: list of BGR arrays / image paths, or None for the camera
"""
import argparse
import json
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from engine import Engine, best_score  # noqa: E402
from face_store import FaceStore  # noqa: E402

# PLACEHOLDER pending face/calibrate.py: the sweep never ran (0 impostor frames). Don't trust it.
THRESHOLD = 0.42
N_FRAMES = 5
BUDGET_S = 2.0
CAM_W, CAM_H = 1280, 720
WARMUP_FRAMES = 5   # let the Brio's auto-exposure settle

_engines = {}


def get_engine(det_size=640):
    """One Engine per det_size per process (model load takes seconds)."""
    if det_size not in _engines:
        _engines[det_size] = Engine(det_size=det_size)
    return _engines[det_size]


def open_camera(index=0):
    """V4L2 + MJPG (raw YUYV at 720p is too slow over USB on the Brio 100)."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'cannot open /dev/video{index} (camera unplugged? run/camera.sh holding it?)')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(WARMUP_FRAMES):
        cap.read()
    return cap


def camera_frames(cap):
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError('camera read failed')
        yield frame


def _as_frame(f):
    if isinstance(f, (str, os.PathLike)):
        img = cv2.imread(str(f))
        if img is None:
            raise RuntimeError(f'cannot read image {f}')
        return img
    return f


def verify(person_id, frames=None, n=N_FRAMES, threshold=THRESHOLD, det_size=640, camera=0,
           engine=None, store=None, budget_s=BUDGET_S):
    """Returns a dict: accepted, best_score, scores, frames_used, faces_found, timings, reason, …"""
    store = store or FaceStore()
    rec = store.get(person_id)
    if rec is None:
        return {'person_id': person_id, 'accepted': False, 'reason': 'not_enrolled', 'best_score': None,
                'threshold': threshold, 'scores': [], 'frames_used': 0, 'faces_found': 0, 'timings': {}}
    enrolled = rec['embeddings']
    engine = engine or get_engine(det_size)

    cap = None
    t_open = 0.0
    if frames is None:
        t = time.perf_counter()
        cap = open_camera(camera)
        t_open = (time.perf_counter() - t) * 1000.0
        source = camera_frames(cap)
    else:
        source = iter(frames)

    scores, per_frame_ms, embed_ms = [], [], []
    best, used, found, reason = None, 0, 0, 'no_match'
    t0 = time.perf_counter()
    try:
        while used < n:
            f = next(source, None)
            if f is None:
                break
            if time.perf_counter() - t0 > budget_s:
                reason = 'budget'
                break
            tf = time.perf_counter()
            used += 1
            emb, _bbox, _det, ms = engine.embed(_as_frame(f))
            embed_ms.append(round(ms, 1))
            s = None
            if emb is not None:
                found += 1
                s = round(best_score(emb, enrolled), 4)
                best = s if best is None else max(best, s)
            scores.append(s)
            per_frame_ms.append(round((time.perf_counter() - tf) * 1000.0, 1))
            if s is not None and s >= threshold:
                reason = 'match'
                break
    finally:
        if cap is not None:
            cap.release()
    total_ms = (time.perf_counter() - t0) * 1000.0
    if found == 0 and reason != 'budget':
        reason = 'no_face'
    return {
        'person_id': person_id, 'accepted': reason == 'match', 'reason': reason,
        'best_score': best, 'threshold': threshold, 'scores': scores,
        'frames_used': used, 'faces_found': found, 'det_size': engine.det_size,
        'timings': {'camera_open_ms': round(t_open, 1), 'embed_ms': embed_ms,
                    'per_frame_ms': per_frame_ms, 'total_ms': round(total_ms, 1),
                    'over_budget': total_ms > budget_s * 1000.0},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('person_id')
    ap.add_argument('--frames', type=int, default=N_FRAMES, help='best-of-N (default %(default)s)')
    ap.add_argument('--threshold', type=float, default=THRESHOLD, help='default %(default)s (placeholder)')
    ap.add_argument('--det-size', type=int, default=640, choices=(320, 640))
    ap.add_argument('--camera', type=int, default=0, help='/dev/videoN')
    ap.add_argument('--images', nargs='+', help='image files instead of the camera')
    ap.add_argument('--json', action='store_true', help='print the raw result dict')
    a = ap.parse_args()

    try:
        r = verify(a.person_id, frames=a.images, n=a.frames, threshold=a.threshold,
                   det_size=a.det_size, camera=a.camera)
    except (RuntimeError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    if a.json:
        print(json.dumps(r, indent=2))
    elif r['reason'] == 'not_enrolled':
        print(f'{a.person_id}: not enrolled')
        return 2
    else:
        t = r['timings']
        sc = ' '.join('-' if s is None else f'{s:.3f}' for s in r['scores'])
        print(f"{'ACCEPT' if r['accepted'] else 'REJECT'} {a.person_id}  best={r['best_score']}  "
              f"threshold={r['threshold']}  reason={r['reason']}")
        print(f"  scores: [{sc}]  faces {r['faces_found']}/{r['frames_used']}  det_size {r['det_size']}")
        print(f"  embed ms: {t['embed_ms']}  total {t['total_ms']:.0f} ms"
              f"{'  ** OVER 2 s BUDGET **' if t['over_budget'] else ''}"
              f"{'  (camera open %.0f ms)' % t['camera_open_ms'] if t['camera_open_ms'] else ''}")
    if r['reason'] == 'not_enrolled':
        return 2
    return 0 if r['accepted'] else 1


if __name__ == '__main__':
    sys.exit(main())
