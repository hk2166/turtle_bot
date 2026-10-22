#!/usr/bin/env python3
# Written 2026-10-22: 1:N face identification (who is this?), the pickup/sender side of the robot.
# New file — not yet run on the robot with a real camera.
"""Identify who is in front of the camera (1:N), the counterpart to verify.py's 1:1 check.

verify.py answers "is this the claimed recipient?". identify.py answers "who is this?", which the
robot needs when a parcel is *loaded* (who is sending / which enrolled person is at the crate) and as
a stronger handover check (confirm the person is recipient X and not merely close to the threshold).

Best-of-N, same 2 s budget as verify.py: embed up to N frames, keep each person's running-best cosine,
stop early once the top person is both over threshold AND clear of the runner-up by `margin` (so two
look-alikes never silently swap). Returns a ranked candidate list either way.

    from identify import identify
    r = identify(frames)              # frames: list of BGR arrays / image paths, or None for the camera
    r['person_id'], r['score'], r['candidates']   # top match or None, its score, [(pid, score), ...]

    python face/identify.py                      # 5 frames from /dev/video0
    python face/identify.py --images a.jpg b.jpg # from files
    python face/identify.py --top 5 --margin 0.05
Exit code: 0 identified, 1 unknown (no confident match), 2 error (nobody enrolled, no camera).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from engine import Engine, cosine  # noqa: E402
from face_store import FaceStore  # noqa: E402
import verify as _verify  # noqa: E402  (reuse THRESHOLD, camera helpers, _as_frame)

# Same placeholder as verify.py until face/calibrate.py has impostor data. A match must also beat the
# runner-up by MARGIN, so identification is stricter than a bare 1:1 accept at the same threshold.
THRESHOLD = _verify.THRESHOLD
MARGIN = 0.04
N_FRAMES = _verify.N_FRAMES
BUDGET_S = _verify.BUDGET_S


def _roster(store):
    """(names, embs) where embs[i] is person names[i]'s (n_i, 512) enrolled embeddings."""
    data = store.load()['people']
    names, embs = [], []
    for pid in sorted(data):
        rec = store.get(pid)
        if rec is not None and len(rec['embeddings']):
            names.append(pid)
            embs.append(rec['embeddings'])
    return names, embs


def identify(frames=None, n=N_FRAMES, threshold=THRESHOLD, margin=MARGIN, det_size=640, camera=0,
             engine=None, store=None, budget_s=BUDGET_S, top=3):
    """Best-of-N 1:N identification. Returns a dict: identified, person_id, score, candidates, …"""
    store = store or FaceStore()
    names, rosters = _roster(store)
    if not names:
        return {'identified': False, 'person_id': None, 'score': None, 'reason': 'nobody_enrolled',
                'candidates': [], 'threshold': threshold, 'margin': margin,
                'frames_used': 0, 'faces_found': 0, 'timings': {}}
    engine = engine or _verify.get_engine(det_size)

    cap = None
    t_open = 0.0
    if frames is None:
        t = time.perf_counter()
        cap = _verify.open_camera(camera)
        t_open = (time.perf_counter() - t) * 1000.0
        source = _verify.camera_frames(cap)
    else:
        source = iter(frames)

    best = np.full(len(names), -1.0, dtype=np.float32)   # running-best cosine per enrolled person
    per_frame_ms, embed_ms = [], []
    used, found, reason = 0, 0, 'unknown'
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
            emb, _bbox, _det, ms = engine.embed(_verify._as_frame(f))
            embed_ms.append(round(ms, 1))
            if emb is not None:
                found += 1
                for i, roster in enumerate(rosters):
                    s = float(np.max(cosine(emb, roster)))
                    if s > best[i]:
                        best[i] = s
            per_frame_ms.append(round((time.perf_counter() - tf) * 1000.0, 1))
            # early stop: a clear, confident winner (over threshold and clear of the runner-up)
            order = np.argsort(best)[::-1]
            top1 = float(best[order[0]])
            top2 = float(best[order[1]]) if len(order) > 1 else -1.0
            if top1 >= threshold and (top1 - top2) >= margin:
                reason = 'match'
                break
    finally:
        if cap is not None:
            cap.release()

    order = np.argsort(best)[::-1]
    candidates = [(names[i], round(float(best[i]), 4)) for i in order[:top] if best[i] > -1.0]
    top1 = float(best[order[0]]) if len(order) else -1.0
    top2 = float(best[order[1]]) if len(order) > 1 else -1.0
    identified = top1 >= threshold and (top1 - top2) >= margin and found > 0
    if found == 0 and reason != 'budget':
        reason = 'no_face'
    elif not identified and reason not in ('budget',):
        reason = 'ambiguous' if (top1 >= threshold and (top1 - top2) < margin) else 'unknown'
    total_ms = (time.perf_counter() - t0) * 1000.0
    return {
        'identified': bool(identified),
        'person_id': names[order[0]] if identified else None,
        'score': round(top1, 4) if top1 > -1.0 else None,
        'runner_up': candidates[1] if len(candidates) > 1 else None,
        'candidates': candidates, 'reason': reason,
        'threshold': threshold, 'margin': margin, 'det_size': engine.det_size,
        'frames_used': used, 'faces_found': found,
        'timings': {'camera_open_ms': round(t_open, 1), 'embed_ms': embed_ms,
                    'per_frame_ms': per_frame_ms, 'total_ms': round(total_ms, 1),
                    'over_budget': total_ms > budget_s * 1000.0},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', type=int, default=N_FRAMES, help='best-of-N (default %(default)s)')
    ap.add_argument('--threshold', type=float, default=THRESHOLD, help='default %(default)s (placeholder)')
    ap.add_argument('--margin', type=float, default=MARGIN, help='top-1 must beat top-2 by this (default %(default)s)')
    ap.add_argument('--det-size', type=int, default=640, choices=(320, 640))
    ap.add_argument('--camera', type=int, default=0, help='/dev/videoN')
    ap.add_argument('--top', type=int, default=3, help='how many candidates to print')
    ap.add_argument('--images', nargs='+', help='image files instead of the camera')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    try:
        r = identify(frames=a.images, n=a.frames, threshold=a.threshold, margin=a.margin,
                     det_size=a.det_size, camera=a.camera, top=a.top)
    except (RuntimeError, ValueError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    if a.json:
        print(json.dumps(r, indent=2))
        return 0 if r['identified'] else (2 if r['reason'] == 'nobody_enrolled' else 1)
    if r['reason'] == 'nobody_enrolled':
        print('nobody enrolled')
        return 2
    cand = '  '.join(f'{pid}:{sc:.3f}' for pid, sc in r['candidates']) or '(none)'
    t = r['timings']
    print(f"{'IDENTIFIED ' + r['person_id'] if r['identified'] else 'UNKNOWN'}  "
          f"best={r['score']}  threshold={r['threshold']} margin={r['margin']}  reason={r['reason']}")
    print(f"  candidates: {cand}   faces {r['faces_found']}/{r['frames_used']}  det_size {r['det_size']}")
    print(f"  total {t['total_ms']:.0f} ms{'  ** OVER 2 s BUDGET **' if t['over_budget'] else ''}")
    return 0 if r['identified'] else 1


if __name__ == '__main__':
    sys.exit(main())
