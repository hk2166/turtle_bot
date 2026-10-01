#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot.
"""Calibrate the verification threshold: collect scored frames, then sweep TAR/FAR.

collect: stand in front of the camera and score every frame against a CLAIMED person.
  genuine  = the claimed person themself;  impostor = someone else claiming to be them.
  desk = laptop/desk height;  mast = the robot's mast camera (~1.25 m, tilted up ~10°).
    python face/calibrate.py collect --person hemant --label genuine --session mast --frames 40
    python face/calibrate.py collect --person hemant --label impostor --session mast --frames 40   # Lakshay stands in
  Rows are appended to ~/.campus_bot/calib/scores.csv (scores only, never images).

analyze: TAR/FAR sweep 0.20–0.70, recommended threshold, timing, desk-vs-mast, best-of-N.
    python face/calibrate.py analyze [--csv FILE] [--relabel lakshaysharma=impostor] [--exclude NAME]

Open questions carried over (RECOVERY.md): the sweep needs impostor rows (last run: 120 genuine,
0 impostor); lakshaysharma's "genuine" frames scored ~0.18 (mislabel or bad enrolment? ask Hemant);
p95 embed was ~900 ms at det 640, so re-collect with --det-size 320.
"""
import argparse
import csv
import os
import sys
import time
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from face_store import FaceStore, data_dir  # noqa: E402

FIELDS = ['timestamp', 'claimed_person', 'label', 'session', 'score', 'det_score', 'embed_ms', 'det_size']
SWEEP = np.round(np.arange(0.20, 0.70 + 1e-9, 0.01), 2)
TARGET_FAR = 0.001          # 0.1 %
PLACEHOLDER = 0.42
FRAME_BUDGET_MS = 400       # 5 frames inside the 2 s verify budget
ANOMALY_GAP = 0.25          # a person's genuine median this far below the others' is suspicious


def default_csv():
    return os.path.join(data_dir(), 'calib', 'scores.csv')


# --------------------------------------------------------------------------------------------- collect
def collect(a):
    from engine import Engine, best_score
    from verify import open_camera

    rec = FaceStore().get(a.person)
    if rec is None:
        print(f'{a.person} is not enrolled; enrol first (face/enroll_service.py)', file=sys.stderr)
        return 2
    os.makedirs(os.path.dirname(a.csv), mode=0o700, exist_ok=True)
    eng = Engine(det_size=a.det_size)
    try:
        cap = open_camera(a.camera)
    except RuntimeError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    new = not os.path.exists(a.csv) or os.path.getsize(a.csv) == 0
    print(f'collecting {a.frames} {a.label} frames claiming "{a.person}" ({a.session}, det {a.det_size}). '
          'Move a little between frames: small head turns, step closer/further. Ctrl-C to stop.')
    got = miss = 0
    try:
        with open(a.csv, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(FIELDS)
            while got < a.frames:
                ok, frame = cap.read()
                if not ok:
                    print('camera read failed', file=sys.stderr)
                    return 2
                emb, _bbox, det, ms = eng.embed(frame)
                del frame
                if emb is None:
                    miss += 1
                    continue
                s = best_score(emb, rec['embeddings'])
                w.writerow([time.strftime('%Y-%m-%dT%H:%M:%S'), a.person, a.label, a.session,
                            f'{s:.4f}', f'{det:.3f}', f'{ms:.1f}', a.det_size])
                f.flush()
                got += 1
                print(f'  {got:3d}/{a.frames}  score {s:.3f}  det {det:.2f}  {ms:5.0f} ms')
                time.sleep(a.interval)
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()
    print(f'{got} rows appended to {a.csv} ({miss} frames had no usable face)')
    return 0


# --------------------------------------------------------------------------------------------- analyze
def load_rows(path, relabel=(), exclude=()):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                r['score'] = float(r['score'])
                r['embed_ms'] = float(r['embed_ms'] or 'nan')
                r['det_size'] = int(r['det_size'] or 0)
            except (KeyError, ValueError):
                continue
            r['label'] = r['label'].strip().lower()
            if r['claimed_person'] in exclude:
                continue
            if r['label'] == 'genuine' and r['claimed_person'] in relabel:
                r['label'] = 'impostor'
            if r['label'] in ('genuine', 'impostor'):
                rows.append(r)
    return rows


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float('nan')


def recommend(gen, imp):
    """Lowest threshold with FAR <= TARGET_FAR (= zero accepts below 1000 impostor rows) -> best TAR."""
    for t in SWEEP:
        far = float((imp >= t).mean())
        if far <= TARGET_FAR:
            return float(t), float((gen >= t).mean()), far
    return None


def best_of_n(rows, label, n, t):
    """Group each (claimed person, session) run into consecutive chunks of n; chunk score = max."""
    runs = defaultdict(list)
    for r in sorted((r for r in rows if r['label'] == label), key=lambda r: r['timestamp']):
        runs[(r['claimed_person'], r['session'])].append(r['score'])
    chunks = [max(s[i:i + n]) for s in runs.values() for i in range(0, len(s) - n + 1, n)]
    if not chunks:
        return None, 0
    return float(np.mean(np.array(chunks) >= t)), len(chunks)


def analyze(a):
    if not os.path.exists(a.csv):
        print(f'no CSV at {a.csv}; run `collect` first', file=sys.stderr)
        return 2
    relabel = {x.split('=')[0] for x in a.relabel if x.endswith('=impostor')}
    rows = load_rows(a.csv, relabel, set(a.exclude))
    gen = np.array([r['score'] for r in rows if r['label'] == 'genuine'])
    imp = np.array([r['score'] for r in rows if r['label'] == 'impostor'])
    print(f'{a.csv}: {len(rows)} rows  genuine {len(gen)}  impostor {len(imp)}'
          f"{'  relabelled→impostor: ' + ','.join(sorted(relabel)) if relabel else ''}"
          f"{'  excluded: ' + ','.join(a.exclude) if a.exclude else ''}")
    if not rows:
        print('nothing to analyze: no genuine/impostor rows')
        return 1

    # Per-person genuine medians: catch mislabels / broken enrolments (the lakshaysharma ~0.18 case).
    print('\n== genuine score per claimed person ==')
    per = defaultdict(list)
    for r in rows:
        if r['label'] == 'genuine':
            per[r['claimed_person']].append(r['score'])
    meds = {p: float(np.median(s)) for p, s in per.items()}
    suspects = []
    for p in sorted(meds):
        others = [m for q, m in meds.items() if q != p]
        ref = float(np.median(others)) if others else None
        flag = ref is not None and meds[p] < ref - ANOMALY_GAP
        if flag:
            suspects.append(p)
        print(f'  {p:20s} n={len(per[p]):4d}  median {meds[p]:.3f}  min {min(per[p]):.3f}'
              f"{f'   ** SUSPECT: others ~{ref:.2f}' if flag else ''}")
    for p in suspects:
        print(f'  !! {p}: genuine frames score far below everyone else. Either the label is wrong (someone else\n'
              f'     stood in: re-run with --relabel {p}=impostor and they become impostor data) or {p}\'s\n'
              f'     enrolment is bad (re-enrol). Ask Hemant which. Until then this skews the sweep;\n'
              f'     compare with --exclude {p}.')

    # TAR/FAR sweep.
    print('\n== TAR/FAR sweep ==')
    if len(gen) == 0:
        print('  no genuine rows: cannot compute TAR. Collect --label genuine frames.')
    if len(imp) == 0:
        print('  NO IMPOSTOR ROWS: FAR cannot be measured, so no threshold can be recommended.\n'
              '  Collect some: have a colleague stand at the mast claiming an enrolled person, e.g.\n'
              '    python face/calibrate.py collect --person hemant --label impostor --session mast --frames 40\n'
              f'  Keep the {PLACEHOLDER} placeholder until then. TAR alone is shown below.')
    print(f"  {'thr':>5} {'TAR':>7} {'FAR':>7} {'imp_acc':>7}")
    for t in SWEEP:
        if round(t * 100) % 5 and abs(t - PLACEHOLDER) > 1e-6:
            continue   # print every 0.05 plus the placeholder
        tar = f'{(gen >= t).mean():7.1%}' if len(gen) else '      -'
        far = f'{(imp >= t).mean():7.2%}' if len(imp) else '      -'
        acc = f'{int((imp >= t).sum()):7d}' if len(imp) else '      -'
        print(f'  {t:5.2f} {tar} {far} {acc}{"   <- current placeholder" if abs(t - PLACEHOLDER) < 1e-6 else ""}')

    rec = None
    if len(gen) and len(imp):
        rec = recommend(gen, imp)
        print(f'\n  genuine p5 {pct(gen, 5):.3f} min {gen.min():.3f} | impostor max {imp.max():.3f} p95 {pct(imp, 95):.3f}')
        if rec:
            print(f'  RECOMMENDED threshold {rec[0]:.2f}: TAR {rec[1]:.1%} (single frame), FAR {rec[2]:.2%} '
                  f'({int((imp >= rec[0]).sum())} of {len(imp)} impostor frames accepted)')
            ok = [t for t in SWEEP if t >= rec[0] and (gen >= t).mean() >= rec[1] - 0.02]
            print(f'  with margin: {max(ok):.2f} (highest threshold that still keeps TAR within 2 points; '
                  'prefer this while impostor data is thin)')
        else:
            print(f'  no threshold in {SWEEP[0]:.2f}–{SWEEP[-1]:.2f} reaches FAR <= 0.1%; '
                  'check impostor labels / enrolment quality')
        if len(imp) < 1.0 / TARGET_FAR:
            print(f'  note: {len(imp)} impostor frames can only resolve FAR to ~{1 / len(imp):.1%}; '
                  f'{int(1 / TARGET_FAR)}+ are needed to really show <= {TARGET_FAR:.1%}. Leave a margin above it.')

    # Timing.
    print('\n== embed timing (ms per frame) ==')
    by_ds = defaultdict(list)
    for r in rows:
        if np.isfinite(r['embed_ms']):
            by_ds[r['det_size']].append(r['embed_ms'])
    for ds in sorted(by_ds):
        ms = np.array(by_ds[ds])
        over = int((ms > FRAME_BUDGET_MS).sum())
        print(f'  det {ds:4d}: n={len(ms):4d}  median {np.median(ms):6.0f}  p95 {pct(ms, 95):6.0f}  max {ms.max():6.0f}'
              f'  >{FRAME_BUDGET_MS} ms: {over}{"  ** worst frame alone blows the 2 s budget" if ms.max() > 2000 else ""}')
    if 320 not in by_ds:
        print('  no det 320 rows yet: collect some with --det-size 320 to compare speed and scores')

    # Angle effect.
    print('\n== angle effect (median genuine score per session) ==')
    sess = defaultdict(list)
    for r in rows:
        if r['label'] == 'genuine' and r['claimed_person'] not in suspects:
            sess[r['session']].append(r['score'])
    for s in sorted(sess):
        print(f'  {s:6s} n={len(sess[s]):4d}  median {np.median(sess[s]):.3f}')
    if 'desk' in sess and 'mast' in sess:
        d = float(np.median(sess['mast']) - np.median(sess['desk']))
        print(f'  mast - desk = {d:+.3f}  ({"no meaningful angle penalty" if d > -0.05 else "mast angle HURTS: check camera tilt/height"})')
    if suspects:
        print(f'  (suspect people excluded here: {", ".join(suspects)})')

    # Best-of-N.
    t = rec[0] if rec else a.threshold
    print(f'\n== best-of-N simulation at threshold {t:.2f}{"" if rec else " (not a recommendation)"} ==')
    for n in (1, 3, 5):
        g, gn = best_of_n(rows, 'genuine', n, t)
        i, im = best_of_n(rows, 'impostor', n, t)
        gs = f'{g:6.1%} of {gn}' if g is not None else '     -'
        is_ = f'{i:6.1%} of {im}' if i is not None else '     - (no impostor rows)'
        print(f'  N={n}: genuine accept {gs:>14}   impostor accept {is_}')
    print('  best-of-N raises TAR but also FAR (an impostor gets N tries); judge FAR at the N you deploy.')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('collect', help='score camera frames against a claimed person')
    c.add_argument('--person', required=True, help='CLAIMED person_id (enrolled)')
    c.add_argument('--label', required=True, choices=('genuine', 'impostor'))
    c.add_argument('--session', required=True, choices=('desk', 'mast'))
    c.add_argument('--frames', type=int, default=40)
    c.add_argument('--det-size', type=int, default=640, choices=(320, 640))
    c.add_argument('--camera', type=int, default=0)
    c.add_argument('--interval', type=float, default=0.3, help='seconds between kept frames')
    c.add_argument('--csv', default=default_csv())
    z = sub.add_parser('analyze', help='TAR/FAR sweep, timing, angle effect, best-of-N')
    z.add_argument('--csv', default=default_csv())
    z.add_argument('--threshold', type=float, default=PLACEHOLDER, help='for best-of-N when no recommendation')
    z.add_argument('--relabel', action='append', default=[], metavar='PERSON=impostor',
                   help="treat PERSON's genuine rows as impostor attempts")
    z.add_argument('--exclude', action='append', default=[], metavar='PERSON')
    a = ap.parse_args()
    return collect(a) if a.cmd == 'collect' else analyze(a)


if __name__ == '__main__':
    sys.exit(main())
