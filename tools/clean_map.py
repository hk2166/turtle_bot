#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — the original cleaning steps and grading metric were NOT
# recovered. This is a conservative stand-in; replace it if the original turns up.
"""Clean a saved map and report its stats.

    python3 tools/clean_map.py maps/<area>.yaml [--deep]
writes maps/<area>_clean.pgm + .yaml and maps/<area>_report.txt.

Cleaning (only small, isolated things are touched; walls are never thinned or moved):
  * occupied specks of ≤ --speck cells with no unknown space around them → free
    (ghosts of legs, chair wheels, lidar noise)
  * movables: free-standing objects (free floor all round) of ≤ --movable cells and ≤ 0.5 m across
    → free: chairs, bins, plant pots as they stood while mapping. Colleagues move them; baked into
    the map they block gaps that are open today. The lidar still sees them live wherever they are.
    Walls, desks and anything touching them are kept. --keep-movables turns this off.
  * enclosed unknown holes of ≤ --hole cells → free (gaps in free space the beams just missed)
--deep additionally:
  * bigger speck/hole limits (6 / 40 cells)
  * keeps only the free floor reachable from the main area through passages wider than ~0.25 m:
    the "free" spray fanning through glass and door slits becomes unknown. Real doorways
    (≥ 0.7 m) are far wider than the cut, so rooms are never lost.
The report is statistics, NOT the original grade (that metric is still unrecovered).
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml

FREE, OCC, UNK = 254, 0, 205


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('--speck', type=int, default=3, help='max occupied blob (cells) to remove')
    ap.add_argument('--hole', type=int, default=25, help='max enclosed unknown hole (cells) to fill')
    ap.add_argument('--movable', type=int, default=30, help='max free-standing object (cells) to drop')
    ap.add_argument('--keep-movables', action='store_true', help="don't drop free-standing objects")
    ap.add_argument('--deep', action='store_true', help='stronger pass: see the header')
    args = ap.parse_args()
    if args.deep:
        args.speck = max(args.speck, 6)
        args.hole = max(args.hole, 40)

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    base_dir = os.path.dirname(os.path.abspath(args.map_yaml))
    img_path = os.path.join(base_dir, meta['image'])
    img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"Can't read {img_path}")
    res = float(meta['resolution'])
    # map_saver writes trinary maps; be tolerant of other thresholds.
    m = np.full(img.shape, UNK, np.uint8)
    m[img >= 250] = FREE
    m[img <= 50] = OCC
    out = m.copy()

    occ = (m == OCC).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
    unk_near = cv2.dilate((m == UNK).astype(np.uint8), np.ones((5, 5), np.uint8))
    specks = movables = 0
    max_span = int(round(0.5 / res))
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        span = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
        speck = area <= args.speck
        movable = not args.keep_movables and area <= args.movable and span <= max_span
        if speck or movable:
            blob = labels == i
            if not unk_near[blob].any():
                out[blob] = FREE
                specks += speck
                movables += not speck

    unk = (out == UNK).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(unk, connectivity=4)
    holes = 0
    h, w = m.shape
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if area <= args.hole and x > 0 and y > 0 and x + bw < w and y + bh < h:
            blob = labels == i
            ring = cv2.dilate(blob.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & ~blob
            if (out[ring] == FREE).mean() > 0.6:
                out[blob] = FREE
                holes += 1

    sprayed = 0
    if args.deep:
        # main-floor extraction: erode free space by ~0.12 m (cuts necks < ~0.25 m), keep the largest
        # piece, grow it back inside the original free mask. Free cells never reached = spray → unknown.
        free = (out == FREE).astype(np.uint8)
        core = cv2.erode(free, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        ncc, lab, st, _ = cv2.connectedComponentsWithStats(core, connectivity=4)
        if ncc > 1:
            main = (lab == (1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])))).astype(np.uint8)
            k = np.ones((3, 3), np.uint8)
            prev = 0
            while True:
                main = cv2.dilate(main, k) & free
                tot = int(main.sum())
                if tot == prev:
                    break
                prev = tot
            spray = free.astype(bool) & ~main.astype(bool)
            sprayed = int(spray.sum())
            out[spray] = UNK

    stem = os.path.splitext(os.path.basename(args.map_yaml))[0]
    clean_img = f'{stem}_clean.pgm'
    cv2.imwrite(os.path.join(base_dir, clean_img), out)
    clean_meta = dict(meta, image=clean_img)
    with open(os.path.join(base_dir, f'{stem}_clean.yaml'), 'w') as f:
        yaml.safe_dump(clean_meta, f, default_flow_style=None, sort_keys=False)

    cell = res * res
    free_a = (out == FREE).sum() * cell
    occ_n = int((out == OCC).sum())
    ys, xs = np.where(out != UNK)
    extent = ((xs.max() - xs.min() + 1) * res, (ys.max() - ys.min() + 1) * res) if len(xs) else (0, 0)
    lines = [
        f'Map {stem}  ({w}×{h} cells at {res} m = {w * res:.1f} × {h * res:.1f} m)',
        f'  mapped extent      {extent[0]:.1f} × {extent[1]:.1f} m',
        f'  free floor         {free_a:.1f} m²',
        f'  occupied cells     {occ_n}',
        f'  specks removed     {specks}  (≤ {args.speck} cells, not near unknown)',
        (f'  movables removed   {movables}  (free-standing, ≤ {args.movable} cells and 0.5 m: chairs, bins)'
         if not args.keep_movables else '  movables kept      (--keep-movables)'),
        f'  holes filled       {holes}  (≤ {args.hole} cells, enclosed)',
        f'  spray removed      {sprayed * cell:.1f} m² of unreachable "free" (glass bleed)' if args.deep else
        '  (run with --deep to also strip unreachable glass-bleed spray)',
        '  NOTE: statistics only. The original grading metric has not been recovered.',
        '  Check by eye: walls single and straight, glass room reads as a wall, loop closed with no doubling.',
    ]
    report = '\n'.join(lines)
    with open(os.path.join(base_dir, f'{stem}_report.txt'), 'w') as f:
        f.write(report + '\n')
    print(report)


if __name__ == '__main__':
    main()
