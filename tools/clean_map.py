#!/usr/bin/env python3
# RECONSTRUCTED from RECOVERY.md (2026-09-30) — the original cleaning steps and grading metric were NOT
# recovered. This is a conservative stand-in; replace it if the original turns up.
"""Clean a saved map and report its stats.

    python3 tools/clean_map.py maps/<area>.yaml
writes maps/<area>_clean.pgm + .yaml and maps/<area>_report.txt.

Cleaning (only small, isolated things are touched; walls are never thinned or moved):
  * occupied specks of ≤ --speck cells with no unknown space around them → free
    (ghosts of legs, chair wheels, lidar noise)
  * enclosed unknown holes of ≤ --hole cells → free (gaps in free space the beams just missed)
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
    args = ap.parse_args()

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
    specks = 0
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] <= args.speck:
            blob = labels == i
            if not unk_near[blob].any():
                out[blob] = FREE
                specks += 1

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
        f'  holes filled       {holes}  (≤ {args.hole} cells, enclosed)',
        '  NOTE: statistics only. The original grading metric has not been recovered.',
        '  Check by eye: walls single and straight, glass room reads as a wall, loop closed with no doubling.',
    ]
    report = '\n'.join(lines)
    with open(os.path.join(base_dir, f'{stem}_report.txt'), 'w') as f:
        f.write(report + '\n')
    print(report)


if __name__ == '__main__':
    main()
