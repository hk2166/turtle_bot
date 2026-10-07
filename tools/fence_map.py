#!/usr/bin/env python3
# Written 2026-10-07: virtual boundary ("fence") baked into the navigation map.
"""Keep the planner inside the office: draw fence lines into a copy of the map.

The lidar sees through glass, so the saved map has phantom free floor beyond the windows and the
planner will happily route through it. Fence segments (drawn on the dashboard, or added here) are
baked as solid walls into maps/<area>_fenced.pgm|yaml, which run/nav.sh then uses automatically.

    python3 tools/fence_map.py bake office          rebuild maps/office_fenced.* from the fence file
    python3 tools/fence_map.py show office          print the stored segments

Fence file: config/fence_<area>.yaml — a list of line segments in map coordinates (metres), the same
coordinates goto.py and the dashboard use. Delete a line there and re-bake to remove it.
"""
import os
import sys

import cv2
import yaml

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def fence_path(area):
    return os.path.join(PROJECT, 'config', f'fence_{area}.yaml')


def load_fence(area):
    try:
        with open(fence_path(area)) as f:
            data = yaml.safe_load(f) or {}
        return [[float(v) for v in seg] for seg in (data.get('lines') or [])]
    except FileNotFoundError:
        return []


def save_fence(area, lines):
    with open(fence_path(area), 'w') as f:
        f.write('# Virtual boundary for maps/%s: line segments [x1, y1, x2, y2] in map metres.\n'
                '# Drawn on the dashboard (FENCE mode) or by hand; apply with: tools/fence_map.py bake %s\n'
                % (area, area))
        yaml.safe_dump({'lines': [[round(float(v), 3) for v in seg] for seg in lines]}, f)


def bake(area):
    """Draw the fence into <area>_clean and write <area>_fenced.pgm/yaml. Returns the yaml path."""
    base_yaml = os.path.join(PROJECT, 'maps', f'{area}_clean.yaml')
    with open(base_yaml) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image'])
                     if not os.path.isabs(meta['image']) else meta['image'], -1)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h = img.shape[0]

    def px(x, y):
        return int(round((x - ox) / res)), int(round(h - 1 - (y - oy) / res))

    out = img.copy()
    for x1, y1, x2, y2 in load_fence(area):
        cv2.line(out, px(x1, y1), px(x2, y2), 0, thickness=2)   # 0 = occupied in the pgm
    fenced_pgm = os.path.join(PROJECT, 'maps', f'{area}_fenced.pgm')
    fenced_yaml = os.path.join(PROJECT, 'maps', f'{area}_fenced.yaml')
    cv2.imwrite(fenced_pgm, out)
    meta['image'] = f'{area}_fenced.pgm'
    with open(fenced_yaml, 'w') as f:
        f.write(f'# {area}_clean + the virtual boundary from config/fence_{area}.yaml (tools/fence_map.py)\n')
        yaml.safe_dump(meta, f, sort_keys=False)
    return fenced_yaml


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ('bake', 'show'):
        sys.exit(__doc__)
    area = sys.argv[2]
    if sys.argv[1] == 'show':
        for seg in load_fence(area):
            print(seg)
        return
    lines = load_fence(area)
    out = bake(area)
    print(f'{out}: {len(lines)} fence segment(s) baked in')


if __name__ == '__main__':
    main()
