#!/usr/bin/env python3
# Written 2026-10-07: stitch two saved maps into one.
"""Merge map B into map A's coordinate frame and write a new map.

    python3 tools/merge_maps.py <A> <B> <out>        e.g.  merge_maps.py office newOfficeMap office_full

Each SLAM run has its own frame, so the tool first FINDS the rigid transform between the maps:
a global search (rotation x FFT correlation of the wall cells), refined against A's distance field.
It prints the alignment score — read it before trusting the result:
    >= 0.60  good lock, merge is sound
    < 0.45   the maps disagree or barely overlap: the merge is refused.
The merged map lives in A's frame, so A's fences, goals and poses stay valid. Writes
maps/<out>.pgm/yaml and maps/<out>_clean.* (ready for run/nav.sh <out>); nothing else is touched.
Occupied beats free beats unknown where the maps overlap.
"""
import os
import sys

import cv2
import numpy as np
import yaml

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(area):
    meta = yaml.safe_load(open(os.path.join(PROJECT, 'maps', f'{area}_clean.yaml')))
    img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image']), -1)
    if img is None:
        sys.exit(f'maps/{meta["image"]} missing')
    # grid with row 0 at the BOTTOM (world y up): occupied 2, free 1, unknown 0
    tri = np.zeros(img.shape, np.uint8)
    tri[img == 254] = 1
    tri[img == 0] = 2
    return tri[::-1].copy(), float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def occ_points(tri, res, ox, oy, cap=4000):
    ys, xs = np.where(tri == 2)
    if len(xs) > cap:
        k = np.random.default_rng(0).choice(len(xs), cap, replace=False)
        ys, xs = ys[k], xs[k]
    return np.column_stack([(xs + 0.5) * res + ox, (ys + 0.5) * res + oy])


def coarse_align(triA, resA, oxA, oyA, triB, resB, oxB, oyB):
    """FFT correlation of blurred wall masks over rotations → (theta, tx, ty) world transform guess."""
    assert abs(resA - resB) < 1e-6, 'different resolutions not supported'
    res = resA
    N = 1 << int(np.ceil(np.log2(max(triA.shape + triB.shape) + 32)))
    A = np.zeros((N, N), np.float32)
    A[:triA.shape[0], :triA.shape[1]] = (triA == 2)
    A = cv2.GaussianBlur(A, (0, 0), 1.5)
    FA = np.fft.rfft2(A)
    maskB = (triB == 2).astype(np.float32)
    hB, wB = maskB.shape
    best = (-1, 0, 0, 0)
    for deg in range(0, 360, 2):
        R = cv2.getRotationMatrix2D((wB / 2, hB / 2), deg, 1.0)
        cos, sin = abs(R[0, 0]), abs(R[0, 1])
        w1, h1 = int(hB * sin + wB * cos) + 2, int(hB * cos + wB * sin) + 2
        R[0, 2] += w1 / 2 - wB / 2
        R[1, 2] += h1 / 2 - hB / 2
        rot = cv2.warpAffine(maskB, R, (w1, h1))
        if h1 > N or w1 > N:
            continue
        B = np.zeros((N, N), np.float32)
        B[:h1, :w1] = rot
        corr = np.fft.irfft2(FA * np.conj(np.fft.rfft2(B)), s=(N, N))
        peak = np.unravel_index(np.argmax(corr), corr.shape)
        score = corr[peak]
        if score > best[0]:
            # shift of rotated-B's pixel grid relative to A's pixel grid
            dy = peak[0] if peak[0] < N // 2 else peak[0] - N
            dx = peak[1] if peak[1] < N // 2 else peak[1] - N
            best = (score, deg, dx, dy, R, (w1, h1))
    score, deg, dx, dy, R, _ = best
    # world transform implied by (rotation R, pixel shift dx/dy): map B-world through pixel space
    def b_world_to_a_world(p):
        bx = (p[0] - oxB) / res - 0.5
        by = (p[1] - oyB) / res - 0.5
        rx = R[0, 0] * bx + R[0, 1] * by + R[0, 2]
        ry = R[1, 0] * bx + R[1, 1] * by + R[1, 2]
        return np.array([oxA + (rx + dx + 0.5) * res, oyA + (ry + dy + 0.5) * res])
    p0 = b_world_to_a_world(np.array([0.0, 0.0]))
    p1 = b_world_to_a_world(np.array([1.0, 0.0]))
    d = p1 - p0
    theta = float(np.arctan2(d[1], d[0]))
    return theta, p0, score


def apply_T(pts, theta, t0):
    c, s = np.cos(theta), np.sin(theta)
    return pts @ np.array([[c, s], [-s, c]]) + t0


def refine(ptsB, dtA, resA, oxA, oyA, theta, t0):
    """Hill-climb (theta, tx, ty) on A's distance field. Returns params + score."""
    def score(th, t):
        p = apply_T(ptsB, th, t)
        cx = ((p[:, 0] - oxA) / resA).astype(int)
        cy = ((p[:, 1] - oyA) / resA).astype(int)
        ok = (cx >= 0) & (cx < dtA.shape[1]) & (cy >= 0) & (cy < dtA.shape[0])
        if ok.sum() < 50:
            return 0.0
        d = dtA[cy[ok], cx[ok]]
        return float((d < 0.10).sum()) / len(ptsB)
    cur = score(theta, t0)
    for step, astep in ((0.10, 0.03), (0.04, 0.012), (0.015, 0.004), (0.005, 0.0015)):
        improved = True
        while improved:
            improved = False
            for dth in (-astep, 0, astep):
                for dx in (-step, 0, step):
                    for dy in (-step, 0, step):
                        if dth == dx == dy == 0:
                            continue
                        sc = score(theta + dth, t0 + [dx, dy])
                        if sc > cur:
                            cur, theta, t0 = sc, theta + dth, t0 + np.array([dx, dy])
                            improved = True
    return theta, t0, cur


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    a, b, out = sys.argv[1:4]
    if not __import__('re').fullmatch(r'[A-Za-z0-9_-]+', out):
        sys.exit('bad output name')
    triA, resA, oxA, oyA = load(a)
    triB, resB, oxB, oyB = load(b)
    print(f'aligning {b} ({triB.shape[1]}x{triB.shape[0]}) onto {a} ({triA.shape[1]}x{triA.shape[0]}) ...')
    theta, t0, _ = coarse_align(triA, resA, oxA, oyA, triB, resB, oxB, oyB)
    dtA = cv2.distanceTransform((triA != 2).astype(np.uint8), cv2.DIST_L2, 3) * resA
    ptsB = occ_points(triB, resB, oxB, oyB)
    theta, t0, sc = refine(ptsB, dtA, resA, oxA, oyA, theta, t0)
    print(f'alignment: rotate {np.degrees(theta):+.1f}°, shift ({t0[0]:+.2f}, {t0[1]:+.2f}) m, '
          f'score {sc:.2f} ({sc:.0%} of {b}\'s walls land on {a}\'s walls)')
    if sc < 0.45:
        sys.exit('REFUSED: alignment too weak — the maps barely overlap or disagree. Nothing written.')

    # merged canvas in A's frame
    hB, wB = triB.shape
    cornersB = np.array([[oxB, oyB], [oxB + wB * resB, oyB], [oxB, oyB + hB * resB],
                         [oxB + wB * resB, oyB + hB * resB]])
    cornersA = apply_T(cornersB, theta, t0)
    x0 = min(oxA, cornersA[:, 0].min()) - 0.25
    y0 = min(oyA, cornersA[:, 1].min()) - 0.25
    x1 = max(oxA + triA.shape[1] * resA, cornersA[:, 0].max()) + 0.25
    y1 = max(oyA + triA.shape[0] * resA, cornersA[:, 1].max()) + 0.25
    W, H = int(np.ceil((x1 - x0) / resA)), int(np.ceil((y1 - y0) / resA))
    merged = np.zeros((H, W), np.uint8)
    ax0 = int(round((oxA - x0) / resA))
    ay0 = int(round((oyA - y0) / resA))
    merged[ay0:ay0 + triA.shape[0], ax0:ax0 + triA.shape[1]] = triA
    # warp B into the merged canvas via 3 point correspondences (B pixels -> merged pixels)
    src = np.float32([[0, 0], [wB, 0], [0, hB]])
    world = np.stack([oxB + src[:, 0] * resB, oyB + src[:, 1] * resB], axis=1)
    worldA = apply_T(world, theta, t0)
    dst = np.float32(np.stack([(worldA[:, 0] - x0) / resA, (worldA[:, 1] - y0) / resA], axis=1))
    M = cv2.getAffineTransform(src, dst)
    warpB = cv2.warpAffine(triB, M, (W, H), flags=cv2.INTER_NEAREST, borderValue=0)
    merged = np.maximum(merged, warpB)
    # back to pgm convention (row 0 = top)
    img = np.full(merged.shape, 205, np.uint8)
    img[merged == 1] = 254
    img[merged == 2] = 0
    img = img[::-1]
    for suffix in ('', '_clean'):
        cv2.imwrite(os.path.join(PROJECT, 'maps', f'{out}{suffix}.pgm'), img)
        with open(os.path.join(PROJECT, 'maps', f'{out}{suffix}.yaml'), 'w') as f:
            f.write(f'# merged by tools/merge_maps.py: {a} + {b} (score {sc:.2f}) in {a}\'s frame\n')
            yaml.safe_dump({'image': f'{out}{suffix}.pgm', 'mode': 'trinary', 'resolution': resA,
                            'origin': [round(float(x0), 4), round(float(y0), 4), 0], 'negate': 0,
                            'occupied_thresh': 0.65, 'free_thresh': 0.196}, f, sort_keys=False)
    print(f'maps/{out}.pgm + maps/{out}_clean.* written ({W}x{H} px = {W*resA:.1f} x {H*resA:.1f} m), '
          f'frame = {a}\'s (fences/poses for {a} stay valid)')


if __name__ == '__main__':
    main()
