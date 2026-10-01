#!/usr/bin/env python3
# Written 2026-10-01: printable AprilTags and the camera calibration checkerboard.
"""Make A4 PDFs to print for camera localization.

    python3 tools/make_tags.py 0-7                 # tags 0..7, one per A4 page, 160 mm
    python3 tools/make_tags.py 8 9 --size-mm 120
    python3 tools/make_tags.py --checkerboard      # for tools/calibrate_camera.py (9x6 corners, 25 mm)

Print at 100% / "actual size" (NOT "fit to page"), then MEASURE the black square: if it isn't the size
printed under it, pass the measured size (tag_size in config/robot.env, --square-mm for calibration).
Tag family tag36h11; every tag in one office must have a different id.
"""
import argparse
import os
import sys

import cv2
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
import tags_common as tc  # noqa: E402

DPI = 300
A4 = (2480, 3508)          # pixels at 300 dpi, portrait
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def mm(v):
    return int(round(v / 25.4 * DPI))


def font(size_mm):
    try:
        return ImageFont.truetype(FONT, mm(size_mm))
    except OSError:
        return ImageFont.load_default()


def tag_page(tag_id, size_mm):
    page = Image.new('L', A4, 255)
    d = ImageDraw.Draw(page)
    dictionary = cv2.aruco.getPredefinedDictionary(tc.DICTIONARY)
    cells = 8                                       # tag36h11: 6x6 data + 1-cell black border each side
    px = mm(size_mm)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, cells * 50, borderBits=1)
    marker = Image.fromarray(marker).resize((px, px), Image.NEAREST)
    x0, y0 = (A4[0] - px) // 2, mm(45)
    page.paste(marker, (x0, y0))
    d.text((A4[0] // 2, mm(20)), f'TOP  ↑   Campus Bot tag {tag_id}', fill=0, font=font(9), anchor='mm')
    y = y0 + px + mm(14)
    d.text((A4[0] // 2, y), f'{tc.FAMILY}  id {tag_id}   black square = {size_mm:g} mm', fill=0,
           font=font(6), anchor='mm')
    # A 100 mm bar: measure it to catch a printer that scaled the page.
    y += mm(14)
    d.rectangle([A4[0] // 2 - mm(50), y, A4[0] // 2 + mm(50), y + mm(3)], fill=0)
    d.text((A4[0] // 2, y + mm(10)), 'this bar must measure 100 mm (print at 100%)', fill=0, font=font(4.5),
           anchor='mm')
    d.text((A4[0] // 2, y + mm(22)),
           'Mount flat on a wall at 1.2–1.8 m, upright (TOP up), not on glass doors or anything that moves.',
           fill=0, font=font(4), anchor='mm')
    return page


def checkerboard_page(cols, rows, square_mm):
    """cols x rows INNER corners → (cols+1) x (rows+1) squares, landscape A4."""
    W, H = A4[1], A4[0]
    page = Image.new('L', (W, H), 255)
    d = ImageDraw.Draw(page)
    sq = mm(square_mm)
    bw, bh = (cols + 1) * sq, (rows + 1) * sq
    if bw > W - mm(20) or bh > H - mm(30):
        raise SystemExit(f'{cols + 1}x{rows + 1} squares of {square_mm} mm do not fit on A4')
    x0, y0 = (W - bw) // 2, (H - bh) // 2 + mm(5)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                d.rectangle([x0 + c * sq, y0 + r * sq, x0 + (c + 1) * sq - 1, y0 + (r + 1) * sq - 1], fill=0)
    d.text((W // 2, mm(10)), f'Campus Bot camera calibration: {cols}x{rows} inner corners, squares {square_mm:g} mm '
           f'(measure one!)  —  tape it flat on cardboard', fill=0, font=font(4.5), anchor='mm')
    return page


def parse_ids(items):
    ids = []
    for it in items:
        if '-' in it:
            a, b = it.split('-', 1)
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(it))
    for i in ids:
        if not 0 <= i < 587:
            raise SystemExit(f'tag36h11 ids are 0..586 (got {i})')
    return ids


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ids', nargs='*', default=['0-7'], help='tag ids, e.g. 0-7 or 3 5 9 (default 0-7)')
    ap.add_argument('--size-mm', type=float, default=160.0, help='black square edge (default 160)')
    ap.add_argument('--checkerboard', action='store_true', help='make the calibration checkerboard instead')
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square-mm', type=float, default=25.0)
    ap.add_argument('-o', '--out', help='PDF path (default ~/Desktop/campus_bot_<tags|checkerboard>.pdf)')
    a = ap.parse_args()
    if a.checkerboard:
        pages = [checkerboard_page(a.cols, a.rows, a.square_mm)]
        out = a.out or os.path.expanduser('~/Desktop/campus_bot_checkerboard.pdf')
    else:
        if not 40 <= a.size_mm <= 190:
            raise SystemExit('--size-mm: 40..190 fits an A4 page')
        ids = parse_ids(a.ids)
        pages = [tag_page(i, a.size_mm) for i in ids]
        out = a.out or os.path.expanduser('~/Desktop/campus_bot_tags.pdf')
    pages[0].save(out, 'PDF', resolution=DPI, save_all=True, append_images=pages[1:])
    print(f'{out}: {len(pages)} page(s). Print at 100% ("actual size"), then measure.')


if __name__ == '__main__':
    main()
