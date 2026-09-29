# RECONSTRUCTED (2026-09-30): shared helpers for the campus_bot nodes and tools. Not yet run on the robot.
"""Mask/config helpers shared by scan_filter.py, preflight.py and calibrate_mask.py."""
import math
import os

import yaml

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MASK_FILE = os.path.join(PROJECT_DIR, 'config', 'scan_mask.yaml')


def load_mask(path=MASK_FILE):
    """Return (sectors_deg, calibrated). sectors_deg is a list of (start, end) in degrees."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return [], False
    sectors = [(float(a), float(b)) for a, b in (data.get('sectors') or [])]
    return sectors, bool(data.get('calibrated', False))


def beam_angles_deg(angle_min, angle_increment, n):
    """Beam angles normalised to [-180, 180)."""
    out = []
    for i in range(n):
        a = math.degrees(angle_min + i * angle_increment)
        out.append((a + 180.0) % 360.0 - 180.0)
    return out


def in_sector(a, start, end):
    """True if angle a (deg) lies in [start, end]; a sector with start > end wraps through ±180."""
    if start <= end:
        return start <= a <= end
    return a >= start or a <= end


def mask_indices(angle_min, angle_increment, n, sectors):
    """Indices of the beams that fall inside any mask sector."""
    angles = beam_angles_deg(angle_min, angle_increment, n)
    return [i for i, a in enumerate(angles) if any(in_sector(a, s, e) for s, e in sectors)]


def load_env(path=os.path.join(PROJECT_DIR, 'config', 'robot.env')):
    """Parse config/robot.env (KEY=value lines, '#' comments) into a dict of strings."""
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if '=' in line:
                    k, v = line.split('=', 1)
                    env[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    return env
