# Written 2026-10-22: the guided ("Face ID style") enrolment behind https://<robot>:8443/enroll. New file.
"""Guided face enrolment from a person's OWN phone or laptop camera, like iOS Face ID setup: they roll
their head in a slow circle while a ring of SECTORS fills in. Each sector is a head direction; it
fills once a good face embedding was captured looking that way. A full ring gives templates from many
angles, so the robot later recognises them whether they face it square-on or glance at it.

The browser only captures and displays: it sends ~4 small JPEG frames a second to the robot, which
embeds each one in memory (never stored), updates the ring, and answers with the next hint. Only the
final embeddings are kept, encrypted (face_store.py).

Head pose comes from the detector's 5 landmarks (eyes, nose, mouth corners): after levelling the eye
line, the nose's sideways offset (in eye-distances) is the turn, and how far the nose sits between the
eyes and the mouth is the tilt. Both are relative to the person's own straight-ahead pose (the first
steady frames), so any face shape works. Directions are in the MIRRORED view the person sees (the
selfie view): "right" is their right.

Modes: 'enroll' (the ring) and 'delete' (a few straight-ahead frames must match the enrolled face
before it is removed — so only you can delete you).
"""
import math
import secrets
import threading
import time

import numpy as np

from engine import best_score
from face_store import check_person_id

N_SECTORS = 12
YAW_SCALE = 0.30          # nose offset (eye-distances) that counts as a full turn (~30 deg)
PITCH_SCALE = 0.15        # change in nose position between eyes and mouth that counts as a full tilt
CENTER_R = 0.35           # |aim| below this = looking straight
SECTOR_R = 0.55           # |aim| above this = looking clearly in a direction (fills a sector)
BASELINE_FRAMES = 3       # steady straight-ahead frames that set the person's neutral pose
PER_SECTOR = 2            # embeddings kept per sector
MIN_DET = 0.5             # turned faces score lower than frontal ones; 0.5 still embeds well
MIN_FACE_FRAC, MAX_FACE_FRAC = 0.10, 0.75   # face width / frame width: too far / too close
MIN_AGREE = 0.25          # a sector frame must look like the straight-ahead frames (same person)
TIMEOUT_S = 60.0
MIN_SECTORS_AT_TIMEOUT = 8
RESULT_HOLD_S = 120.0     # a finished session lingers this long (the page reads the result), then goes
MAX_SESSIONS = 4          # people enrolling at the same time
DIRS = ['right', 'up-right', 'up', 'up-left', 'left', 'down-left', 'down', 'down-right']


def head_pose(kps):
    """(yaw, pitch_ratio, roll_deg) from 5 landmarks [l-eye, r-eye, nose, l-mouth, r-mouth] (image coords).
    yaw: nose offset from the eye midpoint in eye-distances (image +x); pitch_ratio: nose depth between
    eye line (0) and mouth line (1). Roll is removed first so a tilted head doesn't read as a turn."""
    k = np.asarray(kps, dtype=np.float64)
    le, re = k[0], k[1]
    ang = math.atan2(re[1] - le[1], re[0] - le[0])
    c, s = math.cos(-ang), math.sin(-ang)
    R = np.array([[c, -s], [s, c]])
    p = (k - (le + re) / 2.0) @ R.T            # eye line horizontal, eye midpoint at the origin
    iod = max(float(np.linalg.norm(p[1] - p[0])), 1e-6)
    yaw = float(p[2][0]) / iod
    mouth_y = float((p[3][1] + p[4][1]) / 2.0)
    pitch = float(p[2][1]) / mouth_y if mouth_y > 1e-6 else 0.5
    return yaw, pitch, math.degrees(ang)


def sector_of(dx, dy):
    """Sector index for an aim (dx right, dy up) in the mirrored view; 0 = right, counter-clockwise."""
    a = math.atan2(dy, dx) % (2 * math.pi)
    return int(a / (2 * math.pi / N_SECTORS)) % N_SECTORS


def dir_name(sector):
    a = (sector + 0.5) * 360.0 / N_SECTORS
    return DIRS[int(round(a / 45.0)) % 8]


class Session:
    """One person's capture. feed() takes engine.analyze() results; status() is what the page shows."""

    def __init__(self, person_id, store, threshold, mode='enroll'):
        self.pid = check_person_id(person_id)
        self.store, self.threshold, self.mode = store, threshold, mode
        self.token = secrets.token_urlsafe(16)
        self.t0 = self.seen = time.time()
        self.state = 'capturing'              # capturing -> done | failed | cancelled
        self.base, self.center = [], []       # [(yaw, pitch)], [emb]
        self.yaw0 = self.p0 = None
        self.sectors = [[] for _ in range(N_SECTORS)]   # [(det, emb)]
        self.filled = [False] * N_SECTORS
        self.hint, self.aim, self.msg, self.ended = 'Look straight at the camera', None, '', None

    # -------------------------------------------------------------------------- per frame
    def feed(self, a, frame_w):
        self.seen = time.time()
        if self.state != 'capturing':
            return
        if time.time() - self.t0 > TIMEOUT_S:
            return self._timeout()
        if a['n_faces'] == 0:
            self.hint = 'Look at the camera'
            return
        if a['n_faces'] > 1:
            self.hint = 'Only one person in view, please'
            return
        frac = (a['bbox'][2] - a['bbox'][0]) / max(frame_w, 1)
        if frac < MIN_FACE_FRAC:
            self.hint = 'Come a bit closer'
            return
        if frac > MAX_FACE_FRAC:
            self.hint = 'Move back a little'
            return
        if a['emb'] is None or a['det'] < MIN_DET or a['kps'] is None:
            self.hint = 'Hold still a moment — better light on your face helps'
            return
        yaw, pitch, _roll = head_pose(a['kps'])

        if self.yaw0 is None:                 # 1) neutral pose: a few steady, roughly-frontal frames
            if abs(yaw) > 0.15:
                self.hint = 'Look straight at the camera'
                return
            self.base.append((yaw, pitch))
            self.center.append(a['emb'])
            self.hint = 'Hold still…'
            if len(self.base) >= BASELINE_FRAMES:
                self.yaw0 = float(np.median([b[0] for b in self.base]))
                self.p0 = float(np.median([b[1] for b in self.base]))
                if self.mode == 'delete':
                    return self._delete()
                self.hint = 'Now slowly move your head in a circle'
            return

        # 2) the ring. Mirrored view: turning to THEIR right moves the nose toward image-left.
        dx = -(yaw - self.yaw0) / YAW_SCALE
        dy = -(pitch - self.p0) / PITCH_SCALE   # chin up brings the nose toward the eye line
        self.aim = [round(max(-1.5, min(1.5, dx)), 3), round(max(-1.5, min(1.5, dy)), 3)]
        ref = np.mean(np.stack(self.center), axis=0)
        if best_score(a['emb'], ref[None, :]) < MIN_AGREE:
            self.hint = 'Is someone else in view? Only you, please'
            return
        m = math.hypot(dx, dy)
        if m >= SECTOR_R:
            s = self.sectors[sector_of(dx, dy)]
            if len(s) < PER_SECTOR:
                s.append((a['det'], a['emb']))
        missing = [i for i in range(N_SECTORS) if not self.sectors[i]]
        if not missing:
            return self._finish()
        here = sector_of(dx, dy) if m >= CENTER_R else missing[0]
        target = min(missing, key=lambda i: min((i - here) % N_SECTORS, (here - i) % N_SECTORS))
        self.hint = f'Keep circling slowly — now {dir_name(target)}'

    # -------------------------------------------------------------------------- endings
    def _timeout(self):
        filled = sum(1 for s in self.sectors if s)
        if self.mode == 'enroll' and self.yaw0 is not None and filled >= MIN_SECTORS_AT_TIMEOUT:
            return self._finish()
        if self.yaw0 is None:
            return self._end('failed', 'Could not get a clear straight-ahead view. Face the camera, '
                                       'with light in front of you (not behind), and try again.')
        self._end('failed', 'Not enough angles captured. Try again and move slowly in a full circle.')

    def _matches_existing(self):
        existing = self.store.get(self.pid)
        if existing is None:
            return None
        return max(best_score(e, existing['embeddings']) for e in self.center)

    def _finish(self):
        s = self._matches_existing()
        if s is not None and s < self.threshold:   # re-enrolling: only the same person may replace it
            return self._end('failed', f'The ID "{self.pid}" is already enrolled to a different face. '
                                       'Pick another ID, or ask the robot admin to remove the old one.')
        embs = list(self.center) + [e for sec in self.sectors for _d, e in sec]
        n = self.store.enroll(self.pid, embs, consent=True)
        angles = sum(1 for sec in self.sectors if sec)
        self._end('done', f'{"Updated" if s is not None else "Enrolled"} {self.pid}: {n} templates from '
                          f'{angles}/{N_SECTORS} angles. No photos were kept.')

    def _delete(self):
        s = self._matches_existing()
        if s is None:
            return self._end('failed', f'"{self.pid}" is not enrolled — nothing to delete.')
        if s < self.threshold:
            return self._end('failed', f'Your face does not match "{self.pid}", so it was not deleted.')
        self.store.delete(self.pid)
        self._end('done', f'Deleted the face template for {self.pid}. The robot no longer recognises you.')

    def cancel(self):
        if self.state == 'capturing':
            self._end('cancelled', 'Cancelled. Nothing was stored.')

    def _end(self, state, msg):
        self.state, self.msg, self.hint, self.ended = state, msg, msg, time.time()
        # keep only which sectors filled (for the ring); drop every embedding from memory
        self.filled = [bool(x) for x in self.sectors] if state == 'done' else [False] * N_SECTORS
        self.center, self.sectors = [], [[] for _ in range(N_SECTORS)]

    def stale(self):
        now = time.time()
        if self.ended is not None:
            return now - self.ended > RESULT_HOLD_S
        return now - self.seen > 15.0          # the phone stopped sending frames (tab closed)

    def status(self):
        return {'state': self.state, 'mode': self.mode, 'person_id': self.pid, 'hint': self.hint,
                'msg': self.msg, 'center': self.yaw0 is not None,
                'sectors': self.filled if self.ended is not None else [bool(s) for s in self.sectors],
                'aim': self.aim, 'elapsed': round(time.time() - self.t0, 1), 'timeout': TIMEOUT_S}


class Sessions:
    """All enrolment sessions, one per browser (by token). frame() does the inference."""

    def __init__(self, store, engine_fn, infer_lock, threshold):
        self.store, self.engine_fn, self.infer, self.threshold = store, engine_fn, infer_lock, threshold
        self.lock = threading.Lock()
        self.by_token = {}

    def _gc(self):
        for t in [t for t, s in self.by_token.items() if s.stale()]:
            self.by_token[t].cancel()
            del self.by_token[t]

    def start(self, person_id, consent, mode='enroll'):
        if mode not in ('enroll', 'delete'):
            return {'ok': False, 'msg': 'bad mode'}
        if mode == 'enroll' and not consent:
            return {'ok': False, 'msg': 'Tick the consent box first. Nothing is captured without it.'}
        try:
            check_person_id(person_id)
        except ValueError:
            return {'ok': False, 'msg': 'ID: lowercase letters, numbers, . _ - only (e.g. "priya" or "priya.k")'}
        if self.engine_fn() is None:
            return {'ok': False, 'msg': 'The robot is still loading its face model — try again in a few seconds.'}
        with self.lock:
            self._gc()
            if sum(s.state == 'capturing' for s in self.by_token.values()) >= MAX_SESSIONS:
                return {'ok': False, 'msg': 'Several people are enrolling right now — try again in a minute.'}
            s = Session(person_id, self.store, self.threshold, mode=mode)
            self.by_token[s.token] = s
        return {'ok': True, 'token': s.token, **s.status()}

    def get(self, token):
        with self.lock:
            return self.by_token.get(token or '')

    def frame(self, token, img):
        """Feed one decoded BGR frame from the person's browser; returns the updated status."""
        s = self.get(token)
        if s is None:
            return {'state': 'expired', 'hint': 'This session ended — tap Start again.'}
        if s.state == 'capturing' and img is not None:
            eng = self.engine_fn()
            with self.infer:
                a = eng.analyze(img, embed=True)
            s.feed(a, img.shape[1])
        return s.status()

    def status(self, token):
        s = self.get(token)
        return s.status() if s else {'state': 'expired', 'hint': 'This session ended — tap Start again.'}

    def cancel(self, token):
        s = self.get(token)
        if s:
            s.cancel()
        return s is not None
