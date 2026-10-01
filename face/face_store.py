# RECONSTRUCTED from RECOVERY.md/CLAUDE.md (2026-10-01) — not yet run on the robot.
"""Fernet-encrypted face-embedding store. Embeddings only, never images.

    ~/.campus_bot/            0700
        faces.enc             0600  Fernet token of the JSON below
        faces.key             0600  Fernet key (losing it = everyone re-enrols)

Per person: L2-normalised 512-d ArcFace embeddings, their (re-normalised) mean, enrol time and the
consent flag. Never commit either file (see .gitignore).

    from face_store import FaceStore
    s = FaceStore(); s.enroll('hemant', embs, consent=True); s.get('hemant')['embeddings']  # (n, 512)
    python face/face_store.py            # list enrolled people
    python face/face_store.py --delete X # delete one person
"""
import base64
import json
import os
import re
import sys
import tempfile
import threading
import time

import numpy as np
from cryptography.fernet import Fernet, InvalidToken

EMB_DIM = 512
PERSON_RE = re.compile(r'^[a-z0-9][a-z0-9._-]{0,63}$')   # lowercase, e.g. 'lakshaysharma'
_LOCK = threading.Lock()   # one writer at a time inside the enrol service


def data_dir():
    """~/.campus_bot, resolved at call time so tests can point HOME at a scratch dir."""
    return os.path.join(os.path.expanduser('~'), '.campus_bot')


def normalize(v):
    v = np.asarray(v, dtype=np.float32).reshape(-1)
    n = float(np.linalg.norm(v))
    if v.shape[0] != EMB_DIM or not np.isfinite(n) or n < 1e-6:
        raise ValueError(f'bad embedding (dim {v.shape[0]}, norm {n})')
    return v / n


def check_person_id(person_id):
    pid = (person_id or '').strip().lower()
    if not PERSON_RE.match(pid):
        raise ValueError('person_id must be 1-64 chars of a-z 0-9 . _ - (e.g. "lakshaysharma")')
    return pid


def _b64(arr):
    return base64.b64encode(np.asarray(arr, dtype='<f4').tobytes()).decode('ascii')


def _unb64(s):
    return np.frombuffer(base64.b64decode(s), dtype='<f4').copy()


def _write_private(path, data):
    """Atomic write: tmp file (0600) in the same dir, fsync, rename."""
    d = os.path.dirname(path)
    fd, tmp = tempfile.mkstemp(prefix='.tmp-', dir=d)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


class FaceStore:
    def __init__(self, base_dir=None):
        self.dir = base_dir or data_dir()
        self.enc_path = os.path.join(self.dir, 'faces.enc')
        self.key_path = os.path.join(self.dir, 'faces.key')
        self._fernet = None

    # -- key / file handling --------------------------------------------------------------------
    def _ensure_dir(self):
        os.makedirs(self.dir, mode=0o700, exist_ok=True)
        os.chmod(self.dir, 0o700)

    def _key(self, create):
        if self._fernet:
            return self._fernet
        if os.path.exists(self.key_path):
            with open(self.key_path, 'rb') as f:
                key = f.read().strip()
        elif os.path.exists(self.enc_path):
            # Never mint a new key over existing data: that would silently orphan everyone's enrolment.
            raise RuntimeError(f'{self.enc_path} exists but {self.key_path} is missing — restore the key '
                               'or move faces.enc aside (everyone then re-enrols)')
        elif not create:
            return None
        else:
            self._ensure_dir()
            key = Fernet.generate_key()
            _write_private(self.key_path, key + b'\n')
        self._fernet = Fernet(key)
        return self._fernet

    def load(self):
        """Raw dict {'version': 1, 'people': {pid: record}}. Empty if nothing is enrolled yet."""
        if not os.path.exists(self.enc_path):
            return {'version': 1, 'people': {}}
        f = self._key(create=False)
        if f is None:
            raise RuntimeError(f'{self.key_path} missing')
        with open(self.enc_path, 'rb') as fh:
            token = fh.read()
        try:
            return json.loads(f.decrypt(token))
        except InvalidToken:
            raise RuntimeError(f'{self.enc_path} does not decrypt with {self.key_path} (wrong key?)')

    def save(self, data):
        f = self._key(create=True)
        self._ensure_dir()
        _write_private(self.enc_path, f.encrypt(json.dumps(data).encode('utf-8')))

    # -- people ---------------------------------------------------------------------------------
    def enroll(self, person_id, embeddings, consent=True):
        """Replace person_id's enrolment. Returns the number of embeddings stored."""
        if not consent:
            raise ValueError('enrolment needs explicit opt-in consent')
        pid = check_person_id(person_id)
        embs = np.stack([normalize(e) for e in embeddings])
        if len(embs) == 0:
            raise ValueError('no embeddings')
        mean = normalize(embs.mean(axis=0))
        with _LOCK:
            data = self.load()
            data['people'][pid] = {
                'embeddings': [_b64(e) for e in embs],
                'mean': _b64(mean),
                'enrolled_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                'consent': True,
                'n': int(len(embs)),
            }
            self.save(data)
        return len(embs)

    def delete(self, person_id):
        """True if the person existed."""
        pid = check_person_id(person_id)
        with _LOCK:
            data = self.load()
            if pid not in data['people']:
                return False
            del data['people'][pid]
            self.save(data)
        return True

    def list_people(self):
        """[(person_id, n_embeddings, enrolled_at)], sorted."""
        people = self.load()['people']
        return sorted((pid, r['n'], r['enrolled_at']) for pid, r in people.items())

    def get(self, person_id):
        """{'embeddings': (n,512) float32, 'mean': (512,), 'enrolled_at', 'consent'} or None."""
        r = self.load()['people'].get(check_person_id(person_id))
        if r is None:
            return None
        return {'embeddings': np.stack([_unb64(e) for e in r['embeddings']]),
                'mean': _unb64(r['mean']), 'enrolled_at': r['enrolled_at'], 'consent': r['consent']}


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--delete', metavar='PERSON_ID')
    a = ap.parse_args()
    s = FaceStore()
    if a.delete:
        print('deleted' if s.delete(a.delete) else 'not enrolled', a.delete)
        return 0
    people = s.list_people()
    if not people:
        print(f'nobody enrolled ({s.enc_path})')
    for pid, n, at in people:
        print(f'{pid:24s} {n:3d} embeddings  enrolled {at}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
