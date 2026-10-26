# Reconstructed 2026-10-01 from RECOVERY.md/CLAUDE.md; one folder per person since 2026-10-26. Runs on
# the robot (enrolments from the phone page and the robot camera, delivery identify).
"""Face-embedding store, one folder per enrolled person. Embeddings only, never images.

    ~/.campus_bot/                 0700
        faces.key                  0600  Fernet key (losing it = everyone re-enrols)
        faces/                     0700  `ls ~/.campus_bot/faces` = who is registered
            hemant/                0700
                info.json          0600  person_id, n, enrolled_at, consent, source (no biometrics)
                face.enc           0600  Fernet token of {"embeddings": [512-d, ...], "mean": 512-d}
        faces.enc.migrated         the old single-file store, kept after the one-time migration

Per person: L2-normalised 512-d ArcFace embeddings and their (re-normalised) mean. A person's folder is
replaced in one rename, so a crash mid-enrolment never leaves half a person. Never commit any of this
(it lives outside the repo, in the home directory).

    from face_store import FaceStore
    s = FaceStore(); s.enroll('hemant', embs, consent=True, source='robot camera'); s.get('hemant')
    python face/face_store.py            # who is enrolled (and from where)
    python face/face_store.py --delete X # delete one person
"""
import base64
import json
import os
import re
import secrets
import shutil
import sys
import tempfile
import threading
import time

import numpy as np
from cryptography.fernet import Fernet, InvalidToken

EMB_DIM = 512
PERSON_RE = re.compile(r'^[a-z0-9][a-z0-9._-]{0,63}$')   # lowercase, e.g. 'lakshaysharma'
_LOCK = threading.Lock()   # one writer at a time inside a process


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
        self.faces_dir = os.path.join(self.dir, 'faces')
        self.key_path = os.path.join(self.dir, 'faces.key')
        self.legacy_path = os.path.join(self.dir, 'faces.enc')    # the pre-2026-10-26 single file
        self._fernet = None

    # -- key / layout ---------------------------------------------------------------------------
    def _ensure_dirs(self):
        for d in (self.dir, self.faces_dir):
            os.makedirs(d, mode=0o700, exist_ok=True)
            os.chmod(d, 0o700)

    def _has_data(self):
        return os.path.exists(self.legacy_path) or bool(self._folders())

    def _key(self, create):
        if self._fernet:
            return self._fernet
        if os.path.exists(self.key_path):
            with open(self.key_path, 'rb') as f:
                key = f.read().strip()
        elif self._has_data():
            # Never mint a new key over existing data: that would silently orphan everyone's enrolment.
            raise RuntimeError(f'enrolments exist but {self.key_path} is missing — restore the key '
                               'or move ~/.campus_bot/faces aside (everyone then re-enrols)')
        elif not create:
            return None
        else:
            self._ensure_dirs()
            key = Fernet.generate_key()
            _write_private(self.key_path, key + b'\n')
        self._fernet = Fernet(key)
        return self._fernet

    def _folders(self):
        try:
            names = os.listdir(self.faces_dir)
        except OSError:
            return []
        return sorted(n for n in names if PERSON_RE.match(n)
                      and os.path.isfile(os.path.join(self.faces_dir, n, 'info.json'))
                      and os.path.isfile(os.path.join(self.faces_dir, n, 'face.enc')))

    def _migrate(self):
        """One-time: split the old single faces.enc into per-person folders, keep it as .migrated."""
        if not os.path.exists(self.legacy_path):
            return
        with _LOCK:
            if not os.path.exists(self.legacy_path):
                return
            f = self._key(create=False)
            if f is None:
                raise RuntimeError(f'{self.legacy_path} exists but {self.key_path} is missing')
            with open(self.legacy_path, 'rb') as fh:
                try:
                    old = json.loads(f.decrypt(fh.read()))
                except InvalidToken:
                    raise RuntimeError(f'{self.legacy_path} does not decrypt with {self.key_path} (wrong key?)')
            have = set(self._folders())
            for pid, r in old.get('people', {}).items():
                if pid in have or not PERSON_RE.match(pid):
                    continue
                self._write_person(pid, r['embeddings'], r['mean'],
                                   {'enrolled_at': r.get('enrolled_at'), 'consent': bool(r.get('consent')),
                                    'n': int(r.get('n', len(r['embeddings']))),
                                    'source': 'migrated from faces.enc'})
            os.replace(self.legacy_path, self.legacy_path + '.migrated')

    def _write_person(self, pid, embs_b64, mean_b64, info):
        """Build the folder under a temp name, then swap it in with one rename."""
        self._ensure_dirs()
        f = self._key(create=True)
        tmp = os.path.join(self.faces_dir, f'.tmp-{pid}-{secrets.token_hex(4)}')
        os.makedirs(tmp, mode=0o700)
        try:
            _write_private(os.path.join(tmp, 'face.enc'),
                           f.encrypt(json.dumps({'embeddings': embs_b64, 'mean': mean_b64}).encode()))
            _write_private(os.path.join(tmp, 'info.json'),
                           (json.dumps({'person_id': pid, **info}, indent=2) + '\n').encode())
            dest = os.path.join(self.faces_dir, pid)
            old = None
            if os.path.exists(dest):
                old = os.path.join(self.faces_dir, f'.old-{pid}-{secrets.token_hex(4)}')
                os.replace(dest, old)
            os.replace(tmp, dest)
            if old:
                shutil.rmtree(old, ignore_errors=True)
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise

    def _info(self, pid):
        with open(os.path.join(self.faces_dir, pid, 'info.json')) as fh:
            return json.load(fh)

    # -- people ---------------------------------------------------------------------------------
    def enroll(self, person_id, embeddings, consent=True, source=None):
        """Replace person_id's enrolment. Returns the number of embeddings stored."""
        if not consent:
            raise ValueError('enrolment needs explicit opt-in consent')
        pid = check_person_id(person_id)
        embs = np.stack([normalize(e) for e in embeddings])
        if len(embs) == 0:
            raise ValueError('no embeddings')
        mean = normalize(embs.mean(axis=0))
        self._migrate()
        with _LOCK:
            self._write_person(pid, [_b64(e) for e in embs], _b64(mean),
                               {'n': int(len(embs)), 'enrolled_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                                'consent': True, 'source': source or 'unknown'})
        return len(embs)

    def delete(self, person_id):
        """True if the person existed."""
        pid = check_person_id(person_id)
        self._migrate()
        with _LOCK:
            dest = os.path.join(self.faces_dir, pid)
            if not os.path.isdir(dest):
                return False
            gone = os.path.join(self.faces_dir, f'.del-{pid}-{secrets.token_hex(4)}')
            os.replace(dest, gone)
            shutil.rmtree(gone, ignore_errors=True)
        return True

    def people(self):
        """[info dict], sorted by id: person_id, n, enrolled_at, consent, source."""
        self._migrate()
        out = []
        for pid in self._folders():
            try:
                out.append(self._info(pid))
            except (OSError, ValueError):
                continue
        return out

    def list_people(self):
        """[(person_id, n_embeddings, enrolled_at)], sorted."""
        return [(i['person_id'], int(i.get('n', 0)), i.get('enrolled_at')) for i in self.people()]

    def load(self):
        """{'version': 2, 'people': {pid: info}} — who is enrolled; embeddings come from get()."""
        return {'version': 2, 'people': {i['person_id']: i for i in self.people()}}

    def get(self, person_id):
        """{'embeddings': (n,512) float32, 'mean': (512,), 'enrolled_at', 'consent', 'source'} or None."""
        pid = check_person_id(person_id)
        self._migrate()
        folder = os.path.join(self.faces_dir, pid)
        if not os.path.isfile(os.path.join(folder, 'face.enc')):
            return None
        info = self._info(pid)
        f = self._key(create=False)
        if f is None:
            raise RuntimeError(f'{self.key_path} missing')
        with open(os.path.join(folder, 'face.enc'), 'rb') as fh:
            try:
                r = json.loads(f.decrypt(fh.read()))
            except InvalidToken:
                raise RuntimeError(f'{folder}/face.enc does not decrypt with {self.key_path} (wrong key?)')
        return {'embeddings': np.stack([_unb64(e) for e in r['embeddings']]), 'mean': _unb64(r['mean']),
                'enrolled_at': info.get('enrolled_at'), 'consent': bool(info.get('consent')),
                'source': info.get('source')}


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--delete', metavar='PERSON_ID')
    a = ap.parse_args()
    s = FaceStore()
    if a.delete:
        print('deleted' if s.delete(a.delete) else 'not enrolled', a.delete)
        return 0
    people = s.people()
    if not people:
        print(f'nobody enrolled ({s.faces_dir})')
    for i in people:
        print(f"{i['person_id']:24s} {int(i.get('n', 0)):3d} embeddings  enrolled {i.get('enrolled_at')}  "
              f"via {i.get('source', '?')}")
    if people:
        print(f'({len(people)} registered; one folder each in {s.faces_dir})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
