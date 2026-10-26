# Written 2026-10-22: the dashboard/queue side of face recognition — a thin, dependency-free client
# for face/face_service.py. New file.
"""Talk to the warm face service (face/face_service.py) from the ROS/dashboard Python, which has no
insightface/onnxruntime. Pure stdlib, short timeouts, and it NEVER raises: if the service is down or
the camera is missing, every call returns a dict with ok=False so the delivery flow falls back to OTP
or a manual GOT IT instead of blocking.

    import face_client
    face_client.health()                       # {'ok': True, 'model_loaded': True, 'enrolled': 3, ...}
    face_client.verify('hemant')               # {'ok': True, 'accepted': True, 'best_score': 0.55, ...}
    face_client.identify()                      # {'ok': True, 'identified': True, 'person_id': 'hemant', ...}
    face_client.people()                        # ['hemant', 'lakshaysharma', ...]

FACE_URL overrides the base URL (default http://127.0.0.1:8070).
"""
import json
import os
import urllib.error
import urllib.request

BASE = os.environ.get('FACE_URL', 'http://127.0.0.1:8070')
# A verify/identify runs the camera + model: allow it the 2 s budget plus camera-open and slack.
VERIFY_TIMEOUT = float(os.environ.get('FACE_TIMEOUT', '6'))
QUICK_TIMEOUT = 2.0


def _get(path, timeout=QUICK_TIMEOUT):
    try:
        with urllib.request.urlopen(f'{BASE}{path}', timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except Exception:
            return {'ok': False, 'error': f'http {e.code}'}
    except Exception as e:
        return {'ok': False, 'error': f'face service unreachable: {e}'}


def _post(path, payload, timeout=VERIFY_TIMEOUT):
    try:
        data = json.dumps(payload).encode()
        req = urllib.request.Request(f'{BASE}{path}', data=data,
                                     headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except Exception:
            return {'ok': False, 'error': f'http {e.code}'}
    except Exception as e:
        return {'ok': False, 'error': f'face service unreachable: {e}'}


def health(timeout=QUICK_TIMEOUT):
    return _get('/health', timeout)


def available(timeout=QUICK_TIMEOUT):
    """True only if the service is up, the model is loaded and the camera is present."""
    h = health(timeout)
    return bool(h.get('ok') and h.get('camera', {}).get('available'))


def people(timeout=QUICK_TIMEOUT):
    r = _get('/people', timeout)
    return r.get('people', []) if r.get('ok') else []


def frame(annotate=True, timeout=3.0):
    """(jpeg bytes, None) of the face camera with the live name overlay, or (None, why)."""
    try:
        with urllib.request.urlopen(f'{BASE}/frame.jpg{"" if annotate else "?ann=0"}', timeout=timeout) as r:
            return r.read(), None
    except urllib.error.HTTPError as e:
        try:
            return None, e.read().decode('utf-8', 'replace')[:120]
        except Exception:
            return None, f'http {e.code}'
    except Exception as e:
        return None, f'face service unreachable ({e.__class__.__name__})'


def verify(person_id, timeout=VERIFY_TIMEOUT, **kw):
    """1:1. Returns the service result; accepted is only ever True on a real match."""
    return _post('/verify', {'person_id': person_id, **kw}, timeout)


def identify(timeout=VERIFY_TIMEOUT, **kw):
    """1:N. Returns the service result; identified is only ever True on a confident match."""
    return _post('/identify', {**kw}, timeout)


if __name__ == '__main__':
    import sys
    if len(sys.argv) >= 3 and sys.argv[1] == 'verify':
        print(json.dumps(verify(sys.argv[2]), indent=2))
    elif len(sys.argv) >= 2 and sys.argv[1] == 'identify':
        print(json.dumps(identify(), indent=2))
    else:
        print(json.dumps(health(), indent=2))
