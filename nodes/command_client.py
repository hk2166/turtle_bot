# Written 2026-10-29: the dashboard side of voice/typed commands — a dependency-free client for
# nodes/command_service.py (which holds the anthropic SDK in its own venv). New file.
"""Never raises: any failure comes back as {'ok': False, 'error': ...} for the page to show."""
import json
import os
import urllib.error
import urllib.request

BASE = os.environ.get('COMMAND_URL', 'http://127.0.0.1:8071')
TIMEOUT = float(os.environ.get('COMMAND_TIMEOUT', '45'))   # Claude at low effort: a few seconds; SDK retry 1


def _call(path, payload=None, timeout=TIMEOUT):
    try:
        data = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request(f'{BASE}{path}', data=data, method='GET' if data is None else 'POST',
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except Exception:
            return {'ok': False, 'error': f'command service error {e.code}'}
    except Exception:
        return {'ok': False, 'error': 'voice commands are offline right now (command service not running)'}


def health():
    return _call('/health', timeout=2.0)


def parse(text, places, requester=None):
    return _call('/parse', {'text': text, 'places': places, 'requester': requester or {}})


def transcribe(audio_b64, mime='', places=None, timeout=100):
    """Speech -> text on the robot (Whisper in the command service). audio_b64: the browser's recording."""
    return _call('/transcribe', {'audio_b64': audio_b64, 'mime': mime, 'places': places or []}, timeout=timeout)


def vision_multi(mode, views, context=None, timeout=60):
    """Several labelled views at once: views = [(label, bytes, 'image/jpeg' | 'image/png'), ...]."""
    import base64
    views = [v for v in views if v and v[1]]
    if not views:
        return {'ok': False, 'error': 'no camera frame'}
    return _call('/vision', {'mode': mode, 'context': context or {},
                             'images': [{'label': l, 'b64': base64.b64encode(b).decode('ascii'), 'type': t}
                                        for l, b, t in views]}, timeout=timeout)


def vision(mode, jpg, context=None, timeout=TIMEOUT):
    """mode: hazard | blocked | describe | arrival; jpg: bytes of one camera frame."""
    import base64
    if not jpg:
        return {'ok': False, 'error': 'no camera frame (is the mast camera running?)'}
    return _call('/vision', {'mode': mode, 'image_b64': base64.b64encode(jpg).decode('ascii'),
                             'context': context or {}}, timeout=timeout)
