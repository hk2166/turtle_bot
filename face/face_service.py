#!/usr/bin/env python3
# Written 2026-10-22: the always-on face service the delivery flow calls. New file — not yet run on
# the robot with the real (second) camera.
"""Warm face-recognition service: keeps the InsightFace model loaded and owns the face camera, so the
delivery flow can verify a recipient or identify a sender in well under a second of model startup.

Runs in the face venv (it needs insightface/onnxruntime), separate from the ROS/dashboard Python:

    ~/.venvs/face/bin/python face/face_service.py          # 127.0.0.1:8070  (run/face.sh on)

Why a service and not a subprocess per check: loading buffalo_s takes several seconds; a per-delivery
subprocess would blow the 2 s budget every time. This process loads it once and holds the camera warm.

Endpoints (localhost only — nodes/face_client.py is the caller; nothing here is exposed to the WiFi):
    GET  /health                      model loaded?  camera ok?  how many enrolled?
    GET  /people                      enrolled ids (names only)
    POST /verify   {"person_id": X}   1:1 — is the person at the camera X?         -> verify.py result
    POST /identify {}                 1:N — who is at the camera?                   -> identify.py result
Both POSTs also take optional n / threshold / det_size (identify also margin / top). With
{"images": ["/path/a.jpg", ...]} they run against files instead of the camera (for testing/calibration).

Camera: FACE_CAM (default /dev/face_cam, the pinned second camera — see setup/10_face_cam.sh) or
FACE_CAM_INDEX for a bare /dev/videoN. The camera is opened on demand, kept warm, and released after
FACE_CAM_IDLE seconds idle so it never holds the device against enrolment or the Brio. If the camera
is absent (mount not built yet), /health says so and the camera endpoints return reason 'no_camera';
the service still starts and serves /people and the image-based checks.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify as verify_mod  # noqa: E402
import identify as identify_mod  # noqa: E402
from engine import Engine  # noqa: E402
from face_store import FaceStore  # noqa: E402

HOST = os.environ.get('FACE_HOST', '127.0.0.1')
PORT = int(os.environ.get('FACE_PORT', '8070'))
DET_SIZE = int(os.environ.get('FACE_DET_SIZE', '640'))
CAM_W = int(os.environ.get('FACE_CAM_W', '1280'))
CAM_H = int(os.environ.get('FACE_CAM_H', '720'))
CAM_IDLE = float(os.environ.get('FACE_CAM_IDLE', '30'))
WARMUP_FRAMES = 5
MAX_BODY = 1 << 20


def cam_device():
    """The configured face camera: a /dev path, an int index, or None when nothing is attached."""
    dev = os.environ.get('FACE_CAM', '/dev/face_cam')
    if os.environ.get('FACE_CAM_INDEX') not in (None, ''):
        return int(os.environ['FACE_CAM_INDEX'])
    return dev if dev and os.path.exists(dev) else None


class Cam:
    """One warm cv2 capture, opened on demand and reaped after idle. Callers hold `lock` for a whole
    check so the camera and the (single-threaded) model are used by one request at a time."""
    def __init__(self, device, w, h, idle):
        self.device, self.w, self.h, self.idle = device, w, h, idle
        self.cap = None
        self.last = 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._reaper, daemon=True).start()

    def available(self):
        return self.device is not None and (isinstance(self.device, int) or os.path.exists(self.device))

    def _ensure(self):
        if self.cap is not None and self.cap.isOpened():
            return self.cap
        if not self.available():
            raise RuntimeError('no_camera')
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open {self.device} (unplugged? held by another program?)')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(WARMUP_FRAMES):
            cap.read()
        self.cap = cap
        return cap

    def frames(self):
        """Generator of fresh frames from the warm camera; the caller must hold `lock`."""
        cap = self._ensure()
        self.last = time.time()
        while True:
            ok, fr = cap.read()
            if not ok:
                self._release()
                raise RuntimeError('camera read failed (device dropped?)')
            self.last = time.time()
            yield fr

    def _release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

    def _reaper(self):
        while True:
            time.sleep(5)
            with self.lock:
                if self.cap is not None and time.time() - self.last > self.idle:
                    self._release()


class Service:
    def __init__(self):
        self.store = FaceStore()
        self.cam = Cam(cam_device(), CAM_W, CAM_H, CAM_IDLE)
        self.engine = None
        self.model_error = None
        threading.Thread(target=self._load_model, daemon=True).start()

    def _load_model(self):
        try:
            self.engine = Engine(det_size=DET_SIZE)
        except Exception as e:                      # keep serving /health so the failure is visible
            self.model_error = str(e)

    def ready(self):
        return self.engine is not None

    def health(self):
        try:
            n = len(self.store.list_people())
            store_ok, store_err = True, None
        except Exception as e:
            n, store_ok, store_err = 0, False, str(e)
        return {'ok': self.ready() and store_ok, 'model_loaded': self.ready(),
                'model_error': self.model_error, 'det_size': DET_SIZE,
                'camera': {'device': str(self.cam.device), 'available': self.cam.available()},
                'enrolled': n, 'store_ok': store_ok, 'store_error': store_err}

    def _run(self, fn, body, **extra):
        """Shared path for /verify and /identify: images from the body, else the warm camera."""
        if not self.ready():
            return 503, {'ok': False, 'error': 'model_not_loaded', 'detail': self.model_error}
        images = body.get('images')
        kw = {'engine': self.engine, 'store': self.store,
              'n': int(body.get('n', verify_mod.N_FRAMES)),
              'threshold': float(body.get('threshold', fn['threshold'])),
              'det_size': int(body.get('det_size', DET_SIZE))}
        kw.update(extra)
        try:
            if images is not None:
                r = fn['call'](frames=[str(p) for p in images], **kw)
            else:
                with self.cam.lock:
                    r = fn['call'](frames=self.cam.frames(), **kw)
            return 200, {'ok': True, **r}
        except RuntimeError as e:
            msg = str(e)
            reason = 'no_camera' if msg == 'no_camera' else 'camera_error'
            return 200, {'ok': False, 'reason': reason, 'error': msg}
        except Exception as e:
            return 500, {'ok': False, 'reason': 'error', 'error': str(e)}

    def verify(self, body):
        pid = str(body.get('person_id', '')).strip().lower()
        if not pid:
            return 400, {'ok': False, 'error': 'person_id required'}
        return self._run({'call': verify_mod.verify, 'threshold': verify_mod.THRESHOLD}, body, person_id=pid)

    def identify(self, body):
        extra = {'margin': float(body.get('margin', identify_mod.MARGIN)), 'top': int(body.get('top', 3))}
        return self._run({'call': identify_mod.identify, 'threshold': identify_mod.THRESHOLD}, body, **extra)


class Handler(BaseHTTPRequestHandler):
    svc = None

    def _send(self, code, obj):
        payload = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path == '/health':
            self._send(200, self.svc.health())
        elif self.path == '/people':
            try:
                self._send(200, {'ok': True, 'people': [p for p, _n, _at in self.svc.store.list_people()]})
            except Exception as e:
                self._send(500, {'ok': False, 'error': str(e)})
        else:
            self._send(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(min(n, MAX_BODY)) or b'{}') if n else {}
            if not isinstance(body, dict):
                raise ValueError('body must be a JSON object')
        except (ValueError, json.JSONDecodeError) as e:
            return self._send(400, {'ok': False, 'error': f'bad JSON body: {e}'})
        if self.path == '/verify':
            code, obj = self.svc.verify(body)
        elif self.path == '/identify':
            code, obj = self.svc.identify(body)
        else:
            code, obj = 404, {'ok': False, 'error': 'not found'}
        self._send(code, obj)

    def log_message(self, *a):               # quiet: one line per request is noise on the robot
        pass


def main():
    Handler.svc = Service()
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f'face service on http://{HOST}:{PORT}  (camera {cam_device()}, det_size {DET_SIZE}); '
          f'model loading in the background', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
