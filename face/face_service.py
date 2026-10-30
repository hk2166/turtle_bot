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
import hmac
import json
import os
import socket
import ssl
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify as verify_mod  # noqa: E402
import identify as identify_mod  # noqa: E402
from engine import Engine, best_score, MIN_FACE_PX  # noqa: E402
from face_store import FaceStore  # noqa: E402
import enroll_session  # noqa: E402

HOST = os.environ.get('FACE_HOST', '127.0.0.1')
PORT = int(os.environ.get('FACE_PORT', '8070'))
DET_SIZE = int(os.environ.get('FACE_DET_SIZE', '640'))
CAM_W = int(os.environ.get('FACE_CAM_W', '1280'))
CAM_H = int(os.environ.get('FACE_CAM_H', '720'))
CAM_IDLE = float(os.environ.get('FACE_CAM_IDLE', '5'))     # s the grabber keeps decoding after the last use
CAM_FPS = int(os.environ.get('FACE_CAM_FPS', '30'))
DECODE_FPS = float(os.environ.get('FACE_DECODE_FPS', '15'))  # frames actually decoded per second
STREAM_FPS = float(os.environ.get('FACE_STREAM_FPS', '15'))  # dashboard stream rate (per viewer, max)
WARMUP_FRAMES = 5
MAX_BODY = 1 << 20
# Self-service enrolment for the office WiFi (phones/laptops use their own camera, which browsers only
# allow over HTTPS). Self-signed cert, kept OUT of the repo in ~/.campus_bot/tls/.
ENROLL_PORT = int(os.environ.get('ENROLL_PORT', '8443'))
ENROLL_HTTPS = os.environ.get('ENROLL_HTTPS', '1') != '0'
ENROLL_CODE = os.environ.get('ENROLL_CODE', '')          # optional shared access code
TLS_DIR = os.path.expanduser('~/.campus_bot/tls')
MAX_FRAME = 2 << 20
PAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'enroll_page.html')


BRIO_VENDOR = '046d'     # Logitech: the mast Brio belongs to camera_node (AprilTags + EYES preview)


def find_face_cam():
    """The capture node of the first USB camera that is NOT the Brio (the HIKVISION), from sysfs, so no
    udev rule is needed and a /dev/videoN renumbering after a replug doesn't matter."""
    base = '/sys/class/video4linux'
    try:
        names = sorted(os.listdir(base), key=lambda n: int(n[5:]) if n[5:].isdigit() else 999)
    except OSError:
        return None
    for n in names:
        d = os.path.join(base, n)
        try:
            if open(os.path.join(d, 'index')).read().strip() != '0':     # 1 = the metadata node
                continue
            usb = os.path.realpath(os.path.join(d, 'device', '..'))      # the USB device directory
            vendor = open(os.path.join(usb, 'idVendor')).read().strip()
        except OSError:
            continue                                                      # not a USB camera (Pi ISP etc.)
        if vendor != BRIO_VENDOR:
            return f'/dev/{n}'
    return None


def cam_device():
    """The face camera: FACE_CAM / FACE_CAM_INDEX if set, else /dev/face_cam (setup/10_face_cam.sh),
    else the auto-detected non-Brio USB camera; None when nothing is attached."""
    if os.environ.get('FACE_CAM_INDEX') not in (None, ''):
        return int(os.environ['FACE_CAM_INDEX'])
    for dev in (os.environ.get('FACE_CAM'), '/dev/face_cam'):
        if dev and os.path.exists(dev):
            return dev
    return find_face_cam()


class Cam:
    """The face camera, read by ONE background grabber thread while anyone wants frames (a dashboard
    stream, a delivery check). Everyone gets the newest frame instead of a stale buffered one, and
    several consumers never fight over the device. The grabber stops and releases the camera `idle` s
    after the last want, so an unwatched robot spends no CPU decoding video.
    `lock` is kept for callers that want a whole check to themselves (verify/identify)."""
    def __init__(self, device, w, h, idle, fps):
        self.device, self.w, self.h, self.idle, self.fps = device, w, h, idle, fps
        self.rotate = verify_mod.cam_rotation()     # the mount holds it upside-down (config/face_cam.yaml)
        self.lock = threading.Lock()
        self.cond = threading.Condition()
        self.frame, self.seq, self.error = None, 0, None
        self.want_until = 0.0
        self.thread = None

    def available(self):
        return self.device is not None and (isinstance(self.device, int) or os.path.exists(self.device))

    def want(self, secs=None):
        """Keep the grabber running for at least `secs` more (default: idle)."""
        with self.cond:
            self.want_until = max(self.want_until, time.time() + (secs or self.idle))
            if self.thread is None or not self.thread.is_alive():
                self.error = None
                self.thread = threading.Thread(target=self._grab, daemon=True)
                self.thread.start()

    def _open(self):
        if not self.available():
            raise RuntimeError('no_camera')
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open {self.device} (unplugged? held by another program?)')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(WARMUP_FRAMES):
            cap.read()
        return cap

    def _grab(self):
        cap = None
        try:
            cap = self._open()
            # The HIKVISION only does 25/30 fps at 720p, but nothing here needs more than ~15: grab()
            # every frame (cheap, keeps the newest one) and decode (retrieve, the costly part) at DECODE_FPS.
            period, nxt = 1.0 / max(1.0, DECODE_FPS), 0.0
            while time.time() < self.want_until:
                if not cap.grab():
                    raise RuntimeError('camera read failed (device dropped?)')
                if time.time() < nxt:
                    continue
                nxt = time.time() + period * 0.8      # slack: frame timing jitters at 30 fps
                ok, fr = cap.retrieve()
                if not ok:
                    raise RuntimeError('camera decode failed')
                fr = verify_mod.upright(fr, self.rotate)
                with self.cond:
                    self.frame = fr
                    self.seq += 1
                    self.cond.notify_all()
        except Exception as e:
            with self.cond:
                self.error = str(e)
                self.cond.notify_all()
        finally:
            if cap is not None:
                cap.release()
            with self.cond:
                self.frame = None           # never serve a frozen frame after the device is released

    def latest(self, after=-1, timeout=3.0):
        """(frame, seq) newer than `after` (the caller must not modify the frame). RuntimeError if the
        camera can't deliver ('no_camera' when nothing is attached)."""
        self.want()
        deadline = time.time() + timeout
        with self.cond:
            while self.frame is None or self.seq <= after:
                if self.error is not None and not (self.thread and self.thread.is_alive()):
                    raise RuntimeError(self.error)
                left = deadline - time.time()
                if left <= 0:
                    raise RuntimeError(self.error or 'no frames from the face camera')
                self.cond.wait(left)
            return self.frame, self.seq

    def frames(self):
        """Generator of successive new frames (for verify/identify's best-of-N)."""
        seq = -1
        while True:
            f, seq = self.latest(seq)
            yield f


class Service:
    def __init__(self):
        self.store = FaceStore()
        self.cam = Cam(cam_device(), CAM_W, CAM_H, CAM_IDLE, CAM_FPS)
        self.viewers, self.view_t = 0, 0.0      # who is watching the preview (drives the annotator)
        self.vlock = threading.Lock()
        self.ann = {'t': 0.0, 'faces': [], 'w': 1, 'hint': ''}
        self.ann_thread = None
        self.enc_lock = threading.Lock()
        self.enc = {}                           # annotate -> (seq, jpeg): one encode per frame, shared
        self.engine = None
        self.model_error = None
        self.infer = threading.Lock()          # one model call at a time (delivery checks + enrolment)
        self.sessions = enroll_session.Sessions(self.store, lambda: self.engine, self.infer,
                                                verify_mod.THRESHOLD)
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
                'camera': {'device': str(self.cam.device), 'available': self.cam.available(),
                           'rotate': self.cam.rotate},
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
                with self.infer:
                    r = fn['call'](frames=[str(p) for p in images], **kw)
            else:
                if not self.cam.available():
                    raise RuntimeError('no_camera')
                with self.cam.lock, self.infer:
                    r = fn['call'](frames=self.cam.frames(), **kw)
            return 200, {'ok': True, **r}
        except RuntimeError as e:
            msg = str(e)
            reason = 'no_camera' if msg == 'no_camera' else 'camera_error'
            return 200, {'ok': False, 'reason': reason, 'error': msg}
        except Exception as e:
            return 500, {'ok': False, 'reason': 'error', 'error': str(e)}

    # ---------------------------------------------------------------- live preview for the dashboard
    # The live preview: a detector-only pass every ANN_TRACK_EVERY s keeps the boxes on the faces as
    # people move (~50-170 ms on the Pi), and a full recognition every ANN_EVERY s (or at once when a
    # new face walks in) puts names on them (~70 ms more per face). Before 2026-10-30 one recognition
    # every 1.5 s boxed only the largest face and left the box behind for 2.5 s, so it looked lost.
    ANN_EVERY = float(os.environ.get('FACE_NAME_EVERY', '1.5'))
    ANN_TRACK_EVERY = float(os.environ.get('FACE_TRACK_EVERY', '0.35'))
    ANN_IDLE_EVERY = 1.0  # s between passes while nobody is in view (most of the time: saves the CPU)
    ANN_KEEP = 1.0        # s boxes stay drawn after the last pass that saw them

    def _roster_cached(self):
        t, names, embs = getattr(self, '_roster_c', (0.0, [], []))
        if time.time() - t > 10:
            names, embs = identify_mod._roster(self.store)
            self._roster_c = (time.time(), names, embs)
        return names, embs

    PREVIEW_W = 640

    def _touch_view(self, delta=0):
        """A viewer arrived/left (delta) or a snapshot was served (0): keeps the annotator running."""
        with self.vlock:
            self.viewers += delta
            self.view_t = time.time()
            if self.ann_thread is None or not self.ann_thread.is_alive():
                self.ann_thread = threading.Thread(target=self._annotator, daemon=True)
                self.ann_thread.start()

    def _label(self, f):
        """(label, known) for one detected face from a recognition pass."""
        w = min(f.bbox[2] - f.bbox[0], f.bbox[3] - f.bbox[1])
        if w < MIN_FACE_PX or getattr(f, 'embedding', None) is None:
            return 'come closer', False
        names, embs = self._roster_cached()
        if not names:
            return 'nobody enrolled', False
        emb = f.normed_embedding.astype(np.float32)
        scores = [best_score(emb, e) for e in embs]
        i = int(np.argmax(scores))
        second = sorted(scores)[-2] if len(scores) > 1 else -1.0
        if scores[i] >= identify_mod.THRESHOLD and scores[i] - second >= identify_mod.MARGIN:
            return f'{names[i]}  {scores[i]:.2f}', True
        return f'not recognised  {scores[i]:.2f}', False

    @staticmethod
    def _match(box, prev):
        """The previous face this box most likely is (centre within ~0.7 face widths), else None."""
        cx, cy, w = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2, box[2] - box[0]
        best, bd = None, 0.7 * max(w, 1.0)
        for p in prev:
            pb = p['bbox']
            d = np.hypot((pb[0] + pb[2]) / 2 - cx, (pb[1] + pb[3]) / 2 - cy)
            if d < bd:
                best, bd = p, d
        return best

    def _annotator(self):
        """Box and name every face in view while someone watches, off the video path, so the stream
        never waits for the model. Skips a pass when a delivery check holds the model."""
        last_name, flips, nofaces = 0.0, 0, 0
        while self.viewers > 0 or time.time() - self.view_t < 3.0:
            time.sleep(self.ANN_TRACK_EVERY if nofaces < 3 else self.ANN_IDLE_EVERY)
            if not self.ready() or not self.infer.acquire(blocking=False):
                continue
            try:
                frame, _seq = self.cam.latest(timeout=1.0)
                det = self.engine.app.det_model
                bboxes, _k = det.detect(frame, max_num=0, metric='default')
                boxes = [] if bboxes is None else [[float(v) for v in b[:4]] for b in bboxes]
                prev = self.ann.get('faces', [])
                matched = [self._match(b, prev) for b in boxes]
                faces = None
                # names: on schedule, or right away when someone new appears
                if boxes and (time.time() - last_name >= self.ANN_EVERY or any(m is None for m in matched)):
                    faces = self.engine.app.get(frame)
                    last_name = time.time()
                # no face for a while: is the picture upside down? (camera remounted, rotate: not updated)
                hint = self.ann.get('hint', '')
                if boxes:
                    nofaces, flips, hint = 0, 0, ''
                else:
                    nofaces += 1
                    if nofaces % 3 == 0:
                        fb, _ = det.detect(cv2.rotate(frame, cv2.ROTATE_180), max_num=0, metric='default')
                        flips = flips + 1 if fb is not None and len(fb) and float(fb[:, 4].max()) > 0.6 else 0
                        if flips >= 2:
                            hint = 'faces look UPSIDE DOWN: fix rotate in config/face_cam.yaml'
            except RuntimeError:
                continue
            finally:
                self.infer.release()
            out = []
            if faces is not None:
                for f in faces:
                    label, known = self._label(f)
                    out.append({'bbox': [float(v) for v in f.bbox], 'label': label, 'known': known})
            else:
                for b, m in zip(boxes, matched):
                    out.append({'bbox': b, 'label': m['label'] if m else '', 'known': bool(m and m['known'])})
            self.ann = {'t': time.time(), 'faces': out, 'w': frame.shape[1], 'hint': hint}

    def _encoded(self, frame, seq, annotate):
        """JPEG of this frame (downscaled, boxed); encoded once per frame however many viewers."""
        with self.enc_lock:
            hit = self.enc.get(annotate)
            if hit and hit[0] == seq:
                return hit[1]
            h, w = frame.shape[:2]
            s = min(1.0, self.PREVIEW_W / w)
            small = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA) if s < 1 else frame.copy()
            ann = self.ann
            if annotate and time.time() - ann['t'] < self.ANN_KEEP:
                k = small.shape[1] / max(ann.get('w', w), 1)
                for f in ann.get('faces', []):
                    x1, y1, x2, y2 = (int(v * k) for v in f['bbox'])
                    col = (80, 200, 60) if f['known'] else (0, 170, 255)          # BGR: green / amber
                    cv2.rectangle(small, (x1, y1), (x2, y2), col, 2)
                    if f['label']:
                        (tw, th), _ = cv2.getTextSize(f['label'], cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                        ty = max(th + 10, y1)
                        cv2.rectangle(small, (x1, ty - th - 10), (x1 + tw + 10, ty), col, -1)
                        cv2.putText(small, f['label'], (x1 + 5, ty - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (0, 0, 0), 2, cv2.LINE_AA)
            if annotate and ann.get('hint'):
                cv2.rectangle(small, (0, 0), (small.shape[1], 26), (0, 0, 160), -1)
                cv2.putText(small, ann['hint'], (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                            cv2.LINE_AA)
            ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if not ok:
                raise RuntimeError('encode failed')
            jpg = jpg.tobytes()
            self.enc[annotate] = (seq, jpg)
            return jpg

    def preview(self, annotate=True):
        """(jpeg bytes, None) of the face camera now, boxed and named when annotate; (None, why) if not."""
        if not self.cam.available():
            return None, 'no face camera connected'
        if annotate:
            self._touch_view()
        try:
            frame, seq = self.cam.latest()
            return self._encoded(frame, seq, annotate), None
        except RuntimeError as e:
            return None, str(e)

    def stream(self, annotate=True, fps=STREAM_FPS):
        """Generator of JPEGs for one MJPEG viewer: the newest frame each time, at most `fps` a second."""
        if annotate:
            self._touch_view(+1)
        period, seq, nxt = 1.0 / max(1.0, min(fps, 30.0)), -1, 0.0
        try:
            while True:
                wait = nxt - time.time()
                if wait > 0:
                    time.sleep(wait)
                frame, seq = self.cam.latest(seq)
                nxt = time.time() + period
                if annotate:
                    self.view_t = time.time()
                yield self._encoded(frame, seq, annotate)
        finally:
            if annotate:
                self._touch_view(-1)

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
        if self.path.startswith('/stream.mjpg'):         # live MJPEG stream (dashboard, admin)
            q = parse_qs(urlparse(self.path).query)
            annotate = q.get('ann', ['1'])[0] != '0'
            try:
                fps = float(q.get('fps', [STREAM_FPS])[0])
            except ValueError:
                fps = STREAM_FPS
            if not self.svc.cam.available():
                return self._send(503, {'ok': False, 'error': 'no face camera connected'})
            gen = self.svc.stream(annotate, fps)
            try:
                first = next(gen)                         # fail with a clean 503 before any headers
            except RuntimeError as e:
                gen.close()
                return self._send(503, {'ok': False, 'error': str(e)})
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                jpg = first
                while True:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    self.wfile.flush()
                    jpg = next(gen)
            except (BrokenPipeError, ConnectionResetError, OSError, RuntimeError):
                pass                                      # viewer closed the tab / camera dropped
            finally:
                gen.close()
            return
        if self.path.startswith('/frame.jpg'):           # live face-camera preview (dashboard, admin)
            jpg, why = self.svc.preview(annotate='ann=0' not in self.path)
            if jpg is None:
                payload = why.encode()
                self.send_response(503)
                self.send_header('Content-Type', 'text/plain')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpg)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == '/health':
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


# ------------------------------------------------------------------ self-service enrolment (HTTPS)
def _lan_ips():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout.split()
    except Exception:
        out = []
    return [ip for ip in out if not ip.startswith('fe80')]


def ensure_cert():
    """Self-signed cert for <host>.local + the current LAN IPs, regenerated when an IP is new (the robot
    moved to the other WiFi). Returns (crt, key) paths, or None if openssl failed."""
    os.makedirs(TLS_DIR, mode=0o700, exist_ok=True)
    crt, key, sans_f = (os.path.join(TLS_DIR, n) for n in ('enroll.crt', 'enroll.key', 'sans.txt'))
    host = socket.gethostname()
    names = sorted({f'DNS:{host}.local', f'DNS:{host}', 'DNS:localhost', 'IP:127.0.0.1'} |
                   {f'IP:{ip}' for ip in _lan_ips()})
    have = open(sans_f).read().split(',') if os.path.exists(sans_f) else []
    if os.path.exists(crt) and os.path.exists(key) and set(names) <= set(have):
        return crt, key
    r = subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-sha256', '-days', '825',
                        '-keyout', key, '-out', crt, '-subj', f'/CN={host}.local',
                        '-addext', 'subjectAltName=' + ','.join(names)], capture_output=True, text=True)
    if r.returncode != 0:
        print(f'enrolment HTTPS disabled: openssl failed: {r.stderr.strip()[:200]}', flush=True)
        return None
    os.chmod(key, 0o600)
    with open(sans_f, 'w') as f:
        f.write(','.join(names))
    return crt, key


class EnrollHandler(BaseHTTPRequestHandler):
    """The office-WiFi side: only the enrolment page and its API are reachable here."""
    svc = None

    def _send(self, code, body, ctype='application/json', headers=()):
        payload = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(payload)

    def _body(self, limit):
        n = int(self.headers.get('Content-Length', 0) or 0)
        if n > limit:
            raise ValueError('too large')
        return self.rfile.read(n) if n else b''

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ('/', '/enroll/'):
            return self._send(302, b'', 'text/plain', (('Location', '/enroll'),))
        if u.path == '/enroll':
            with open(PAGE_FILE, 'rb') as f:
                return self._send(200, f.read(), 'text/html; charset=utf-8')
        if u.path == '/api/enroll/info':
            return self._send(200, {'code_required': bool(ENROLL_CODE), 'model_ready': self.svc.ready()})
        if u.path == '/api/enroll/status':
            return self._send(200, self.svc.sessions.status(parse_qs(u.query).get('token', [''])[0]))
        self._send(404, {'ok': False, 'error': 'not found'})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            if u.path == '/api/enroll/frame':
                tok = parse_qs(u.query).get('token', [''])[0]
                buf = self._body(MAX_FRAME)
                img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR) if buf else None
                del buf
                st = self.svc.sessions.frame(tok, img)
                del img                         # the frame never outlives this request
                return self._send(200, st)
            body = json.loads(self._body(MAX_BODY) or b'{}')
            if not isinstance(body, dict):
                raise ValueError('body must be a JSON object')
        except (ValueError, json.JSONDecodeError) as e:
            return self._send(400, {'ok': False, 'msg': f'bad request: {e}'})
        if u.path == '/api/enroll/start':
            if ENROLL_CODE and not hmac.compare_digest(str(body.get('code', '')).encode(), ENROLL_CODE.encode()):
                time.sleep(1.0)
                return self._send(200, {'ok': False, 'msg': 'Wrong access code.'})
            return self._send(200, self.svc.sessions.start(str(body.get('person_id', '')),
                                                           bool(body.get('consent')),
                                                           str(body.get('mode', 'enroll'))))
        if u.path == '/api/enroll/cancel':
            return self._send(200, {'ok': self.svc.sessions.cancel(str(body.get('token', '')))})
        self._send(404, {'ok': False, 'error': 'not found'})

    def log_message(self, *a):
        pass


class TLSServer(ThreadingHTTPServer):
    """HTTPS on every address, IPv4 and IPv6 (resp-desktop.local resolves to IPv6 on the Airtel WiFi).
    The TLS handshake runs in the request thread, so a phone sitting on the cert warning can't block
    accept() for everyone else."""
    address_family = socket.AF_INET6
    daemon_threads = True

    def __init__(self, addr, handler, ctx):
        self.ctx = ctx
        super().__init__(addr, handler)

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()

    def get_request(self):
        sock, addr = self.socket.accept()
        sock.settimeout(15)
        return self.ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False), addr

    def finish_request(self, request, client_address):
        try:
            request.do_handshake()
        except (ssl.SSLError, OSError):         # e.g. a browser closing on the cert warning
            return
        super().finish_request(request, client_address)

    def handle_error(self, request, client_address):
        pass


def start_enroll_server(svc):
    if not ENROLL_HTTPS:
        return None
    cert = ensure_cert()
    if cert is None:
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(*cert)
    EnrollHandler.svc = svc
    srv = TLSServer(('::', ENROLL_PORT), EnrollHandler, ctx)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'self-service enrolment on https://{socket.gethostname()}.local:{ENROLL_PORT}/enroll '
          f'(self-signed cert; access code {"ON" if ENROLL_CODE else "off"})', flush=True)
    return srv


def main():
    Handler.svc = Service()
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f'face service on http://{HOST}:{PORT}  (camera {cam_device()}, det_size {DET_SIZE}); '
          f'model loading in the background', flush=True)
    start_enroll_server(Handler.svc)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
