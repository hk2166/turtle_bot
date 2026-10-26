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
from engine import Engine, best_score  # noqa: E402
from face_store import FaceStore  # noqa: E402
import enroll_session  # noqa: E402

HOST = os.environ.get('FACE_HOST', '127.0.0.1')
PORT = int(os.environ.get('FACE_PORT', '8070'))
DET_SIZE = int(os.environ.get('FACE_DET_SIZE', '640'))
CAM_W = int(os.environ.get('FACE_CAM_W', '1280'))
CAM_H = int(os.environ.get('FACE_CAM_H', '720'))
CAM_IDLE = float(os.environ.get('FACE_CAM_IDLE', '30'))
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
    """One warm cv2 capture, opened on demand and reaped after idle. Callers hold `lock` for a whole
    check so the camera and the (single-threaded) model are used by one request at a time."""
    def __init__(self, device, w, h, idle):
        self.device, self.w, self.h, self.idle = device, w, h, idle
        self.rotate = verify_mod.cam_rotation()     # the mount holds it upside-down (config/face_cam.yaml)
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
            yield verify_mod.upright(fr, self.rotate)

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
    # s between recognitions on the preview. Each costs ~350 ms of CPU (2 threads), so 1.5 s keeps one
    # viewer near a quarter of a core; at 0.7 s with 4 threads it was ~90% (measured 2026-10-26).
    ANN_EVERY = 1.5
    ANN_KEEP = 2.5        # s a box stays drawn after its recognition

    def _roster_cached(self):
        t, names, embs = getattr(self, '_roster_c', (0.0, [], []))
        if time.time() - t > 10:
            names, embs = identify_mod._roster(self.store)
            self._roster_c = (time.time(), names, embs)
        return names, embs

    PREVIEW_SHARE = 0.12  # s: every viewer polling within this window gets the same JPEG (CPU stays flat)

    def preview(self, annotate=True, width=640):
        """(jpeg bytes, None) of the face camera now, boxed and named when annotate; (None, why) if not.
        Shared: several dashboard tabs polling at once cost one capture + encode, not one each."""
        cache = getattr(self, '_pv', {})
        hit = cache.get(annotate)
        if hit and time.time() - hit[0] < self.PREVIEW_SHARE:
            return hit[1], None
        jpg, why = self._preview(annotate, width)
        if jpg is not None:
            cache[annotate] = (time.time(), jpg)
            self._pv = cache
        return jpg, why

    def _preview(self, annotate, width):
        if not self.cam.available():
            return None, 'no face camera connected'
        if not self.cam.lock.acquire(timeout=1.5):        # a delivery check is using the camera
            return None, 'face camera busy'
        try:
            frame = next(self.cam.frames())
        except RuntimeError as e:
            return None, str(e)
        finally:
            self.cam.lock.release()
        now = time.time()
        ann = getattr(self, '_ann', {'t': 0.0, 'bbox': None})
        if annotate and self.ready() and now - ann['t'] > self.ANN_EVERY and self.infer.acquire(blocking=False):
            try:
                a = self.engine.analyze(frame, embed=True)
            finally:
                self.infer.release()
            label, known = '', False
            if a['emb'] is not None:
                names, embs = self._roster_cached()
                if names:
                    scores = [best_score(a['emb'], e) for e in embs]
                    i = int(np.argmax(scores))
                    second = sorted(scores)[-2] if len(scores) > 1 else -1.0
                    if scores[i] >= identify_mod.THRESHOLD and scores[i] - second >= identify_mod.MARGIN:
                        label, known = f'{names[i]}  {scores[i]:.2f}', True
                    else:
                        label = f'not recognised  {scores[i]:.2f}'
                else:
                    label = 'nobody enrolled'
            elif a['bbox'] is not None:
                label = 'face unclear'
            ann = self._ann = {'t': now, 'bbox': a['bbox'], 'label': label, 'known': known}
        if annotate and ann.get('bbox') is not None and now - ann['t'] < self.ANN_KEEP:
            x1, y1, x2, y2 = (int(v) for v in ann['bbox'])
            col = (80, 200, 60) if ann['known'] else (0, 170, 255)          # BGR: green / amber
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3)
            if ann['label']:
                (tw, th), _ = cv2.getTextSize(ann['label'], cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                ty = max(th + 12, y1)
                cv2.rectangle(frame, (x1, ty - th - 12), (x1 + tw + 12, ty), col, -1)
                cv2.putText(frame, ann['label'], (x1 + 6, ty - 7), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 0), 2, cv2.LINE_AA)
        h, w = frame.shape[:2]
        if w > width:
            frame = cv2.resize(frame, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return (jpg.tobytes(), None) if ok else (None, 'encode failed')

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
