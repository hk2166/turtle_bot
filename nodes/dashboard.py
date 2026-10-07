#!/usr/bin/env python3
# Written 2026-10-06: the web mission-control panel.
"""Campus Bot dashboard: a web panel served from the robot (http://<robot>:8088).

    run/dashboard.sh                    start/stop it (any browser on the office WiFi)

Shows the live map with the robot, its plan and goals (click the map to send the robot there),
the camera feed, power/WiFi/temperature/navigation status, and a STOP button.
Navigation itself must be running (run/nav.sh); the panel only talks to it.
The camera is opened only while someone is actually watching the feed, and released after.
"""
import json
import math
import os
import subprocess
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
import yaml
sys_path_added = True
import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CompressedImage
from nav2_msgs.srv import LoadMap
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.expanduser('~/.campus_bot/logs')
LOGS = {n: os.path.join(LOG_DIR, n + '.log') for n in
        ('nav', 'bringup', 'auto', 'tags', 'slam', 'agent', 'dashboard')}
ANSI = re.compile(r'\x1b\[[0-9;]*m')
PORT = int(os.environ.get('DASH_PORT', '8088'))
AREA_FILE = os.path.expanduser('~/.campus_bot/run/dash_map')
CAM_RESERVED = os.path.expanduser('~/.campus_bot/run/camera_reserved')   # touched by run/camera.sh on


def _initial_area():
    try:
        a = open(AREA_FILE).read().strip()
        if a and os.path.exists(os.path.join(PROJECT, 'maps', f'{a}_clean.yaml')):
            return a
    except OSError:
        pass
    return os.environ.get('DASH_MAP', 'office')


def remember_area(a):
    try:
        os.makedirs(os.path.dirname(AREA_FILE), exist_ok=True)
        open(AREA_FILE, 'w').write(a)
    except OSError:
        pass


AREA = _initial_area()
NAV_STATE = {GoalStatus.STATUS_ACCEPTED: 'GOAL ACCEPTED', GoalStatus.STATUS_EXECUTING: 'DRIVING',
             GoalStatus.STATUS_CANCELING: 'CANCELING', GoalStatus.STATUS_SUCCEEDED: 'GOAL REACHED',
             GoalStatus.STATUS_CANCELED: 'GOAL CANCELED', GoalStatus.STATUS_ABORTED: 'GOAL FAILED'}


class Ros(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, None, spin_thread=True)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_stop = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan = []
        self.nav_state = 'IDLE'
        self.goal = None
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        from rclpy.qos import DurabilityPolicy as DP, QoSProfile as QP
        self.grid = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m),
                                 QP(depth=1, durability=DP.TRANSIENT_LOCAL))
        self.costmap = None    # the planner's view: static map + live obstacles + inflation
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'costmap', m),
                                 QP(depth=1, durability=DP.TRANSIENT_LOCAL))
        from rclpy.qos import qos_profile_sensor_data
        self.cam_jpg = None
        self.cam_t = 0.0

        def on_img(m):
            self.cam_jpg = bytes(m.data)
            self.cam_t = time.time()
        self.create_subscription(CompressedImage, '/camera/image/compressed', on_img, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 10)

    def on_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[::3]]

    def on_status(self, msg):
        if msg.status_list:
            self.nav_state = NAV_STATE.get(msg.status_list[-1].status, 'IDLE')

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.1))
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:
            return None

    def send_goal(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_goal.publish(m)
        self.goal = (x, y)

    def send_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25 ** 2
        m.pose.covariance[35] = math.radians(15) ** 2
        self.pub_init.publish(m)

    def reload_map(self, yaml_path):
        if not self.load_map_cli.wait_for_service(timeout_sec=3.0):
            return 'map server not running (fence saved; applies on next nav start)'
        req = LoadMap.Request()
        req.map_url = yaml_path
        fut = self.load_map_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10:
            time.sleep(0.1)
        if fut.done() and fut.result() and fut.result().result == 0:
            return 'boundary live: planner now fenced in'
        return 'map reload failed (fence saved; applies on next nav start)'

    def stop(self):
        p = self.pose()
        if p:
            self.send_goal(p[0], p[1], p[2])   # "come to where you are" preempts the current goal
        for _ in range(5):
            self.pub_stop.publish(Twist())
            time.sleep(0.05)
        self.goal = None


class Camera:
    """Grabs frames only while a browser polled /frame.jpg in the last few seconds
    (snapshot polling renders in every browser, unlike MJPEG streams in Safari).
    The robot's camera_node always wins: the device is released (and not reopened) while
    run/camera.sh has just reserved it or camera_node's preview is arriving."""
    def __init__(self, ros):
        self.ros = ros
        self.lock = threading.Lock()
        self.jpg = None
        self.err = b''
        self.last_req = 0.0
        self.running = False

    def reserved(self):
        try:
            if time.time() - os.path.getmtime(CAM_RESERVED) < 20.0:
                return True
        except OSError:
            pass
        return self.ros.cam_jpg is not None and time.time() - self.ros.cam_t < 3.0

    def _loop(self):
        import sys
        sys.path.insert(0, os.path.join(PROJECT, 'nodes'))
        import tags_common as tc
        cap = tc.open_camera()
        if cap is None:
            with self.lock:
                self.err = b'camera busy or unplugged'
                self.running = False
            return
        try:
            while time.time() - self.last_req < 5.0 and not self.reserved():
                ok, img = cap.read()
                if not ok:
                    break
                img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with self.lock:
                        self.jpg = jpg.tobytes()
                time.sleep(0.12)
        finally:
            cap.release()
            with self.lock:
                self.running = False
                self.jpg = None

    def get(self):
        with self.lock:
            self.last_req = time.time()
            if not self.running and not self.reserved():
                self.running = True
                self.err = b''
                threading.Thread(target=self._loop, daemon=True).start()
            return self.jpg, self.err


class State:
    """Cached slow readings (power, wifi, temperature, what's running)."""
    def __init__(self):
        self.cache = {}
        self.t = 0.0
        self.lock = threading.Lock()
        self.fit_msg = ''
        self.job = ''
        self.busy = False

    def slow(self):
        with self.lock:
            if time.time() - self.t < 5:
                return self.cache
            out = {}
            try:
                p = subprocess.run(['bash', os.path.join(PROJECT, 'run', 'power_check.sh'), '--brief'],
                                   capture_output=True, text=True, timeout=5)
                out['power'] = p.stdout.strip()[:60] or 'n/a'
            except Exception:
                out['power'] = 'n/a'
            try:
                w = subprocess.run(['iw', 'dev', 'wlan0', 'link'], capture_output=True, text=True, timeout=3).stdout
                out['wifi'] = next((ln.split()[1] + ' dBm' for ln in w.splitlines() if 'signal' in ln), 'n/a')
            except Exception:
                out['wifi'] = 'n/a'
            try:
                t = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True, timeout=3).stdout
                out['temp'] = t.strip().replace('temp=', '')
            except Exception:
                out['temp'] = 'n/a'
            out['nav_up'] = subprocess.run(['pgrep', '-f', 'component_container_isolated'],
                                           capture_output=True).returncode == 0
            self.cache, self.t = out, time.time()
            return out


def pid_alive(name):
    try:
        pid = int(open(os.path.expanduser(f'~/.campus_bot/run/{name}.pid')).read().strip())
        os.kill(pid, 0)
        return True
    except (OSError, ValueError):
        return False


def list_maps():
    import glob
    return sorted(os.path.basename(p)[:-len('_clean.yaml')]
                  for p in glob.glob(os.path.join(PROJECT, 'maps', '*_clean.yaml')))


def content_bbox(known, margin=8, min_cells=8):
    """Rows/cols holding a meaningful amount of mapped cells (stray speckle ignored), padded."""
    rows = np.flatnonzero(known.sum(axis=1) >= min_cells)
    cols = np.flatnonzero(known.sum(axis=0) >= min_cells)
    if not len(rows) or not len(cols):
        rows = np.flatnonzero(known.any(axis=1))
        cols = np.flatnonzero(known.any(axis=0))
    if not len(rows):
        return None
    return (max(0, rows[0] - margin), min(known.shape[0] - 1, rows[-1] + margin),
            max(0, cols[0] - margin), min(known.shape[1] - 1, cols[-1] + margin))


def render_grid(grid):
    """Live /map (while mapping), cropped to the mapped area, same green theme as the static map."""
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    a0 = np.asarray(grid.data, np.int16).reshape(grid.info.height, grid.info.width)  # row 0 = bottom
    bb = content_bbox(a0 >= 0)
    if bb:
        r0, r1, c0, c1 = bb
        a0 = a0[r0:r1 + 1, c0:c1 + 1]
        ox += c0 * res
        oy += r0 * res
    a = a0[::-1]
    v = np.full((*a.shape, 3), 14, np.uint8)
    v[(a >= 0) & (a < 50)] = (24, 34, 24)
    v[a >= 50] = (120, 255, 120)
    ok, png = cv2.imencode('.png', v)
    return (png.tobytes(),
            {'resolution': res, 'origin': [ox, oy], 'A': world_to_px_mat(res, ox, oy, a.shape[0])},
            a.shape)


def world_to_px_mat(res, ox, oy, h):
    """3x3: world metres -> image pixels (row 0 = top)."""
    return np.array([[1.0 / res, 0.0, -ox / res],
                     [0.0, -1.0 / res, (h - 1) + oy / res],
                     [0.0, 0.0, 1.0]])


def dominant_angle(occ_rc):
    """Angle (deg) of the dominant wall direction of occupied pixels, in (-45, 45]."""
    pts = occ_rc[:, ::-1].astype(np.float32)        # (x=col, y=row)
    a = cv2.minAreaRect(pts)[2]
    while a > 45:
        a -= 90
    while a <= -45:
        a += 90
    return a


def home_file(area):
    return os.path.join(PROJECT, 'config', f'home_{area}.yaml')


def get_home(area):
    """(x, y, yaw, how): the saved home, else the nearest safe open spot to the mapping start (0,0)."""
    try:
        h = yaml.safe_load(open(home_file(area))) or {}
        return float(h['x']), float(h['y']), float(h.get('yaw', 0.0)), 'saved home'
    except (OSError, KeyError, TypeError, ValueError):
        pass
    fenced = os.path.join(PROJECT, 'maps', f'{area}_fenced.yaml')
    base = fenced if os.path.exists(fenced) else os.path.join(PROJECT, 'maps', f'{area}_clean.yaml')
    meta = yaml.safe_load(open(base))
    img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image']), -1)
    res, ox, oy = float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])
    H = img.shape[0]
    blocked = ((img == 0) | (img == 205)).astype(np.uint8)
    dt = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 3) * res
    ys, xs = np.where(dt >= 0.45)          # robot half-diagonal 0.29 m + margin
    if not len(xs):
        return 0.0, 0.0, 0.0, 'mapping start (no safe spot found)'
    wx = ox + (xs + 0.5) * res
    wy = oy + (H - 1 - ys + 0.5) * res
    i = int(np.argmin(np.hypot(wx, wy)))
    return float(wx[i]), float(wy[i]), 0.0, f'nearest open spot to the mapping start ({np.hypot(wx[i], wy[i]):.1f} m from it)'


_SNAP = {}   # map path -> (mtime, dt metres, res, ox, oy, H)


def snap_reachable(ros, x, y, within=2.5):
    """Snap (x, y) to the nearest spot the robot can REACH on the planner's live costmap (static map
    + fence + currently seen obstacles + inflation): the goal always lands in the robot's own pocket
    of free space, so "no valid path" can only mean the way is truly blocked. None = can't tell
    (no costmap / no pose yet); (None, None, None) = nothing reachable within `within` m."""
    m, p = ros.costmap, ros.pose()
    if m is None or p is None:
        return None
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    h, w = m.info.height, m.info.width
    g = np.asarray(m.data, np.int16).reshape(h, w)      # -1 unknown, 0..98 drivable, 99/100 blocked
    trav = ((g >= 0) & (g < 99)).astype(np.uint8)       # 99 = too close to a wall for the footprint
    _, labels = cv2.connectedComponents(trav, connectivity=8)
    rr, rc = int((p[1] - oy) / res), int((p[0] - ox) / res)
    if not (0 <= rr < h and 0 <= rc < w):
        return None
    if not trav[rr, rc]:                                # scan noise can blot the robot's own cell
        ys, xs = np.nonzero(trav[max(rr - 16, 0):rr + 17, max(rc - 16, 0):rc + 17])
        if not len(xs):
            return None
        d = (ys + max(rr - 16, 0) - rr) ** 2 + (xs + max(rc - 16, 0) - rc) ** 2
        i = int(np.argmin(d))
        rr, rc = ys[i] + max(rr - 16, 0), xs[i] + max(rc - 16, 0)
    region = labels == labels[rr, rc]
    gr, gc = int((y - oy) / res), int((x - ox) / res)
    r = int(within / res)
    y0, y1 = max(gr - r, 0), min(gr + r + 1, h)
    x0, x1 = max(gc - r, 0), min(gc + r + 1, w)
    ys, xs = np.where(region[y0:y1, x0:x1])
    if not len(xs):
        return None, None, None
    d = np.hypot(ys + y0 - gr, xs + x0 - gc) * res
    score = d + g[ys + y0, xs + x0] * 0.004             # prefer lower cost: don't park against furniture
    i = int(np.argmin(score))
    if d[i] > within:
        return None, None, None
    nx, ny = ox + (xs[i] + x0 + 0.5) * res, oy + (ys[i] + y0 + 0.5) * res
    return nx, ny, math.hypot(nx - x, ny - y)


def snap_goal(x, y, need=0.35, within=2.0):
    """(x', y', moved) with x',y' the nearest point to (x, y) that has `need` m of clearance on the
    served map (fence included), searching `within` m; None when there is no such point (goal in a
    wall, under a desk, outside the fence or off the map). need 0.35 = half-diagonal 0.29 + margin."""
    fenced = os.path.join(PROJECT, 'maps', f'{AREA}_fenced.yaml')
    base = fenced if os.path.exists(fenced) else os.path.join(PROJECT, 'maps', f'{AREA}_clean.yaml')
    try:
        mtime = os.path.getmtime(base)
        if _SNAP.get(base, (None,))[0] != mtime:
            meta = yaml.safe_load(open(base))
            img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image']), -1)
            blocked = ((img == 0) | (img == 205)).astype(np.uint8)
            dt = cv2.distanceTransform(1 - blocked, cv2.DIST_L2, 3) * float(meta['resolution'])
            _SNAP.clear()   # one map at a time
            _SNAP[base] = (mtime, dt, float(meta['resolution']),
                           float(meta['origin'][0]), float(meta['origin'][1]), img.shape[0])
    except (OSError, KeyError, TypeError, ValueError):
        return x, y, 0.0                       # no saved map (live mapping): send as clicked
    _, dt, res, ox, oy, H = _SNAP[base]
    cx, cy = int((x - ox) / res), int(H - 1 - (y - oy) / res)
    if 0 <= cy < dt.shape[0] and 0 <= cx < dt.shape[1] and dt[cy, cx] >= need:
        return x, y, 0.0
    r = int(within / res)
    y0, y1 = max(cy - r, 0), min(cy + r + 1, dt.shape[0])
    x0, x1 = max(cx - r, 0), min(cx + r + 1, dt.shape[1])
    ys, xs = np.where(dt[y0:y1, x0:x1] >= need)
    if not len(xs):
        return None
    d2 = (ys + y0 - cy) ** 2 + (xs + x0 - cx) ** 2
    i = int(np.argmin(d2))
    if d2[i] > r * r:
        return None
    nx = ox + (xs[i] + x0 + 0.5) * res
    ny = oy + (H - 1 - (ys[i] + y0) + 0.5) * res
    return nx, ny, math.hypot(nx - x, ny - y)


def load_map():
    fenced = os.path.join(PROJECT, 'maps', f'{AREA}_fenced.yaml')
    base = fenced if os.path.exists(fenced) else os.path.join(PROJECT, 'maps', f'{AREA}_clean.yaml')
    if not os.path.exists(base):   # brand-new map being created: nothing saved yet
        blank = np.full((200, 200, 3), 14, np.uint8)
        okb, png = cv2.imencode('.png', blank)
        return png.tobytes(), {'resolution': 0.05, 'origin': [-5.0, -5.0],
                               'A': world_to_px_mat(0.05, -5.0, -5.0, 200)}, (200, 200)
    meta = yaml.safe_load(open(base))
    img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image']), -1)
    # crop the big unknown margins so the office fills the pane; shift the origin to match
    bb = content_bbox(img != 205)        # pgm: row 0 = TOP
    if bb:
        r0, r1, c0, c1 = bb
        H = img.shape[0]
        meta = dict(meta)
        meta['origin'] = [meta['origin'][0] + c0 * meta['resolution'],
                          meta['origin'][1] + (H - 1 - r1) * meta['resolution'], 0]
        img = img[r0:r1 + 1, c0:c1 + 1]
    v = np.full((*img.shape, 3), 14, np.uint8)
    v[img == 254] = (24, 34, 24)        # free floor: very dark green-grey
    v[img == 205] = (14, 16, 14)        # unknown: near black
    v[img == 0] = (120, 255, 120)       # walls: phosphor green (BGR)

    A = world_to_px_mat(meta['resolution'], meta['origin'][0], meta['origin'][1], img.shape[0])
    occ = np.column_stack(np.where(img == 0))
    if len(occ) > 80:
        # straighten the office: rotate so the dominant wall direction runs horizontal
        ang = dominant_angle(occ)
        h0, w0 = v.shape[:2]
        R = cv2.getRotationMatrix2D((w0 / 2, h0 / 2), ang, 1.0)
        cos, sin = abs(R[0, 0]), abs(R[0, 1])
        w1, h1 = int(h0 * sin + w0 * cos), int(h0 * cos + w0 * sin)
        R[0, 2] += w1 / 2 - w0 / 2
        R[1, 2] += h1 / 2 - h0 / 2
        v = cv2.warpAffine(v, R, (w1, h1), flags=cv2.INTER_NEAREST, borderValue=(14, 16, 14))
        A = np.vstack([R, [0, 0, 1]]) @ A
        known = np.any(v != (14, 16, 14), axis=2)
        bb = content_bbox(known)
        if bb:
            r0, r1, c0, c1 = bb
            v = v[r0:r1 + 1, c0:c1 + 1]
            A = np.array([[1, 0, -c0], [0, 1, -r0], [0, 0, 1.0]]) @ A
    ok, png = cv2.imencode('.png', v)
    meta = dict(meta)
    meta['A'] = A
    return png.tobytes(), meta, v.shape[:2]


MAP_PNG, MAP_META, MAP_SHAPE = load_map()


def refresh_map():
    global MAP_PNG, MAP_META, MAP_SHAPE
    MAP_PNG, MAP_META, MAP_SHAPE = load_map()
PAGE = open(os.path.join(PROJECT, 'nodes', 'dashboard.html'), 'rb').read()


class Handler(BaseHTTPRequestHandler):
    ros = None
    cam = None
    state = None

    def log_message(self, *a):
        pass

    def send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send(200, 'text/html; charset=utf-8', PAGE)
        elif self.path.startswith('/map.png'):
            if pid_alive('slam') and self.ros.grid is not None:
                png, _, _ = render_grid(self.ros.grid)
                self.send(200, 'image/png', png)
            else:
                self.send(200, 'image/png', MAP_PNG)
        elif self.path == '/api/state':
            s = self.state.slow()
            mapping = pid_alive('slam')
            if mapping and self.ros.grid is not None:
                _, m_meta, shape = render_grid(self.ros.grid)
            else:
                m_meta, shape = MAP_META, MAP_SHAPE
            A = np.asarray(m_meta['A'])
            Ainv = np.linalg.inv(A)
            d = {'pose': self.ros.pose(), 'plan': self.ros.plan, 'goal': self.ros.goal,
                 'mode': 'MAPPING' if mapping else ('NAV' if s['nav_up'] else 'DOWN'),
                 'maps': list_maps(), 'current': AREA, 'job': self.state.job,
                 'nav': ('MAPPING %s — drive in MANUAL, then SAVE MAP' % AREA) if mapping
                        else (self.ros.nav_state if s['nav_up'] else 'NAV DOWN (pick a map → NAVIGATE)'),
                 'fit': self.state.fit_msg, 'fence': __import__('fence_map').load_fence(AREA),
                 'home': list(get_home(AREA)[:3]) if os.path.exists(os.path.join(PROJECT, 'maps', f'{AREA}_clean.yaml')) else None,
                 'map': {'A': A[:2].tolist(), 'Ainv': Ainv[:2].tolist(),
                         'h': shape[0], 'w': shape[1]}, **s}
            self.send(200, 'application/json', json.dumps(d).encode())
        elif self.path.startswith('/api/logs'):
            name = (self.path.split('f=')[1].split('&')[0] if 'f=' in self.path else 'nav')
            f = LOGS.get(name)
            lines = []
            if f and os.path.exists(f):
                with open(f, 'rb') as fh:
                    fh.seek(max(0, os.path.getsize(f) - 40000))
                    text = fh.read().decode('utf-8', 'replace')
                for ln in text.splitlines()[-45:]:
                    ln = ANSI.sub('', ln)
                    # drop the noisy prefix: [component]... [stamp] [node]: message → node: message
                    ln2 = re.sub(r'^\[[a-zA-Z_0-9-]+-\d+\] ', '', ln)
                    m = re.match(r'^\[(INFO|WARN|ERROR|DEBUG)\] \[([0-9.]+)\] \[([^]]*)\]: (.*)$', ln2)
                    if m:
                        t = time.strftime('%H:%M:%S', time.localtime(float(m.group(2))))
                        lines.append(f'{t} {m.group(1)[0]} {m.group(3).split(".")[-1]}: {m.group(4)}'[:150])
                    else:
                        lines.append(ln2[:150])
            self.send(200, 'text/plain; charset=utf-8', '\n'.join(lines).encode())
        elif self.path.startswith('/frame.jpg'):
            jpg, err = self.cam.get()
            if not jpg and self.ros.cam_jpg and time.time() - self.ros.cam_t < 3.0:
                jpg = self.ros.cam_jpg          # camera_node owns the device: use its published preview
            if jpg:
                self.send(200, 'image/jpeg', jpg)
            else:
                self.send(503, 'text/plain', err or b'warming up')
        else:
            self.send(404, 'text/plain', b'not here')

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(n) or b'{}')
        if self.path == '/api/drive':
            # manual teleop: one Twist per POST. The browser streams while a control is held; when the
            # stream stops (release, hidden tab, WiFi drop) the firmware's 300 ms watchdog halts the robot.
            # hard caps = the robot's limits (robot.env): top ~0.8 m/s, 1.2 rad/s; firmware ramps accel
            vx = max(-0.8, min(0.8, float(body.get('vx', 0.0))))
            wz = max(-1.2, min(1.2, float(body.get('wz', 0.0))))
            t = Twist()
            t.linear.x, t.angular.z = vx, wz
            self.ros.pub_stop.publish(t)
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/goal':
            # Snap the click to the nearest spot the robot can actually reach: a click a little
            # inside a desk, a wall, the fence or a sealed-off pocket used to make Nav2 fail the
            # goal outright ("no valid path", 2026-10-08). Live costmap first, saved map as fallback.
            snapped = snap_reachable(self.ros, body['x'], body['y'])
            if snapped is None:                                   # nav not up yet: static-map snap
                snapped = snap_goal(body['x'], body['y'])
            if snapped is None or snapped[0] is None:
                self.state.fit_msg = 'no reachable open floor within 2.5 m of that click: not sent'
                self.send(200, 'application/json', b'{"ok":false}')
                return
            gx, gy, moved = snapped
            p = self.ros.pose()
            yaw = body.get('yaw')
            if yaw is None:
                yaw = math.atan2(gy - p[1], gx - p[0]) if p else 0.0
            self.ros.send_goal(gx, gy, yaw)
            if moved > 0.1:
                self.state.fit_msg = f'goal nudged {moved:.1f} m onto open floor ({gx:.2f}, {gy:.2f})'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/home':
            x, y, yaw, how = get_home(AREA)
            self.ros.send_goal(x, y, yaw)
            self.state.fit_msg = f'going home: ({x:.2f}, {y:.2f}) — {how}'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/home_set':
            p = self.ros.pose()
            if p is None:
                self.state.fit_msg = 'no robot pose: set the pose first, then SET HOME'
            else:
                with open(home_file(AREA), 'w') as f:
                    f.write(f'# home for maps/{AREA} (dashboard SET HOME); delete to use the mapping start\n')
                    yaml.safe_dump({'x': round(p[0], 3), 'y': round(p[1], 3), 'yaw': round(p[2], 4)}, f)
                self.state.fit_msg = f'home saved: ({p[0]:.2f}, {p[1]:.2f}) @ {math.degrees(p[2]):.0f}°'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/stop':
            self.ros.stop()
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/pose':
            self.ros.send_pose(body['x'], body['y'], body.get('yaw', 0.0))
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/use_map':
            name = body.get('name', '')
            if name not in list_maps():
                self.send(400, 'application/json', b'{"ok":false}')
                return
            if self.state.busy:
                self.send(200, 'application/json', b'{"ok":false}')
                return

            def run():
                global AREA
                self.state.busy = True
                try:
                    self.state.job = f'switching to map {name}: stopping the stack ...'
                    subprocess.run(['bash', os.path.join(PROJECT, 'run', 'stop.sh'), '--keep-dashboard'],
                                   capture_output=True, timeout=120)
                    AREA = name
                    remember_area(name)
                    refresh_map()
                    self.state.job = f'starting navigation on {name} (takes ~1 min) ...'
                    subprocess.Popen(['setsid', 'bash', os.path.join(PROJECT, 'run', 'nav.sh'),
                                      name, '--yes', '--no-rviz'],
                                     stdout=open(os.path.expanduser('~/.campus_bot/logs/nav_run.log'), 'w'),
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
                    for _ in range(40):
                        time.sleep(3)
                        if pid_alive('nav'):
                            break
                    self.state.job = f'navigation on {name}: set the pose (REFRESH / RESET / shift-click), then click goals'
                finally:
                    self.state.busy = False
            threading.Thread(target=run, daemon=True).start()
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/map_start':
            name = body.get('name', '')
            import re as _re
            if not _re.fullmatch(r'[A-Za-z0-9_-]{1,40}', name) or self.state.busy:
                self.send(400, 'application/json', b'{"ok":false}')
                return

            def run():
                global AREA
                self.state.busy = True
                try:
                    self.state.job = f'creating map {name}: stopping navigation, starting SLAM ...'
                    r = subprocess.run(['bash', os.path.join(PROJECT, 'run', 'map_ctl.sh'), 'start', name],
                                       capture_output=True, text=True, timeout=300)
                    if r.returncode == 0:
                        AREA = name
                        remember_area(name)
                        refresh_map()
                        self.state.job = f'MAPPING {name}: drive with the MANUAL tab, watch the map grow, then SAVE MAP'
                    else:
                        tail = (r.stdout + r.stderr).strip().splitlines()[-2:]
                        self.state.job = 'map start failed: ' + ' / '.join(t.strip() for t in tail)[:110]
                finally:
                    self.state.busy = False
            threading.Thread(target=run, daemon=True).start()
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/map_save':
            def run():
                self.state.busy = True
                try:
                    self.state.job = f'saving map {AREA} ...'
                    r = subprocess.run(['bash', os.path.join(PROJECT, 'run', 'map_ctl.sh'), 'save', AREA],
                                       capture_output=True, text=True, timeout=180)
                    refresh_map()
                    self.state.job = (f'map {AREA} saved — pick it in the list and NAVIGATE'
                                      if r.returncode == 0 else 'save failed: see dashboard.log')
                finally:
                    self.state.busy = False
            threading.Thread(target=run, daemon=True).start()
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/map_discard':
            subprocess.run(['bash', os.path.join(PROJECT, 'run', 'map_ctl.sh'), 'discard'],
                           capture_output=True, timeout=60)
            self.state.job = 'mapping discarded'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/fence':
            import fence_map
            lines = fence_map.load_fence(AREA)
            pts = body.get('points') or []
            if body.get('clear'):
                lines = []
            else:
                for a, b in zip(pts, pts[1:]):
                    lines.append([a[0], a[1], b[0], b[1]])
            fence_map.save_fence(AREA, lines)
            fence_map.bake(AREA)
            refresh_map()
            msg = self.ros.reload_map(os.path.join(PROJECT, 'maps', f'{AREA}_fenced.yaml'))
            self.state.fit_msg = f'{len(lines)} fence segment(s) — {msg}'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/fit':
            mode = body.get('mode', 'start')
            pose_now = self.ros.pose()

            def run():
                self.state.fit_msg = 'fitting scan to map ...'
                if mode == 'here':
                    if pose_now is None:
                        self.state.fit_msg = 'no current pose: use RESET POSE or FIT WIDE'
                        return
                    x, y, yaw = pose_now
                    r = subprocess.run(['python3', os.path.join(PROJECT, 'tools', 'fit_pose.py'),
                                        f'{x:.3f}', f'{y:.3f}', f'{math.degrees(yaw):.1f}',
                                        '--radius', '1.0', '--angle', '30'],
                                       capture_output=True, text=True)
                    try:
                        fx, fy, fyaw, score = (float(v) for v in r.stdout.split())
                    except ValueError:
                        self.state.fit_msg = ('fit failed: ' + (r.stdout + r.stderr).strip())[:120]
                        return
                    if score < 0.5:
                        self.state.fit_msg = f'no confident fit near here ({score:.0%}): pose left unchanged'
                        return
                    self.ros.send_pose(fx, fy, fyaw)
                    self.state.fit_msg = f'pose refreshed: {fx:.2f}, {fy:.2f} @ {math.degrees(fyaw):.0f}° ({score:.0%} on walls)'
                else:
                    r = subprocess.run(['bash', os.path.join(PROJECT, 'run', 'reset_pose.sh')] +
                                       (['--wide'] if mode == 'wide' else []),
                                       capture_output=True, text=True)
                    lines = [ln.strip() for ln in (r.stdout + r.stderr).splitlines() if 'fit' in ln or '✗' in ln]
                    self.state.fit_msg = (lines[-1] if lines else 'fit finished')[:120]
            threading.Thread(target=run, daemon=True).start()
            self.send(200, 'application/json', b'{"ok":true}')
        else:
            self.send(404, 'text/plain', b'not here')


def main():
    rclpy.init()
    ros = Ros()
    threading.Thread(target=lambda: rclpy.spin(ros), daemon=True).start()
    Handler.ros = ros
    Handler.cam = Camera(ros)
    Handler.state = State()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f'dashboard on http://0.0.0.0:{PORT}  (map: {AREA})', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
