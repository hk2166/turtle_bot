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
AREA = os.environ.get('DASH_MAP', 'office')
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
    (snapshot polling renders in every browser, unlike MJPEG streams in Safari)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.jpg = None
        self.err = b''
        self.last_req = 0.0
        self.running = False

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
            while time.time() - self.last_req < 5.0:
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
            if not self.running:
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


def render_grid(grid):
    """Live /map (while mapping) in the same green theme as the static map."""
    a = np.asarray(grid.data, np.int16).reshape(grid.info.height, grid.info.width)[::-1]
    v = np.full((*a.shape, 3), 14, np.uint8)
    v[(a >= 0) & (a < 50)] = (24, 34, 24)
    v[a >= 50] = (120, 255, 120)
    ok, png = cv2.imencode('.png', v)
    return png.tobytes(), {'resolution': grid.info.resolution,
                           'origin': [grid.info.origin.position.x, grid.info.origin.position.y]}, a.shape


def load_map():
    fenced = os.path.join(PROJECT, 'maps', f'{AREA}_fenced.yaml')
    base = fenced if os.path.exists(fenced) else os.path.join(PROJECT, 'maps', f'{AREA}_clean.yaml')
    if not os.path.exists(base):   # brand-new map being created: nothing saved yet
        blank = np.full((200, 200, 3), 14, np.uint8)
        okb, png = cv2.imencode('.png', blank)
        return png.tobytes(), {'resolution': 0.05, 'origin': [-5.0, -5.0]}, (200, 200)
    meta = yaml.safe_load(open(base))
    img = cv2.imread(os.path.join(PROJECT, 'maps', meta['image']), -1)
    v = np.full((*img.shape, 3), 14, np.uint8)
    v[img == 254] = (24, 34, 24)        # free floor: very dark green-grey
    v[img == 205] = (14, 16, 14)        # unknown: near black
    v[img == 0] = (120, 255, 120)       # walls: phosphor green (BGR)
    ok, png = cv2.imencode('.png', v)
    return png.tobytes(), meta, img.shape


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
            res = MAP_META['resolution']
            o = MAP_META['origin']
            mapping = pid_alive('slam')
            if mapping and self.ros.grid is not None:
                _, m_meta, m_shape = render_grid(self.ros.grid)
                res, o = m_meta['resolution'], {'0': m_meta['origin'][0], '1': m_meta['origin'][1]}
                o = m_meta['origin']
                shape = m_shape
            else:
                res, o, shape = MAP_META['resolution'], MAP_META['origin'], MAP_SHAPE
            d = {'pose': self.ros.pose(), 'plan': self.ros.plan, 'goal': self.ros.goal,
                 'mode': 'MAPPING' if mapping else ('NAV' if s['nav_up'] else 'DOWN'),
                 'maps': list_maps(), 'current': AREA, 'job': self.state.job,
                 'nav': ('MAPPING %s — drive in MANUAL, then SAVE MAP' % AREA) if mapping
                        else (self.ros.nav_state if s['nav_up'] else 'NAV DOWN (pick a map → NAVIGATE)'),
                 'fit': self.state.fit_msg, 'fence': __import__('fence_map').load_fence(AREA),
                 'map': {'res': res, 'ox': o[0], 'oy': o[1],
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
            vx = max(-0.35, min(0.35, float(body.get('vx', 0.0))))
            wz = max(-0.9, min(0.9, float(body.get('wz', 0.0))))
            t = Twist()
            t.linear.x, t.angular.z = vx, wz
            self.ros.pub_stop.publish(t)
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/goal':
            p = self.ros.pose()
            yaw = body.get('yaw')
            if yaw is None:
                yaw = math.atan2(body['y'] - p[1], body['x'] - p[0]) if p else 0.0
            self.ros.send_goal(body['x'], body['y'], yaw)
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
                    refresh_map()
                    self.state.job = f'starting navigation on {name} (takes ~1 min) ...'
                    subprocess.Popen(['setsid', 'bash', os.path.join(PROJECT, 'run', 'nav.sh'),
                                      name, '--no-camera', '--yes'],
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
    Handler.cam = Camera()
    Handler.state = State()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f'dashboard on http://0.0.0.0:{PORT}  (map: {AREA})', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
