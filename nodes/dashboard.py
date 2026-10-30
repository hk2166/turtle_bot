#!/usr/bin/env python3
# Written 2026-10-06: the web mission-control panel.
"""Campus Bot dashboard: a web panel served from the robot (http://<robot>:8088).

    run/dashboard.sh                    start/stop it (any browser on the office WiFi)

Shows the live map with the robot, its plan and goals (click the map to send the robot there),
the camera feed, power/WiFi/temperature/navigation status, and a STOP button.
Navigation itself must be running (run/nav.sh); the panel only talks to it.
The camera is opened only while someone is actually watching the feed, and released after.
"""
import ipaddress
import json
import math
import secrets
import urllib.request
from urllib.parse import parse_qs, urlparse
import socket
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

import fleet
import face_client   # talks to the warm face service; returns ok:false (never raises) when it's off
import command_client  # voice/typed commands -> trip plans (nodes/command_service.py, Claude)
def _brio_rotate():
    """CAMERA_IMAGE_ROTATE from config/robot.env (the run scripts source it but don't export it)."""
    try:
        m = re.search(r'^CAMERA_IMAGE_ROTATE=(\d+)', open(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'robot.env')).read(), re.M)
        deg = int(m.group(1)) % 360 if m else 0
    except (OSError, ValueError):
        deg = 0
    return {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(deg)


BRIO_ROTATE = _brio_rotate()
COMMAND_GAP = float(os.environ.get('COMMAND_GAP', '4'))
COMMAND_DAILY_CAP = int(os.environ.get('COMMAND_DAILY_CAP', '300'))
HTTPS_PORT = int(os.environ.get('DASH_HTTPS_PORT', '8444'))   # same pages over https: browsers only give
                                                               # a page the microphone on a secure origin

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
        # clean 960 px frame at 1 Hz for the vision features (camera_node only encodes it while subscribed)
        self.snap_jpg, self.snap_t = None, 0.0

        def on_snap(m):
            self.snap_jpg, self.snap_t = bytes(m.data), time.time()
        self.create_subscription(CompressedImage, '/camera/snapshot/compressed', on_snap, qos_profile_sensor_data)
        # Nav2's own speed-limit channel (controller_server listens on `speed_limit`): the hazard spotter
        # slows the robot through it; 0.0 = no limit
        from nav2_msgs.msg import SpeedLimit
        self._SpeedLimit = SpeedLimit
        self.pub_speed = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 10)
        # Cancel every Nav2 goal (the slip watcher's stop): stop() can't be used there, because it sends a
        # "come to where you are" goal and during a slip "where you are" is the drifted, wrong pose.
        from action_msgs.srv import CancelGoal
        self._CancelGoal = CancelGoal
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

    def cancel_all_goals(self):
        if not self.cancel_cli.service_is_ready():
            return False
        self.cancel_cli.call_async(self._CancelGoal.Request())   # zero goal id + stamp = every goal
        self.goal = None
        return True

    def snapshot(self, max_age=3.0):
        """Newest camera frame for vision: the clean snapshot, else the (smaller, tag-annotated) preview."""
        now = time.time()
        if self.snap_jpg and now - self.snap_t < max_age:
            return self.snap_jpg
        if self.cam_jpg and now - self.cam_t < max_age:
            return self.cam_jpg
        return None

    def set_speed_limit(self, percent=None):
        """Limit the Nav2 controller to `percent` % of its max speed; None lifts the limit."""
        m = self._SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = True
        m.speed_limit = float(percent) if percent else 0.0
        self.pub_speed.publish(m)

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
                if BRIO_ROTATE is not None:          # mounted upside down: same turn camera_node applies
                    img = cv2.rotate(img, BRIO_ROTATE)
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
                out['power_ok'] = p.returncode     # 0 clean, 1 low, 2 unreadable (pre-flight uses this)
            except Exception:
                out['power'] = 'n/a'
                out['power_ok'] = 2
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


_face = {'t': 0.0, 'health': {'ok': False}}


def face_health():
    """Cached face-service health (polled at most every 10 s): {ok, available, enrolled, ...}."""
    now = time.time()
    if now - _face['t'] > 10:
        h = face_client.health()
        _face['health'] = {'ok': bool(h.get('ok')),
                           'available': bool(h.get('ok') and h.get('camera', {}).get('available')),
                           'enrolled': h.get('enrolled', 0), 'model_loaded': bool(h.get('model_loaded'))}
        _face['t'] = now
    return _face['health']


# 🅿 HANDBRAKE (2026-10-30): a parked robot must not move, whoever asks. A file, so it survives restarts
# and reboots, and the other nodes (auto_clear, unstick) can check it too. While it exists: the queue
# dispatches nothing, map goals / HOME / manual drive / voice trips are refused, and brake_guard cancels
# any Nav2 goal that starts anyway (RViz, tools/goto.py, ...). Standing still, the DM542s keep holding
# torque, so the wheels resist being pushed. Not a safety-rated e-stop: a program writing /cmd_vel directly
# (tools/teleop_keys.py, drive_hold.py) isn't stopped by it; the master switch / an e-stop is.
HANDBRAKE_FILE = os.path.expanduser('~/.campus_bot/run/handbrake')


def handbrake_on():
    return os.path.exists(HANDBRAKE_FILE)


def set_handbrake(on):
    if on:
        os.makedirs(os.path.dirname(HANDBRAKE_FILE), exist_ok=True)
        with open(HANDBRAKE_FILE, 'w') as f:
            f.write(f'parked from the dashboard at {time.strftime("%F %T")}\n')
    else:
        try:
            os.remove(HANDBRAKE_FILE)
        except OSError:
            pass


def brake_guard(ros, log):
    """While parked: cancel any Nav2 goal that becomes active and send a stop."""
    last_log = 0.0
    while True:
        time.sleep(0.25)
        try:
            if handbrake_on() and ros.nav_state in ('DRIVING', 'GOAL ACCEPTED'):
                ros.cancel_all_goals()
                ros.pub_stop.publish(Twist())
                if time.time() - last_log > 5:
                    last_log = time.time()
                    log('🅿 handbrake: a goal tried to move the robot and was cancelled')
        except Exception as e:
            log(f'handbrake guard error: {e}')


_cmdh = {'t': 0.0, 'h': {'ok': False}}


def command_health():
    """Cached (10 s): is the command service up, and does it have an API key?"""
    if time.time() - _cmdh['t'] > 10:
        h = command_client.health()
        _cmdh['h'] = {'ok': bool(h.get('ok')), 'key': bool(h.get('key_configured')), 'model': h.get('model')}
        _cmdh['t'] = time.time()
    return _cmdh['h']


VISION_DAILY_CAP = int(os.environ.get('VISION_DAILY_CAP', '1500'))    # looks/day, all vision features
HAZARD_EVERY = float(os.environ.get('HAZARD_EVERY', '3'))             # s between looks while driving
HAZARD_SLOW_PCT = float(os.environ.get('HAZARD_SLOW_PCT', '40'))      # % of max speed near a hazard
HAZARD_SLOW_S = float(os.environ.get('HAZARD_SLOW_S', '8'))           # s it stays slow after a sighting
HAZARD_OFF_FILE = os.path.expanduser('~/.campus_bot/run/hazard_off')  # exists = spotter switched off
PRICE_IN, PRICE_OUT = 4e-6, 20e-6        # Claude Opus 5.5 $ per token, for the dashboard's cost estimate
# Legroom (camera, judged against the robot's own 38 cm-wide frame in view): while driving, slow down too
# when the camera sees less than this much room, even with no named hazard (footprint 0.44 x 0.38 m).
ROOM_MIN_AHEAD = float(os.environ.get('ROOM_MIN_AHEAD', '0.7'))
ROOM_MIN_WIDTH = float(os.environ.get('ROOM_MIN_WIDTH', '0.5'))


class Vision:
    """The mast camera + Claude (nodes/command_service.py) for SLOW navigation decisions:
    - hazard spotter: while Nav2 drives, one look every HAZARD_EVERY s for things the 2D lidar misses
      (low floor objects, overhangs, glass); on a medium/high one, Nav2's speed limit drops for a few
      seconds. Advisory only: an answer takes seconds, so it never steers or stops the robot; collision
      safety stays with the lidar, bumper and the 300 ms watchdog. A wrong answer = briefly slower.
    - ask(): one look for fleet ('blocked', 'arrival') and the WHAT DO YOU SEE button ('describe').
    Budget: VISION_DAILY_CAP looks a day across all of it; cost is estimated from the token usage."""

    def __init__(self, ros, log):
        self.ros, self.log = ros, log
        self.lock = threading.Lock()
        self.day, self.calls, self.cost = '', 0, 0.0
        self.last_hazard, self.limited, self.slow_until, self.inflight = None, False, 0.0, False
        self.logged = {}
        threading.Thread(target=self._hazard_loop, daemon=True).start()

    @staticmethod
    def hazard_on():
        return not os.path.exists(HAZARD_OFF_FILE)

    def set_hazard(self, on):
        if on:
            try:
                os.remove(HAZARD_OFF_FILE)
            except OSError:
                pass
        else:
            os.makedirs(os.path.dirname(HAZARD_OFF_FILE), exist_ok=True)
            open(HAZARD_OFF_FILE, 'w').close()
            if self.limited:
                self.ros.set_speed_limit(None)
                self.limited = False

    def _spend(self):
        with self.lock:
            today = time.strftime('%F')
            if self.day != today:
                self.day, self.calls, self.cost = today, 0, 0.0
            if self.calls >= VISION_DAILY_CAP:
                return False
            self.calls += 1
            return True

    def ask(self, mode, context=None, timeout=45):
        """One look through the camera: (result dict, None) or (None, why). Never raises."""
        jpg = self.ros.snapshot()
        if jpg is None:
            return None, 'no camera frame (is the mast camera running?)'
        if not command_health().get('key'):
            return None, 'no Anthropic API key'
        if not self._spend():
            return None, f'today\'s vision limit ({VISION_DAILY_CAP} looks) is used up'
        r = command_client.vision(mode, jpg, context, timeout=timeout)
        u = r.get('usage') or {}
        with self.lock:
            self.cost += u.get('in', 0) * PRICE_IN + u.get('out', 0) * PRICE_OUT
        if r.get('ok'):
            self.note_room(r['result'])
        return (r['result'], None) if r.get('ok') else (None, r.get('error'))

    def note_room(self, res):
        """Keep the latest legroom estimate (clear ahead / path width, metres) from any vision answer."""
        def num(k):
            try:
                v = float(res.get(k))
                return v if 0 <= v <= 50 else None
            except (TypeError, ValueError):
                return None
        room = {'clear_ahead_m': num('clear_ahead_m'), 'path_width_m': num('path_width_m'), 't': time.time()}
        if room['clear_ahead_m'] is not None or room['path_width_m'] is not None:
            self.last_room = room
        return room

    def status(self):
        lh, lr = self.last_hazard, getattr(self, 'last_room', None)
        return {'hazard_on': self.hazard_on(), 'calls_today': self.calls, 'cap': VISION_DAILY_CAP,
                'cost_today': round(self.cost, 2), 'slowed': self.limited,
                'last_hazard': lh if lh and time.time() - lh['t'] < 120 else None,
                'room': lr if lr and time.time() - lr['t'] < 120 else None}

    def _hazard_loop(self):
        last = 0.0
        while True:
            time.sleep(0.5)
            try:
                now = time.time()
                if self.limited and now > self.slow_until:
                    self.ros.set_speed_limit(None)
                    self.limited = False
                    self.log('hazard spotter: back to normal speed')
                if (self.ros.nav_state != 'DRIVING' or not self.hazard_on() or self.inflight
                        or now - last < HAZARD_EVERY or not command_health().get('key')):
                    continue
                last, self.inflight = now, True
                threading.Thread(target=self._check, daemon=True).start()
            except Exception as e:
                self.inflight = False
                self.log(f'hazard spotter error: {e}')

    def _check(self):
        try:
            res, _err = self.ask('hazard', {'state': 'driving autonomously to a goal'}, timeout=20)
            if not res or self.ros.nav_state != 'DRIVING':
                return
            room = self.note_room(res)
            hz = [h for h in res.get('hazards', [])
                  if h.get('severity') in ('medium', 'high') and h.get('distance') != 'over_3m']
            tight = []
            if room['clear_ahead_m'] is not None and room['clear_ahead_m'] < ROOM_MIN_AHEAD:
                tight.append(f"only ~{room['clear_ahead_m']:.1f} m clear ahead")
            if room['path_width_m'] is not None and room['path_width_m'] < ROOM_MIN_WIDTH:
                tight.append(f"path ~{room['path_width_m']:.2f} m wide (needs ~0.5)")
            if not hz and not tight:
                return
            pct = 25.0 if any(h['severity'] == 'high' for h in hz) else HAZARD_SLOW_PCT
            self.ros.set_speed_limit(pct)
            self.limited, self.slow_until = True, time.time() + HAZARD_SLOW_S
            text = '; '.join([f"{h['what']} ({h['distance'].replace('_', ' ')}, {h['where'].replace('_', ' ')})"
                              for h in hz] + tight)
            self.last_hazard = {'text': text, 't': time.time(), 'pct': pct}
            if time.time() - self.logged.get(text.lower(), 0) > 20:
                self.logged[text.lower()] = time.time()
                self.log(f'⚠ hazard ahead: {text} → slowing to {pct:.0f}%')
        finally:
            self.inflight = False


def face_identify(where='check', job=None):
    """Who is at the crate, as a short label, or None to log nothing. Log-only, never raises, never
    blocks a delivery. Skips silently unless the face service is up with a camera and someone enrolled."""
    try:
        if not face_health().get('available'):
            return None
        r = face_client.identify()
        if not r.get('ok'):
            return None
        if r.get('identified'):
            return f"{r['person_id']} ({r['score']})"
        if r.get('reason') in ('unknown', 'ambiguous'):
            return 'someone not recognised'
        return None            # no_face / no_camera / nobody_enrolled: nothing worth logging
    except Exception:
        return None


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


CALL_PAGE = open(os.path.join(PROJECT, 'nodes', 'call.html'), 'rb').read()
LOGIN_PAGE = open(os.path.join(PROJECT, 'nodes', 'login.html'), 'rb').read()
# Open to anyone on the WiFi: the call page and what it needs. Everything else is admin-only.
# '/' is the call page for everyone; mission control lives at /admin (login: ⚙ ADMIN, bottom right)
# The camera preview is view-only and public too: the call page shows what the robot sees.
PUBLIC_GET = ('/', '/call', '/login', '/api/places', '/api/queue', '/frame.jpg', '/enroll', '/face')
ENROLL_PORT = int(os.environ.get('ENROLL_PORT', '8443'))    # face/face_service.py's HTTPS enrolment page
# The internet-facing copy (run/vps_tunnel.sh carries it to the VPS): listens on the Pi's localhost only,
# and there NOTHING is open without the admin login (on the office WiFi the call page is open to all).
# Tunnelled visitors arrive from 127.0.0.1, so this port must never get the "the robot itself" trust.
REMOTE_PORT = int(os.environ.get('DASH_REMOTE_PORT', '8090'))
REMOTE_OPEN = ('/login', '/api/login', '/api/lan')
# ---- same WiFi? (2026-10-30) Someone on the internet link whose public address is the robot's own is
# behind the same office router: the page offers the direct WiFi address (much faster video), with a
# one-time pass so they stay logged in. The VPS's Traefik puts the visitor's address in X-Real-Ip.
VPS_ENV = os.path.expanduser('~/.campus_bot/vps.env')
NET = {'v4': '', 'v6': '', 'lan': '', 't': 0.0}
HANDOFF = {}                     # one-time pass -> (admin session token, expiry)
GIF = (b'GIF89a\x01\x00\x01\x00\x80\x00\x00\x00\x00\x00\xff\xff\xff!\xf9\x04\x01\x00\x00\x00\x00'
       b',\x00\x00\x00\x00\x01\x00\x01\x00\x00\x02\x02D\x01\x00;')


def remote_url():
    try:
        m = re.search(r'^VPS_URL=(\S+)', open(VPS_ENV).read(), re.M)
        return m.group(1) if m else ''
    except OSError:
        return ''


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))              # no packet is sent: just picks the outgoing interface
        return s.getsockname()[0]
    except OSError:
        return ''
    finally:
        s.close()


def public_ip_watch():
    """The robot's own public addresses (they change: Airtel), refreshed every 5 min."""
    while True:
        for k, url in (('v4', 'https://api.ipify.org'), ('v6', 'https://api6.ipify.org')):
            try:
                with urllib.request.urlopen(url, timeout=5) as r:
                    NET[k] = r.read().decode().strip()[:64]
            except Exception:
                pass                            # keep the last known one
        NET['lan'], NET['t'] = lan_ip(), time.time()
        time.sleep(300)


def same_network(ip):
    ip = (ip or '').strip()
    if ip.startswith('::ffff:'):
        ip = ip[7:]
    try:
        a = ipaddress.ip_address(ip)
        if a.version == 4:
            return bool(NET['v4']) and ip == NET['v4']
        return bool(NET['v6']) and ipaddress.ip_address(NET['v6']) in ipaddress.ip_network(f'{ip}/64', strict=False)
    except ValueError:
        return False


PUBLIC_POST = ('/api/login', '/api/request', '/api/request/cancel', '/api/request/done')
# admin actions that take the robot away from the queue (it pauses, a trip in progress goes back in front)
TAKES_CONTROL = {'/api/goal': 'map goal', '/api/stop': 'STOP', '/api/drive': 'manual drive',
                 '/api/home': 'HOME', '/api/use_map': 'map switch', '/api/map_start': 'mapping'}


class Handler(BaseHTTPRequestHandler):
    ros = None
    cam = None
    state = None
    auth = None
    queue = None
    vision = None
    remote = False               # True on REMOTE_PORT (the internet, via the VPS)
    _fails = []                  # times of wrong passwords on the remote port (slows guessing)

    def log_message(self, *a):
        pass

    def send(self, code, ctype, body, headers=()):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def json(self, obj, code=200, headers=()):
        self.send(code, 'application/json', json.dumps(obj).encode(), headers)

    def token(self):
        for part in (self.headers.get('Cookie') or '').split(';'):
            k, _, v = part.strip().partition('=')
            if k == 'cb_admin':
                return v
        return ''

    def is_admin(self):
        # the robot itself (scripts, curl on the Pi) is trusted; everyone else needs a login
        local = not self.remote and self.client_address[0] in ('127.0.0.1', '::1', '::ffff:127.0.0.1')
        return local or self.auth.valid(self.token())

    def remote_gate(self, route, post):
        """On the remote port only the login is open. True = refused (already answered)."""
        if not self.remote or route in REMOTE_OPEN or self.auth.valid(self.token()):
            return False
        if post:
            self.json({'ok': False, 'msg': 'log in first'}, 401)
        else:
            self.send(302, 'text/plain', b'login', (('Location', '/login'),))
        return True

    # Each command is one Claude call (a cent or two): the call page is open to the whole office WiFi, so
    # one command per device every COMMAND_GAP s and COMMAND_DAILY_CAP a day. The admin isn't limited.
    _cmd = {'day': '', 'n': 0, 'last': {}}
    _stt_last = {}               # device -> time of its last transcription (local, free: a light limit)

    def command_allowed(self, device, admin):
        st, today = Handler._cmd, time.strftime('%F')
        if st['day'] != today:
            st.update(day=today, n=0, last={})
        if not admin:
            if not device:
                return 'reload the page and try again'
            if time.time() - st['last'].get(device, 0) < COMMAND_GAP:
                return 'one command every few seconds, please'
            if st['n'] >= COMMAND_DAILY_CAP:
                return 'today\'s limit for voice commands is used up: call the robot to a place instead'
            st['last'][device] = time.time()
        st['n'] += 1
        return None

    def proxy_stream(self, url):
        """Pass an MJPEG stream (the face service's) through to the browser until either side closes."""
        import urllib.request
        try:
            up = urllib.request.urlopen(url, timeout=6)
        except Exception as e:
            msg = getattr(e, 'read', lambda: b'')() or str(e).encode()
            self.send(503, 'text/plain', msg[:200])
            return
        try:
            self.send_response(200)
            self.send_header('Content-Type', up.headers.get('Content-Type', 'multipart/x-mixed-replace; boundary=frame'))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            while True:
                chunk = up.read1(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            up.close()

    def eyes_stream(self, fps=15.0):
        """The mast camera as MJPEG: each new preview frame pushed as soon as it exists (no polling)."""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        last, period = None, 1.0 / fps
        try:
            while True:
                jpg, _err = self.cam.get()
                if not jpg and self.ros.cam_jpg and time.time() - self.ros.cam_t < 3.0:
                    jpg = self.ros.cam_jpg
                if jpg and jpg is not last:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    self.wfile.flush()
                    last = jpg
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def frame(self):
        jpg, err = self.cam.get()
        if not jpg and self.ros.cam_jpg and time.time() - self.ros.cam_t < 3.0:
            jpg = self.ros.cam_jpg          # camera_node owns the device: use its published preview
        if jpg:
            self.send(200, 'image/jpeg', jpg)
        else:
            self.send(503, 'text/plain', err or b'warming up')

    def client_ip(self):
        """The visitor's real address: on the remote port the VPS's Traefik sets X-Real-Ip."""
        if self.remote:
            return self.headers.get('X-Real-Ip') or (self.headers.get('X-Forwarded-For') or '').split(',')[-1]
        return self.client_address[0]

    def lan_routes(self, route):
        """Same-WiFi switching. True = answered."""
        if route == '/api/lan':
            if not self.remote:                 # already direct: tell the page where the internet link is
                self.json({'direct': True, 'remote': remote_url()})
            else:
                same = same_network(self.client_ip()) and bool(NET['lan'])
                self.json({'direct': False, 'same': same,
                           'lan': f'https://{NET["lan"]}:{HTTPS_PORT}' if same else ''})
            return True
        if route == '/api/lan/handoff' and self.remote and self.is_admin():
            if not (same_network(self.client_ip()) and NET['lan']):
                self.json({'ok': False})
                return True
            now = time.time()
            for k in [k for k, (_t, e) in HANDOFF.items() if e < now]:
                HANDOFF.pop(k, None)
            p = secrets.token_urlsafe(24)
            HANDOFF[p] = (self.auth.issue(), now + 60)
            self.json({'ok': True, 'url': f'https://{NET["lan"]}:{HTTPS_PORT}/handoff?t={p}'})
            return True
        if route == '/handoff' and not self.remote:
            tok, exp = HANDOFF.pop(parse_qs(urlparse(self.path).query).get('t', [''])[0], ('', 0))
            if tok and exp > time.time():
                self.send(302, 'text/plain', b'admin', (('Location', '/admin'), ('Set-Cookie',
                          f'cb_admin={tok}; Path=/; HttpOnly; SameSite=Lax; Max-Age={fleet.SESSION_S}')))
            else:
                self.send(302, 'text/plain', b'login', (('Location', '/login'),))
            return True
        if route == '/lan_ping.gif' and not self.remote:   # "can this browser reach me directly?"
            self.send(200, 'image/gif', GIF, (('Access-Control-Allow-Origin', '*'),))
            return True
        return False

    def do_GET(self):
        route = self.path.split('?')[0]
        if self.remote_gate(route, False):
            return
        if self.lan_routes(route):
            return
        if route in ('/', '/index.html', '/call'):
            self.send(200, 'text/html; charset=utf-8', CALL_PAGE)
            return
        if route == '/login':
            if self.auth.valid(self.token()):          # already logged in: straight to mission control
                self.send(302, 'text/plain', b'admin', (('Location', '/admin'),))
            else:
                self.send(200, 'text/html; charset=utf-8', LOGIN_PAGE)
            return
        if route == '/api/places':
            self.json({'map': AREA, 'places': [{'id': p['id'], 'name': p['name']} for p in fleet.load_places(AREA)]})
            return
        if route == '/api/queue':
            dev = self.path.split('device=')[1].split('&')[0] if 'device=' in self.path else ''
            self.json({**self.queue.public_view(dev), 'commands': command_health().get('ok', False)})
            return
        if route == '/frame.jpg':
            self.frame()
            return
        if route in ('/enroll', '/face'):
            # self-service face registration lives on the face service's HTTPS port (phones only give a
            # page the camera over https); keep whatever host name the visitor used
            h = self.headers.get('Host') or 'resp-desktop.local'
            host = h[:h.index(']') + 1] if h.startswith('[') else h.split(':')[0]
            self.send(302, 'text/plain', b'enroll', (('Location', f'https://{host}:{ENROLL_PORT}/enroll'),))
            return
        if not self.is_admin():
            if route == '/admin':
                self.send(302, 'text/plain', b'login', (('Location', '/login'),))
            else:
                self.send(401, 'text/plain', b'admin login required')
            return
        if route == '/admin':
            self.send(200, 'text/html; charset=utf-8', PAGE)
        elif route == '/face_stream.mjpg':      # the HIKVISION live (MJPEG), boxed + named (admin only)
            q = self.path.split('?', 1)[1] if '?' in self.path else ''
            self.proxy_stream(f'{face_client.BASE}/stream.mjpg?{q}')
        elif route == '/eyes_stream.mjpg':      # the mast camera live (MJPEG)
            self.eyes_stream()
        elif route == '/face_frame.jpg':        # the HIKVISION, boxed + named by the face service (admin only)
            jpg, why = face_client.frame(annotate='ann=0' not in self.path)
            if jpg is None:
                self.send(503, 'text/plain', why.encode())
            else:
                self.send(200, 'image/jpeg', jpg)
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
                         'h': shape[0], 'w': shape[1]},
                 'places': fleet.load_places(AREA), 'queue': self.queue.admin_view(),
                 'cam_ok': self.ros.cam_jpg is not None and time.time() - self.ros.cam_t < 3.0,
                 'face': face_health(), 'commands': command_health(), 'vision': self.vision.status(),
                 'handbrake': handbrake_on(), **s}
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
        else:
            self.send(404, 'text/plain', b'not here')

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            body = json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            body = {}
        if self.remote_gate(self.path.split('?')[0], True):
            return
        # ---- open to everyone (the call page)
        if self.path == '/api/login':
            if self.remote:
                now = time.time()
                Handler._fails = [t for t in Handler._fails if now - t < 600]
                if len(Handler._fails) >= 10:
                    self.json({'ok': False, 'msg': 'too many wrong passwords: wait 10 minutes'})
                    return
            tok = self.auth.login(str(body.get('password', '')))
            if tok is None and self.remote:
                Handler._fails.append(time.time())
            if tok is None:
                self.json({'ok': False, 'msg': 'wrong password' if fleet.Auth.configured() else
                           'no admin password yet: on the robot run  python3 tools/set_admin_password.py'})
            else:
                self.json({'ok': True}, headers=(('Set-Cookie', f'cb_admin={tok}; Path=/; HttpOnly; '
                                                  f'SameSite=Strict; Max-Age={fleet.SESSION_S}'
                                                  + ('; Secure' if self.remote else '')),))
            return
        if self.path == '/api/request':
            job, msg = self.queue.request(body.get('name'), body.get('place'), body.get('device'))
            self.json({'ok': job is not None, 'msg': msg,
                       **({'id': job['id'], 'token': job['token']} if job else {})})
            return
        if self.path == '/api/request/cancel':
            self.json({'ok': self.queue.cancel(body.get('id'), body.get('token'))})
            return
        if self.path == '/api/request/done':
            self.json({'ok': self.queue.done(body.get('id'), body.get('token'))})
            return
        # ---- voice / typed commands (call page and mission control). Claude only proposes a plan; the
        # page shows it and nothing moves until a person taps CONFIRM (/api/command/run).
        if self.path == '/api/command/parse':
            admin = self.is_admin()
            why = self.command_allowed(str(body.get('device') or ''), admin)
            if why:
                print(time.strftime('%H:%M:%S'), 'command refused:', why, flush=True)   # used to be silent
                self.json({'ok': False, 'error': why})
                return
            places = [{'id': p['id'], 'name': p['name']} for p in fleet.load_places(AREA)]
            r = command_client.parse(str(body.get('text') or '')[:500], places,
                                     {'name': str(body.get('name') or '')[:40], 'place_id': body.get('place_id')})
            print(time.strftime('%H:%M:%S'), 'command:', repr(str(body.get('text') or '')[:120]), '->',
                  (r.get('plan') or {}).get('summary') or r.get('error') or (r.get('plan') or {}).get('question'),
                  flush=True)
            self.json(r)
            return
        if self.path == '/api/command/transcribe':     # the page's recording -> text, Whisper on the robot
            dev = str(body.get('device') or '')[:64]
            last = Handler._stt_last.get(dev, 0)
            if not self.is_admin() and (not dev or time.time() - last < 2.0):
                self.json({'ok': False, 'error': 'one recording every couple of seconds, please'})
                return
            Handler._stt_last[dev] = time.time()
            # people's names come from the places ("Hemant's Desk" -> Hemant): hints for spelling them right
            names = []
            for p in fleet.load_places(AREA):
                names.append(p['name'])
                person = re.sub(r"'s\b.*$|\s+(desk|office|sir)\b.*$", '', p['name'], flags=re.I).strip()
                if person and person.lower() != p['name'].lower():
                    names.append(person)
            r = command_client.transcribe(body.get('audio'), str(body.get('mime') or ''), names)
            print(time.strftime('%H:%M:%S'), 'heard:', repr(r.get('text')) if r.get('ok') else r.get('error'),
                  f"({r.get('ms')} ms)" if r.get('ms') else '', flush=True)
            self.json(r)
            return
        if self.path == '/api/command/run':
            plan = body.get('plan') or {}
            admin = self.is_admin()
            if handbrake_on():
                self.json({'ok': False, 'msg': '🅿 the robot is parked (handbrake on): ask the admin to release it'})
                return
            if not plan.get('understood'):
                self.json({'ok': False, 'msg': 'nothing to run: say the command again'})
                return
            job, msg = self.queue.mission(body.get('name') or ('admin' if admin else ''), body.get('device'),
                                          plan.get('legs'), item=plan.get('item'), summary=plan.get('summary'),
                                          command=body.get('text'), admin=admin)
            self.json({'ok': job is not None, 'msg': msg,
                       **({'id': job['id'], 'token': job['token']} if job else {})})
            return
        if self.path == '/api/vision/describe':       # "what do you see?" (button or voice), one look
            why = self.command_allowed(str(body.get('device') or ''), self.is_admin())
            if why:
                self.json({'ok': False, 'error': why})
                return
            res, err = self.vision.ask('describe', {'asked by': 'the operator' if self.is_admin() else 'someone'})
            text = None
            if res:
                text = res['description']
                if res.get('in_the_way') and res['in_the_way'].lower().strip('.') not in ('nothing', 'none'):
                    text += f" In the way: {res['in_the_way']}"
                try:
                    text += (f" Legroom: about {float(res['clear_ahead_m']):.1f} m clear ahead, path about "
                             f"{float(res['path_width_m']):.2f} m wide.")
                except (KeyError, TypeError, ValueError):
                    pass
            if text:
                print(time.strftime('%H:%M:%S'), 'vision: sees:', text, flush=True)
            self.json({'ok': bool(res), 'text': text, 'error': err})
            return
        if self.path == '/api/trip/confirm':          # LOADED / GOT IT at the current stop of a trip
            ok, msg = self.queue.confirm(body.get('id'), token=body.get('token'), admin=self.is_admin(),
                                         who=body.get('who'))
            self.json({'ok': ok, 'msg': msg})
            return
        # ---- admin only
        if not self.is_admin():
            self.send(401, 'text/plain', b'admin login required')
            return
        if self.path == '/api/handbrake':
            on = bool(body.get('on'))
            set_handbrake(on)
            if on:
                self.queue.admin_took_control('handbrake')   # a trip in progress goes back in line
                self.ros.cancel_all_goals()
                for _ in range(5):
                    self.ros.pub_stop.publish(Twist())
                msg = ('🅿 HANDBRAKE ON: the robot will not move (goals, HOME, manual drive, queue trips, '
                       'auto-relocation and wedge escape are all held)')
            else:
                msg = '🅿 handbrake released: queue trips wait until you RESUME QUEUE'
            self.state.fit_msg = msg
            print(time.strftime('%H:%M:%S'), msg, flush=True)
            self.json({'ok': True, 'on': handbrake_on()})
            return
        moves = self.path in ('/api/goal', '/api/home') or (self.path == '/api/drive' and (
            float(body.get('vx', 0) or 0) or float(body.get('wz', 0) or 0)))
        if moves and handbrake_on():
            self.state.fit_msg = '🅿 handbrake is on: release it to move the robot'
            self.json({'ok': False, 'msg': 'handbrake is on'})
            return
        # manual drive: only a real motion command takes control. The manual tab streams zeros while
        # nothing is held, and each one used to cancel the queue's trip and re-pause the queue.
        if self.path in TAKES_CONTROL and not (self.path == '/api/drive' and not (
                float(body.get('vx', 0) or 0) or float(body.get('wz', 0) or 0))):
            self.queue.admin_took_control(TAKES_CONTROL[self.path])
        if self.path in ('/api/use_map', '/api/map_start'):
            self.queue.clear('map changed: open requests cancelled (places belong to the old map)', go_home=False)
        if self.path == '/api/vision/hazard':          # hazard spotter on/off (admin)
            self.vision.set_hazard(bool(body.get('on')))
            self.state.fit_msg = f'hazard spotter {"ON" if self.vision.hazard_on() else "OFF"}'
            self.json({'ok': True, 'on': self.vision.hazard_on()})
            return
        if self.path == '/api/face/identify':
            # on-demand "who's at the crate" — used when loading a parcel (pickup). Log only.
            r = face_client.identify()
            if r.get('ok') and r.get('identified'):
                msg = f"face: {r['person_id']} ({r['score']})"
            elif r.get('ok'):
                msg = f"face: {r.get('reason', 'no match')}"
            else:
                msg = f"face: unavailable ({r.get('error', 'service off')})"
            self.state.fit_msg = msg
            print(time.strftime('%H:%M:%S'), msg, flush=True)
            self.json({'ok': bool(r.get('ok')), 'msg': msg, 'result': r})
            return
        if self.path == '/api/logout':
            self.auth.logout(self.token())
            self.json({'ok': True}, headers=(('Set-Cookie', 'cb_admin=; Path=/; Max-Age=0'),))
        elif self.path == '/api/places/add':
            if body.get('here'):
                p = self.ros.pose()
                if p is None:
                    self.json({'ok': False, 'msg': 'the robot has no position yet'})
                    return
                x, y, yaw = p
            else:
                snapped = snap_reachable(self.ros, body['x'], body['y']) or snap_goal(body['x'], body['y'])
                if snapped is None or snapped[0] is None:
                    self.json({'ok': False, 'msg': 'no reachable floor near there'})
                    return
                x, y, yaw = snapped[0], snapped[1], None
            place, msg = fleet.add_place(AREA, body.get('name', ''), x, y, yaw)
            self.state.fit_msg = msg
            self.json({'ok': place is not None, 'msg': msg})
        elif self.path == '/api/places/move':
            snapped = snap_reachable(self.ros, body['x'], body['y']) or snap_goal(body['x'], body['y'])
            if snapped is None or snapped[0] is None:
                self.json({'ok': False, 'msg': 'no reachable floor near there: pin not moved'})
                return
            place, msg = fleet.move_place(AREA, body.get('id'), snapped[0], snapped[1])
            self.state.fit_msg = msg
            self.json({'ok': place is not None, 'msg': msg})
        elif self.path == '/api/places/delete':
            self.json({'ok': fleet.delete_place(AREA, body.get('id'))})
        elif self.path == '/api/queue/cancel':
            self.json({'ok': self.queue.cancel(body.get('id'), admin=True)})
        elif self.path == '/api/queue/clear':
            self.json({'ok': True, 'n': self.queue.clear()})
        elif self.path == '/api/queue/pause':
            self.queue.set_paused(bool(body.get('paused')))
            self.json({'ok': True})
        elif self.path == '/api/drive':
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
            elif 'lines' in body:   # the eraser sends the whole edited set back
                lines = [[float(v) for v in seg] for seg in body['lines'] if len(seg) == 4]
            else:
                for a, b in zip(pts, pts[1:]):
                    lines.append([a[0], a[1], b[0], b[1]])
            fence_map.save_fence(AREA, lines)
            fence_map.bake(AREA)
            refresh_map()
            msg = self.ros.reload_map(os.path.join(PROJECT, 'maps', f'{AREA}_fenced.yaml'))
            self.state.fit_msg = f'{len(lines)} fence segment(s) — {msg}'
            self.send(200, 'application/json', b'{"ok":true}')
        elif self.path == '/api/stuck':
            # nodes/slip_watch.py: the steps say the robot is driving but the lidar says it isn't (pushing
            # against something below the scan plane, or wheel slip). Stop the push, then put the pose back
            # where the robot really is, before AMCL follows the step count off the map.
            try:
                x, y, yaw = float(body['x']), float(body['y']), float(body['yaw'])
            except (KeyError, TypeError, ValueError):
                self.send(400, 'text/plain', b'x, y, yaw required')
                return
            claimed = max(0.0, float(body.get('claimed') or 0.3))
            what = str(body.get('what') or 'pushing against something the lidar can\'t see')[:120]
            in_queue = self.queue.blocked_now(f'stuck: {what}')
            self.ros.cancel_all_goals()
            for _ in range(3):
                self.ros.pub_stop.publish(Twist())

            def refit():
                radius = min(1.5, max(0.5, claimed + 0.3))      # cover however far the steps claimed
                r = subprocess.run(['python3', os.path.join(PROJECT, 'tools', 'fit_pose.py'),
                                    f'{x:.3f}', f'{y:.3f}', f'{math.degrees(yaw):.1f}',
                                    '--radius', f'{radius:.2f}', '--angle', '30'],
                                   capture_output=True, text=True)
                try:
                    fx, fy, fyaw, score = (float(v) for v in r.stdout.split())
                except ValueError:
                    fx = fy = fyaw = None
                    score = 0.0
                if fx is not None and score >= 0.5:
                    self.ros.send_pose(fx, fy, fyaw)
                    how = f'pose re-fitted to the walls ({score:.0%})'
                else:                                  # the lidar said it didn't move: last good pose
                    self.ros.send_pose(x, y, yaw)
                    how = 'pose put back where it got stuck (no confident fit)'
                msg = (f'⚠ stuck: {what}. The wheels claimed {claimed:.2f} m the lidar didn\'t see; '
                       f'{"trip will retry" if in_queue else "goal cancelled"}, {how}')
                self.state.fit_msg = msg[:200]
                print(time.strftime('%H:%M:%S'), msg, flush=True)
            threading.Thread(target=refit, daemon=True).start()
            self.json({'ok': True, 'queue': in_queue})
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
                    # search around a spot that is ON this map: where the robot thinks it is, else
                    # its last saved pose, else HOME. The old default, the mapping start (0,0), is
                    # off the asdf3 map and got set as the pose when the fit failed (2026-10-13).
                    gx, gy, gyaw = pose_now if pose_now else (None, None, None)
                    if gx is None:
                        try:
                            lp = yaml.safe_load(open(os.path.expanduser(f'~/.campus_bot/run/last_pose_{AREA}.yaml')))
                            gx, gy, gyaw = float(lp['x']), float(lp['y']), float(lp['yaw'])
                        except (OSError, KeyError, TypeError, ValueError):
                            gx, gy, gyaw, _ = get_home(AREA)
                    r = subprocess.run(['bash', os.path.join(PROJECT, 'run', 'reset_pose.sh')] +
                                       (['--wide'] if mode == 'wide' else []) +
                                       [f'{gx:.3f}', f'{gy:.3f}', f'{math.degrees(gyaw):.1f}'],
                                       capture_output=True, text=True)
                    lines = [ln.strip() for ln in (r.stdout + r.stderr).splitlines() if 'fit' in ln or '✗' in ln]
                    self.state.fit_msg = (lines[-1] if lines else 'fit finished')[:120]
            threading.Thread(target=run, daemon=True).start()
            self.send(200, 'application/json', b'{"ok":true}')
        else:
            self.send(404, 'text/plain', b'not here')


class DualStackServer(ThreadingHTTPServer):
    """IPv6 and IPv4 on one socket. resp-desktop.local resolves to both, and a Mac that tried the
    IPv6 address found nothing listening (dashboard was IPv4-only) on the Airtel network (2026-10-13)."""
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


class TLSDualStackServer(DualStackServer):
    """The same pages over https (HTTPS_PORT), so phones and laptops will give the page the microphone for
    voice commands. Self-signed cert shared with the enrolment page (~/.campus_bot/tls, made by the face
    service). The handshake runs in the request thread, so a browser stuck on the cert warning can't block
    everyone else."""
    daemon_threads = True

    def __init__(self, addr, handler, ctx):
        self.ctx = ctx
        super().__init__(addr, handler)

    def get_request(self):
        sock, addr = self.socket.accept()
        sock.settimeout(30)
        return self.ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False), addr

    def finish_request(self, request, client_address):
        import ssl
        try:
            request.do_handshake()
            request.settimeout(None)            # long-lived MJPEG streams must not time out
        except (ssl.SSLError, OSError):
            return
        super().finish_request(request, client_address)

    def handle_error(self, request, client_address):
        pass


def start_https():
    import ssl
    crt, key = (os.path.expanduser(f'~/.campus_bot/tls/enroll.{x}') for x in ('crt', 'key'))
    if not (os.path.exists(crt) and os.path.exists(key)):
        print(f'https dashboard off: no cert in ~/.campus_bot/tls yet (run/face.sh on makes it)', flush=True)
        return None
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(crt, key)
    srv = TLSDualStackServer(('::', HTTPS_PORT), Handler, ctx)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'dashboard also on https://{socket.gethostname()}.local:{HTTPS_PORT} (voice commands need https)',
          flush=True)
    return srv


def main():
    rclpy.init()
    ros = Ros()
    threading.Thread(target=lambda: rclpy.spin(ros), daemon=True).start()
    Handler.ros = ros
    Handler.cam = Camera(ros)
    Handler.state = State()
    Handler.auth = fleet.Auth()
    nav = fleet.Navigator(ros)

    def qlog(m):
        print(time.strftime('%H:%M:%S'), m, flush=True)
        Handler.state.fit_msg = m

    Handler.vision = Vision(ros, qlog)
    Handler.queue = fleet.Queue(ros, nav, lambda: AREA, lambda: get_home(AREA),
                                lambda: ros.pose() is not None and not pid_alive('slam'), qlog,
                                snap_fn=lambda x, y: snap_reachable(ros, x, y) or snap_goal(x, y),
                                face_fn=face_identify,
                                vision_fn=lambda mode, ctx: Handler.vision.ask(mode, ctx),
                                brake_fn=handbrake_on)
    threading.Thread(target=brake_guard, args=(ros, qlog), daemon=True).start()
    if handbrake_on():
        print('🅿 handbrake is ON (parked): the robot will not move until it is released', flush=True)
    if not fleet.Auth.configured():
        print('no admin password yet: python3 tools/set_admin_password.py (the call page works anyway)',
              flush=True)
    srv = DualStackServer(('::', PORT), Handler)
    print(f'dashboard on http://0.0.0.0:{PORT}  (map: {AREA})', flush=True)
    try:
        start_https()
    except Exception as e:                      # never let https trouble take the plain dashboard down
        print(f'https dashboard off: {e}', flush=True)
    threading.Thread(target=public_ip_watch, daemon=True).start()
    try:
        rsrv = ThreadingHTTPServer(('127.0.0.1', REMOTE_PORT), type('RemoteHandler', (Handler,), {'remote': True}))
        rsrv.daemon_threads = True
        threading.Thread(target=rsrv.serve_forever, daemon=True).start()
        print(f'remote (VPS) copy on 127.0.0.1:{REMOTE_PORT}: login required for everything', flush=True)
    except OSError as e:
        print(f'remote copy off: {e}', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
