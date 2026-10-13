# Written 2026-10-13: office-wide use of the robot (admin password, named places, request queue).
"""Admin login, pinned places and the "call the robot" queue, used by nodes/dashboard.py.

Admin: the mission-control panel needs a password (set it on the robot with
    python3 tools/set_admin_password.py
stored salted + hashed in ~/.campus_bot/admin.yaml, never in the repo). A login gives a 12 h session
cookie. Requests from the robot itself (127.0.0.1) count as admin, so scripts on the Pi keep working.

Places: admin pins named spots per map in config/places_<map>.yaml ("Hemant's desk", "Pantry").

Queue (anyone on the office WiFi, page /call): enter your name, pick a place, call the robot.
- One open request per person: per device AND per name, so nobody stacks requests.
- First come, first served. The robot drives to the place, waits until the caller taps GOT IT (or
  ARRIVE_WAIT seconds), then takes the next request; with none left it drives home.
- The admin can cancel any request, clear the queue, pause/resume it. Any admin control of the robot
  (goal, STOP, manual drive, map switch) pauses the queue and puts a trip in progress back at the front.
Persisted in ~/.campus_bot/run/queue.json, so a dashboard restart keeps the line.
"""
import hashlib
import hmac
import json
import math
import os
import re
import secrets
import threading
import time

import yaml

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADMIN_FILE = os.path.expanduser('~/.campus_bot/admin.yaml')
QUEUE_FILE = os.path.expanduser('~/.campus_bot/run/queue.json')
HOLD_FILE = os.path.expanduser('~/.campus_bot/run/hold_position')   # fresh = auto_clear stays put
ARRIVE_WAIT = float(os.environ.get('ARRIVE_WAIT', '120'))
# A blocked trip is retried: Nav2 already fights for ~1-2 min (12 recovery rounds, 1 Hz replans);
# when it still gives up, the blocker is usually a person or a parked chair, which moves. So wait
# RETRY_DELAY and try again, TRIP_TRIES attempts in all, re-snapping the goal to floor that is
# reachable at that moment (someone standing on the exact spot moves the goal next to them).
RETRY_DELAY = float(os.environ.get('RETRY_DELAY', '45'))
TRIP_TRIES = int(os.environ.get('TRIP_TRIES', '3'))
SESSION_S = 12 * 3600
MAX_WAITING = 20
OPEN = ('waiting', 'driving', 'arrived')


# ------------------------------------------------------------------------------------------ admin auth
def hash_password(password, salt=None, iterations=200_000):
    salt = salt or secrets.token_hex(16)
    h = hashlib.pbkdf2_hmac('sha256', password.encode(), bytes.fromhex(salt), iterations).hex()
    return {'salt': salt, 'iterations': iterations, 'hash': h}


class Auth:
    def __init__(self):
        self.sessions = {}            # token -> expiry
        self.lock = threading.Lock()

    @staticmethod
    def configured():
        return os.path.exists(ADMIN_FILE)

    def login(self, password):
        try:
            rec = yaml.safe_load(open(ADMIN_FILE))
            h = hash_password(password, rec['salt'], int(rec['iterations']))['hash']
        except (OSError, KeyError, TypeError, ValueError):
            return None
        if not hmac.compare_digest(h, rec['hash']):
            time.sleep(1.0)           # slow down guessing
            return None
        token = secrets.token_urlsafe(32)
        with self.lock:
            now = time.time()
            self.sessions = {t: e for t, e in self.sessions.items() if e > now}
            self.sessions[token] = now + SESSION_S
        return token

    def logout(self, token):
        with self.lock:
            self.sessions.pop(token, None)

    def valid(self, token):
        with self.lock:
            return bool(token) and self.sessions.get(token, 0) > time.time()


# ------------------------------------------------------------------------------------------ places
def places_file(area):
    return os.path.join(PROJECT, 'config', f'places_{area}.yaml')


def load_places(area):
    try:
        return (yaml.safe_load(open(places_file(area))) or {}).get('places') or []
    except OSError:
        return []


def save_places(area, places):
    with open(places_file(area), 'w') as f:
        f.write(f'# named places for maps/{area}, pinned on the dashboard (admin); the call page lists them\n')
        yaml.safe_dump({'places': places}, f, sort_keys=False, allow_unicode=True)


def add_place(area, name, x, y, yaw=None):
    name = re.sub(r'\s+', ' ', str(name)).strip()[:40]
    if not name:
        return None, 'name required'
    places = load_places(area)
    if any(p['name'].lower() == name.lower() for p in places):
        return None, f'"{name}" already exists'
    base = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-') or 'place'
    pid, k = base, 2
    while any(p['id'] == pid for p in places):
        pid, k = f'{base}-{k}', k + 1
    p = {'id': pid, 'name': name, 'x': round(float(x), 3), 'y': round(float(y), 3),
         'yaw': None if yaw is None else round(float(yaw), 4)}
    places.append(p)
    save_places(area, places)
    return p, f'pinned "{name}" at ({p["x"]:.2f}, {p["y"]:.2f})'


def move_place(area, pid, x, y):
    """Admin dragged a pin: new spot, same name/id. The facing is dropped (set again with PIN ROBOT
    SPOT if it matters); requests already in the queue keep the spot they were made for."""
    places = load_places(area)
    p = next((p for p in places if p['id'] == pid), None)
    if p is None:
        return None, 'that pin no longer exists'
    p.update(x=round(float(x), 3), y=round(float(y), 3), yaw=None)
    save_places(area, places)
    return p, f'moved "{p["name"]}" to ({p["x"]:.2f}, {p["y"]:.2f})'


def delete_place(area, pid):
    places = load_places(area)
    keep = [p for p in places if p['id'] != pid]
    save_places(area, keep)
    return len(keep) < len(places)


# ------------------------------------------------------------------------------------------ queue
class Navigator:
    """Nav2 NavigateToPose through its action, so the queue knows how *its* goal ended."""
    def __init__(self, node):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self.NavigateToPose = NavigateToPose
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.handle = None
        self.result = None            # None while running; 'succeeded' | 'aborted' | 'canceled' | 'rejected'
        self.seq = 0

    def ready(self):
        return self.client.server_is_ready()

    def go(self, x, y, yaw):
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        g = self.NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.seq += 1
        seq = self.seq
        self.result, self.handle = None, None
        names = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                 GoalStatus.STATUS_CANCELED: 'canceled'}

        def on_result(f):
            if seq == self.seq:
                try:
                    self.result = names.get(f.result().status, 'aborted')
                except Exception:
                    self.result = 'aborted'

        def on_goal(f):
            if seq != self.seq:
                return
            try:
                h = f.result()
            except Exception:
                h = None
            if h is None or not h.accepted:
                self.result = 'rejected'
                return
            self.handle = h
            h.get_result_async().add_done_callback(on_result)

        self.client.send_goal_async(g).add_done_callback(on_goal)

    def cancel(self):
        self.seq += 1                 # ignore whatever the old goal reports
        if self.handle is not None:
            try:
                self.handle.cancel_goal_async()
            except Exception:
                pass
        self.handle, self.result = None, None


class Queue:
    def __init__(self, ros, nav, area_fn, home_fn, ready_fn, log_fn, snap_fn=None):
        self.ros, self.nav = ros, nav
        self.area_fn, self.home_fn, self.ready_fn, self.log = area_fn, home_fn, ready_fn, log_fn
        self.snap_fn = snap_fn        # (x, y) -> (x', y', moved) on currently reachable floor, or None
        self.lock = threading.RLock()
        self.jobs, self.paused, self.msg, self.homing = [], False, '', False
        self.want_home = False         # a trip ended: drive home once the robot is free (not only within 5 s)
        self._load()
        for j in self.jobs:            # a restart mid-trip: that trip starts again from the front
            if j['status'] == 'driving':
                j['status'] = 'waiting'
            elif j['status'] == 'arrived':     # already delivered: don't drive back to them
                self._finish(j, 'done', 'dashboard restarted while waiting for GOT IT')
        threading.Thread(target=self._run, daemon=True).start()

    # ---- persistence
    def _load(self):
        try:
            d = json.load(open(QUEUE_FILE))
            self.jobs, self.paused = d.get('jobs', []), bool(d.get('paused', False))
        except (OSError, ValueError):
            pass

    def _save(self):
        os.makedirs(os.path.dirname(QUEUE_FILE), exist_ok=True)
        # finished jobs oldest-first by finish time, so the trim drops the OLDEST (the open jobs are
        # kept in front, which used to leave the finished ones newest-first and trim the newest)
        done = sorted((j for j in self.jobs if j['status'] not in OPEN),
                      key=lambda j: j.get('finished') or 0)[-30:]
        self.jobs = [j for j in self.jobs if j['status'] in OPEN] + done
        with open(QUEUE_FILE + '.tmp', 'w') as f:
            json.dump({'jobs': self.jobs, 'paused': self.paused}, f)
        os.replace(QUEUE_FILE + '.tmp', QUEUE_FILE)

    # ---- queries
    def _open(self):
        return [j for j in self.jobs if j['status'] in OPEN]

    def _position(self, job):
        waiting = [j for j in self.jobs if j['status'] == 'waiting']
        active = any(j['status'] in ('driving', 'arrived') for j in self.jobs)
        return waiting.index(job) + 1 + int(active) if job in waiting else 0

    def robot_status(self):
        if self.paused:
            return 'paused'
        if not self.ready_fn():
            return 'offline'
        return 'busy' if any(j['status'] in OPEN for j in self.jobs) else 'ready'

    def public_view(self, device=''):
        with self.lock:
            lst = [{'id': j['id'], 'name': j['name'], 'place': j['place_name'], 'status': j['status'],
                    'pos': self._position(j)} for j in self._open()]
            # this device's open request, else its most recent one (list order is NOT creation order)
            own = [j for j in self.jobs if device and j['device'] == device]
            mine = next((j for j in own if j['status'] in OPEN), None) or \
                max(own, key=lambda j: j.get('created') or 0, default=None)
            m = None
            if mine is not None and (mine['status'] in OPEN or time.time() - (mine.get('finished') or 0) < 600):
                note = mine.get('note', '')
                left = mine.get('not_before', 0) - time.time()
                if mine['status'] == 'waiting' and left > 0:
                    note = f'path blocked: retrying in {int(left)} s (try {mine["tries"] + 1}/{TRIP_TRIES})'
                m = {'id': mine['id'], 'status': mine['status'], 'place': mine['place_name'],
                     'pos': self._position(mine), 'note': note,
                     'wait_left': max(0, int(ARRIVE_WAIT - (time.time() - mine['arrived'])))
                     if mine['status'] == 'arrived' else None}
            return {'robot': self.robot_status(), 'queue': lst, 'mine': m}

    def admin_view(self):
        with self.lock:
            return {'paused': self.paused, 'msg': self.msg, 'robot': self.robot_status(),
                    'jobs': [{k: j.get(k) for k in ('id', 'name', 'place_name', 'status', 'created', 'note')}
                             | {'pos': self._position(j)} for j in self._recent(25)]}

    def _recent(self, n):
        """All open jobs plus the newest finished ones, oldest first, n in all (open never cut off)."""
        opn = self._open()
        done = sorted((j for j in self.jobs if j['status'] not in OPEN), key=lambda j: j.get('finished') or 0)
        keep = opn + (done[-(n - len(opn)):] if n > len(opn) else [])
        return sorted(keep, key=lambda j: j.get('created') or 0)

    # ---- user actions
    def request(self, name, place_id, device):
        name = re.sub(r'\s+', ' ', str(name or '')).strip()[:40]
        device = str(device or '')[:64]
        if not name:
            return None, 'please enter your name'
        if not device:
            return None, 'reload the page and try again'
        place = next((p for p in load_places(self.area_fn()) if p['id'] == place_id), None)
        if place is None:
            return None, 'that place no longer exists: reload the page'
        with self.lock:
            for j in self._open():
                if j['device'] == device:
                    return None, f'this phone already has an open request (for {j["name"]}): cancel it first'
                if j['name'].lower() == name.lower():
                    return None, f'{j["name"]} already has a request ({j["status"]}): cancel it first'
            if sum(j['status'] == 'waiting' for j in self.jobs) >= MAX_WAITING:
                return None, 'the queue is full right now: try again in a few minutes'
            job = {'id': secrets.token_hex(4), 'name': name, 'device': device, 'token': secrets.token_hex(12),
                   'place_id': place['id'], 'place_name': place['name'], 'x': place['x'], 'y': place['y'],
                   'yaw': place.get('yaw'), 'status': 'waiting', 'created': time.time(),
                   'tries': 0, 'not_before': 0,
                   'started': None, 'arrived': None, 'finished': None, 'note': ''}
            self.jobs.append(job)
            self._save()
            self.log(f'queue: {name} called the robot to {place["name"]}')
            return job, 'ok'

    def _finish(self, job, status, note=''):
        job['status'], job['note'], job['finished'] = status, note, time.time()
        self.want_home = True

    def cancel(self, jid, token=None, admin=False):
        with self.lock:
            j = next((j for j in self.jobs if j['id'] == jid and j['status'] in OPEN), None)
            if j is None or not (admin or hmac.compare_digest(str(token or ''), j['token'])):
                return False
            if j['status'] == 'driving':
                self.nav.cancel()
            self._finish(j, 'cancelled', 'by the admin' if admin else 'by the caller')
            self._save()
            self.log(f'queue: request of {j["name"]} cancelled {j["note"]}')
            return True

    def done(self, jid, token):
        with self.lock:
            j = next((j for j in self.jobs if j['id'] == jid and j['status'] == 'arrived'), None)
            if j is None or not hmac.compare_digest(str(token or ''), j['token']):
                return False
            self._finish(j, 'done', 'confirmed by the caller')
            self._save()
            return True

    # ---- admin actions
    def clear(self, note='queue cleared by the admin', go_home=True):
        with self.lock:
            n = 0
            for j in self._open():
                if j['status'] == 'driving':
                    self.nav.cancel()
                self._finish(j, 'cancelled', note)
                n += 1
            if not go_home:            # map switch: the old map's home means nothing now
                self.want_home = False
            self._save()
            self.log(f'queue: {note} ({n} request(s))')
            return n

    def set_paused(self, paused):
        with self.lock:
            self.paused = bool(paused)
            self.msg = 'paused by the admin' if paused else ''
            self._save()

    def admin_took_control(self, what):
        """The admin drove/stopped/re-targeted the robot: pause, and put a trip in progress back in front."""
        with self.lock:
            self.homing = self.want_home = False
            for j in self.jobs:
                if j['status'] == 'driving':
                    self.nav.cancel()
                    j['status'], j['started'] = 'waiting', None
                    self.jobs.remove(j)
                    first = next((i for i, k in enumerate(self.jobs) if k['status'] == 'waiting'), len(self.jobs))
                    self.jobs.insert(first, j)
                    break
            if any(j['status'] == 'waiting' for j in self.jobs) and not self.paused:
                self.paused = True
                self.msg = f'paused: admin took control ({what}). RESUME QUEUE when done'
                self.log(f'queue: {self.msg}')
            self._save()

    # ---- dispatcher
    def _run(self):
        while True:
            time.sleep(1.0)
            try:
                self._tick()
            except Exception as e:          # never let the dispatcher die
                self.log(f'queue dispatcher error: {e}')

    def _tick(self):
        with self.lock:
            active = next((j for j in self.jobs if j['status'] in ('driving', 'arrived')), None)
            if active and active['status'] == 'driving':
                r = self.nav.result
                if r == 'succeeded':
                    active['status'], active['arrived'] = 'arrived', time.time()
                    self.log(f'queue: arrived at {active["place_name"]} for {active["name"]}')
                    self._save()
                elif r in ('aborted', 'rejected'):
                    active['tries'] = active.get('tries', 0) + 1
                    if active['tries'] < TRIP_TRIES:
                        active['status'] = 'waiting'
                        active['not_before'] = time.time() + RETRY_DELAY
                        active['note'] = f'path blocked: retry {active["tries"] + 1}/{TRIP_TRIES}'
                        self.log(f'queue: path to {active["place_name"]} blocked: retrying in '
                                 f'{int(RETRY_DELAY)} s (try {active["tries"] + 1}/{TRIP_TRIES})')
                    else:
                        self._finish(active, 'failed',
                                     f"couldn't reach the place in {TRIP_TRIES} tries (blocked)")
                        self.log(f'queue: trip to {active["place_name"]} for {active["name"]} failed '
                                 f'after {TRIP_TRIES} tries')
                    self._save()
                elif r == 'canceled':      # someone else (RViz, a script) took the robot: back in line, paused
                    active['status'], active['started'] = 'waiting', None
                    self.paused = True
                    self.msg = 'paused: the trip was cancelled outside the queue. RESUME QUEUE when done'
                    self.log(f'queue: {self.msg}')
                    self._save()
                return
            if active and active['status'] == 'arrived':
                with open(HOLD_FILE, 'w'):     # waiting for the caller: no idle relocation
                    pass
                if time.time() - active['arrived'] > ARRIVE_WAIT:
                    self._finish(active, 'done', f'no GOT IT within {int(ARRIVE_WAIT)} s')
                    self._save()
                return
            if self.paused or not self.ready_fn() or not self.nav.ready():
                return
            now = time.time()
            nxt = next((j for j in self.jobs if j['status'] == 'waiting' and now >= j.get('not_before', 0)), None)
            if nxt is None:
                if any(j['status'] == 'waiting' for j in self.jobs):
                    return                 # only retries cooling down: hold position, don't go home
                if self.want_home and not self.homing:
                    x, y, yaw, _ = self.home_fn()
                    self.nav.go(x, y, yaw)
                    self.homing, self.want_home = True, False
                    self.log('queue: empty, going home')
                return
            self.homing = self.want_home = False
            gx, gy = nxt['x'], nxt['y']
            if self.snap_fn:               # aim at floor reachable NOW (the pinned spot may be occupied)
                try:
                    sn = self.snap_fn(gx, gy)
                    if sn and sn[0] is not None:
                        gx, gy = sn[0], sn[1]
                except Exception:
                    pass
            pose = self.ros.pose()
            yaw = nxt['yaw'] if nxt['yaw'] is not None else (
                math.atan2(gy - pose[1], gx - pose[0]) if pose else 0.0)
            nxt['status'], nxt['started'], nxt['note'] = 'driving', time.time(), ''
            self.nav.go(gx, gy, yaw)
            self.ros.goal = (gx, gy)
            retry = f' (try {nxt["tries"] + 1}/{TRIP_TRIES})' if nxt.get('tries') else ''
            self.log(f'queue: driving to {nxt["place_name"]} for {nxt["name"]}{retry}')
            self._save()
