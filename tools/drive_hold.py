#!/usr/bin/env python3
# Written 2026-10-01 (new tool, not a reconstruction).
"""Hold-to-drive: the robot moves ONLY while a key is physically held down.

A terminal only ever sees key *presses* (plus auto-repeat), so terminal teleops have to guess
"released" from a timeout. This opens a small window instead, which gets real key-down / key-up
events from the desktop:

  key down  → drive          key up  → stop          window loses focus → stop

  ↑ / W  forward      ↓ / S  reverse      ← / A  turn left      → / D  turn right
  hold two (e.g. ↑ + ←) to arc            SPACE  stop (keys held at the time stay ignored until let go)
  + / -  speed ±0.05 m/s                  Esc or closing the window: ramp to a stop, then quit

Starting and stopping follow the tower-stability ramp (≤ 0.5 m/s²), so "stop" takes up to about
half a second from full mapping speed. That is the hard limit in CLAUDE.md, not a timer. While
moving it republishes /cmd_vel at 20 Hz, well inside the ESP32's 300 ms watchdog. When stopped it
goes quiet so it never fights Nav2.

Run it through run/drive.sh (that also starts the micro-ROS agent), or alone if the agent is up:
  python3 tools/drive_hold.py [--speed 0.25] [--turn 0.6]
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import load_env  # noqa: E402

KEYS = {  # keysym (lower case): (linear sign, angular sign)
    'up': (1, 0), 'w': (1, 0), 'down': (-1, 0), 's': (-1, 0),
    'left': (0, 1), 'a': (0, 1), 'right': (0, -1), 'd': (0, -1),
}
RATE_HZ = 20
# X/XWayland auto-repeat turns one long hold into release+press pairs a few ms apart. A release is
# only believed if no press of the same key follows within this window.
REPEAT_FILTER_MS = 40
# Remote desktop (xrdp): if the WiFi drops while a key is held, the key-up never arrives and this
# window, which runs ON the robot, would keep driving (the ESP32 watchdog can't help: commands keep
# coming). So in an xrdp session the link to the remote computer is checked while moving: the RDP
# connection must show traffic (data or ACKs) within LINK_LOST_MS, or every key counts as released.
RDP_PORT = int(os.environ.get('CAMPUS_RDP_PORT', '3389'))
LINK_LOST_MS = 1000
LINK_CHECK_S = 0.2
SPINNER = '◐◓◑◒'


def rdp_link_age_ms(port=RDP_PORT):
    """ms since the remote-desktop client last sent data or ACKed ours; None = no RDP connection."""
    try:
        out = subprocess.run(['ss', '-tinH', 'state', 'established', f'( sport = :{port} )'],
                             capture_output=True, text=True, timeout=0.5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    ages = []
    for conn in re.split(r'\n(?=\S)', out.strip()):       # one entry per connection (info lines indented)
        rcv, ack = re.search(r'lastrcv:(\d+)', conn), re.search(r'lastack:(\d+)', conn)
        if rcv or ack:
            ages.append(min(int(m.group(1)) for m in (rcv, ack) if m))
    return min(ages) if ages else None


def ramp(cur, tgt, step):
    return cur + max(-step, min(step, tgt - cur))


class HoldDrive:
    def __init__(self, args, env):
        self.max_lin = float(env.get('MAX_LIN_SPEED', 0.8))
        self.max_ang = float(env.get('MAX_ANG_SPEED', 1.2))
        self.acc_lin = min(float(env.get('MAX_LIN_ACCEL', 0.5)), 0.5)   # never above 0.5 m/s²
        self.acc_ang = float(env.get('MAX_ANG_ACCEL', 1.0))
        self.speed = min(args.speed, self.max_lin)
        self.turn = min(args.turn, self.max_ang)
        self.held = set()
        self.pending_release = {}
        self.v = self.w = 0.0
        self.quiet = 0          # zero messages still to send after stopping
        self.bumper = False
        self.odom = None
        self.closing = False
        self.close_deadline = 0.0
        self.blocked = set()    # keys still held when SPACE was pressed: ignored until really released
        self.remote = bool(os.environ.get('XRDP_SESSION'))   # running inside an xrdp remote desktop
        self.link_lost = False
        self.next_link_check = 0.0
        self.spin = 0

        rclpy.init()
        self.node = rclpy.create_node('drive_hold')
        self.pub = self.node.create_publisher(Twist, env.get('CMD_VEL_TOPIC', '/cmd_vel'), 10)
        self.node.create_subscription(Odometry, env.get('ODOM_TOPIC', '/odom'),
                                      lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        self.node.create_subscription(Bool, '/bumper_tripped', lambda m: setattr(self, 'bumper', m.data), 10)

        self.root = tk.Tk()
        self.root.title('Campus Bot: hold to drive')
        self.root.geometry('460x300')
        self.root.configure(bg='#202020')
        font = ('DejaVu Sans Mono', 13)
        self.big = tk.Label(self.root, text='STOPPED', font=('DejaVu Sans', 28, 'bold'), bg='#202020', fg='#cccccc')
        self.big.pack(pady=(18, 6))
        self.status = tk.Label(self.root, font=font, bg='#202020', fg='#dddddd', justify='left')
        self.status.pack(pady=4)
        tk.Label(self.root, bg='#202020', fg='#888888', font=('DejaVu Sans', 10), justify='left',
                 text='Hold ↑↓←→ / WASD to drive. Let go = stop.  SPACE stop.  +/- speed.  Esc quit.\n'
                      'Keep this window focused: clicking away stops the robot.').pack(side='bottom', pady=10)

        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.bind('<FocusOut>', lambda e: self.release_all())
        self.root.protocol('WM_DELETE_WINDOW', self.quit)
        self.last_tick = time.monotonic()
        # ROS callbacks run in their own thread so they never slow the 20 Hz loop below. It has its own
        # executor and is stopped and joined before the node is destroyed (destroying a node under a
        # running rclpy.spin() aborts the whole process).
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spinning = True
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()
        self.root.after(int(1000 / RATE_HZ), self.tick)
        self.root.after(300, self._grab_focus)
        # Tk swallows KeyboardInterrupt inside callbacks, so Ctrl-C in the terminal (or a kill from
        # run/stop.sh) would otherwise leave the window up. Route both to the normal stop-and-quit.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: self.root.after(0, self.quit))

    def _grab_focus(self):
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.focus_force()
        self.root.after(500, lambda: self.root.attributes('-topmost', False))

    def _spin(self):
        while self.spinning:
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:   # context shut down underneath us
                return

    # ---------------- keyboard
    def on_press(self, e):
        k = e.keysym.lower()
        if k in self.pending_release:              # auto-repeat: the "release" was fake
            self.root.after_cancel(self.pending_release.pop(k))
        if self.closing:
            return
        if k in KEYS:
            if k not in self.blocked:
                self.held.add(k)
        elif k == 'space':
            # Stop: drop every target and block the keys still held (their auto-repeat would restart
            # the robot) until they're really released. The loop ramps to zero at ≤ 0.5 m/s², not a
            # step to zero, which would be the tip-over risk the ramp exists for.
            self.blocked |= self.held
            self.held.clear()
        elif k in ('plus', 'equal', 'kp_add'):
            self.speed = min(self.max_lin, round(self.speed + 0.05, 2))
        elif k in ('minus', 'kp_subtract'):
            self.speed = max(0.05, round(self.speed - 0.05, 2))
        elif k == 'escape':
            self.quit()

    def on_release(self, e):
        k = e.keysym.lower()
        if k in KEYS and k not in self.pending_release:
            self.pending_release[k] = self.root.after(REPEAT_FILTER_MS, self._really_released, k)

    def _really_released(self, k):
        self.pending_release.pop(k, None)
        self.held.discard(k)
        self.blocked.discard(k)

    def release_all(self):
        for job in self.pending_release.values():
            self.root.after_cancel(job)
        self.pending_release.clear()
        self.held.clear()

    # ---------------- control loop
    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.1)     # real elapsed time, so the ramp is exact at any loop rate
        self.last_tick = now
        if self.remote and (self.held or self.v or self.w) and now >= self.next_link_check:
            self.next_link_check = now + LINK_CHECK_S
            self.check_link()
        lin = sum(KEYS[k][0] for k in self.held)
        ang = sum(KEYS[k][1] for k in self.held)
        tv = self.speed * max(-1, min(1, lin))
        tw = self.turn * max(-1, min(1, ang))
        if self.bumper and tv > 0:
            tv = 0.0                               # bumper: back off or turn only
        self.v = ramp(self.v, tv, self.acc_lin * dt)
        self.w = ramp(self.w, tw, self.acc_ang * dt)
        if abs(self.v) < 1e-3:
            self.v = 0.0
        if abs(self.w) < 1e-3:
            self.w = 0.0

        moving = self.v != 0.0 or self.w != 0.0
        if moving:
            self.quiet = 5                         # after stopping, send a few zeros, then go silent
        if moving or self.quiet > 0:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.v, self.w
            self.pub.publish(msg)
            if not moving:
                self.quiet -= 1
        if self.closing and (not moving or now > self.close_deadline):
            self._finish()
            return
        self.draw(moving)
        self.root.after(int(1000 / RATE_HZ), self.tick)

    def check_link(self):
        age = rdp_link_age_ms()
        lost = age is None or age > LINK_LOST_MS
        if lost and not self.link_lost:
            # Treat every held key as released (ramped stop) and ignore them until really re-pressed.
            for job in self.pending_release.values():
                self.root.after_cancel(job)
            self.pending_release.clear()
            self.blocked |= self.held
            self.held.clear()
            print(f'remote link lost ({"no RDP connection" if age is None else f"{age} ms silent"}): stopping',
                  flush=True)
        self.link_lost = lost

    def draw(self, moving):
        listeners = self.pub.get_subscription_count()
        if moving:
            self.spin += 1          # changes every tick while moving: keeps remote-desktop traffic (and ACKs) flowing
        live = SPINNER[self.spin % len(SPINNER)] if moving else ''
        if self.link_lost:
            self.big.config(text='LINK LOST ' + live, fg='#ff5050')
        elif self.bumper:
            self.big.config(text='BUMPER', fg='#ff5050')
        elif moving:
            self.big.config(text=('DRIVING ' if self.held else 'STOPPING ') + live,
                            fg='#50e050' if self.held else '#e0c050')
        else:
            self.big.config(text='STOPPED', fg='#cccccc')
        lines = [f'cmd   v {self.v:+.2f} m/s   w {self.w:+.2f} rad/s',
                 f'max   {self.speed:.2f} m/s   {self.turn:.2f} rad/s',
                 f'keys  {" ".join(sorted(self.held)) or "-"}']
        if self.odom:
            p = self.odom.pose.pose.position
            lines.append(f'odom  x {p.x:+.2f}  y {p.y:+.2f}')
        lines.append('ESP32 listening on /cmd_vel' if listeners else '!! nobody subscribed to /cmd_vel (agent down?)')
        if self.remote:
            lines.append('remote desktop: stops if the link is silent > 1 s')
        self.status.config(text='\n'.join(lines), fg='#dddddd' if listeners else '#ff9050')

    def quit(self):
        """Esc, closing the window, Ctrl-C: ramp to a stop (tick() does it), then close."""
        if self.closing:
            return
        self.closing = True
        self.release_all()
        self.close_deadline = time.monotonic() + 3.0   # 0.8 m/s at 0.5 m/s² takes 1.6 s
        self.big.config(text='STOPPING', fg='#e0c050')

    def _finish(self):
        for _ in range(5):
            self.pub.publish(Twist())
        self.root.destroy()

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.pub.publish(Twist())
            finally:
                self.spinning = False
                self.spin_thread.join(timeout=2.0)
                self.executor.shutdown()
                self.node.destroy_node()
                rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=None, help='drive speed m/s (default MAP_LIN_SPEED)')
    ap.add_argument('--turn', type=float, default=None, help='turn speed rad/s (default MAP_ANG_SPEED)')
    args = ap.parse_args()
    env = load_env()
    args.speed = args.speed if args.speed is not None else float(env.get('MAP_LIN_SPEED', 0.25))
    args.turn = args.turn if args.turn is not None else float(env.get('MAP_ANG_SPEED', 0.5))
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        sys.exit('No display: run this on the Pi desktop (monitor or xrdp), not over plain SSH.')
    HoldDrive(args, env).run()


if __name__ == '__main__':
    main()
