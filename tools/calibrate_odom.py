#!/usr/bin/env python3
# Written 2026-09-30 (new tool, not a reconstruction): the calibration CLAUDE.md asks for.
"""Wheel calibration: drive 5 m straight → wheel diameter; spin 10 turns → wheel separation.

Needs the sensors up (run/remap.sh, or the bring-up alone) and clear space around the robot.
Any key or Ctrl-C ramps the robot to a stop (≤ 0.5 m/s²).

  python3 tools/calibrate_odom.py straight [--distance 5] [--wheel-dia 0.100]
      Put tape on the floor at the front caster, drive, then measure how far it REALLY went.
      New WHEEL_DIA = WHEEL_DIA × measured / odom. It also reports how much the robot curved
      (a sign of the left and right wheels being unequal).

  python3 tools/calibrate_odom.py spin [--turns 10] [--wheel-sep 0.350]
      Mark the robot's heading with tape. It spins until the gyro says N turns, then asks you to
      confirm the real count. Commanded vs real rotation gives the EFFECTIVE wheel separation,
      which includes carpet slip (the last session measured turns only ~60% efficient on carpet).
      New WHEEL_SEP = WHEEL_SEP × commanded / real.

Pass the values currently in the firmware (--wheel-dia / --wheel-sep) to get the new ones directly.
Every run is appended to ~/.campus_bot/logs/calibrate_odom.csv.
"""
import argparse
import csv
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'nodes'))
from campus_common import load_env  # noqa: E402

LOG = os.path.expanduser('~/.campus_bot/logs/calibrate_odom.csv')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def find_topic(node, preferred, type_name):
    topics = dict(node.get_topic_names_and_types())
    if preferred in topics and type_name in topics[preferred]:
        return preferred
    return next((n for n, t in topics.items() if type_name in t and not n.startswith('/sim/')), None)


class Cal:
    def __init__(self, args, env):
        self.args = args
        self.node = rclpy.create_node('calibrate_odom')
        n = self.node
        t0 = time.monotonic()   # DDS discovery can take a few seconds: keep looking
        self.odom_t = None
        while self.odom_t is None and time.monotonic() - t0 < 6.0:
            rclpy.spin_once(n, timeout_sec=0.2)
            self.odom_t = find_topic(n, env.get('ODOM_TOPIC', '/odom'), 'nav_msgs/msg/Odometry')
        self.imu_t = find_topic(n, env.get('IMU_TOPIC', '/imu/data_raw'), 'sensor_msgs/msg/Imu')
        if not self.odom_t:
            sys.exit('No odometry topic. Start the sensors first (run/remap.sh or launch/bringup.launch.py).')
        self.cmd_t = env.get('CMD_VEL_TOPIC', '/cmd_vel')
        self.pub = n.create_publisher(Twist, self.cmd_t, 10)
        self.odom = self.truth = None
        self.gyro_z = None
        n.create_subscription(Odometry, self.odom_t, lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        if self.imu_t:
            n.create_subscription(Imu, self.imu_t, lambda m: setattr(self, 'gyro_z', m.angular_velocity.z),
                                  qos_profile_sensor_data)
        if args.truth_topic:
            n.create_subscription(Odometry, args.truth_topic, lambda m: setattr(self, 'truth', m), 10)
        t0 = time.monotonic()
        while (self.odom is None or (args.truth_topic and self.truth is None)) and time.monotonic() - t0 < 5:
            rclpy.spin_once(n, timeout_sec=0.1)
        if self.odom is None:
            sys.exit(f'No messages on {self.odom_t}.')
        if n.count_subscribers(self.cmd_t) == 0:
            sys.exit(f'Nobody listens on {self.cmd_t}: the ESP32 isn\'t taking commands.')
        print(f'odom {self.odom_t} · gyro {self.imu_t or "none (using odom heading)"} · commands {self.cmd_t}')

    def send(self, v):
        """Publish the ramped command: m/s for 'straight', turns/s (→ rad/s) for 'spin'."""
        msg = Twist()
        if self.args.test == 'straight':
            msg.linear.x = v
        else:
            msg.angular.z = v * 2 * math.pi
        self.pub.publish(msg)

    def pose(self, m):
        p = m.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def drive(self, target, rate, accel, measure):
        """Ramp up to `rate`, hold, and ramp down so that measure() (progress, same units as target)
        ends at `target`. Returns (commanded integral, aborted)."""
        tty_ok = sys.stdin.isatty()
        fd = sys.stdin.fileno() if tty_ok else None
        old = termios.tcgetattr(fd) if tty_ok else None
        v = cmd_int = 0.0
        aborted = False
        last = time.monotonic()
        # Ctrl-C during the run = "any key": ramp down like any other stop (rclpy's own handler would
        # have shut ROS down first, so no stop command could be sent).
        self.interrupted = False
        prev_int = signal.signal(signal.SIGINT, lambda *_: setattr(self, 'interrupted', True))
        try:
            if tty_ok:
                tty.setcbreak(fd)
            while True:
                for _ in range(20):          # every waiting callback, so odom/gyro are fresh (not ~250 ms old)
                    rclpy.spin_once(self.node, timeout_sec=0.0)
                if tty_ok and select.select([fd], [], [], 0)[0]:
                    os.read(fd, 64)          # the whole key (an arrow key is 3 bytes)
                    aborted = True
                if self.interrupted:
                    aborted = True
                now = time.monotonic()
                dt = now - last
                last = now
                done = measure()
                remaining = target - done
                stop_dist = v * v / (2 * accel)
                want = 0.0 if (aborted or remaining <= stop_dist) else rate
                v = v + max(-accel * dt, min(accel * dt, want - v))
                if want == 0.0 and v < 0.02 and (aborted or remaining < target * 0.002 or v <= 0):
                    break
                v = max(v, 0.0)
                cmd_int += v * dt
                self.send(v)
                print(f'\r  {done:7.3f} / {target:.3f}   cmd {v:5.2f}   (any key = stop)  ', end='', flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            aborted = True
        finally:
            signal.signal(signal.SIGINT, prev_int)
            if tty_ok:                       # terminal first: it must come back even if the sends fail
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                termios.tcflush(fd, termios.TCIFLUSH)   # leftover key bytes would land in the next input()
            try:
                for _ in range(5):
                    self.send(0.0)
                    time.sleep(0.05)
            except Exception as e:
                print(f'\n  ! could not send the stop command ({e}); the ESP32 watchdog stops it within 300 ms')
        print()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:          # let the robot settle and the last odom arrive
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return cmd_int, aborted

    def ask_float(self, prompt, default):
        if self.args.yes:
            print(f'{prompt} {default:.3f}')
            return default
        while True:
            try:
                s = input(prompt + f' [{default:.3f}] ').strip()
            except EOFError:
                s = ''
            if not s:
                return default
            try:
                return float(s)
            except ValueError:
                print('  a number, please (for example 4.97)')


def straight(cal, args):
    x0, y0, th0 = cal.pose(cal.odom)
    tx0 = cal.pose(cal.truth) if cal.truth else None
    heading = {'start': th0}

    def progress():
        x, y, _ = cal.pose(cal.odom)
        return math.hypot(x - x0, y - y0)

    print(f'Driving {args.distance} m straight at {args.speed} m/s. Mark the floor at the front caster now.')
    if not args.yes:
        input('  Clear path? Press Enter to go … ')
    cmd_d, aborted = cal.drive(args.distance, args.speed, args.accel, progress)
    x, y, th = cal.pose(cal.odom)
    odom_d = math.hypot(x - x0, y - y0)
    curve = math.degrees(wrap(th - heading['start']))
    default = odom_d
    if cal.truth and tx0:
        tx, ty, _ = cal.pose(cal.truth)
        default = math.hypot(tx - tx0[0], ty - tx0[1])
    real = cal.ask_float('  Measured distance with a tape (m)?', default)
    scale = real / odom_d if odom_d > 0.05 and real > 0 else float('nan')
    if scale != scale:
        print('  ! too little distance (odometry or measured) to compute a factor: nothing to apply')
    print(f'\n  commanded {cmd_d:.3f} m · odometry {odom_d:.3f} m · measured {real:.3f} m'
          f'{"  (ABORTED early: still valid, just less precise)" if aborted else ""}')
    print(f'  odometry scale error: {100 * (1 / scale - 1):+.1f}%  → multiply WHEEL_DIA by {scale:.4f}')
    if args.wheel_dia:
        print(f'  WHEEL_DIA {args.wheel_dia:.4f} → {args.wheel_dia * scale:.4f} m')
    print(f'  heading change while driving "straight": {curve:+.1f}° (gyro/odom)')
    if abs(curve) > 2 and odom_d > 1:
        sep = args.wheel_sep or 0.35
        diff = math.radians(curve) * sep / odom_d
        side = 'RIGHT' if curve > 0 else 'LEFT'
        print(f'  → it curves {"left" if curve > 0 else "right"}: the {side} wheel travels ~{100 * abs(diff):.1f}% '
              f'further (unequal wheels, or one motor losing steps)')
    return {'test': 'straight', 'target': args.distance, 'commanded': cmd_d, 'odom': odom_d, 'real': real,
            'factor': scale, 'curve_deg': curve, 'aborted': aborted}


def spin(cal, args):
    state = {'imu': 0.0, 'odom': 0.0, 'last_odom_yaw': cal.pose(cal.odom)[2], 'truth': 0.0,
             'last_truth_yaw': cal.pose(cal.truth)[2] if cal.truth else 0.0, 't': time.monotonic()}

    def progress():
        now = time.monotonic()
        dt = now - state['t']
        state['t'] = now
        if cal.gyro_z is not None:
            state['imu'] += cal.gyro_z * dt
        yaw = cal.pose(cal.odom)[2]
        state['odom'] += wrap(yaw - state['last_odom_yaw'])
        state['last_odom_yaw'] = yaw
        if cal.truth:
            ty = cal.pose(cal.truth)[2]
            state['truth'] += wrap(ty - state['last_truth_yaw'])
            state['last_truth_yaw'] = ty
        return (state['imu'] if cal.gyro_z is not None else state['odom']) / (2 * math.pi)

    print(f'Spinning {args.turns} turns (counter-clockwise) at {args.rate} rad/s. Mark the heading with tape.')
    if not args.yes:
        input('  Clear space (≥ 0.5 m all round)? Press Enter to go … ')
    # drive() works in "turns": rate and accel converted to turns/s.
    k = 2 * math.pi
    cmd_turns, aborted = cal.drive(args.turns, args.rate / k, args.ang_accel / k, progress)
    progress()
    gyro_turns = state['imu'] / k if cal.gyro_z is not None else None
    odom_turns = state['odom'] / k
    default = state['truth'] / k if cal.truth else (gyro_turns if gyro_turns is not None else odom_turns)
    real = cal.ask_float('  How many turns did it REALLY do (by the tape mark)?', default)
    factor = cmd_turns / real if real > 0.05 and cmd_turns > 0.05 else float('nan')
    if factor != factor:
        print('  ! too little rotation (commanded or real) to compute a factor: nothing to apply')
    print(f'\n  commanded {cmd_turns:.3f} turns · gyro {gyro_turns if gyro_turns is None else round(gyro_turns, 3)}'
          f' · odom {odom_turns:.3f} · real {real:.3f}{"  (ABORTED early)" if aborted else ""}')
    print(f'  turn efficiency {100 / factor:.0f}% (the wheels have to be told {factor:.3f}× the rotation)')
    print(f'  → multiply WHEEL_SEP by {factor:.4f} (effective separation, includes floor slip)')
    if args.wheel_sep:
        print(f'  WHEEL_SEP {args.wheel_sep:.4f} → {args.wheel_sep * factor:.4f} m')
    if gyro_turns is not None and abs(gyro_turns - real) / max(real, 1e-6) > 0.02:
        print(f'  ! the gyro counted {gyro_turns:.3f} vs {real:.3f} real: gyro scale off by '
              f'{100 * (gyro_turns / real - 1):+.1f}%. Heading comes from the gyro, so fix that in the firmware too.')
    print('  Repeat on the office carpet AND on hard floor: slip differs; use the carpet value for mapping.')
    return {'test': 'spin', 'target': args.turns, 'commanded': cmd_turns, 'odom': odom_turns, 'real': real,
            'factor': factor, 'gyro': gyro_turns, 'aborted': aborted}


def main():
    env = load_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['straight', 'spin'])
    ap.add_argument('--distance', type=float, default=5.0)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--turns', type=float, default=10.0)
    ap.add_argument('--rate', type=float, default=0.5, help='spin rate, rad/s')
    ap.add_argument('--accel', type=float, default=float(env.get('MAX_LIN_ACCEL', 0.5)))
    ap.add_argument('--ang-accel', type=float, default=float(env.get('MAX_ANG_ACCEL', 1.0)))
    ap.add_argument('--wheel-dia', type=float, help='WHEEL_DIA currently in the firmware (m)')
    ap.add_argument('--wheel-sep', type=float, help='WHEEL_SEP currently in the firmware (m)')
    ap.add_argument('--truth-topic', help='for testing with the simulator: /sim/ground_truth')
    ap.add_argument('--yes', action='store_true', help="don't prompt (accept defaults; for testing)")
    args = ap.parse_args()
    args.accel = min(args.accel, 0.5)
    args.speed = min(args.speed, 0.4)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        cal = Cal(args, env)
        res = straight(cal, args) if args.test == 'straight' else spin(cal, args)
    except KeyboardInterrupt:        # at a prompt: nothing is moving
        print('\n  cancelled, nothing logged')
        rclpy.try_shutdown()
        sys.exit(130)
    cal.node.destroy_node()
    rclpy.try_shutdown()
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    new = not os.path.exists(LOG)
    with open(LOG, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['time', 'test', 'target', 'commanded', 'odom', 'real', 'factor', 'extra', 'aborted'])
        extra = res.get('curve_deg', res.get('gyro'))
        w.writerow([datetime.now().isoformat(timespec='seconds'), res['test'], res['target'],
                    f"{res['commanded']:.4f}", f"{res['odom']:.4f}", f"{res['real']:.4f}", f"{res['factor']:.4f}",
                    extra, res['aborted']])
    print(f'  logged to {LOG}')


if __name__ == '__main__':
    main()
