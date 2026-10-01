#!/usr/bin/env python3
# RECONSTRUCTED (2026-09-30) — not yet run on the robot.
"""Keyboard driving for mapping, with the robot's motion limits built in.

Why not teleop_twist_keyboard: it publishes once per key press (the ESP32's 300 ms watchdog then
stops the robot between key repeats), it jumps straight to full speed (tower stability needs
≤ 0.5 m/s²), and it uses `q` for speed-up, while run/remap.sh uses `q` for "finish and save".

Hold a key to drive; let go and the robot ramps to a stop. Commands are republished at 20 Hz while
moving, then the node goes quiet so it never fights Nav2 for /cmd_vel.

Exit status: 0 after `q` (save the map), 130 after Ctrl-C.
"""
import argparse
import math
import os
import re
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

HELP = """
 Drive (hold the key):          u  i  o      i / ,   forward / back   (or the arrow keys)
                                j  k  l      j / l   turn left / right
                                m  ,  .      u o m . arcs
 k or SPACE  stop               + / -   drive speed ±10%     ] / [   turn speed ±10%
 q  stop, FINISH and SAVE the map          Ctrl-C  stop and quit
"""

MOVES = {  # key: (linear sign, angular sign)
    'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), 'o': (1, -1), 'm': (-1, -1), '.': (-1, 1),
}


# Arrow keys arrive as ESC [ A..D (or ESC O A..D). Map them to the drive keys and drop any other
# escape sequence, so its bytes ('[', 'A', …) are never read as commands.
ESCAPES = re.compile(r'\x1b[\[O][0-9;]*[A-Za-z~]')
ARROWS = {'A': 'i', 'B': ',', 'C': 'l', 'D': 'j'}


def read_keys(fd):
    """Everything waiting on the terminal, as drive keys. Reads the fd directly: select() can't see
    bytes already pulled into sys.stdin's buffer, which made each keypress act on an older key."""
    data = os.read(fd, 64)
    while data.endswith(b'\x1b') or re.search(rb'\x1b[\[O][0-9;]*$', data):   # escape split across reads
        if not select.select([fd], [], [], 0.02)[0]:
            break
        more = os.read(fd, 64)
        if not more:        # end of input
            break
        data += more
    text = data.decode(errors='ignore')
    text = ESCAPES.sub(lambda m: ARROWS.get(m.group(0)[-1], '') if len(m.group(0)) == 3 or m.group(0)[1] == 'O'
                       else '', text)
    text = re.sub(r'\x1b[\[O]?[0-9;]*$', '', text)   # an escape cut off at the end: drop it
    return text.replace('\x1b', '')


def ramp(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/cmd_vel')
    ap.add_argument('--speed', type=float, default=0.25, help='starting drive speed, m/s')
    ap.add_argument('--turn', type=float, default=0.5, help='starting turn speed, rad/s')
    ap.add_argument('--max-speed', type=float, default=0.8)
    ap.add_argument('--max-turn', type=float, default=1.2)
    ap.add_argument('--accel', type=float, default=0.5, help='m/s² (≤ 0.5: tower stability)')
    ap.add_argument('--ang-accel', type=float, default=1.0, help='rad/s²')
    ap.add_argument('--purpose', choices=['map', 'drive'], default='map',
                    help="map: q finishes and saves the map (run/remap.sh); drive: q just quits (run/drive.sh)")
    ap.add_argument('--hold', type=float, default=0.6,
                    help='s a key press keeps driving (must cover the keyboard auto-repeat delay)')
    args, ros_args = ap.parse_known_args()
    if not sys.stdin.isatty():
        sys.exit('teleop_keys.py needs an interactive terminal.')
    accel = min(args.accel, 0.5)

    # No rclpy signal handler: on Ctrl-C it would shut the context down before the `finally` below
    # could ramp the robot to a stop (every publish there failed silently). Python's KeyboardInterrupt
    # still arrives.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('teleop_keys')
    pub = node.create_publisher(Twist, args.topic, 10)

    speed, turn = min(args.speed, args.max_speed), min(args.turn, args.max_turn)
    target_v = target_w = v = w = 0.0
    hold_until = 0.0
    quiet_since = time.monotonic()
    exit_code = 0
    dt = 0.05
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP if args.purpose == 'map' else
          HELP.replace('q  stop, FINISH and SAVE the map', 'q  stop and quit                  '))
    try:
        tty.setcbreak(fd)
        last = time.monotonic()
        while rclpy.ok():
            ready, _, _ = select.select([fd], [], [], dt)
            now = time.monotonic()
            finish = False
            for key in (read_keys(fd) if ready else ''):
                if key in MOVES:
                    ls, as_ = MOVES[key]
                    target_v, target_w = ls * speed, as_ * turn
                    hold_until = now + args.hold
                elif key in ('k', ' '):
                    target_v = target_w = 0.0
                    hold_until = 0.0
                elif key in ('+', '='):
                    speed = min(speed * 1.1, args.max_speed)
                elif key in ('-', '_'):
                    speed = max(speed / 1.1, 0.05)
                elif key == ']':
                    turn = min(turn * 1.1, args.max_turn)
                elif key == '[':
                    turn = max(turn / 1.1, 0.1)
                elif key == 'q':
                    finish = True
                    break
            if finish:
                break
            if now > hold_until:
                target_v = target_w = 0.0
            step = now - last
            last = now
            v = ramp(v, target_v, accel * step)
            w = ramp(w, target_w, args.ang_accel * step)
            moving = abs(v) > 1e-4 or abs(w) > 1e-4 or target_v or target_w
            if moving:
                quiet_since = now
            if moving or now - quiet_since < 0.5:   # a few zeros after stopping, then go quiet
                msg = Twist()
                msg.linear.x, msg.angular.z = (v if abs(v) > 1e-4 else 0.0), (w if abs(w) > 1e-4 else 0.0)
                pub.publish(msg)
            sys.stdout.write(f'\r  drive {speed:4.2f} m/s  turn {turn:4.2f} rad/s  |  now v={v:+5.2f} '
                             f'w={w:+5.2f} ({math.degrees(w):+4.0f}°/s)   ')
            sys.stdout.flush()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # Ramp down (≤ accel) rather than slam to zero, then send zeros.
        try:
            while abs(v) > 1e-3 or abs(w) > 1e-3:
                v = ramp(v, 0.0, accel * dt)
                w = ramp(w, 0.0, args.ang_accel * dt)
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                pub.publish(msg)
                time.sleep(dt)
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(dt)
        except Exception:
            pass
        print('\n  stopped.')
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
