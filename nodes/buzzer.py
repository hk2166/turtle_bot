#!/usr/bin/env python3
# Written 2026-10-09: the robot's voice. 2026-10-10: two ACTIVE buzzers, front (GPIO12) and rear
# (GPIO13), each fed through 4 x 20 ohm (salvaged from the LED strip) = ~15 mA from the 3.3 V pin.
"""Plays a sound for what the robot is doing, so people nearby know without looking at a screen.

    python3 nodes/buzzer.py                 run (started at boot by run/autostart.sh when BUZZER=1)
    python3 nodes/buzzer.py --test          play every sound once, with its name
    python3 nodes/buzzer.py --dry [--test]  print instead of sounding (no hardware needed)

One-off sounds: hello (node start), ready (robot knows where it is), go (new goal), arrived,
failed (goal given up), stuck (a recovery move starts), found (position set by camera/user),
battery (low, every 60 s; /battery_state once a voltage sensor is fitted).
While it lasts: reversing (beep-beep on the REAR buzzer: the back has no bumper), blocked (double
pip on the FRONT every 4 s: on a goal but not moving), moving (tiny FRONT tick every 5 s on a goal;
BUZZER_MOVING=0 turns it off). Everything else plays on both.
BUZZER_TYPE=active (default): the buzzers make their own tone, so sounds are rhythms (full-on PWM).
BUZZER_TYPE=passive: 50% square waves, real notes (12 mm transducers are loudest near 2.7 kHz).
Hardware: /run/campus-bot-buzzer/{front,rear} -> RP1 PWM0 channels 0/1 (setup/08_buzzer.sh).
"""
import argparse
import os
import queue
import threading
import time

PWM_DIR = '/run/campus-bot-buzzer'
TYPE = os.environ.get('BUZZER_TYPE', 'active')
# BUZZER_LINES: which buzzers are wired ("front", or "front,rear"). A sound meant for a missing one
# plays on the ones that exist, so a single buzzer still gives every sound.
BOTH = tuple(n for n in os.environ.get('BUZZER_LINES', 'front,rear').replace(' ', '').split(',') if n) or ('front',)
WHERE = {'reversing': ('rear',), 'blocked': ('front',), 'moving': ('front',)}   # default: all wired


def lines_for(name):
    want = [n for n in WHERE.get(name, BOTH) if n in BOTH]
    return tuple(want) or BOTH

# passive: (frequency Hz or 0 for a rest, seconds)
NOTES = {
    'hello':    [(2093, .12), (2637, .12), (3136, .12), (4186, .25)],
    'ready':    [(3136, .08), (0, .04), (4186, .15)],
    'go':       [(2093, .08), (2637, .12)],
    'arrived':  [(2093, .10), (2637, .10), (3136, .10), (4186, .30)],
    'failed':   [(3136, .15), (2637, .15), (2093, .15), (1568, .45)],
    'stuck':    [(2637, .20), (0, .08), (1976, .35)],
    'found':    [(3136, .06), (0, .03), (3520, .06)],
    'battery':  [(1568, .30), (0, .15), (1568, .30)],
    'reversing': [(2700, .40), (0, .40)],
    'blocked':  [(2700, .06), (0, .06), (2700, .06)],
    'moving':   [(3500, .025)],
}
# active: (1 = on / 0 = off, seconds); each one a rhythm you can tell apart by ear
B, _ = 1, 0
RHYTHMS = {
    'hello':    [(B, .08), (_, .08), (B, .08), (_, .08), (B, .08), (_, .08), (B, .30)],   # · · · —
    'ready':    [(B, .05), (_, .06), (B, .05)],                                         # ··
    'go':       [(B, .10)],                                                             # ·
    'arrived':  [(B, .08), (_, .06), (B, .08), (_, .06), (B, .40)],                     # · · —
    'failed':   [(B, .60), (_, .15), (B, .60)],                                         # — —
    'stuck':    [(B, .25), (_, .10), (B, .06), (_, .10), (B, .25)],                     # — · —
    'found':    [(B, .03), (_, .05), (B, .03)],                                         # tiny ··
    'battery':  [(B, .50), (_, .50), (B, .50), (_, .50), (B, .50)],                     # slow — — —
    'reversing': [(B, .40), (_, .40)],
    'blocked':  [(B, .06), (_, .10), (B, .06)],
    'moving':   [(B, .02)],
}
SOUNDS = RHYTHMS if TYPE == 'active' else NOTES


class Pwm:
    """One buzzer line. Active buzzer: steady high (100% duty) = sound. Passive: 50% square wave."""
    def __init__(self, path, dry):
        self.path, self.dry, self.on = path, dry, False

    def _w(self, name, v):
        with open(os.path.join(self.path, name), 'w') as f:
            f.write(str(v))

    def tone(self, hz):
        if self.dry:
            return
        if hz <= 0:
            self.off()
            return
        if TYPE == 'active':
            period, duty = 1_000_000, 1_000_000        # 1 kHz carrier at 100% = plain DC "on"
        else:
            period = int(1e9 / hz)
            duty = period // 2
        self._w('duty_cycle', 0)                      # duty must never exceed the new period
        self._w('period', period)
        self._w('duty_cycle', duty)
        if not self.on:
            self._w('enable', 1)
            self.on = True

    def off(self):
        if not self.dry and self.on:
            self._w('duty_cycle', 0)
            self._w('enable', 0)
        self.on = False


class Player:
    def __init__(self, dry):
        missing = not os.path.exists(os.path.join(PWM_DIR, 'front'))
        if missing and not dry:
            print(f'no {PWM_DIR}/front (run sudo setup/08_buzzer.sh and reboot): printing sounds instead',
                  flush=True)
        self.dry = dry or missing
        self.lines = {n: Pwm(os.path.join(PWM_DIR, n), self.dry) for n in BOTH}
        self.q = queue.Queue()
        self.playing = False
        threading.Thread(target=self._run, daemon=True).start()

    def play(self, name):
        if self.dry:
            print(f'{time.strftime("%H:%M:%S")} ♪ {name} ({"+".join(lines_for(name))})', flush=True)
        self.q.put(name)

    def off(self):
        for line in self.lines.values():
            line.off()

    def _run(self):
        while True:
            name = self.q.get()
            self.playing = True
            where = [self.lines[n] for n in lines_for(name)]
            try:
                for hz, sec in SOUNDS[name]:
                    for line in where:
                        line.tone(hz)
                    time.sleep(sec)
            finally:
                self.off()
                self.playing = False

    def busy(self):
        return self.playing or not self.q.empty()


def run_ros(player, moving_ticks):
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import BatteryState

    ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)

    class Buzzer(Node):
        def __init__(self):
            super().__init__('buzzer')
            self.goals = {}            # goal id -> last status (navigate_to_pose)
            self.recovering = set()    # behaviour action names currently running
            self.rec_goals = {}
            self.goal_active = False
            self.cmd_vx, self.cmd_t = 0.0, 0.0
            self.speed, self.speed_t = 0.0, 0.0
            self.still_since = None
            self.next_pattern = 0.0
            self.ready = False
            self.batt_next = 0.0
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav, 10)
            for b in ('spin', 'backup', 'wait', 'drive_on_heading'):
                self.create_subscription(GoalStatusArray, f'/{b}/_action/status',
                                         lambda m, b=b: self.on_recovery(b, m), 10)
            self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                     lambda m: player.play('found'), 10)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
            self.create_timer(0.1, self.tick)
            player.play('hello')

        def on_nav(self, m):
            for s in m.status_list:
                gid = bytes(s.goal_info.goal_id.uuid)
                old = self.goals.get(gid)
                if old == s.status:
                    continue
                self.goals[gid] = s.status
                if old is None and s.status in ACTIVE:
                    player.play('go')
                elif s.status == GoalStatus.STATUS_SUCCEEDED and old in ACTIVE:
                    player.play('arrived')
                elif s.status == GoalStatus.STATUS_ABORTED and old in ACTIVE:
                    player.play('failed')
            self.goal_active = any(v in ACTIVE for v in self.goals.values())
            if len(self.goals) > 50:
                self.goals = {k: v for k, v in self.goals.items() if v in ACTIVE}

        def on_recovery(self, name, m):
            running = any(s.status in ACTIVE for s in m.status_list)
            if running and name not in self.recovering and name != 'wait':
                player.play('stuck')
            (self.recovering.add if running else self.recovering.discard)(name)

        def on_cmd(self, m):
            self.cmd_vx, self.cmd_t = m.linear.x, time.monotonic()

        def on_odom(self, m):
            t = m.twist.twist
            self.speed, self.speed_t = abs(t.linear.x) + 0.3 * abs(t.angular.z), time.monotonic()

        def on_amcl(self, _):
            if not self.ready:
                self.ready = True
                player.play('ready')

        def on_battery(self, m):
            now = time.monotonic()
            low = m.voltage > 1 and m.voltage < float(os.environ.get('BATT_WARN_V', '25.6'))
            if low and now >= self.batt_next:
                player.play('battery')
                self.batt_next = now + 60

        def tick(self):
            now = time.monotonic()
            reversing = self.cmd_vx < -0.02 and now - self.cmd_t < 0.5
            moving = self.speed > 0.03 and now - self.speed_t < 1.0
            if self.goal_active and not moving and not self.recovering:
                self.still_since = self.still_since or now
            else:
                self.still_since = None
            if player.busy() or now < self.next_pattern:
                return
            if reversing:
                player.play('reversing'); self.next_pattern = now + 0.8
            elif self.still_since and now - self.still_since > 2.5:
                player.play('blocked'); self.next_pattern = now + 4.0
            elif moving_ticks and self.goal_active and moving:
                player.play('moving'); self.next_pattern = now + 5.0

    rclpy.init()
    node = Buzzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        player.off()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--test', action='store_true', help='play every sound once')
    ap.add_argument('--dry', action='store_true', help='print instead of sounding')
    a = ap.parse_args()
    player = Player(a.dry)
    if a.test:
        for name in SOUNDS:
            print(f'{name:10s} {" ".join(f"{hz}Hz/{int(s*1000)}ms" if hz else f"rest/{int(s*1000)}ms" for hz, s in SOUNDS[name])}', flush=True)
            player.play(name)
            while player.busy():
                time.sleep(0.05)
            time.sleep(0.6)
        player.off()
        return
    run_ros(player, os.environ.get('BUZZER_MOVING', '1') == '1')


if __name__ == '__main__':
    main()
