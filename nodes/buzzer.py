#!/usr/bin/env python3
# Written 2026-10-09: the robot's voice. 5 passive buzzers in parallel on one transistor, GPIO12 PWM.
"""Plays a sound for what the robot is doing, so people nearby know without looking at a screen.

    python3 nodes/buzzer.py                 run (started at boot by run/autostart.sh when BUZZER=1)
    python3 nodes/buzzer.py --test          play every sound once, with its name
    python3 nodes/buzzer.py --dry [--test]  print instead of sounding (no hardware needed)

One-off sounds: hello (node start), ready (robot knows where it is), go (new goal), arrived,
failed (goal given up), stuck (a recovery move starts), found (position set by camera/user),
battery (low, every 60 s; /battery_state once a voltage sensor is fitted).
While it lasts: reversing (beep-beep, the back has no bumper), blocked (double pip every 4 s:
on a goal but not moving), moving (tiny tick every 5 s on a goal; BUZZER_MOVING=0 turns it off).
Hardware: /run/campus-bot-buzzer -> RP1 PWM0 channel 0 (setup/08_buzzer.sh). 50% duty square wave;
12 mm transducers are loudest near 2.7 kHz, so the tunes stay between 1.5 and 4.2 kHz.
"""
import argparse
import os
import queue
import threading
import time

PWM = '/run/campus-bot-buzzer'

# (frequency Hz or 0 for a rest, seconds)
SOUNDS = {
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


class Pwm:
    """The buzzer line: square wave of a frequency, or silence."""
    def __init__(self, dry):
        self.dry = dry or not os.path.exists(PWM)
        self.on = False
        if self.dry and not dry:
            print(f'no {PWM} (run sudo setup/08_buzzer.sh and reboot): printing sounds instead', flush=True)

    def _w(self, name, v):
        with open(os.path.join(PWM, name), 'w') as f:
            f.write(str(v))

    def tone(self, hz):
        if self.dry:
            return
        if hz <= 0:
            self.off()
            return
        period = int(1e9 / hz)
        self._w('duty_cycle', 0)          # duty must never exceed the new period
        self._w('period', period)
        self._w('duty_cycle', period // 2)
        if not self.on:
            self._w('enable', 1)
            self.on = True

    def off(self):
        if not self.dry and self.on:
            self._w('enable', 0)
        self.on = False


class Player:
    def __init__(self, dry):
        self.pwm = Pwm(dry)
        self.q = queue.Queue()
        self.playing = False
        threading.Thread(target=self._run, daemon=True).start()

    def play(self, name):
        if self.pwm.dry:
            print(f'{time.strftime("%H:%M:%S")} ♪ {name}', flush=True)
        self.q.put(name)

    def _run(self):
        while True:
            name = self.q.get()
            self.playing = True
            try:
                for hz, sec in SOUNDS[name]:
                    self.pwm.tone(hz)
                    time.sleep(sec)
            finally:
                self.pwm.off()
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
        player.pwm.off()
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
        player.pwm.off()
        return
    run_ros(player, os.environ.get('BUZZER_MOVING', '1') == '1')


if __name__ == '__main__':
    main()
