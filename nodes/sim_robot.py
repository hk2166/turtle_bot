#!/usr/bin/env python3
# Written 2026-09-30 for testing without hardware (new file, not a reconstruction).
"""Fake Campus Bot: stands in for the RPLIDAR C1 + ESP32 so the mapping flow can be tested on the Pi
with nothing plugged in.   run/remap.sh <area> --sim   starts it instead of the lidar driver and the
micro-ROS agent.

Publishes what the real robot does:
  /scan          LaserScan, 10 Hz, 500 beams, frame `laser`, 12 m range, including the 4 column
                 slivers (so the mask and preflight are exercised)
  /odom          Odometry, 20 Hz: distance 3% over-read, heading from the gyro (so SLAM has real work)
                 Turns are only 85% efficient (carpet slip), like the real robot's turn-slip problem.
  /sim/ground_truth  the true pose (Odometry), for checking tools such as calibrate_odom.py
  /imu/data_raw  Imu, 20 Hz, gyro with noise
Subscribes /cmd_vel (Twist) with the firmware's 300 ms watchdog and 0.5 m/s² ramp.
Does NOT broadcast odom → base_link TF (remap.sh then starts odom_to_tf.py, like it may on the robot).
The world is a made-up 16 × 10 m office: walls, a glass-free meeting room, desk blocks, planters.
"""
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


def box(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def world_segments():
    s = box(-2, -3, 14, 7)                                  # outer walls
    s += [(6, 3, 6, 7), (6, 3, 8, 3), (9, 3, 11, 3), (11, 3, 11, 7)]   # meeting room, door 8–9 m
    for x in (1.0, 3.5):                                     # desk rows
        s += box(x, -1.5, x + 1.6, -0.7)
        s += box(x, 1.2, x + 1.6, 2.0)
    s += box(7.5, -1.8, 8.3, -1.0) + box(12.5, -2.6, 13.4, -1.8)   # planters / cabinets
    s += box(12.8, 4.5, 13.6, 6.6)                           # café counter
    s += [(-2, 5.0, 0.5, 5.0)]                               # partition
    return np.array(s, dtype=float)


# 2020 columns at the corners of the 400×300 plate (robot frame), 20 mm square.
COLUMNS = [(0.17, 0.12), (0.17, -0.12), (-0.17, 0.12), (-0.17, -0.12)]


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')
        self.world = world_segments()
        self.beams = 500
        self.x, self.y, self.th = 0.0, 0.0, 0.0             # true pose
        self.ox, self.oy, self.oth = 0.0, 0.0, 0.0          # odometry estimate
        self.v = self.w = self.w_true = 0.0
        self.cmd = (0.0, 0.0)
        self.last_cmd = -1.0
        self.slip_lin = 1.03                                 # odom over-reads distance by 3%
        self.turn_eff = 0.85                                 # carpet: wheels turn the robot only 85% as far
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)   # for testing tools
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(0.05, self.publish_odom)
        self.get_logger().info('SIMULATED robot running (no hardware).')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd = (max(-0.8, min(0.8, msg.linear.x)), max(-2.0, min(2.0, msg.angular.z)))
        self.last_cmd = self.now_s()

    def blocked(self, x, y):
        a, b = self.world[:, :2], self.world[:, 2:]
        p = np.array([x, y])
        ab = b - a
        t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-9), 0, 1)
        d = np.linalg.norm(a + ab * t[:, None] - p, axis=1)
        return d.min() < 0.26

    def step(self):
        tv, tw = self.cmd if self.now_s() - self.last_cmd < 0.3 else (0.0, 0.0)   # 300 ms watchdog
        dv, dw = 0.5 * self.dt, 1.5 * self.dt
        self.v += max(-dv, min(dv, tv - self.v))
        self.w += max(-dw, min(dw, tw - self.w))
        nx = self.x + self.v * math.cos(self.th) * self.dt
        ny = self.y + self.v * math.sin(self.th) * self.dt
        if self.blocked(nx, ny):                             # bumper
            self.v = 0.0
        else:
            self.x, self.y = nx, ny
        w_true = self.w * self.turn_eff
        self.th += w_true * self.dt
        self.w_true = w_true
        # Odometry: distance from wheel steps (with scale error), heading from the gyro (like the firmware).
        ov = self.v * self.slip_lin * (1 + random.gauss(0, 0.01))
        ow = w_true + random.gauss(0, 0.002)
        self.oth += ow * self.dt
        self.ox += ov * math.cos(self.oth) * self.dt
        self.oy += ov * math.sin(self.oth) * self.dt

    def publish_scan(self):
        segs = list(self.world)
        c, s = math.cos(self.th), math.sin(self.th)
        for cx, cy in COLUMNS:
            for x0, y0, x1, y1 in box(cx - 0.01, cy - 0.01, cx + 0.01, cy + 0.01):
                segs.append((self.x + c * x0 - s * y0, self.y + s * x0 + c * y0,
                             self.x + c * x1 - s * y1, self.y + s * x1 + c * y1))
        segs = np.array(segs)
        inc = 2 * math.pi / self.beams
        ang = -math.pi + inc * np.arange(self.beams) + self.th
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)            # N×2
        a, e = segs[:, :2], segs[:, 2:] - segs[:, :2]               # M×2
        o = np.array([self.x, self.y])
        denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]   # N×M
        ao = a - o
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (ao[None, :, 0] * e[None, :, 1] - ao[None, :, 1] * e[None, :, 0]) / denom
            u = (ao[None, :, 0] * d[:, None, 1] - ao[None, :, 1] * d[:, None, 0]) / denom
        t = np.where((np.abs(denom) > 1e-12) & (t > 0.05) & (u >= 0) & (u <= 1), t, np.inf)
        r = t.min(axis=1)
        r = np.where(r <= 12.0, r + np.random.normal(0, 0.01, r.shape), np.inf)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min, msg.angle_max, msg.angle_increment = -math.pi, math.pi - inc, inc
        msg.scan_time, msg.time_increment = 0.1, 0.1 / self.beams
        msg.range_min, msg.range_max = 0.05, 12.0
        msg.ranges = r.astype(float).tolist()
        self.scan_pub.publish(msg)

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.ox, self.oy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.oth / 2), math.cos(self.oth / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = now, 'imu'
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = self.w_true + random.gauss(0, 0.003)
        imu.angular_velocity.x, imu.angular_velocity.y = random.gauss(0, 0.003), random.gauss(0, 0.003)
        imu.linear_acceleration.z = 9.81 + random.gauss(0, 0.02)
        self.imu_pub.publish(imu)
        g = Odometry()
        g.header.stamp, g.header.frame_id, g.child_frame_id = now, 'map', 'base_link'
        g.pose.pose.position.x, g.pose.pose.position.y = self.x, self.y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.truth_pub.publish(g)


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
