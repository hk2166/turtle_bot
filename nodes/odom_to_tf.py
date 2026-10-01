#!/usr/bin/env python3
# RECONSTRUCTED (2026-09-30); calibration scaling added 2026-10-01.
"""Broadcast odom → base_link on /tf from the ESP32's /odom messages.

Only started by run/remap.sh when nothing else provides that transform (it isn't known whether the
original firmware broadcast it itself). Running two publishers of the same transform would make the
TF tree jump, so remap.sh probes first.

odom_lin_scale / odom_ang_scale correct the firmware's wheel constants WITHOUT reflashing: the raw
trajectory is re-integrated step by step with each step's distance and rotation scaled
(tools/calibrate_odom.py measures the scales; they live in config/robot.env). Set them back to 1.0
if the firmware is ever reflashed with corrected WHEEL_DIA/WHEEL_SEP.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


MAX_SKEW_NS = 200_000_000   # 0.2 s; the link itself adds ~20–40 ms


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lin_scale = float(self.declare_parameter('odom_lin_scale', 1.0).value)
        self.ang_scale = float(self.declare_parameter('odom_ang_scale', 1.0).value)
        self.last_raw = None          # (x, y, yaw) as the firmware reported it
        self.cx = self.cy = self.cyaw = 0.0
        if self.lin_scale != 1.0 or self.ang_scale != 1.0:
            self.get_logger().info(f'calibration: distances x{self.lin_scale}, rotations x{self.ang_scale}')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'{topic} → TF {self.odom_frame} → {self.base_frame}')
        self.restamping = None

    def on_odom(self, msg):
        t = TransformStamped()
        # Trust the firmware's stamp only if it agrees with our clock. One that isn't synced (zero,
        # or time since boot, i.e. 1970) would put every transform in the past: TF lookups fail and
        # slam_toolbox drops every scan.
        now = self.get_clock().now()
        skewed = abs((now - Time.from_msg(msg.header.stamp, clock_type=now.clock_type)).nanoseconds) > MAX_SKEW_NS
        t.header.stamp = now.to_msg() if skewed else msg.header.stamp
        if skewed != self.restamping:
            self.restamping = skewed
            if skewed:
                self.get_logger().warn('odom stamps are off our clock by > 0.2 s: using the Pi\'s time')
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.lin_scale == 1.0 and self.ang_scale == 1.0:
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, 0.0
            t.transform.rotation = q
        else:
            if self.last_raw is None:
                self.cx, self.cy, self.cyaw = p.x, p.y, yaw
            else:
                lx, ly, lyaw = self.last_raw
                c, s_ = math.cos(-lyaw), math.sin(-lyaw)
                dx = c * (p.x - lx) - s_ * (p.y - ly)          # this step, in the robot's frame
                dy = s_ * (p.x - lx) + c * (p.y - ly)
                dyaw = math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw))
                dx, dy, dyaw = dx * self.lin_scale, dy * self.lin_scale, dyaw * self.ang_scale
                mid = self.cyaw + dyaw / 2.0                   # integrate along the arc's mean heading
                self.cx += math.cos(mid) * dx - math.sin(mid) * dy
                self.cy += math.sin(mid) * dx + math.cos(mid) * dy
                self.cyaw = math.atan2(math.sin(self.cyaw + dyaw), math.cos(self.cyaw + dyaw))
            self.last_raw = (p.x, p.y, yaw)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.cx, self.cy, 0.0
            t.transform.rotation.z = math.sin(self.cyaw / 2.0)
            t.transform.rotation.w = math.cos(self.cyaw / 2.0)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
