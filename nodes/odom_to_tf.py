#!/usr/bin/env python3
# RECONSTRUCTED (2026-09-30) — not yet run on the robot.
"""Broadcast odom → base_link on /tf from the ESP32's /odom messages.

Only started by run/remap.sh when nothing else provides that transform (it isn't known whether the
original firmware broadcast it itself). Running two publishers of the same transform would make the
TF tree jump, so remap.sh probes first.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'{topic} → TF {self.odom_frame} → {self.base_frame}')

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # If the firmware's clock isn't synced (stamp 0), use ours, or TF lookups will fail.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
