# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
"""Sensor bring-up: lidar driver + column filter + micro-ROS agent + static TFs.

Started by run/remap.sh (which reads config/robot.env and passes everything below as arguments).
By hand:  ros2 launch launch/bringup.launch.py lidar_port:=/dev/ttyUSB0 esp32_port:=/dev/ttyACM0
          ros2 launch launch/bringup.launch.py sim:=true       # no hardware: nodes/sim_robot.py

TF tree:  odom → base_link (ESP32 or odom_to_tf.py)  and static  base_link → laser / imu_link / camera
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
NODES = os.path.join(os.path.dirname(HERE), 'nodes')

ARGS = {
    'sim': 'false',
    'lidar_port': '/dev/ttyUSB0', 'lidar_baud': '460800', 'lidar_scan_mode': 'Standard',
    'esp32_port': '/dev/esp32', 'agent_baud': '115200',
    'scan_topic': '/scan', 'scan_filtered_topic': '/scan_filtered',
    'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': '0.215', 'laser_yaw': '0.0',
    'imu_x': '0.0', 'imu_y': '0.0', 'imu_z': '0.06',
    'camera_x': '0.0', 'camera_y': '0.0', 'camera_z': '1.25', 'camera_pitch': '-0.1745',
}


def static_tf(name, child, x, y, z, yaw='0', pitch='0', roll='0'):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=f'static_tf_{name}',
        arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw, '--pitch', pitch, '--roll', roll,
                   '--frame-id', 'base_link', '--child-frame-id', child],
        output='screen')


def generate_launch_description():
    sim = LC('sim')
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            # --- real hardware ---
            Node(
                package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': LC('lidar_port'),
                    'serial_baudrate': LC('lidar_baud'),
                    'frame_id': 'laser',
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': LC('lidar_scan_mode'),
                }],
                remappings=[('scan', LC('scan_topic'))],
                output='screen', condition=UnlessCondition(sim)),
            Node(
                package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
                arguments=['serial', '--dev', LC('esp32_port'), '-b', LC('agent_baud')],
                output='screen', condition=UnlessCondition(sim)),
            # --- no hardware ---
            ExecuteProcess(cmd=['python3', os.path.join(NODES, 'sim_robot.py')],
                           name='sim_robot', output='screen', condition=IfCondition(sim)),
            # --- always ---
            ExecuteProcess(
                cmd=['python3', os.path.join(NODES, 'scan_filter.py'), '--ros-args',
                     '-p', ['in_topic:=', LC('scan_topic')],
                     '-p', ['out_topic:=', LC('scan_filtered_topic')]],
                name='scan_filter', output='screen'),
            static_tf('laser', 'laser', LC('laser_x'), LC('laser_y'), LC('laser_z'), LC('laser_yaw')),
            # the firmware stamps /imu/data_raw with frame imu_link (seen on the robot 2026-10-01)
            static_tf('imu', 'imu_link', LC('imu_x'), LC('imu_y'), LC('imu_z')),
            static_tf('camera', 'camera', LC('camera_x'), LC('camera_y'), LC('camera_z'),
                      '0', LC('camera_pitch')),
        ])
