# RECONSTRUCTED from RECOVERY.md (2026-09-30) — not yet run on the robot.
"""slam_toolbox (online async) with config/slam_toolbox.yaml. Started by run/remap.sh.

On this Jazzy build slam_toolbox is a lifecycle node, so this reuses the upstream launch file, which
configures and activates it (a plain Node would sit unconfigured and never publish /map).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'slam_toolbox.yaml')


def generate_launch_description():
    upstream = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream),
            launch_arguments={'slam_params_file': CONFIG, 'use_sim_time': 'false', 'autostart': 'true'}.items()),
    ])
