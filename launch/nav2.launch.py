# Written 2026-10-08: stock Nav2 bring-up with a lifecycle bond timeout the Pi can live with.
"""nav2_bringup's bringup_launch.py, unchanged, plus bond_timeout = 10 s for its lifecycle managers.

Started by run/remap.sh (navigation). Takes the same arguments as bringup_launch.py:
    ros2 launch launch/nav2.launch.py map:=maps/<area>_clean.yaml params_file:=config/nav2_params.yaml

Why: each lifecycle manager drops ALL of Nav2 when one server misses its heartbeat for bond_timeout
(stock 4 s, not settable from nav2_params.yaml because the stock launch doesn't pass the params file
to the managers). On 2026-10-08 route_server missed one at load ~5-8, Nav2 reset itself, and the
restart segfaulted in docking_server: navigation gone mid-session. SetParameter reaches every node in
this group, composed ones included. Nodes that don't declare bond_timeout ignore it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

BOND_TIMEOUT = 10.0   # s; a real hang still stops the robot at once: the ESP32's 300 ms cmd watchdog


def generate_launch_description():
    stock = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    return LaunchDescription([GroupAction([
        SetParameter(name='bond_timeout', value=BOND_TIMEOUT),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stock)),
    ])])
