"""nav2_sim.launch.py — Nav2 (slam mode) for the Gazebo sim.

Thin wrapper over nav2.launch.py presetting the sim args (same as
sim_autonomy.launch.py uses): slam:=true, use_sim_time:=true, the shared
nav2_params.yaml, the krr_house map placeholder, autostart:=true. Only
use_sim_time differs from nav2_hardware.launch.py.

    ros2 launch lupin_navigation nav2_sim.launch.py

Run after slam_sim.launch.py has published /map. Expect ~15-30 s of activation
logs ending in "Managed nodes are active".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('lupin_navigation')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
            ),
            launch_arguments=[
                ('slam', 'true'),
                ('use_sim_time', 'true'),
                ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
                ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
                ('autostart', 'true'),
            ],
        ),
    ])
