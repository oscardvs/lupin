"""slam_sim.launch.py — SLAM for the Gazebo sim.

Thin wrapper over _slam_core.launch.py with use_sim_time:=true. Brings up
slam_toolbox (→ /map, map→odom TF) + slam_reset_node (/lupin/nav/clear_map),
the same core hardware uses — only use_sim_time differs.

    ros2 launch lupin_navigation slam_sim.launch.py

slam_toolbox needs /scan + the robot's TF, so drive the sim base a little
(teleop) once Gazebo is up so it has scan context. Then start nav2_sim.launch.py
once /map is alive.
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
                os.path.join(pkg_nav, 'launch', '_slam_core.launch.py'),
            ),
            launch_arguments=[('use_sim_time', 'true')],
        ),
    ])
