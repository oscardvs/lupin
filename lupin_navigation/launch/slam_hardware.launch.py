"""slam_hardware.launch.py — SLAM for the real Mirte.

Thin wrapper over _slam_core.launch.py with use_sim_time:=false. Brings up
slam_toolbox (→ /map, map→odom TF) + slam_reset_node (/lupin/nav/clear_map).

    ros2 launch lupin_navigation slam_hardware.launch.py

Run it, confirm /map (RViz or `ros2 topic echo --once --qos-reliability reliable
--qos-durability transient_local /map nav_msgs/msg/OccupancyGrid`), warm the TF
(`ros2 run lupin_bringup wait_for_tf odom base_link 30.0`), THEN start
nav2_hardware.launch.py.
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
            launch_arguments=[('use_sim_time', 'false')],
        ),
    ])
