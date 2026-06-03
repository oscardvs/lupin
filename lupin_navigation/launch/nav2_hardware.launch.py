"""nav2_hardware.launch.py — Nav2 (slam mode) for the real Mirte.

Thin wrapper over nav2.launch.py presetting the hardware args that
hardware.launch.py uses: slam:=true, use_sim_time:=false, the shared
nav2_params.yaml, the krr_house map placeholder, autostart:=true.

    ros2 launch lupin_navigation nav2_hardware.launch.py

Run ONLY after slam_hardware.launch.py has published /map and the odom→base_link
TF is warm (`ros2 run lupin_bringup wait_for_tf odom base_link 30.0`), or the
Nav2 lifecycle wedges on "Invalid frame ID base_link". Expect ~15-30 s of
activation logs ending in "Managed nodes are active". `map:=` is a placeholder:
in slam mode map_server isn't instantiated, but RewrittenYaml needs a real path.
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
                ('use_sim_time', 'false'),
                ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
                ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
                ('autostart', 'true'),
            ],
        ),
    ])
