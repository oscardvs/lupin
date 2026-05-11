"""hardware_full.launch.py — DEPRECATED alias.

`hardware.launch.py` is now the single laptop-side entry point with
modular flags. The old "full mission" behaviour is just:

    ros2 launch lupin_bringup hardware.launch.py mission:=true

This file is kept as a backwards-compatible shim for muscle memory and
existing scripts. It forwards every launch argument unchanged plus sets
`mission:=true`. Will be removed in a future cleanup pass.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')

    forward = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'hardware.launch.py'),
        ),
        launch_arguments=[('mission', 'true')],
    )

    return LaunchDescription([
        LogInfo(msg='[lupin_bringup] hardware_full is DEPRECATED — use '
                    '`hardware.launch.py mission:=true`. Forwarding now.'),
        forward,
    ])
