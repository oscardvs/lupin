"""sim_hmi.launch.py — TERMINAL 3 of the layered sim bringup.

The operator surface: rosbridge_websocket (:9090), the Lupin web HMI (:8090),
and RViz. Bring this up last (or skip RViz/web for a headless run). Keeping it
in its own terminal means a flaky web build or a stale :9090 never blocks the
robot + autonomy layers — and you can restart just the HMI without touching the
running sim.

    ros2 launch lupin_bringup sim_hmi.launch.py
    # open http://localhost:8090
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_web = get_package_share_directory('lupin_web')
    pkg_rosbridge = get_package_share_directory('rosbridge_server')

    args = [
        DeclareLaunchArgument('web_port', default_value='8090'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_rosbridge, 'launch', 'rosbridge_websocket_launch.xml'),
        ),
    )

    # lupin_web co-launches its own rosbridge on :9090; we already start one
    # above, so tell it to skip (otherwise "Address already in use [9090]").
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web, 'launch', 'lupin_web.launch.py'),
        ),
        launch_arguments=[
            ('port', LaunchConfiguration('web_port')),
            ('rosbridge', 'false'),
        ],
    )

    rviz_config = os.path.join(pkg_bringup, 'rviz', 'full_bringup_viz.rviz')
    src_rviz = os.path.expanduser(
        '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz')
    if os.path.isfile(src_rviz):
        rviz_config = src_rviz
    rviz = Node(
        package='rviz2', executable='rviz2', name='lupin_rviz',
        arguments=['-d', rviz_config], output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([*args, rosbridge, web, rviz])
