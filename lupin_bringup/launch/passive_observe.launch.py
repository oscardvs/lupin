"""passive_observe.launch.py — fill the twin during teleop / manual SLAM.

Following DEMO_DAY_WIRED with NO mission running, the map and tag *pins* already
update, and (after the 2026-06-09 aggregator change) flower/pest markers pin too.
This optional terminal adds the missing climate path so the heatmap also builds:

    greenhouse_bridge   → /greenhouse_bridge/get_tag_reading      (climate oracle)
    passive_observer    → polls the oracle per discovered tag while the mission
                          is idle → KIND_TAG_READING on /floranova/observations

Run it as one extra terminal AFTER the base stack (Nav2 / SLAM / twin / perception
T1-T8). It defers automatically while a mission is active.

MUTUALLY EXCLUSIVE with mission_stack.launch.py (T9): both launch
`greenhouse_bridge` (same node name + service). Run THIS for teleop/observe
sessions; run mission_stack for autonomous runs — never both.

Usage:
    ros2 launch lupin_bringup passive_observe.launch.py
    ros2 launch lupin_bringup passive_observe.launch.py refresh_period_s:=30.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')

    # Same tag oracle config mission_stack feeds the bridge — keep in sync.
    tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')

    args = [
        DeclareLaunchArgument(
            'refresh_period_s', default_value='0.0',
            description='0 = read each tag once; > 0 = re-poll after N seconds '
                        '(track time-of-day drift in the sim oracle).',
        ),
        DeclareLaunchArgument(
            'tag_file', default_value=tag_locations,
            description='Greenhouse tag-oracle config fed to the bridge.',
        ),
        DeclareLaunchArgument(
            'observations_topic', default_value='/floranova/observations',
            description="Twin's observation intake the readings are published on.",
        ),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Frame the tag poses (and emitted observations) are in.',
        ),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', LaunchConfiguration('tag_file'))],
    )

    observer = Node(
        package='lupin_perception',
        executable='passive_observer',
        name='passive_observer',
        output='screen',
        parameters=[{
            'refresh_period_s': LaunchConfiguration('refresh_period_s'),
            'observations_topic': LaunchConfiguration('observations_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] passive_observe: greenhouse_bridge + '
                    'passive_observer. Fills the twin (climate/heatmap) during '
                    'teleop / SLAM tests. Do NOT run alongside mission_stack.'),
        bridge,
        observer,
    ])
