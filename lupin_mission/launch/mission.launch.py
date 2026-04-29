"""Launch the mission orchestrator on its own.

This launch file is intentionally minimal: it brings up only the orchestrator
node and assumes Nav2, the greenhouse bridge, and (in sim) the greenhouse
Gazebo world are already running. Composing them is the caller's job — see
the package README for the full bringup recipe.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_sequence',
            default_value='[]',
            description='YAML list of tag IDs to visit, e.g. "[1, 5, 12]". '
                        'Empty list = visit every tag in tag_locations.json '
                        'in numeric-string order.',
        ),
        DeclareLaunchArgument('approach_yaw', default_value='0.0'),
        DeclareLaunchArgument('nav_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('service_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('dependency_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('nav_retry_limit', default_value='1'),
        DeclareLaunchArgument('frame_id', default_value='map'),

        Node(
            package='lupin_mission',
            executable='mission_orchestrator',
            name='mission_orchestrator',
            output='screen',
            parameters=[{
                'tag_sequence': LaunchConfiguration('tag_sequence'),
                'approach_yaw': LaunchConfiguration('approach_yaw'),
                'nav_timeout_sec': LaunchConfiguration('nav_timeout_sec'),
                'service_timeout_sec': LaunchConfiguration('service_timeout_sec'),
                'dependency_timeout_sec': LaunchConfiguration('dependency_timeout_sec'),
                'nav_retry_limit': LaunchConfiguration('nav_retry_limit'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
