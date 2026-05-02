"""Launch the Lupin digital-twin node.

Standalone — assumes the orchestrator is already publishing on the
configured observations topic. Composable from `sim_full.launch.py`
or any hardware bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'observations_topic', default_value='/floranova/observations',
        ),
        DeclareLaunchArgument('state_topic', default_value='/twin/state'),
        DeclareLaunchArgument(
            'field_service_name', default_value='/twin/get_field',
        ),
        DeclareLaunchArgument('state_publish_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('buffer_len', default_value='32'),
        # IDW tuning. Defaults match the brief; override per-mission if
        # tag density / sensor footprint changes.
        DeclareLaunchArgument('idw_power', default_value='2.0'),
        DeclareLaunchArgument('idw_falloff_radius_m', default_value='1.5'),
        DeclareLaunchArgument('idw_max_distance_m', default_value='1.5'),
    ]
    node = Node(
        package='lupin_twin',
        executable='twin_node',
        name='lupin_twin',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'observations_topic': LaunchConfiguration('observations_topic'),
            'state_topic': LaunchConfiguration('state_topic'),
            'field_service_name': LaunchConfiguration('field_service_name'),
            'state_publish_rate_hz': LaunchConfiguration('state_publish_rate_hz'),
            'frame_id': LaunchConfiguration('frame_id'),
            'buffer_len': LaunchConfiguration('buffer_len'),
            'idw_power': LaunchConfiguration('idw_power'),
            'idw_falloff_radius_m': LaunchConfiguration('idw_falloff_radius_m'),
            'idw_max_distance_m': LaunchConfiguration('idw_max_distance_m'),
        }],
    )
    return LaunchDescription([*args, node])
