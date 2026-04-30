"""Launch the greenhouse bridge node.

All sim-time controls are exposed as launch args. The defaults are sentinels
that mean "don't override the upstream config":
  - debug_time_of_day=-1.0 → leave whatever's in greenhouse_config.yaml
  - speedup_factor=0.0    → leave whatever's in greenhouse_config.yaml
  - debug_seed=-1         → leave whatever's in greenhouse_config.yaml

Setting debug_time_of_day or debug_seed flips the sim's debug_mode on.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag_file', default_value=''),
        DeclareLaunchArgument('sim_config_file', default_value=''),
        DeclareLaunchArgument('debug_time_of_day', default_value='-1.0'),
        DeclareLaunchArgument('speedup_factor', default_value='0.0'),
        DeclareLaunchArgument('debug_seed', default_value='-1'),

        Node(
            package='lupin_greenhouse_bridge',
            executable='bridge_node',
            name='greenhouse_bridge',
            output='screen',
            parameters=[{
                'tag_file': LaunchConfiguration('tag_file'),
                'sim_config_file': LaunchConfiguration('sim_config_file'),
                'debug_time_of_day': LaunchConfiguration('debug_time_of_day'),
                'speedup_factor': LaunchConfiguration('speedup_factor'),
                'debug_seed': LaunchConfiguration('debug_seed'),
            }],
        ),
    ])
