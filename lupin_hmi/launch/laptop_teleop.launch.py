import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lupin_hmi')
    joy_config_file = os.path.join(pkg_dir, 'config', 'ds4_config.yaml')

    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[joy_config_file],
            remappings=[('cmd_vel', '/cmd_vel_manual')],
            output='screen'
        )
    ])