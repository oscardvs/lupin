import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lupin_hmi')

    # Path to Gazebo Simulation
    gazebo_launch_path = os.path.join(
        get_package_share_directory('mirte_gazebo'), 'launch', 'gazebo_mirte_master_empty.launch.xml'
    )

    # Path to your new PS4 config file
    joy_config_file = os.path.join(pkg_dir, 'config', 'ds4_config.yaml')

    return LaunchDescription([
        # 1. Start Gazebo
        IncludeLaunchDescription(AnyLaunchDescriptionSource(gazebo_launch_path)),

        # 2. Start Hardware Drivers (PS4 Controller)
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[joy_config_file],
            remappings=[('cmd_vel', '/cmd_vel_manual')],
            output='screen'
        ),

        # 3. Start Custom Logic
        Node(package='lupin_hmi', executable='cmd_vel_mux', name='cmd_vel_mux'),
        Node(package='lupin_hmi', executable='arm_teleop', name='arm_teleop')
    ])
