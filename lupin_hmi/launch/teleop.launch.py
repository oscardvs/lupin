import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('lupin_hmi')
    joy_config_file = os.path.join(pkg_dir, 'config', 'ds4_config.yaml')

    use_sim = LaunchConfiguration('use_sim')

    # Resolved at launch time so mirte_gazebo is only required when use_sim:=true.
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'), 'launch', 'gazebo_mirte_master_empty.launch.xml'
    ])

    # Sim controller subscribes to *_unstamped; real Mirte firmware subscribes to cmd_vel.
    cmd_vel_topic = PythonExpression([
        "'/mirte_base_controller/cmd_vel_unstamped' if '",
        use_sim,
        "'.lower() == 'true' else '/mirte_base_controller/cmd_vel'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='When true, also bring up Gazebo and target the sim cmd_vel topic. '
                        'Default false drives the real Mirte over the LAN.',
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(use_sim),
        ),

        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[joy_config_file],
            remappings=[('cmd_vel', '/cmd_vel_manual')],
            output='screen',
        ),

        Node(
            package='lupin_hmi', executable='cmd_vel_mux', name='cmd_vel_mux',
            parameters=[{'cmd_vel_topic': cmd_vel_topic}],
            output='screen',
        ),
        Node(package='lupin_hmi', executable='arm_teleop', name='arm_teleop'),
    ])
