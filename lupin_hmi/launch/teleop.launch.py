import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('lupin_hmi')
    joy_config_file = os.path.join(pkg_dir, 'config', 'ds4_config.yaml')

    use_sim = LaunchConfiguration('use_sim')
    world = LaunchConfiguration('world')

    # All sim flavours go through gazebo_mirte_master_empty.launch.xml; the world
    # argument it forwards to gzserver is what changes between flavours. Path
    # resolution is lazy so mirte_gazebo is only required when use_sim:=true.
    empty_world_path = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'), 'launch', 'gazebo_mirte_master_empty.launch.xml'
    ])
    krr_house_world = PathJoinSubstitution([
        FindPackageShare('robocup_home_simulation'), 'worlds', 'KRR_Course_Small_house.world'
    ])
    plasys_models = PathJoinSubstitution([
        FindPackageShare('plasys_house_world'), 'models'
    ])
    aws_models = PathJoinSubstitution([
        FindPackageShare('aws_robomaker_small_house_world'), 'models'
    ])

    sim_empty = PythonExpression([
        "'", use_sim, "'.lower() == 'true' and '", world, "' == 'empty'"
    ])
    sim_nav = PythonExpression([
        "'", use_sim, "'.lower() == 'true' and '", world, "' == 'navigation'"
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
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description="Sim world: 'empty' (featureless ground plane, fastest iteration) "
                        "or 'navigation' (KRR small house, has walls — required for SLAM). "
                        'Ignored when use_sim:=false.',
            choices=['empty', 'navigation'],
        ),

        # Empty world: vendor launch defaults to worlds/empty.world; we don't pass anything.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(empty_world_path),
            condition=IfCondition(sim_empty),
        ),

        # Navigation world (KRR small house): the world references models from the
        # plasys + AWS small-house packages, so GAZEBO_MODEL_PATH must include them or
        # gzserver fails to load the world. Vendor's nav launch does this — we replicate
        # only that part because we don't want their Nav2/AMCL stack alongside slam_toolbox.
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                ':', plasys_models, ':', aws_models,
            ],
            condition=IfCondition(sim_nav),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(empty_world_path),
            launch_arguments={'world': krr_house_world}.items(),
            condition=IfCondition(sim_nav),
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
