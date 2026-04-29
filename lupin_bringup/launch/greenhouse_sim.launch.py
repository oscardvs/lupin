"""greenhouse_sim.launch.py — bring up Gazebo in the greenhouse world.

Loads the SDF emitted by ``scripts/generate_greenhouse_world.py`` (whose
coordinate frame matches mdp-greenhouse's ``tag_locations.json``) and spawns
the MIRTE Master with the standard vendor pipeline.

Why this isn't ``IncludeLaunchDescription(gazebo_mirte_master_empty)``: the
vendor empty-world launch hardcodes the spawn pose ``(1.05, 0.51, 0.02)``,
which falls inside our south perimeter wall (the wall sits at ``y≈0.6``).
Forwarding ``world=`` would give us an immediately stuck robot. So we
replicate the vendor launch's contents here but expose the spawn pose as
launch args, defaulting to a spot in the greenhouse's south aisle.

The robot URDF already includes an Astra Pro Plus depth-camera Gazebo plugin
(publishing on ``/camera/...``); no extra sensor wiring is needed here.

The AWS-small-house ``sim.launch.py`` (on the ``sim`` branch) is intentionally
untouched — this launch is an alternative entry point, not a replacement.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lupin_bringup = get_package_share_directory('lupin_bringup')
    pkg_mirte_gazebo = get_package_share_directory('mirte_gazebo')

    default_world = os.path.join(pkg_lupin_bringup, 'worlds', 'greenhouse.world')
    world_generated_launch = os.path.join(
        pkg_mirte_gazebo, 'launch', 'gazebo_mirte_world_generated.launch.xml'
    )
    spawn_mirte_launch = os.path.join(
        pkg_mirte_gazebo, 'launch', 'spawn_mirte_master.launch.xml'
    )
    twist_mux_config = PathJoinSubstitution([pkg_mirte_gazebo, 'config', 'twist_mux.yaml'])

    args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Path to the SDF world (default: generated greenhouse.world).',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch the Gazebo client GUI.',
        ),
        # South aisle of the greenhouse, facing +Y (toward the tables).
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='1.5'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
    ]

    gazebo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(world_generated_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'generated_world': LaunchConfiguration('world'),
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(spawn_mirte_launch),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    arm_controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'mirte_master_arm_controller',
            'mirte_master_gripper_controller',
        ],
        parameters=[{'use_sim_time': True}],
    )

    base_controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=['pid_wheels_controller', 'mirte_base_controller'],
        parameters=[{'use_sim_time': True}],
    )

    # The vendor empty-world launch publishes a constant zero twist at 100 Hz so
    # twist_mux always has a "lowest priority" stream to fall back on. Mirror it.
    zero_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/zero_cmd_vel', 'geometry_msgs/msg/Twist', '{}',
            '-r', '100',
        ],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    return LaunchDescription([
        *args,
        gazebo,
        spawn_robot,
        arm_controllers,
        base_controllers,
        zero_cmd_vel,
        twist_mux,
    ])
