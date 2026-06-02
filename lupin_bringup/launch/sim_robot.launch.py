"""sim_robot.launch.py — TERMINAL 1 of the layered sim bringup.

The foundation: Gazebo + the greenhouse world + the Mirte robot + its
ros2_control controllers, plus the robot-side glue (twist_mux, arm shim/preset,
gripper bridge, /amcl_pose seed, sim battery). Nothing here depends on SLAM,
Nav2, perception or the HMI — so you can bring this up FIRST and confirm the
controllers actually spawn before layering anything on top:

    ros2 launch lupin_bringup sim_robot.launch.py
    # in another shell, once Gazebo is up:
    ros2 control list_controllers      # expect joint_state_broadcaster +
                                       # pid_wheels_controller + base + arm + gripper, all active

If controllers DON'T come up here, it's a robot/ros2_control problem (not a
perception/nav one) — see the runbook's troubleshooting table. Run
`scripts/sim-clean.sh` first if a previous launch left a zombie gzserver.

Then: TERMINAL 2 = sim_autonomy.launch.py, TERMINAL 3 = sim_hmi.launch.py.
(`sim_full.launch.py` still does all of this in one shot for a quick demo.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_hmi = get_package_share_directory('lupin_hmi')
    lupin_models_path = os.path.join(pkg_bringup, 'models')

    args = [
        DeclareLaunchArgument('spawn_x', default_value='2.0'),
        DeclareLaunchArgument('spawn_y', default_value='1.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708'),
        DeclareLaunchArgument('seed_amcl', default_value='true'),
        DeclareLaunchArgument('joystick', default_value='true'),
    ]

    greenhouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'greenhouse_sim.launch.py'),
        ),
        launch_arguments=[
            ('x', LaunchConfiguration('spawn_x')),
            ('y', LaunchConfiguration('spawn_y')),
            ('yaw', LaunchConfiguration('spawn_yaw')),
        ],
    )

    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[
            os.path.join(pkg_hmi, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True},
        ],
        # Sim drives the vendor gazebo_ros_planar_move plugin, which subscribes to
        # /cmd_vel. (Hardware remaps cmd_vel_out to /mirte_base_controller/cmd_vel
        # instead — this terminal topic is the one documented sim<->hardware drive
        # difference; the joy/manual/auto bus above is identical on both.)
        remappings=[('cmd_vel_out', '/cmd_vel')],
        output='log',
    )

    arm_sim_shim = Node(
        package='lupin_hmi', executable='arm_sim_shim', name='arm_sim_shim',
        parameters=[{'use_sim_time': True}], output='log',
    )
    arm_preset_server = Node(
        package='lupin_hmi', executable='arm_preset_server', name='arm_preset_server',
        parameters=[{'use_sim_time': True}], output='log',
    )
    gripper_action_bridge = Node(
        package='lupin_hmi', executable='gripper_action_bridge', name='gripper_action_bridge',
        parameters=[{'use_sim_time': True}], output='log',
    )

    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'true')],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )

    seed = ExecuteProcess(
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
        output='log',
        condition=IfCondition(LaunchConfiguration('seed_amcl')),
    )
    sim_battery_publisher = Node(
        package='lupin_bringup', executable='sim_battery_publisher', name='sim_battery_publisher',
        parameters=[{'use_sim_time': True, 'initial_charge': 1.0, 'drain_rate_per_sec': 0.001}],
        output='log',
    )

    return LaunchDescription([
        *args,
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', lupin_models_path),
        greenhouse_sim,
        twist_mux,
        arm_sim_shim,
        arm_preset_server,
        gripper_action_bridge,
        xbox_teleop,
        seed,
        sim_battery_publisher,
    ])
