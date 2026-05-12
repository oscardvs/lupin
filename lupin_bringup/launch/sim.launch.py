"""sim.launch.py — Team Lupin sim entry point.

Single command to bring up Gazebo + the MIRTE Master + ros2_control, with an
optional Nav2 + RViz layer on top. Internally this just composes the vendor
mirte_gazebo launches behind a stable args interface, so feature MRs in this
repo have a single thing to launch instead of remembering vendor file names.

Args
----
world (str)   Gazebo world path. Default: 'worlds/empty.world' (empty room,
              fastest iteration). Ignored when nav:=true (the vendor nav
              launch hardcodes the KRR small-house world so AMCL has
              features to localise against).
gui   (bool)  Show the Gazebo client GUI. Default: True.
nav   (bool)  Bring up Nav2 + RViz on top of the sim. Default: False.

Known limitation (track as separate MR)
---------------------------------------
The MIRTE URDF embeds a `gazebo_planar_move` (P3D) plugin that subscribes to
/cmd_vel and teleports the robot via Gazebo's pose API. With nav:=true the
robot reaches goals visually, but the mecanum wheels do not drive: the
ros2_control mecanum controller subscribes to
/mirte_base_controller/cmd_vel_unstamped, and Nav2 publishes to /cmd_vel.

The naïve corrective relay (/cmd_vel → /mirte_base_controller/cmd_vel_unstamped)
creates a feedback loop with the vendor twist_mux, so it is NOT applied here.
A proper fix needs either (a) spawning the robot from a stripped URDF (no P3D
plugin) and remapping Nav2 to publish directly to the controller's input, or
(b) replacing the vendor twist_mux with one that breaks the loop. Until then,
nav:=true is a *visual* tool — paths plan and the robot follows them via
teleport, but wheel-encoder odom (/mirte_base_controller/odom) lags true
motion and there is no wheel-slip / friction / acceleration fidelity.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_mirte_gazebo = get_package_share_directory('mirte_gazebo')
    pkg_bringup = get_package_share_directory('lupin_bringup')
    lupin_models_path = os.path.join(pkg_bringup, 'models')
    empty_launch = os.path.join(
        pkg_mirte_gazebo, 'launch', 'gazebo_mirte_master_empty.launch.xml'
    )
    nav_launch = os.path.join(
        pkg_mirte_gazebo, 'launch', 'gazebo_mirte_master_navigation.launch.py'
    )

    args = [
        DeclareLaunchArgument(
            'world', default_value='worlds/empty.world',
            description='Gazebo world path. Ignored when nav:=true.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='True',
            description='Launch the Gazebo client GUI.',
        ),
        DeclareLaunchArgument(
            'nav', default_value='False',
            description='Launch Nav2 + RViz. Forces the KRR small-house world.',
        ),
    ]

    nav = LaunchConfiguration('nav')

    sim_only = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(empty_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
        condition=UnlessCondition(nav),
    )

    sim_with_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        condition=IfCondition(nav),
    )

    return LaunchDescription([AppendEnvironmentVariable('GAZEBO_MODEL_PATH', lupin_models_path),*args, sim_only, sim_with_nav])
