"""greenhouse_sim.launch.py — bring up Gazebo in the greenhouse world.

Loads the SDF emitted by ``scripts/generate_greenhouse_world.py`` (whose
coordinate frame matches mdp-greenhouse's ``tag_locations.json``) and spawns
the MIRTE Master with the standard vendor pipeline.

Why this isn't ``IncludeLaunchDescription(gazebo_mirte_master_empty)``: the
vendor empty-world launch hardcodes the spawn pose ``(1.05, 0.51, 0.02)``,
which in the 1.0.8 demo layout lands in a perimeter corner against a table.
Forwarding ``world=`` would give us an immediately stuck robot. So we
replicate the vendor launch's contents here but expose the spawn pose as
launch args, defaulting to the greenhouse's central aisle (see x/y/yaw below).

Gazebo classic env: this launch sets ``GAZEBO_PLUGIN_PATH`` /
``GAZEBO_RESOURCE_PATH`` / ``GAZEBO_MODEL_PATH`` / ``OGRE_RESOURCE_PATH``
to the Ubuntu-22.04 / gazebo-classic-11 defaults via ``SetEnvironmentVariable``
so the launch works without sourcing ``/usr/share/gazebo/setup.sh`` first.
This was the recurring "spawn_entity hangs / Scene shared_ptr null" footgun
every teammate hit on first run; the launch silently fixed it now.
The Append semantics preserve any user-set values.

The robot URDF already includes an Astra Pro Plus depth-camera Gazebo plugin
(publishing on ``/camera/...``); no extra sensor wiring is needed here.

The AWS-small-house ``sim.launch.py`` (on the ``sim`` branch) is intentionally
untouched — this launch is an alternative entry point, not a replacement.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Mirrors the exports from /usr/share/gazebo/setup.sh on Ubuntu 22.04 +
# gazebo-classic-11 (the documented Lupin sim environment). Hardcoded
# because the alternative — relying on every teammate to source another
# setup.sh — has burned everyone at least once.
_GAZEBO_PLUGIN_PATH = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
_GAZEBO_RESOURCE_PATH = '/usr/share/gazebo-11'
_GAZEBO_MODEL_PATH = '/usr/share/gazebo-11/models'
_OGRE_RESOURCE_PATH = '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'


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

    # Append rather than overwrite so anything the user already set
    # (custom plugin dirs, vendor models) stays in front. SetEnvironmentVariable
    # is used for OGRE_RESOURCE_PATH because it is typically unset and
    # appending to "" would leave a leading colon.
    gazebo_env = [
        AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', _GAZEBO_PLUGIN_PATH),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', _GAZEBO_RESOURCE_PATH),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', _GAZEBO_MODEL_PATH),
        SetEnvironmentVariable(
            'OGRE_RESOURCE_PATH',
            os.environ.get('OGRE_RESOURCE_PATH') or _OGRE_RESOURCE_PATH,
        ),
    ]

    args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Path to the SDF world (default: generated greenhouse.world).',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch the Gazebo client GUI.',
        ),
        # Central aisle of the demo greenhouse (between vertical tables Table6/7),
        # facing +Y up the greenhouse. ~0.23 m footprint clearance each side in
        # the 1.0.8 layout (the old y=1.5 now lands on Table8/9).
        DeclareLaunchArgument('x', default_value='2.0'),
        DeclareLaunchArgument('y', default_value='3.0'),
        # base_link spawns on the floor. Measured TF base_link->front_left_wheel
        # z = +0.055 and wheel radius 0.05, so the wheel bottoms sit base_link+0.005
        # → base_link at z=0.0 grounds the robot (5 mm gap, invisible). The base is
        # <kinematic> (vendor design) and planar_move holds it at exactly this z —
        # there is NO gravity settling, so z is the final resting height, not a drop.
        # (The old 0.10 left it hovering 10.5 cm; the "wheels 0.0965 m below
        # base_link" note it was based on had the sign and magnitude both wrong.)
        DeclareLaunchArgument('z', default_value='0.0'),
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

    # Option A (vendor planar_move drive): we deliberately do NOT spawn the
    # ros2_control mecanum base controllers (pid_wheels_controller /
    # mirte_base_controller). planar_move drives the kinematic base directly and is
    # the SINGLE odom->base_link TF source; also running the mecanum controller
    # would publish a second, competing odom TF (the Nav2 "timestamp earlier than
    # transform cache" drops). The arm/gripper controllers are still spawned above,
    # and joint_state_broadcaster still reports the (free-spinning) wheel joints.

    # planar_move advertises odometry on /odom; the rest of the stack subscribes to
    # /mirte_base_controller/odom exactly as on hardware (nav2_params odom_topic,
    # HMI odomTopic). Relay so the topic name matches hardware 1:1 — least-friction
    # port. The odom->base_link TF planar_move publishes is already identical.
    odom_relay = Node(
        package='topic_tools', executable='relay',
        name='sim_odom_relay',
        arguments=['/odom', '/mirte_base_controller/odom'],
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # The vendor empty-world launch publishes a constant zero twist at 100 Hz so
    # twist_mux always has a "lowest priority" stream to fall back on. Mirror it
    # but route output to the launch log only — `ros2 topic pub` prints every
    # publish on stdout and would otherwise drown the console at 100 Hz.
    zero_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/zero_cmd_vel', 'geometry_msgs/msg/Twist', '{}',
            '-r', '100',
        ],
        output='log',
    )

    # The single command bus (lupin twist_mux: joy/manual/auto + the idle-zero
    # fallback) lives in sim_robot.launch.py and publishes to /cmd_vel, which
    # planar_move consumes. We keep only the constant-zero publisher here as
    # planar_move's idle-stop guard (planar_move holds the last twist forever
    # otherwise); sim_robot's twist_mux routes /zero_cmd_vel in as lowest priority.
    return LaunchDescription([
        *gazebo_env,
        *args,
        gazebo,
        spawn_robot,
        arm_controllers,
        odom_relay,
        zero_cmd_vel,
    ])
