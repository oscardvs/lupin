"""sim_full.launch.py — bring up the WHOLE simulation stack in one shot.

Composes:
    1. greenhouse_sim.launch.py        — Gazebo + MIRTE + greenhouse world
    2. slam_toolbox/online_async_launch — online SLAM (greenhouse has no map)
    3. lupin_navigation/nav2.launch.py  — Nav2 in slam mode (no AMCL/map_server)
    4. greenhouse_bridge.launch.py      — sensor service for tag readings
    5. mission_orchestrator             — v2 lifecycle node (idles in READY)
    6. rosbridge_websocket              — :9090 for the web HMI
    7. lupin_web.launch.py              — Vite preview on :8090 + web_video :8091
    8. seed_amcl_pose                   — one-shot synthetic /amcl_pose so the
                                          orchestrator's PREPARE.LOCALIZING gate
                                          clears (no real AMCL in slam mode)

This is *the* "I want to test everything" entry point. Heavy and slow on
first start: Gazebo loading the greenhouse world is ~30-60 s, slam_toolbox
needs scan context before Nav2 will plan anywhere, and the orchestrator
will sit in BOOT for ~30 s while it waits for the Nav2 action server to
finish lifecycle activation.

After bringup:
    1. Open http://localhost:8090
    2. (one-time) In Settings → Drive, set cmd_vel topic to ``/cmd_vel_manual``
       so the web joystick goes through the sim's twist_mux instead of
       racing Nav2 on the controller topic.
    3. Drive a small loop in the Teleop tab so slam_toolbox has scan context.
    4. ``ros2 service call /mission/start lupin_msgs/srv/StartMission \\
         "{mission_type: 'InspectionMission', tag_sequence: ['1','2']}"``

Most arguments are forwarded to the included launches with the same name.
The launch is opinionated about what slam_toolbox / Nav2 / mission needs —
override individual args via ``--launch-arguments``.

Known fragility:
- All eight components start in parallel. DDS discovery + the orchestrator's
  dependency wait absorb the ordering, but on a cold laptop you may see
  ~30 s of "waiting for nav2..." chatter before things go green.
- If the orchestrator hits ``dependency_timeout_s`` before Nav2's lifecycle
  finishes, it transitions to FAULT. We bump the default to 120 s here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_web = get_package_share_directory('lupin_web')
    pkg_rosbridge = get_package_share_directory('rosbridge_server')
    pkg_slam = get_package_share_directory('slam_toolbox')

    args = [
        # forwarded to greenhouse_sim
        DeclareLaunchArgument(
            'spawn_x', default_value='1.0',
            description='Mirte spawn X in the greenhouse world (m).',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.5',
            description='Mirte spawn Y in the greenhouse world (m).',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Mirte spawn yaw in the greenhouse world (rad).',
        ),
        # web HMI
        DeclareLaunchArgument(
            'web_port', default_value='8090',
            description='HTTP port for the Lupin Web HMI Vite preview.',
        ),
        DeclareLaunchArgument(
            'enable_web', default_value='true',
            description='Bring up rosbridge + lupin_web. Set to false for '
                        'a headless smoke test.',
        ),
        # mission tunables
        DeclareLaunchArgument(
            'dependency_timeout_s', default_value='120.0',
            description='How long the mission orchestrator waits for Nav2 + '
                        'bridge before transitioning to FAULT. Bumped from '
                        'the 30 s default because Nav2 lifecycle activation '
                        'in slam mode takes longer than that on a cold start.',
        ),
        DeclareLaunchArgument(
            'seed_amcl', default_value='true',
            description='Publish a synthetic /amcl_pose to clear the '
                        'PREPARE.LOCALIZING gate. Slam mode has no AMCL, so '
                        'without this the orchestrator faults at PREPARE.',
        ),
    ]

    # ── 1. Gazebo + MIRTE + greenhouse world ────────────────────────────
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

    # ── 2. slam_toolbox (online async) ──────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('slam_params_file', os.path.join(
                pkg_nav, 'config', 'slam_toolbox_sim.yaml',
            )),
        ],
    )

    # ── 3. Nav2 (slam mode) ─────────────────────────────────────────────
    # Delayed slightly so Gazebo's controller_manager has time to publish
    # /scan and TFs before Nav2's lifecycle managers ask for them.
    nav2 = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
            ),
            launch_arguments=[('slam', 'true')],
        )],
    )

    # ── 4. Greenhouse bridge ────────────────────────────────────────────
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
    )

    # ── 5. Mission orchestrator ─────────────────────────────────────────
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
        ],
    )

    # ── 6. rosbridge_websocket :9090 ────────────────────────────────────
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_rosbridge, 'launch', 'rosbridge_websocket_launch.xml'),
        ),
        condition=_when('enable_web'),
    )

    # ── 7. Lupin web HMI :8090 ──────────────────────────────────────────
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web, 'launch', 'lupin_web.launch.py'),
        ),
        launch_arguments=[('port', LaunchConfiguration('web_port'))],
        condition=_when('enable_web'),
    )

    # ── 8. AMCL pose seed (one-shot, after the orchestrator subscribes) ─
    seed = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
            output='log',
            condition=_when('seed_amcl'),
        )],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] sim_full: starting full sim chain '
                    '(Gazebo + slam + Nav2 + bridge + orchestrator + web)'),
        greenhouse_sim,
        slam,
        nav2,
        bridge,
        mission,
        rosbridge,
        web,
        seed,
    ])


def _when(arg_name: str):
    """Tiny helper — IfCondition needs a substitution, so wrap a launch arg
    as a truthy/falsy condition for the include."""
    from launch.conditions import IfCondition
    return IfCondition(LaunchConfiguration(arg_name))
