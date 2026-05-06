"""hardware_full.launch.py — Lupin full mission stack against the real Mirte.

Extends `hardware.launch.py` with the mission orchestrator, digital twin,
and greenhouse bridge. The Lupin Web HMI and rosbridge are NOT launched
here — they're served by lupin-web.service and mirte-ros.service on the
robot itself.

Phases:

    Phase 1 (t=0 — fire and forget):
        - twist_mux                   — chassis bus arbitration
        - arm_preset_server           — named arm-pose service
        - greenhouse_bridge           — sensor service for tag readings
        - mission_orchestrator        — v2 lifecycle node (idles in READY)
        - lupin_twin                  — aggregates /floranova/observations
        - seed_amcl_pose              — one-shot synthetic /amcl_pose
        - rviz2                       — interactive UI (laptop)
        - xbox teleop (optional)      — joystick on the laptop

    Phase 2 (when /scan first publishes):
        - slam_toolbox online_async   — owns /map + map→odom

    Phase 3 (when /map first publishes):
        - nav2 (slam mode)            — controller_server, planner, etc.

After bringup:
    1. Open http://172.20.10.2:8090 (Lupin HMI on the robot via lupin-web.service).
    2. Drive a small loop with the Xbox controller (or web Teleop tab) so
       slam_toolbox has scan context.
    3. Trigger a mission via the HMI Mission tab or:
         ros2 service call /mission/start lupin_msgs/srv/StartMission \\
           "{mission_type: 'InspectionMission', tag_sequence: ['1','2']}"

Sim-only components NOT launched here:
- greenhouse_sim.launch.py (Gazebo + world spawn)
- arm_sim_shim (real Mirte serves Hiwonder names natively via telemetrix)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_twin = get_package_share_directory('lupin_twin')
    pkg_hmi = get_package_share_directory('lupin_hmi')

    args = [
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
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the persistent config at '
                        'rviz/full_bringup_viz.rviz.',
        ),
        DeclareLaunchArgument(
            'joystick', default_value='false',
            description='Bring up Xbox controller teleop on the laptop.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_nav, 'config', 'slam_toolbox_sim.yaml',
            ),
            description='slam_toolbox params yaml (shared sim/hardware).',
        ),
    ]

    # Tag layout — for now the bridge consumes the same widened sim layout
    # so the orchestrator can be smoke-tested against an oracle bridge on
    # hardware before real perception lands. Per project_approach_pose_pipeline
    # the operator can tune approach poses in approach_overrides.yaml without
    # rebuilding.
    # TODO: replace with `tag_locations_hardware.json` once the demo space
    # has been measured. Until then the oracle just emits whatever the file
    # says — Nav2 will plan to those map-frame poses, which won't match the
    # physical room until we map and re-survey.
    tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')
    approach_overrides = os.path.join(pkg_bringup, 'config', 'approach_overrides.yaml')

    # ── Sentinel: wait for /scan from the robot's RPLidar ──────────────
    wait_for_scan = ExecuteProcess(
        name='wait_for_scan',
        cmd=['ros2', 'topic', 'echo', '--once', '/scan',
             'sensor_msgs/msg/LaserScan'],
        output='log',
    )

    # Spawned directly (not via online_async_launch.py) so we can pin
    # `respawn=True`. The /lupin/nav/clear_map service exposed by
    # `slam_reset_node` SIGTERMs this process to wipe the map; respawn
    # then brings it back up with an empty pose graph.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': False},
        ],
        respawn=True,
        respawn_delay=1.0,
        output='screen',
    )
    on_scan_ready = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_scan,
        on_exit=[
            LogInfo(msg='[lupin_bringup] /scan online — starting slam_toolbox'),
            slam_node,
        ],
    ))

    # ── slam_reset — owns /lupin/nav/clear_map (Trigger). HMI hits this
    # to wipe the SLAM map; node SIGTERMs slam_toolbox + clears costmaps.
    slam_reset_node = Node(
        package='lupin_navigation',
        executable='slam_reset_node',
        name='slam_reset_node',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ── Sentinel: wait for /map from slam_toolbox ──────────────────────
    wait_for_map = ExecuteProcess(
        name='wait_for_map',
        cmd=['ros2', 'topic', 'echo', '--once',
             '--qos-reliability', 'reliable',
             '--qos-durability', 'transient_local',
             '/map', 'nav_msgs/msg/OccupancyGrid'],
        output='log',
    )

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
        ),
        launch_arguments=[
            ('slam', 'true'),
            ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
            ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
            ('use_sim_time', 'false'),
            ('autostart', 'true'),
        ],
    )
    on_map_ready = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map,
        on_exit=[
            LogInfo(msg='[lupin_bringup] /map online — starting Nav2 lifecycle '
                        '(autostart=true; expect ~15-30 s to ACTIVE)'),
            nav2_include,
        ],
    ))

    # ── Greenhouse bridge (oracle, real-perception swap is later work) ─
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', tag_locations)],
    )

    # ── Mission orchestrator ───────────────────────────────────────────
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
            ('tag_locations_file', tag_locations),
            ('approach_overrides_file', approach_overrides),
        ],
    )

    # ── Digital twin ───────────────────────────────────────────────────
    twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_twin, 'launch', 'twin.launch.py'),
        ),
    )

    # ── twist_mux — chassis bus arbitration → real Mirte's stamped sink ─
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[
            os.path.join(pkg_hmi, 'config', 'twist_mux.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('cmd_vel_out', '/mirte_base_controller/cmd_vel'),
        ],
        output='log',
    )

    # ── arm_preset_server — works on both targets ──────────────────────
    arm_preset_server = Node(
        package='lupin_hmi', executable='arm_preset_server',
        name='arm_preset_server',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'false')],
        condition=_when('joystick'),
    )

    # ── AMCL pose seed (one-shot) ──────────────────────────────────────
    seed = ExecuteProcess(
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
        output='log',
        condition=_when('seed_amcl'),
    )

    # ── RViz ───────────────────────────────────────────────────────────
    rviz_config = _resolve_rviz_config()
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lupin_rviz',
        arguments=['-d', rviz_config],
        output='log',
        condition=_when('rviz'),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] hardware_full: Nav2 + slam_toolbox + '
                    'orchestrator + twin + bridge + RViz against the real '
                    'Mirte. HMI is at http://172.20.10.2:8090 (lupin-web.service).'),
        bridge,
        mission,
        twin,
        twist_mux,
        arm_preset_server,
        slam_reset_node,
        xbox_teleop,
        seed,
        rviz,
        wait_for_scan,
        wait_for_map,
        on_scan_ready,
        on_map_ready,
    ])


def _when(arg_name: str):
    from launch.conditions import IfCondition
    return IfCondition(LaunchConfiguration(arg_name))


def _resolve_rviz_config() -> str:
    candidates = [
        os.environ.get('LUPIN_RVIZ_CONFIG'),
        os.path.expanduser(
            '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz',
        ),
        os.path.join(
            get_package_share_directory('lupin_bringup'),
            'rviz', 'full_bringup_viz.rviz',
        ),
    ]
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return os.path.expanduser(
        '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz',
    )
