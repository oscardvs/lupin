"""mission_stack.launch.py — the autonomous-mission bundle as ONE terminal.

The granular DEMO_DAY 8-terminal layout brings up the *base* stack (drive,
SLAM, Nav2, HMI, perception overlay/fusion) but deliberately leaves the
autonomous mission off so the operator can drive/inspect manually. This
launch is the missing piece: it starts exactly the three nodes that
`hardware.launch.py mission:=true` bundles, with the same config, so a demo
that wants explore→monitor→flower autonomy can add it as a single terminal
WITHOUT relaunching everything via the unified bringup:

    greenhouse_bridge   → /greenhouse_bridge/get_tag_reading (tag oracle)
    mission_orchestrator→ /mission/{start,pause,resume,abort} + /mission/state
    seed_amcl_pose      → one-shot /amcl_pose so PREPARE.LOCALIZING clears

Assumes the base stack (Nav2, SLAM, twin, perception) is already up — i.e.
run this AFTER the DEMO_DAY T1–T8 terminals, in its own terminal, with the
two §4 `source` lines. The orchestrator idles in READY until /mission/start.

Usage:
    ros2 launch lupin_bringup mission_stack.launch.py
    ros2 launch lupin_bringup mission_stack.launch.py discovery_goal:=3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_mission = get_package_share_directory('lupin_mission')

    # Same config the unified hardware.launch.py feeds the mission bundle —
    # keep these in sync (see hardware.launch.py "Mission pipeline").
    tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')
    approach_overrides = os.path.join(pkg_bringup, 'config', 'approach_overrides.yaml')

    args = [
        DeclareLaunchArgument(
            'discovery_goal', default_value='5',
            description='ExplorationMission: number of distinct tags to discover '
                        'before switching to the monitoring loop. Overridable per '
                        '/mission/start request.',
        ),
        DeclareLaunchArgument(
            'dependency_timeout_s', default_value='120.0',
            description='How long the orchestrator waits for Nav2 + bridge before '
                        'FAULT. 120 s covers a cold Nav2 lifecycle activation in '
                        'slam mode (the 30 s node default is too tight on hardware).',
        ),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', tag_locations)],
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
            ('tag_locations_file', tag_locations),
            ('approach_overrides_file', approach_overrides),
            ('discovery_goal', LaunchConfiguration('discovery_goal')),
        ],
    )

    # The orchestrator subscribes to /amcl_pose at t=0 and caches whatever the
    # seed publishes; PREPARE.LOCALIZING clears the instant a mission starts.
    seed = ExecuteProcess(
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
        output='log',
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] mission_stack: greenhouse_bridge + '
                    'mission_orchestrator + seed_amcl_pose. Orchestrator idles '
                    'in READY until /mission/start. Requires Nav2/SLAM/twin up.'),
        bridge,
        mission,
        seed,
    ])
