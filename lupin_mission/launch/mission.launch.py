"""Launch the v2 mission orchestrator on its own.

Brings up only the orchestrator node — assumes Nav2, the greenhouse
bridge, and (in sim) the greenhouse Gazebo world are already running.
The orchestrator idles in READY until ``/mission/start`` is invoked;
mission no longer auto-runs on bringup.

Composing the full bringup is the caller's job — see the package README.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # Dependencies
        DeclareLaunchArgument('nav_action_name', default_value='navigate_to_pose'),
        DeclareLaunchArgument(
            'bridge_service_name',
            default_value='/greenhouse_bridge/get_tag_reading',
        ),
        DeclareLaunchArgument('estop_topic', default_value='/e_stop_state'),
        DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('dependency_timeout_s', default_value='30.0'),
        # Localization (PREPARE)
        DeclareLaunchArgument('map_yaml_path', default_value=''),
        DeclareLaunchArgument('localization_timeout_s', default_value='15.0'),
        DeclareLaunchArgument(
            'localization_covariance_threshold', default_value='0.25',
        ),
        DeclareLaunchArgument('tag_locations_file', default_value=''),
        # Inspection
        DeclareLaunchArgument('approach_yaw', default_value='0.0'),
        DeclareLaunchArgument('approach_standoff_m', default_value='0.5'),
        DeclareLaunchArgument('approach_overrides_file', default_value=''),
        DeclareLaunchArgument('nav_timeout_s', default_value='60.0'),
        DeclareLaunchArgument('nav_max_attempts', default_value='2'),
        DeclareLaunchArgument(
            'nav_localization_cov_threshold', default_value='0.25',
        ),
        DeclareLaunchArgument('scan_timeout_s', default_value='5.0'),
        DeclareLaunchArgument('flower_scan_dwell_s', default_value='0.0'),
        # Exploration / monitoring (ExplorationMission)
        DeclareLaunchArgument('discovery_goal', default_value='5'),
        DeclareLaunchArgument('exploration_timeout_s', default_value='180.0'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('require_visual_confirmation', default_value='false'),
        DeclareLaunchArgument(
            'visual_confirmation_service', default_value='/perception/confirm_tag',
        ),
        DeclareLaunchArgument('visual_confirmation_timeout_s', default_value='3.0'),
        # Per-pot arm patrol (optional; sim flower-scan demo — default off)
        DeclareLaunchArgument('arm_patrol_enabled', default_value='false'),
        DeclareLaunchArgument('arm_preset_service', default_value='/lupin/arm/preset'),
        DeclareLaunchArgument('arm_inspect_preset', default_value='inspect'),
        DeclareLaunchArgument('arm_travel_preset', default_value='home'),
        DeclareLaunchArgument('arm_travel_settle_s', default_value='0.0'),
        # Returning
        DeclareLaunchArgument('dock_timeout_s', default_value='60.0'),
        # Publishing
        DeclareLaunchArgument('state_publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('mission_id_prefix', default_value='lupin'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        # In sim, pass true so the orchestrator stamps observations on /clock —
        # otherwise the twin (sim time) sees them as stale and the map pins fade.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    # tag_sequence and dock_pose are array-typed parameters that don't
    # round-trip cleanly through LaunchConfiguration string parsing — we
    # leave them at the orchestrator's declared defaults (empty / origin).
    # Callers that need to preset them should pass --ros-args -p directly,
    # or write a YAML parameter file.
    node = Node(
        package='lupin_mission',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'nav_action_name': LaunchConfiguration('nav_action_name'),
            'bridge_service_name': LaunchConfiguration('bridge_service_name'),
            'estop_topic': LaunchConfiguration('estop_topic'),
            'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
            'dependency_timeout_s': LaunchConfiguration('dependency_timeout_s'),
            'map_yaml_path': LaunchConfiguration('map_yaml_path'),
            'localization_timeout_s': LaunchConfiguration('localization_timeout_s'),
            'localization_covariance_threshold': LaunchConfiguration(
                'localization_covariance_threshold',
            ),
            'tag_locations_file': LaunchConfiguration('tag_locations_file'),
            'approach_yaw': LaunchConfiguration('approach_yaw'),
            'approach_standoff_m': LaunchConfiguration('approach_standoff_m'),
            'approach_overrides_file': LaunchConfiguration('approach_overrides_file'),
            'nav_timeout_s': LaunchConfiguration('nav_timeout_s'),
            'nav_max_attempts': LaunchConfiguration('nav_max_attempts'),
            'nav_localization_cov_threshold': LaunchConfiguration(
                'nav_localization_cov_threshold',
            ),
            'scan_timeout_s': LaunchConfiguration('scan_timeout_s'),
            'flower_scan_dwell_s': LaunchConfiguration('flower_scan_dwell_s'),
            'discovery_goal': LaunchConfiguration('discovery_goal'),
            'exploration_timeout_s': LaunchConfiguration('exploration_timeout_s'),
            'map_topic': LaunchConfiguration('map_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'require_visual_confirmation': LaunchConfiguration(
                'require_visual_confirmation',
            ),
            'visual_confirmation_service': LaunchConfiguration(
                'visual_confirmation_service',
            ),
            'visual_confirmation_timeout_s': LaunchConfiguration(
                'visual_confirmation_timeout_s',
            ),
            'arm_patrol_enabled': LaunchConfiguration('arm_patrol_enabled'),
            'arm_preset_service': LaunchConfiguration('arm_preset_service'),
            'arm_inspect_preset': LaunchConfiguration('arm_inspect_preset'),
            'arm_travel_preset': LaunchConfiguration('arm_travel_preset'),
            'arm_travel_settle_s': LaunchConfiguration('arm_travel_settle_s'),
            'dock_timeout_s': LaunchConfiguration('dock_timeout_s'),
            'state_publish_rate_hz': LaunchConfiguration('state_publish_rate_hz'),
            'mission_id_prefix': LaunchConfiguration('mission_id_prefix'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    return LaunchDescription([*args, node])
