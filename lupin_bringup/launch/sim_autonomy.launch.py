"""sim_autonomy.launch.py — TERMINAL 2 of the layered sim bringup.

The brain: SLAM (slam_toolbox), Nav2 (slam mode), the greenhouse bridge, the
perception stack (tag_annotator + perception_aggregator + sim flower detector),
the mission orchestrator, and the digital twin.

Bring this up AFTER sim_robot.launch.py is running and the controllers are
active (slam_toolbox needs /scan + the robot's TF; the mission orchestrator
needs Nav2 + the bridge). slam_toolbox subscribes to /scan and Nav2 (slam mode)
subscribes to /map, so they self-sequence — no event cascade needed across
terminals; you'll just see a few "waiting" retries until /scan and /map appear.

    ros2 launch lupin_bringup sim_autonomy.launch.py

Then start a mission:
    ros2 service call /mission/start lupin_msgs/srv/StartMission \
      "{mission_type: 'ExplorationMission', discovery_goal: 4}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_twin = get_package_share_directory('lupin_twin')
    pkg_slam = get_package_share_directory('slam_toolbox')

    args = [
        DeclareLaunchArgument('dependency_timeout_s', default_value='120.0'),
    ]

    widened_tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')
    approach_overrides = os.path.join(pkg_bringup, 'config', 'approach_overrides.yaml')

    # SLAM — subscribes /scan, owns /map + map→odom.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('slam_params_file', os.path.join(pkg_nav, 'config', 'slam_toolbox_sim.yaml')),
        ],
    )

    # Nav2 (slam mode) — subscribes /map.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
        ),
        launch_arguments=[
            ('slam', 'true'),
            ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
            ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
            ('use_sim_time', 'true'),
            ('autostart', 'true'),
        ],
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', widened_tag_locations)],
    )

    # Perception: ArUco tag detector (body cam) + aggregator + sim flower detector.
    tag_annotator = Node(
        package='lupin_perception', executable='tag_annotator', name='tag_annotator',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'detections_topic': '/camera/tag_detections_json',
            'tag_size_m': 0.036,
            'tf_frame_prefix': 'tag_',
            'image_qos': 'reliable',
            'fallback_intrinsics': [554.254691191187, 554.254691191187, 320.5, 240.5],
        }],
        output='screen',
    )
    perception_aggregator = Node(
        package='lupin_perception', executable='perception_aggregator', name='perception_aggregator',
        parameters=[{
            'use_sim_time': True,
            'tag_detections_topic': '/camera/tag_detections_json',
            'mission_state_topic': '/mission/state',
            'discovered_tags_topic': '/perception/discovered_tags',
            'observations_topic': '/floranova/observations',
            'confirm_service': '/perception/confirm_tag',
            'map_frame': 'map',
            'tf_frame_prefix': 'tag_',
            'min_sightings': 3,
            'max_tag_distance_m': 2.5,
        }],
        output='screen',
    )
    sim_flower_detector = Node(
        package='lupin_perception', executable='sim_flower_detector', name='sim_flower_detector',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/gripper_camera/image_raw',
            'detections_topic': '/yolo/detections',
            'image_qos': 'reliable',
            'publish_overlay': True,
        }],
        output='screen',
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
            ('tag_locations_file', widened_tag_locations),
            ('approach_overrides_file', approach_overrides),
            ('arm_patrol_enabled', 'true'),
            ('flower_scan_dwell_s', '4.0'),
            # Compact travel pose between pots ('home' is arm-horizontal-forward,
            # ~0.28 m reach → it clips the pots while driving).
            ('arm_travel_preset', 'tuck'),
            # Let the arm finish folding before driving (preset takes ~3 s).
            ('arm_travel_settle_s', '3.2'),
            # Sim runs on /clock — keep the orchestrator's observation stamps on
            # sim time so the twin doesn't treat the map pins as stale.
            ('use_sim_time', 'true'),
        ],
    )

    twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_twin, 'launch', 'twin.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'true')],
    )

    return LaunchDescription([
        *args,
        slam, nav2, bridge,
        tag_annotator, perception_aggregator, sim_flower_detector,
        mission, twin,
    ])
