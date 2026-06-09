"""sim_autonomy.launch.py — TERMINAL 2 of the layered sim bringup.

The brain: SLAM (slam_toolbox), Nav2 (slam mode), the greenhouse bridge, the
perception stack (tag_annotator + perception_aggregator + sim flower detector),
the passive climate observer (fills the twin heatmap during idle/teleop), the
mission orchestrator, and the digital twin.

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
    # tag_size_m = detectable black-border edge of the rendered tag36h11:
    # texture plane 0.04*0.9 = 0.036 m, minus the 10x10 PNG's 1-cell white quiet
    # zone (black square = 8/10 of plane) -> 0.036*0.8 = 0.0288 m. See sim_full.
    tag_annotator = Node(
        package='lupin_perception', executable='tag_annotator', name='tag_annotator',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'detections_topic': '/camera/tag_detections_json',
            'tag_size_m': 0.0288,
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
    # PRODUCER for the box-geometry contract: known layout registered to the map.
    box_layout_publisher = Node(
        package='lupin_perception', executable='box_layout_publisher', name='box_layout_publisher',
        parameters=[{
            'use_sim_time': True,
            'discovered_tags_topic': '/perception/discovered_tags',
            'box_geometry_topic': '/perception/box_geometry_json',
            'tag_locations_file': widened_tag_locations,
            'min_sightings': 3,
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
    # Passive climate poller — fills the twin's heatmap/per-tag readings while
    # NO mission is running (teleop / manual SLAM). It watches discovered tags,
    # polls the greenhouse bridge per tag, and republishes KIND_TAG_READING on
    # /floranova/observations. It defers entirely while a mission is active (the
    # orchestrator owns observation then), so it is safe to always run alongside
    # the mission stack. use_sim_time:=true so its observation stamps are on /clock
    # and the twin doesn't treat the readings as stale (same reason the mission
    # node sets it above).
    passive_observer = Node(
        package='lupin_perception', executable='passive_observer', name='passive_observer',
        parameters=[{
            'use_sim_time': True,
            # Re-poll every 30 s so the heatmap tracks the sim oracle's
            # time-of-day drift instead of freezing on the first reading.
            'refresh_period_s': 30.0,
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
            # Compact travel/stow pose between pots. FK (base_link): 'zero' =
            # gripper straight UP at x=0.09 m (inside the base footprint); 'tuck'
            # and 'home' actually extend 0.23 / 0.13 m forward and clip in the
            # tight aisles. Vertical 'zero' is also ~zero shoulder gravity-moment.
            ('arm_travel_preset', 'zero'),
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
        tag_annotator, perception_aggregator, box_layout_publisher, sim_flower_detector,
        passive_observer,
        mission, twin,
    ])
