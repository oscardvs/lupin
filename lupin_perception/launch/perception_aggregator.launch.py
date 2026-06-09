"""Launch the perception aggregator (tag discovery + flower fusion).

Consumes the tag detector's JSON/TF and the YOLO detector's JSON, and emits
the /perception/discovered_tags feed, KIND_FLOWER observations, and the
/perception/confirm_tag service. Light (no torch) — but its YOLO *input*
(yolo_detector) is heavy, so run both off-robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('tag_detections_topic', default_value='/camera/tag_detections_json'),
        DeclareLaunchArgument('yolo_detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('mission_state_topic', default_value='/mission/state'),
        DeclareLaunchArgument('discovered_tags_topic', default_value='/perception/discovered_tags'),
        DeclareLaunchArgument('observations_topic', default_value='/floranova/observations'),
        DeclareLaunchArgument('confirm_service', default_value='/perception/confirm_tag'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('tf_frame_prefix', default_value='tag_'),
        DeclareLaunchArgument('min_sightings', default_value='3'),
        DeclareLaunchArgument('max_tag_distance_m', default_value='2.5'),
        DeclareLaunchArgument('box_geometry_topic', default_value='/perception/box_geometry_json'),
        DeclareLaunchArgument(
            'tag_locations_file', default_value='',
            description='tag_locations.json with the real planter rectangles + '
                        'tag coords (empty -> bundled greenhouse_sim package).'),
        DeclareLaunchArgument(
            'flower_box_gate', default_value='true',
            description='Drop YOLO blooms whose projected map position falls '
                        "outside current_target's registered planter rectangle, "
                        'so an adjacent bench / over-reaching pan sweep is not '
                        'misattributed. Engages only once a MapBox is registered '
                        'for the target; a no-op in sim (blooms place in-bench).'),
        DeclareLaunchArgument(
            'flower_box_gate_margin_m', default_value='0.10',
            description='Metric slack on the rectangle for the gate above, so '
                        'nominal-FOV error does not drop edge-of-bench blooms.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    aggregator = Node(
        package='lupin_perception',
        executable='perception_aggregator',
        name='perception_aggregator',
        output='screen',
        parameters=[{
            'tag_detections_topic': LaunchConfiguration('tag_detections_topic'),
            'yolo_detections_topic': LaunchConfiguration('yolo_detections_topic'),
            'mission_state_topic': LaunchConfiguration('mission_state_topic'),
            'discovered_tags_topic': LaunchConfiguration('discovered_tags_topic'),
            'observations_topic': LaunchConfiguration('observations_topic'),
            'confirm_service': LaunchConfiguration('confirm_service'),
            'map_frame': LaunchConfiguration('map_frame'),
            'tf_frame_prefix': LaunchConfiguration('tf_frame_prefix'),
            'min_sightings': LaunchConfiguration('min_sightings'),
            'max_tag_distance_m': LaunchConfiguration('max_tag_distance_m'),
            'flower_box_gate': LaunchConfiguration('flower_box_gate'),
            'flower_box_gate_margin_m': LaunchConfiguration('flower_box_gate_margin_m'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    box_layout_publisher = Node(
        package='lupin_perception',
        executable='box_layout_publisher',
        name='box_layout_publisher',
        output='screen',
        parameters=[{
            'discovered_tags_topic': LaunchConfiguration('discovered_tags_topic'),
            'box_geometry_topic': LaunchConfiguration('box_geometry_topic'),
            'tag_locations_file': LaunchConfiguration('tag_locations_file'),
            'min_sightings': LaunchConfiguration('min_sightings'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([*args, aggregator, box_layout_publisher])
