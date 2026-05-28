"""Launch both perception pipelines from one entry point.

This keeps the existing single-purpose launch files intact while giving
operators one command that starts the AprilTag detector and the YOLO
detector together.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'tag_image_topic',
            default_value='/camera/color/image_raw',
            description='Image topic forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_camera_info_topic',
            default_value='/camera/color/camera_info',
            description='CameraInfo topic forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_detections_topic',
            default_value='/camera/tag_detections_json',
            description='Detection JSON topic forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_size_m',
            default_value='0.04',
            description='Tag edge length forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_tf_frame_prefix',
            default_value='tag_',
            description='TF frame prefix forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_image_qos',
            default_value='sensor_data',
            description='Image QoS forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'tag_use_sim_time',
            default_value='false',
            description='use_sim_time forwarded to perception.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_image_topic',
            default_value='/gripper_camera/image_raw',
            description='Image topic forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_annotated_image_topic',
            default_value='/yolo/image_detections',
            description='Annotated image topic forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_detections_topic',
            default_value='/yolo/detections',
            description='Detection topic forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('lupin_perception'),
                'models',
                'best.pt',
            ]),
            description='Model path forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_conf',
            default_value='0.25',
            description='Confidence threshold forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_imgsz',
            default_value='640',
            description='Inference size forwarded to yolo_detector.launch.py.',
        ),
        DeclareLaunchArgument(
            'yolo_start_viewer',
            default_value='true',
            description='Viewer toggle forwarded to yolo_detector.launch.py.',
        ),
    ]

    tag_annotator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lupin_perception'),
                'launch',
                'perception.launch.py',
            ])
        ),
        launch_arguments=[
            ('image_topic', LaunchConfiguration('tag_image_topic')),
            ('camera_info_topic', LaunchConfiguration('tag_camera_info_topic')),
            ('detections_topic', LaunchConfiguration('tag_detections_topic')),
            ('tag_size_m', LaunchConfiguration('tag_size_m')),
            ('tf_frame_prefix', LaunchConfiguration('tag_tf_frame_prefix')),
            ('image_qos', LaunchConfiguration('tag_image_qos')),
            ('use_sim_time', LaunchConfiguration('tag_use_sim_time')),
        ],
    )

    yolo_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lupin_perception'),
                'launch',
                'yolo_detector.launch.py',
            ])
        ),
        launch_arguments=[
            ('image_topic', LaunchConfiguration('yolo_image_topic')),
            ('annotated_image_topic', LaunchConfiguration('yolo_annotated_image_topic')),
            ('detections_topic', LaunchConfiguration('yolo_detections_topic')),
            ('model_path', LaunchConfiguration('yolo_model_path')),
            ('conf', LaunchConfiguration('yolo_conf')),
            ('imgsz', LaunchConfiguration('yolo_imgsz')),
            ('start_viewer', LaunchConfiguration('yolo_start_viewer')),
        ],
    )

    return LaunchDescription([*args, tag_annotator, yolo_detector])