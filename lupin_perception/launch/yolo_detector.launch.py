"""Launch the YOLO detector/tracker and an image viewer for annotated detections."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'image_topic',
            default_value='/gripper_camera/image_raw/compressed',
            description='Compressed camera image stream to run YOLO detections/tracking on.',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/yolo/image_detections',
            description='Annotated image topic with YOLO bounding boxes and track IDs.',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo/detections',
            description='JSON detection/tracking metadata topic.',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('lupin_perception'),
                'models',
                'best.pt',
            ]),
            description='Path to the YOLO .pt model.',
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.25',
            description='YOLO confidence threshold.',
        ),
        DeclareLaunchArgument(
            'imgsz',
            default_value='640',
            description='YOLO inference image size.',
        ),
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Enable YOLO tracking instead of frame-by-frame detection.',
        ),
        DeclareLaunchArgument(
            'tracker',
            default_value='bytetrack.yaml',
            description='Tracker config file. Common options: bytetrack.yaml or botsort.yaml.',
        ),
        DeclareLaunchArgument(
            'start_viewer',
            default_value='true',
            description='Open rqt_image_view on the annotated detections feed.',
        ),
    ]

    yolo_detector = Node(
        package='lupin_perception',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf': LaunchConfiguration('conf'),
            'imgsz': LaunchConfiguration('imgsz'),
            'enable_tracking': LaunchConfiguration('enable_tracking'),
            'tracker': LaunchConfiguration('tracker'),
        }],
        remappings=[
            ('/gripper_camera/image_raw/compressed', LaunchConfiguration('image_topic')),
            ('yolo/image_detections', LaunchConfiguration('annotated_image_topic')),
            ('yolo/detections', LaunchConfiguration('detections_topic')),
        ],
    )

    detection_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='yolo_detection_viewer',
        output='screen',
        arguments=[LaunchConfiguration('annotated_image_topic')],
        condition=IfCondition(LaunchConfiguration('start_viewer')),
    )

    return LaunchDescription([*args, yolo_detector, detection_viewer])