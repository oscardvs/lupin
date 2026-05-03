import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find where the config file lives after installation
    config = os.path.join(
        get_package_share_directory('lupin_perception'),
        'config',
        'tags.yaml'
    )

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[config],
            remappings=[
                # Map standard detector topics to MIRTE's camera
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                ('tag_detections_image', '/camera/image_raw_boxed')
            ],
            output='screen'
        ),

        Node(
            package='lupin_perception',
            executable='tag_annotator',
            name='tag_annotator',
            output='screen'
        )
    ])