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
            package='lupin_perception',
            executable='tag_annotator',
            name='tag_annotator',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])