"""Launch the `tag_annotator` AprilTag detector against the real Mirte.

Defaults match the vendor Orbbec stack on the MIRTE Master (astra_camera
publishes /camera/color/image_raw + /camera/color/camera_info). Override
any of the launch args to retarget at the throttled `/lupin/camera/...`
topic, the gripper cam, or a custom calibrated camera_info.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='Camera image stream to detect AprilTags on. Vendor '
                        'Orbbec default; switch to /lupin/camera/color/'
                        'image_raw to match the HMI throttle, or to '
                        '/gripper_camera/image_raw for the wrist cam.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='CameraInfo source used to populate the intrinsic '
                        'matrix K and distortion coefficients. Must be '
                        'rectified for the chosen image_topic.',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/camera/tag_detections_json',
            description='Where the HMI subscribes for the overlay JSON.',
        ),
        DeclareLaunchArgument(
            'tag_size_m', default_value='0.10',
            description='Physical edge length of the printed tags, in metres.',
        ),
        DeclareLaunchArgument(
            'tf_frame_prefix', default_value='tag_',
            description='Child-frame prefix for the broadcast tag transforms.',
        ),
        DeclareLaunchArgument(
            'image_qos', default_value='sensor_data',
            description='QoS profile for the image subscription: '
                        '"sensor_data" (BEST_EFFORT) connects to both the '
                        'vendor RELIABLE driver and BEST_EFFORT republishers '
                        '(topic_tools throttle); "reliable" matches the '
                        'sim-style profile.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set true only when replaying a bag with /clock; the '
                        'real robot has no /clock publisher.',
        ),
    ]

    tag_annotator = Node(
        package='lupin_perception',
        executable='tag_annotator',
        name='tag_annotator',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'tag_size_m': LaunchConfiguration('tag_size_m'),
            'tf_frame_prefix': LaunchConfiguration('tf_frame_prefix'),
            'image_qos': LaunchConfiguration('image_qos'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([*args, tag_annotator])
