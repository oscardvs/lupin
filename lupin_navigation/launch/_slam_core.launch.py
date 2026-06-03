"""_slam_core.launch.py — shared SLAM core (slam_toolbox + map-reset service).

NOT run directly — use the environment wrappers instead:
    ros2 launch lupin_navigation slam_hardware.launch.py   (use_sim_time:=false)
    ros2 launch lupin_navigation slam_sim.launch.py        (use_sim_time:=true)

Brings up exactly what hardware.launch.py's slam:=true does:

    slam_toolbox (async)  → /map + map→odom TF        (respawn=True)
    slam_reset_node       → /lupin/nav/clear_map srv   (HMI "Erase map" SIGTERMs
                            slam_toolbox; respawn brings it back with a blank graph)

The single env difference (use_sim_time) is a launch arg, not baked into the
yaml — the slam_toolbox params are shared sim/hardware.

Ordering note: Nav2 must come up AFTER /map exists and the odom→base_link TF is
warm, or its lifecycle wedges on "Invalid frame ID base_link". Run SLAM, confirm
/map (RViz, or `ros2 topic echo --once --qos-reliability reliable
--qos-durability transient_local /map nav_msgs/msg/OccupancyGrid`), then launch
nav2_<env>.launch.py. hardware.launch.py automates this with sentinels.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('lupin_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (true for Gazebo). Hardware = false. '
                        'Normally set by the slam_hardware / slam_sim wrapper.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_nav, 'config', 'slam_toolbox_sim.yaml'),
            description='slam_toolbox params yaml. Shared sim/hardware — only '
                        'use_sim_time differs and is set via the launch arg.',
        ),
    ]

    # Spawned directly (not via online_async_launch.py) with respawn=True so the
    # /lupin/nav/clear_map service can SIGTERM it to wipe the map and have it
    # respawn with an empty pose graph. Mirrors hardware.launch.py's slam_node.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=1.0,
        output='screen',
    )

    # Owns /lupin/nav/clear_map (Trigger). The HMI "Erase map" button hits this;
    # it SIGTERMs slam_toolbox (respawn above brings it back blank).
    slam_reset_node = Node(
        package='lupin_navigation',
        executable='slam_reset_node',
        name='slam_reset_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    return LaunchDescription([*args, slam_node, slam_reset_node])
