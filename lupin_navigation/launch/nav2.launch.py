"""nav2.launch.py — Lupin Nav2 entry point.

Brings up the full Nav2 lifecycle stack (localization + navigation)
configured for the MIRTE Master mecanum base in the Gazebo sim:

- localization: map_server + AMCL (OmniMotionModel, base_link, headless
  initial pose at the vendor spawn coords) + lifecycle_manager_localization
- navigation:   controller_server (MPPI/Omni) + planner_server +
  behavior_server + bt_navigator + waypoint_follower +
  velocity_smoother + lifecycle_manager_navigation

Wiring detail that matters: velocity_smoother's smoothed output is remapped
to /cmd_vel_auto so it feeds the lupin twist_mux as the lowest-priority
(autonomy) input. We do NOT publish to /cmd_vel (which in sim is the
vendor twist_mux output to the gazebo_planar_move teleport plugin) — Nav2
hits the same arbitration chain the Xbox dead-man and web HMI hit, and
twist_mux's priority+timeout logic stays in charge.

slam:=true mode: drops the localization half (map_server + AMCL +
lifecycle_manager_localization) so an external slam_toolbox node can own
/map and the map→odom TF instead. Use this when navigating in a world
without a saved map (e.g. the greenhouse) — slam_toolbox builds the map
online and Nav2 plans against it. Run slam_toolbox separately, e.g.:

    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \\
        slam_params_file:=$(ros2 pkg prefix lupin_navigation)/share/\\
        lupin_navigation/config/slam_toolbox_sim.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('lupin_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    lifecycle_nodes_localization = ['map_server', 'amcl']
    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Substitute the launch-time map path and use_sim_time into params.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml,
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto-activate the lifecycle managers'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'krr_house.yaml'),
            description='Full path to the .yaml map for map_server',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 ROS 2 params file',
        ),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description=(
                'If true, skip map_server/AMCL/localization lifecycle manager '
                'so an external slam_toolbox node can own /map and the '
                'map→odom TF. Run slam_toolbox separately.'
            ),
        ),

        # ── Localization ────────────────────────────────────────────────
        # Skipped under slam:=true — slam_toolbox publishes /map and the
        # map→odom TF instead, so map_server/AMCL would conflict.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            condition=UnlessCondition(slam),
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            condition=UnlessCondition(slam),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes_localization,
                'bond_timeout': 20.0,  # see navigation manager below — WiFi discovery lag
            }],
            condition=UnlessCondition(slam),
        ),

        # ── Navigation ──────────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            # Standard Nav2 chain: controller_server -> velocity_smoother -> /cmd_vel.
            # We keep cmd_vel_nav as the inter-stage hop; final output is rewired
            # below at the smoother.
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                # Hook into the lupin twist_mux's autonomous input (priority 10).
                # Do NOT publish to /cmd_vel here — that goes to gazebo_planar_move
                # and would bypass both the mux and the mecanum drive controller.
                ('cmd_vel_smoothed', '/cmd_vel_auto'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes_navigation,
                # Raised from Nav2's 4.0s default: over the FastDDS discovery-server
                # WiFi link the lifecycle<->server bond can take >4s to form on a cold
                # bringup (discovery round-trips through the robot's server at :11811),
                # which spuriously trips "unable to be reached after 4.00s by bond" and
                # aborts bringup. 20s absorbs the lag; on a wired link the bond still
                # forms in <1s so the larger ceiling is harmless.
                'bond_timeout': 20.0,
            }],
        ),
    ])
