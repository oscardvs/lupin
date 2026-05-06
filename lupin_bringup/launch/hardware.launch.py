"""hardware.launch.py — Lupin Nav2 stack against the real Mirte.

Laptop-side bring-up that adds Nav2 + online slam_toolbox + RViz on top of
the robot's already-running vendor stack (mirte-ros.service: telemetrix,
ros2_control, RPLidar, cameras, rosbridge :9090, vendor web_video_server).
The Lupin Web HMI is served separately by lupin-web.service on :8090; this
launch does NOT start either rosbridge or the HMI — they're already up and
re-launching would just bind-conflict.

Topology after launch:

    Robot (mirte-ros.service):
        rplidar       → /scan
        controllers   → /mirte_base_controller/odom + TF (odom→base_link)
        controllers   ← /mirte_base_controller/cmd_vel  (Twist, BEST_EFFORT)
        rosbridge_websocket :9090
        web_video_server :8181 (vendor, localhost only)

    Laptop (this launch):
        slam_toolbox  → /map, map→odom TF
        nav2          → /cmd_vel_auto via velocity_smoother
        twist_mux     → /mirte_base_controller/cmd_vel
        rviz2         (interactive)

    Browser (lupin-web.service on robot, port :8090):
        publishes /cmd_vel_manual and /goal_pose via rosbridge

Online SLAM is the default — drive around with the joystick (Xbox or web)
or RViz "2D Goal Pose", and slam_toolbox builds the map as we move. The
map can be saved with `nav2_map_server map_saver_cli` (transient_local
durability — see project_mirte_sim_nav_quirks for the flag) once the demo
space is mapped.

Deviations from sim_full.launch.py
----------------------------------
- No `greenhouse_sim` — robot is the source of /scan, /odom, /tf.
- No `arm_sim_shim` — real Mirte exposes Hiwonder services natively.
- No `rosbridge` / `lupin_web` Includes — both run on the robot via systemd.
- All `use_sim_time:=false`.
- twist_mux output → /mirte_base_controller/cmd_vel (stamped, not _unstamped).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')  # noqa: F841
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_hmi = get_package_share_directory('lupin_hmi')
    pkg_slam = get_package_share_directory('slam_toolbox')

    args = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz alongside Nav2 with the persistent '
                        'config at rviz/full_bringup_viz.rviz. Ctrl+S in '
                        'RViz writes back to that exact path.',
        ),
        DeclareLaunchArgument(
            'joystick', default_value='false',
            description='Bring up Xbox controller teleop (joy_node + '
                        'teleop_twist_joy + arm_teleop). Set true if a USB '
                        'gamepad is plugged into the laptop; false avoids '
                        'noisy joy_node logs when no controller is present.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_nav, 'config', 'slam_toolbox_sim.yaml',
            ),
            description='slam_toolbox params yaml. Sim and hardware share '
                        'this — the only sim-specific bit (use_sim_time) is '
                        'overridden via launch arg, not the yaml.',
        ),
    ]

    # ── Sentinel: wait for /scan from the robot's RPLidar ──────────────
    # On hardware the robot's rplidar_node is already running, so /scan is
    # usually live before this fires. The cascade still helps when the
    # laptop launches before DDS discovery has propagated topics.
    wait_for_scan = ExecuteProcess(
        name='wait_for_scan',
        cmd=['ros2', 'topic', 'echo', '--once', '/scan',
             'sensor_msgs/msg/LaserScan'],
        output='log',
    )

    # ── slam_toolbox starts after /scan is up ──────────────────────────
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('slam_params_file', LaunchConfiguration('slam_params_file')),
        ],
    )
    on_scan_ready = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_scan,
        on_exit=[
            LogInfo(msg='[lupin_bringup] /scan online — starting slam_toolbox'),
            slam_include,
        ],
    ))

    # ── Sentinel: wait for /map from slam_toolbox ──────────────────────
    # /map is RELIABLE+TRANSIENT_LOCAL; tell echo to match so the QoS
    # negotiation actually connects.
    wait_for_map = ExecuteProcess(
        name='wait_for_map',
        cmd=['ros2', 'topic', 'echo', '--once',
             '--qos-reliability', 'reliable',
             '--qos-durability', 'transient_local',
             '/map', 'nav_msgs/msg/OccupancyGrid'],
        output='log',
    )

    # ── Nav2 (slam mode) starts after /map is up ───────────────────────
    # `map:=krr_house.yaml` is forwarded even though slam:=true means
    # map_server isn't instantiated — RewrittenYaml still substitutes the
    # path into the params blob and a literal '' makes it explode.
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
        ),
        launch_arguments=[
            ('slam', 'true'),
            ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
            ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
            ('use_sim_time', 'false'),
            ('autostart', 'true'),
        ],
    )
    on_map_ready = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map,
        on_exit=[
            LogInfo(msg='[lupin_bringup] /map online — starting Nav2 lifecycle '
                        '(autostart=true; expect ~15-30 s to ACTIVE)'),
            nav2_include,
        ],
    ))

    # ── twist_mux — same priority arbitration as sim, hardware sink ────
    # Three Twist sources are arbitrated by priority + timeout:
    #   /cmd_vel_joy    Xbox dead-man teleop      (prio 100) ── operator
    #   /cmd_vel_manual web HMI joystick widget   (prio  50) ── remote operator
    #   /cmd_vel_auto   Nav2 velocity_smoother    (prio  10) ── autonomy
    # Output goes to /mirte_base_controller/cmd_vel (Twist, BEST_EFFORT) —
    # the topic the real Mirte's mecanum controller listens on. (Sim uses
    # _unstamped; see project_drive_topic memory.)
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[
            os.path.join(pkg_hmi, 'config', 'twist_mux.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[
            ('cmd_vel_out', '/mirte_base_controller/cmd_vel'),
        ],
        output='log',
    )

    # ── arm_preset_server — named arm-pose service ─────────────────────
    # Same node as sim — emits JointTrajectory on /mirte_master_arm_controller/
    # joint_trajectory which the real Mirte's controller listens on natively.
    arm_preset_server = Node(
        package='lupin_hmi', executable='arm_preset_server',
        name='arm_preset_server',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ── Xbox controller teleop (optional) ──────────────────────────────
    # Joy → teleop_twist_joy → /cmd_vel_joy (twist_mux input, priority 100).
    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'false')],
        condition=_when('joystick'),
    )

    # ── RViz with persistent source-tree config ─────────────────────────
    rviz_config = _resolve_rviz_config()
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lupin_rviz',
        arguments=['-d', rviz_config],
        output='log',
        condition=_when('rviz'),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] hardware: Nav2 + online slam_toolbox '
                    '+ RViz against the real Mirte. /scan → slam_toolbox; '
                    '/map → Nav2.'),
        # Phase 1 — fire-and-forget at t=0:
        twist_mux,
        arm_preset_server,
        xbox_teleop,
        rviz,
        # Sentinels: tiny "wait for topic" processes that exit on first
        # message receipt. Their exit fires the next stage.
        wait_for_scan,
        wait_for_map,
        # Event handlers: chain the cascade.
        on_scan_ready,
        on_map_ready,
    ])


def _when(arg_name: str):
    """Tiny helper — IfCondition needs a substitution, so wrap a launch arg
    as a truthy/falsy condition for the include."""
    from launch.conditions import IfCondition
    return IfCondition(LaunchConfiguration(arg_name))


def _resolve_rviz_config() -> str:
    """Resolve the RViz config path; prefers the source-tree path so Ctrl+S
    in RViz writes back to a location that survives `colcon build` and shows
    up in `git status` for committing."""
    candidates = [
        os.environ.get('LUPIN_RVIZ_CONFIG'),
        os.path.expanduser(
            '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz',
        ),
        os.path.join(
            get_package_share_directory('lupin_bringup'),
            'rviz', 'full_bringup_viz.rviz',
        ),
    ]
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return os.path.expanduser(
        '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz',
    )
