"""hardware.launch.py — Lupin laptop-side bring-up against the real Mirte.

Single, modular entry point for everything that runs on the operator's
laptop on top of the robot's onboard services. Each subsystem is behind
a boolean flag so the operator can opt in or out without editing files.

Topology after launch (defaults):

    Robot (already running via systemd):
        mirte-ros.service               telemetrix, controllers,
                                        RPLidar → /scan, cameras, vendor
                                        rosbridge :9090 (idle — HMI
                                        doesn't connect here), vendor
                                        web_video_server :8181
        lupin-onboard.service           twist_mux (cmd_vel arbitration),
                                        arm_preset_server,
                                        gripper_action_bridge
        lupin-cameras-throttle.service  /camera/* → /lupin/camera/* @ 1 Hz

    Laptop (this launch):
        lupin_web (Vite + web_video + rosbridge)              (web:=true)
        slam_toolbox        → /map, map→odom TF              (slam:=true)
        slam_reset_node     → /lupin/nav/clear_map service   (slam:=true)
        nav2 (slam mode)    → /cmd_vel_auto via smoother     (nav2:=true)
        lupin_twin          → /twin/state + /twin/get_field  (twin:=true)
        greenhouse_bridge   → /floranova/* oracle            (mission:=true)
        mission_orchestrator→ /mission/start + lifecycle     (mission:=true)
        seed_amcl_pose      → one-shot /amcl_pose            (mission:=true)
        xbox_teleop         → /cmd_vel_joy + arm_teleop      (joystick:=true)
        rviz2               → interactive UI                 (rviz:=true)

Flags (all booleans, default in parens):

    web (true)        lupin_web — Vite preview (HTTPS :8090) +
                      rosbridge_websocket :9090 + web_video_server :8091.
                      Open https://<laptop-ip>:8090 to use the HMI.
    slam (true)       slam_toolbox + slam_reset_node. Owns /map.
    nav2 (true)       Nav2 stack. Waits for /map before activating.
    twin (true)       lupin_twin aggregator. Cheap; HMI consumes it.
    mission (false)   greenhouse_bridge + mission_orchestrator + AMCL
                      pose seed as a bundle. Turn on for mission runs.
    rviz (true)       RViz2 with the persistent full_bringup_viz config.
    joystick (false)  Xbox controller teleop on the laptop.

Common invocations:

    # Default operator mode — HMI + SLAM + Nav2 + twin + RViz.
    # Open https://<laptop-ip>:8090 once the Vite line "ready in NNN ms"
    # shows. The rosbridge pill in the top bar goes green within a couple
    # of seconds.
    ros2 launch lupin_bringup hardware.launch.py

    # Headless smoke test — no HMI, no RViz window.
    ros2 launch lupin_bringup hardware.launch.py web:=false rviz:=false

    # Full mission run (bridge + orchestrator on top of the default HMI +
    # Nav2 + slam + RViz).
    ros2 launch lupin_bringup hardware.launch.py mission:=true

    # Skip the autonomy stack — just the HMI on top of a parked robot.
    ros2 launch lupin_bringup hardware.launch.py slam:=false nav2:=false

The sentinel cascade (wait_for_scan → slam → wait_for_map → wait_for_tf →
nav2) only fires when its target subsystem is enabled, so disabling one
stage doesn't block the rest.
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_twin = get_package_share_directory('lupin_twin')
    pkg_hmi = get_package_share_directory('lupin_hmi')
    pkg_web = get_package_share_directory('lupin_web')

    args = [
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='Bring up slam_toolbox (+ slam_reset_node). '
                        'Owns /map and the map→odom TF. Turn off when '
                        'using a saved map with a separate map_server.',
        ),
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='Bring up the Nav2 stack in slam mode. Waits for '
                        '/map before activating.',
        ),
        DeclareLaunchArgument(
            'twin', default_value='true',
            description='Bring up lupin_twin (aggregates '
                        '/floranova/observations into /twin/state). Cheap '
                        'and the HMI Twin tab is a pure consumer.',
        ),
        DeclareLaunchArgument(
            'mission', default_value='false',
            description='Bring up the mission pipeline as a bundle: '
                        'greenhouse_bridge (oracle), mission_orchestrator '
                        'lifecycle node, and a one-shot /amcl_pose seed. '
                        'Turn on for end-to-end mission runs.',
        ),
        DeclareLaunchArgument(
            'web', default_value='true',
            description='Bring up the Lupin Web HMI on the laptop: Vite '
                        'preview (HTTPS :8090) + rosbridge_websocket :9090 '
                        '+ web_video_server :8091. Open https://<laptop-ip>'
                        ':8090 in a browser to use it. Default true so the '
                        'one-shot bringup gives the operator a working HMI '
                        'without a second command. The Pi is no longer in '
                        'the JSON-encoding path — that load lives here. '
                        'Vendor rosbridge :9090 on the robot stays running '
                        'but sits idle.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the persistent config at '
                        'rviz/full_bringup_viz.rviz. Ctrl+S in RViz writes '
                        'back to that exact path.',
        ),
        DeclareLaunchArgument(
            'joystick', default_value='false',
            description='Bring up Xbox controller teleop (joy_node + '
                        'teleop_twist_joy + arm_teleop) on the laptop. '
                        'False avoids noisy joy_node logs when no '
                        'controller is plugged in.',
        ),
        DeclareLaunchArgument(
            'dependency_timeout_s', default_value='120.0',
            description='How long the mission orchestrator waits for Nav2 '
                        '+ bridge before transitioning to FAULT. Bumped '
                        'from the 30 s default because Nav2 lifecycle '
                        'activation in slam mode takes longer than that '
                        'on a cold start.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_nav, 'config', 'slam_toolbox_sim.yaml',
            ),
            description='slam_toolbox params yaml. Sim and hardware share '
                        'this — the only sim-specific bit (use_sim_time) '
                        'is overridden via launch arg, not the yaml.',
        ),
    ]

    # Tag layout — for now the bridge consumes the same widened sim layout
    # so the orchestrator can be smoke-tested against an oracle bridge on
    # hardware before real perception lands. Per project_approach_pose_pipeline
    # the operator can tune approach poses in approach_overrides.yaml without
    # rebuilding. Replace with tag_locations_hardware.json once the demo
    # space is measured.
    tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')
    approach_overrides = os.path.join(pkg_bringup, 'config', 'approach_overrides.yaml')

    # ── SLAM ───────────────────────────────────────────────────────────
    # Sentinel: wait for /scan from the robot's RPLidar. On hardware the
    # robot's rplidar_node is already running, so /scan is usually live
    # before this fires. The cascade still helps when the laptop launches
    # before DDS discovery has propagated topics.
    wait_for_scan = ExecuteProcess(
        name='wait_for_scan',
        cmd=['ros2', 'topic', 'echo', '--once', '/scan',
             'sensor_msgs/msg/LaserScan'],
        output='log',
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # Spawned directly (not via online_async_launch.py) so we can pin
    # respawn=True. The /lupin/nav/clear_map service exposed by
    # slam_reset_node SIGTERMs this process to wipe the map; respawn then
    # brings it back up with an empty pose graph.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': False},
        ],
        respawn=True,
        respawn_delay=1.0,
        output='screen',
    )
    on_scan_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[
                LogInfo(msg='[lupin_bringup] /scan online — starting slam_toolbox'),
                slam_node,
            ],
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # slam_reset — owns /lupin/nav/clear_map (Trigger). HMI hits this to
    # wipe the SLAM map; node SIGTERMs slam_toolbox + clears costmaps.
    slam_reset_node = Node(
        package='lupin_navigation',
        executable='slam_reset_node',
        name='slam_reset_node',
        parameters=[{'use_sim_time': False}],
        output='log',
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # ── Nav2 ───────────────────────────────────────────────────────────
    # Sentinel: wait for /map. RELIABLE+TRANSIENT_LOCAL; tell echo to
    # match so the QoS negotiation actually connects. When slam=false the
    # operator is responsible for providing /map (separate map_server);
    # this sentinel still fires correctly whether slam_toolbox or
    # map_server publishes it.
    wait_for_map = ExecuteProcess(
        name='wait_for_map',
        cmd=['ros2', 'topic', 'echo', '--once',
             '--qos-reliability', 'reliable',
             '--qos-durability', 'transient_local',
             '/map', 'nav_msgs/msg/OccupancyGrid'],
        output='log',
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

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

    # ── TF-ready sentinel between /map and Nav2 lifecycle ─────────────
    # On hardware, a cold DDS-over-WiFi /tf subscription on the laptop
    # takes ~5–10 s to populate the buffer with the robot's odom →
    # base_link transform. Nav2's local_costmap activation does a single
    # canTransform() call with a short retry budget and bails with
    # "Invalid frame ID base_link" if TF isn't hot yet, leaving the
    # lifecycle stuck. This sentinel blocks until an external listener
    # resolves the transform, then exits — gating Nav2's launch.
    wait_for_tf = ExecuteProcess(
        name='wait_for_tf',
        cmd=['ros2', 'run', 'lupin_bringup', 'wait_for_tf',
             'odom', 'base_link', '30.0'],
        output='log',
        condition=IfCondition(LaunchConfiguration('nav2')),
    )
    on_map_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map,
            on_exit=[
                LogInfo(msg='[lupin_bringup] /map online — waiting for tf '
                            'odom→base_link to warm up before Nav2'),
                wait_for_tf,
            ],
        ),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )
    on_tf_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_tf,
            on_exit=[
                LogInfo(msg='[lupin_bringup] tf hot — starting Nav2 lifecycle '
                            '(autostart=true; expect ~15-30 s to ACTIVE)'),
                nav2_include,
            ],
        ),
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # ── Mission pipeline (bridge + orchestrator + AMCL seed) ───────────
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', tag_locations)],
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
            ('tag_locations_file', tag_locations),
            ('approach_overrides_file', approach_overrides),
        ],
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    # The orchestrator subscribes to /amcl_pose at t=0 and caches whatever
    # the seed publishes — no race with the slam/Nav2 cascade.
    # PREPARE.LOCALIZING clears immediately when a mission is started.
    seed = ExecuteProcess(
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
        output='log',
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    # ── Digital twin (HMI live-state aggregator) ───────────────────────
    twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_twin, 'launch', 'twin.launch.py'),
        ),
        condition=IfCondition(LaunchConfiguration('twin')),
    )

    # ── Lupin Web HMI — Vite + rosbridge + web_video_server ───────────
    # tls:=true + rosbridge:=true mirrors what the old laptop systemd unit
    # ran. Co-located rosbridge so the JSON-encoding load lives on the
    # laptop, not the Pi (Phase-2 split per project_offload_strategy).
    # When you don't want the HMI (headless smoke tests, CI), pass
    # web:=false.
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web, 'launch', 'lupin_web.launch.py'),
        ),
        launch_arguments=[
            ('mode', 'preview'),
            ('tls', 'true'),
            ('rosbridge', 'true'),
        ],
        condition=IfCondition(LaunchConfiguration('web')),
    )

    # ── Xbox controller teleop (optional, joystick on the laptop) ──────
    # Joy → teleop_twist_joy → /cmd_vel_joy (twist_mux input, priority 100).
    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'false')],
        condition=IfCondition(LaunchConfiguration('joystick')),
    )

    # ── RViz with persistent source-tree config ────────────────────────
    rviz_config = _resolve_rviz_config()
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lupin_rviz',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # NOTE: twist_mux, arm_preset_server, and gripper_action_bridge USED
    # to live in this launch; they moved to lupin-onboard.service on the
    # robot so the operator can drive the chassis, move the arm, and
    # operate the gripper the moment the robot finishes booting — no
    # laptop launch required. See onboard.launch.py.

    return LaunchDescription([
        *args,
        LogInfo(msg=['[lupin_bringup] hardware: laptop-side bring-up. ',
                     'web=', LaunchConfiguration('web'),
                     ' slam=', LaunchConfiguration('slam'),
                     ' nav2=', LaunchConfiguration('nav2'),
                     ' twin=', LaunchConfiguration('twin'),
                     ' mission=', LaunchConfiguration('mission'),
                     ' rviz=', LaunchConfiguration('rviz'),
                     ' joystick=', LaunchConfiguration('joystick')]),
        # Phase 1 — fire-and-forget at t=0 (each gated by its own flag):
        web,
        slam_reset_node,
        bridge,
        mission,
        twin,
        seed,
        xbox_teleop,
        rviz,
        # Sentinels: tiny "wait for topic" processes that exit on first
        # message receipt. Their exit fires the next stage.
        wait_for_scan,
        wait_for_map,
        # Event handlers: chain the cascade.
        on_scan_ready,
        on_map_ready,
        on_tf_ready,
    ])


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
