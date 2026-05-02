"""sim_full.launch.py — bring up the WHOLE simulation stack in one shot.

Composes, with an event-driven cascade between the upstream-dependent
stages:

    Phase 1 (t=0 — fire and forget):
        - greenhouse_sim.launch.py    — Gazebo + MIRTE + greenhouse world
        - greenhouse_bridge           — sensor service for tag readings
        - mission_orchestrator        — v2 lifecycle node (idles in READY)
        - rosbridge_websocket         — :9090 for the web HMI
        - lupin_web.launch.py         — Vite preview on :8090 + web_video :8091
        - seed_amcl_pose              — one-shot synthetic /amcl_pose

    Phase 2 (when /scan first publishes):
        - slam_toolbox online_async   — online SLAM, owns /map + map→odom

    Phase 3 (when /map first publishes):
        - lupin_navigation nav2.launch.py — Nav2 in slam mode (no AMCL)

The chain uses tiny `ros2 topic echo --once` sentinels and
RegisterEventHandler(OnProcessExit) — no fixed delays. If Gazebo never
publishes /scan (crash, missing plugin), slam_toolbox simply doesn't
fire and you can see exactly which stage stalled instead of drowning
in retry warnings.

The orchestrator subscribes to /amcl_pose at t=0 and caches whatever
the seed publishes — no race with the cascade. PREPARE.LOCALIZING
clears immediately whenever a mission is started.

After bringup:
    1. Open http://localhost:8090
    2. (one-time) In Settings → Drive, set cmd_vel topic to ``/cmd_vel_manual``
       so the web joystick goes through the sim's twist_mux instead of
       racing Nav2 on the controller topic.
    3. Drive a small loop in the Teleop tab so slam_toolbox has scan context.
    4. ``ros2 service call /mission/start lupin_msgs/srv/StartMission \\
         "{mission_type: 'InspectionMission', tag_sequence: ['1','2']}"``

Most arguments are forwarded to the included launches with the same name.
The launch is opinionated about what slam_toolbox / Nav2 / mission needs —
override individual args via ``--launch-arguments``.

Known fragility:
- All eight components start in parallel. DDS discovery + the orchestrator's
  dependency wait absorb the ordering, but on a cold laptop you may see
  ~30 s of "waiting for nav2..." chatter before things go green.
- If the orchestrator hits ``dependency_timeout_s`` before Nav2's lifecycle
  finishes, it transitions to FAULT. We bump the default to 120 s here.
"""

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_web = get_package_share_directory('lupin_web')
    pkg_rosbridge = get_package_share_directory('rosbridge_server')
    pkg_slam = get_package_share_directory('slam_toolbox')

    args = [
        # forwarded to greenhouse_sim — defaults match greenhouse_sim's own
        # defaults (south aisle facing the tables). The (1.0, 0.5) corner
        # spawn was wrong: it lands the robot against the south wall.
        DeclareLaunchArgument(
            'spawn_x', default_value='2.0',
            description='Mirte spawn X in the greenhouse world (m).',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='1.5',
            description='Mirte spawn Y in the greenhouse world (m).',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='1.5708',
            description='Mirte spawn yaw in the greenhouse world (rad). '
                        '1.5708 (90°) faces the tables along +X.',
        ),
        # web HMI
        DeclareLaunchArgument(
            'web_port', default_value='8090',
            description='HTTP port for the Lupin Web HMI Vite preview.',
        ),
        DeclareLaunchArgument(
            'enable_web', default_value='true',
            description='Bring up rosbridge + lupin_web. Set to false for '
                        'a headless smoke test.',
        ),
        # mission tunables
        DeclareLaunchArgument(
            'dependency_timeout_s', default_value='120.0',
            description='How long the mission orchestrator waits for Nav2 + '
                        'bridge before transitioning to FAULT. Bumped from '
                        'the 30 s default because Nav2 lifecycle activation '
                        'in slam mode takes longer than that on a cold start.',
        ),
        DeclareLaunchArgument(
            'seed_amcl', default_value='true',
            description='Publish a synthetic /amcl_pose to clear the '
                        'PREPARE.LOCALIZING gate. Slam mode has no AMCL, so '
                        'without this the orchestrator faults at PREPARE.',
        ),
        # rviz
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz alongside the sim, loaded with the '
                        'persistent config at rviz/full_bringup_viz.rviz '
                        'in the source tree. Ctrl+S in RViz writes back to '
                        'that exact path so the layout survives colcon '
                        'build and can be committed to git.',
        ),
    ]

    # ── 1. Gazebo + MIRTE + greenhouse world ────────────────────────────
    greenhouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'greenhouse_sim.launch.py'),
        ),
        launch_arguments=[
            ('x', LaunchConfiguration('spawn_x')),
            ('y', LaunchConfiguration('spawn_y')),
            ('yaw', LaunchConfiguration('spawn_yaw')),
        ],
    )

    # Event-driven cascade: each downstream component starts only after
    # the upstream readiness signal it depends on actually appears,
    # instead of relying on fixed delays.
    #
    #   greenhouse_sim → /scan publishes  ──► slam_toolbox starts
    #                                          ↓
    #                         slam publishes /map  ──► Nav2 starts
    #
    # Sentinels are tiny `ros2 topic echo --once` processes that exit on
    # first message receipt. Their exit fires a RegisterEventHandler
    # that kicks the next stage. If a sentinel never sees a message
    # (Gazebo crashed, slam_toolbox never started), nothing further runs
    # — visible as "next stage didn't fire" rather than a flood of
    # "waiting for transform" warnings the operator has to learn to
    # ignore.

    # ── 2a. Sentinel: wait for /scan from the Gazebo lidar plugin ──────
    wait_for_scan = ExecuteProcess(
        name='wait_for_scan',
        cmd=['ros2', 'topic', 'echo', '--once', '/scan',
             'sensor_msgs/msg/LaserScan'],
        output='log',
    )

    # ── 2b. slam_toolbox starts after /scan is up ──────────────────────
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('slam_params_file', os.path.join(
                pkg_nav, 'config', 'slam_toolbox_sim.yaml',
            )),
        ],
    )
    on_scan_ready = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_scan,
        on_exit=[
            LogInfo(msg='[lupin_bringup] /scan online — starting slam_toolbox'),
            slam_include,
        ],
    ))

    # ── 3a. Sentinel: wait for /map from slam_toolbox ──────────────────
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

    # ── 3b. Nav2 (slam mode) starts after /map is up ───────────────────
    # `map:=krr_house.yaml` is forwarded even though slam:=true means
    # map_server isn't instantiated — RewrittenYaml still substitutes
    # the path into the params blob and a literal '' makes it explode.
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py'),
        ),
        launch_arguments=[
            ('slam', 'true'),
            ('params_file', os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
            ('map', os.path.join(pkg_nav, 'maps', 'krr_house.yaml')),
            ('use_sim_time', 'true'),
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

    # ── 4. Greenhouse bridge ────────────────────────────────────────────
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
    )

    # ── 5. Mission orchestrator ─────────────────────────────────────────
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
        ],
    )

    # ── 6. rosbridge_websocket :9090 ────────────────────────────────────
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_rosbridge, 'launch', 'rosbridge_websocket_launch.xml'),
        ),
        condition=_when('enable_web'),
    )

    # ── 7. Lupin web HMI :8090 ──────────────────────────────────────────
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web, 'launch', 'lupin_web.launch.py'),
        ),
        launch_arguments=[('port', LaunchConfiguration('web_port'))],
        condition=_when('enable_web'),
    )

    # ── 9. RViz with persistent source-tree config ──────────────────────
    # Point RViz at the source-tree path (preferred) so Ctrl+S writes back
    # to a location that survives `colcon build` and shows up in
    # `git status` for committing. Falls back to the installed share copy
    # when running on a deploy that doesn't have the workspace tree.
    rviz_config = _resolve_rviz_config()
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lupin_rviz',
        arguments=['-d', rviz_config],
        output='log',
        condition=_when('rviz'),
    )

    # ── 8b. cmd_vel_mux (Nav2 + manual → /mirte_base_controller/cmd_vel_unstamped) ─
    # Nav2's velocity_smoother outputs to /cmd_vel_auto on purpose — see
    # lupin_navigation/launch/nav2.launch.py. Without this mux, nothing
    # subscribes to /cmd_vel_auto and Nav2 commands fall on the floor;
    # symptom is controller_server logging "Failed to make progress" after
    # ~10 s. The mux's output topic is the sim's twist_mux input (priority
    # 200), which then forwards to /cmd_vel → gazebo_planar_move.
    cmd_vel_mux = Node(
        package='lupin_hmi', executable='cmd_vel_mux', name='cmd_vel_mux',
        parameters=[{
            'cmd_vel_topic': '/mirte_base_controller/cmd_vel_unstamped',
            'use_sim_time': True,
        }],
        output='log',
    )

    # ── 8. AMCL pose seed (one-shot, fires alongside the orchestrator) ─
    # The orchestrator subscribes to /amcl_pose at startup, so the seed
    # message lands the moment it's published — no need to chain on
    # downstream readiness. The orchestrator caches the latest pose and
    # uses it whenever PREPARE.LOCALIZING is entered, including for
    # multi-mission re-runs.
    seed = ExecuteProcess(
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose'],
        output='log',
        condition=_when('seed_amcl'),
    )

    # HMI URL banner — printed before any process starts so the user can
    # scroll up to find it later. Modern terminals (gnome-terminal, kitty,
    # iTerm, VS Code) auto-detect http:// strings and make them
    # ctrl/cmd-clickable. We compute the LAN IP at launch-time (single
    # call, single value) and substitute the configured port.
    lan_ip = _get_lan_ip()
    hmi_banner = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════════════════════════╗\n',
        '║  Lupin HMI                                                   ║\n',
        '║    local:   http://localhost:', LaunchConfiguration('web_port'), '\n',
        '║    LAN:     http://', lan_ip, ':', LaunchConfiguration('web_port'), '\n',
        '║    rosbridge: ws://', lan_ip, ':9090\n',
        '╚══════════════════════════════════════════════════════════════╝',
    ])

    return LaunchDescription([
        *args,
        hmi_banner,
        LogInfo(msg='[lupin_bringup] sim_full: starting full sim chain '
                    '(Gazebo + bridge + orchestrator + web + rviz; '
                    'slam_toolbox waits for /scan, Nav2 waits for /map)'),
        # Phase 1 — fire-and-forget at t=0:
        greenhouse_sim,
        bridge,
        mission,
        rosbridge,
        web,
        seed,
        cmd_vel_mux,
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
    """Resolve the RViz config path for sim_full.

    Prefers the source-tree path so Ctrl+S in RViz writes to a location
    that survives `colcon build` and shows up in `git status` for
    committing. Falls back to the installed share copy if the workspace
    tree isn't where we expect (e.g. running on a deployed system).
    """
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
    # Nothing exists yet — return the source path so RViz creates it
    # there on first save.
    return os.path.expanduser(
        '~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz',
    )


def _get_lan_ip() -> str:
    """Best-effort primary IPv4 address for the host.

    Opens a UDP socket toward 8.8.8.8 (no traffic is actually sent — connect()
    on UDP just picks the route's source address) and reads getsockname().
    Falls back to 127.0.0.1 if there's no route. Used purely to print a
    LAN-reachable Lupin HMI URL in the bringup banner — modern terminals
    auto-link http:// strings.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 1))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


