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
                                        arm_calibrate_server,
                                        gripper_action_bridge
                                        (joy_node / teleop_twist_joy moved
                                        to laptop — see joystick: arg)
        lupin-cameras.service           kills vendor cams, relaunches at
                                        config-driven low FPS on the same
                                        vendor topic names
        lupin-auto-home.service         oneshot at boot — self-heals stuck
                                        controllers (vendor first-boot
                                        race) then sends arm to 'home'.

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
        tag_annotator       → tag_<id> TFs + overlay JSON    (perception:=true)
        perception_aggregator → /perception/discovered_tags,
                              KIND_FLOWER obs, confirm_tag    (perception:=true)
        yolo_detector       → /yolo/detections (flowers/bug) (yolo:=true)
        rviz2               → interactive UI                 (rviz:=true)

Flags (all booleans, default in parens):

    web (true)        lupin_web — Vite preview (HTTPS :8090) +
                      rosbridge_websocket :9090 + web_video_server :8091.
                      Open https://<laptop-ip>:8090 to use the HMI.
    robot_ip          Robot address for the HMI camera-tab video proxy
                      (defaults to the ROS_DISCOVERY_SERVER IP, else the AP
                      192.168.42.1). Pass robot_ip:=10.42.0.1 for wired.
    slam (true)       slam_toolbox + slam_reset_node. Owns /map.
    nav2 (true)       Nav2 stack. Waits for /map before activating.
    twin (true)       lupin_twin aggregator. Cheap; HMI consumes it.
    mission (false)   greenhouse_bridge + mission_orchestrator + AMCL
                      pose seed as a bundle. Turn on for mission runs.
    rviz (true)       RViz2 with the persistent full_bringup_viz config.
    joystick (false)  Xbox controller teleop on the laptop.
    perception (true) lupin_perception/tag_annotator — AprilTag detection
                      against the Orbbec RGB stream. Publishes tag_<id>
                      TFs and JSON detections for the HMI overlay.

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
import re
import socket

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


def _apply_dds_env_from_ros_env_sh() -> str | None:
    """Source ~/.config/lupin/ros-env.sh into this process's env if it exists.

    Why: the laptop talks to the robot's FastDDS discovery server (see
    `lupin_bringup/README.md` "Laptop ↔ robot DDS over WiFi"). The env vars
    that wire that up live in `~/.config/lupin/ros-env.sh`, generated by
    `scripts/setup-laptop-dds-env.sh`. A user's bashrc sources that file,
    so any fresh terminal already has the env set — but a terminal opened
    *before* setup ran does not, and `ros2 launch` from such a terminal
    will spawn rosbridge + Nav2 + RViz with empty DDS env, so the HMI
    silently shows no data.

    Doing it here makes the launch self-configuring: whichever terminal
    you run it from, the children all inherit a consistent env. Returns
    a short banner string (or None) for the user-facing log line.
    """
    path = os.path.expanduser('~/.config/lupin/ros-env.sh')
    if not os.path.isfile(path):
        return None

    # Tiny parser — only handles `export FOO=bar` and `export FOO="bar"`
    # lines, which is all the generated file contains. Anything fancier
    # belongs in a real sourcing path, not a launch file.
    pat = re.compile(r'^\s*export\s+([A-Z_][A-Z0-9_]*)=("([^"]*)"|(\S+))\s*$')
    applied = []
    with open(path) as f:
        for line in f:
            m = pat.match(line)
            if not m:
                continue
            name = m.group(1)
            value = m.group(3) if m.group(3) is not None else m.group(4)
            if os.environ.get(name) != value:
                os.environ[name] = value
                applied.append(name)

    if not applied:
        return None
    return f'[lupin_bringup] applied DDS env from {path}: {", ".join(applied)}'


def _default_robot_ip() -> str:
    """Best-effort robot IP for laptop→robot links that need an explicit
    address (currently the HMI camera-tab video proxy target).

    Derives it from ROS_DISCOVERY_SERVER when set — that env var is the
    robot's FastDDS discovery-server endpoint ('<ip>:11811', written by
    setup-laptop-dds-env.sh and already applied to this process by
    _apply_dds_env_from_ros_env_sh() above). So whichever IP the operator
    pointed DDS at — robot AP 192.168.42.1, or a wired 10.42.0.1 — is
    reused for the video target automatically, with no second place to edit
    when going wired. Falls back to the robot's own AP address.
    """
    ds = os.environ.get('ROS_DISCOVERY_SERVER', '')
    host = ds.split(':')[0].strip() if ds else ''
    return host or '192.168.42.1'


def generate_launch_description() -> LaunchDescription:
    # Apply the DDS env first so _default_robot_ip() below can read the
    # discovery-server IP it sets.
    dds_env_banner = _apply_dds_env_from_ros_env_sh()
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')
    pkg_nav = get_package_share_directory('lupin_navigation')
    pkg_mission = get_package_share_directory('lupin_mission')
    pkg_twin = get_package_share_directory('lupin_twin')
    pkg_hmi = get_package_share_directory('lupin_hmi')
    pkg_web = get_package_share_directory('lupin_web')
    pkg_perception = get_package_share_directory('lupin_perception')

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
            'robot_ip', default_value=_default_robot_ip(),
            description='Robot address the laptop HMI uses for the camera-tab '
                        'video proxy: Vite forwards /_video to '
                        'http://<robot_ip>:8091, where lupin-cameras.service '
                        'serves MJPEG. Defaults to the IP in '
                        'ROS_DISCOVERY_SERVER (so a wired link reuses its '
                        'wired IP automatically), else the robot AP '
                        '192.168.42.1. Override for an unusual link, e.g. '
                        'robot_ip:=10.42.0.1 for the wired demo.',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the persistent config at '
                        'rviz/full_bringup_viz.rviz. Ctrl+S in RViz writes '
                        'back to that exact path.',
        ),
        DeclareLaunchArgument(
            'perception', default_value='true',
            description='Bring up lupin_perception: tag_annotator (Orbbec '
                        'AprilTag detection → tag_<id> TFs + overlay JSON) '
                        'AND perception_aggregator (TF-projects discovered '
                        'tags into /perception/discovered_tags, emits '
                        'KIND_FLOWER observations, serves /perception/'
                        'confirm_tag). Both are light. Turn off if the '
                        'camera driver is down.',
        ),
        DeclareLaunchArgument(
            'yolo', default_value='true',
            description='Bring up the YOLO flower/anomaly detector '
                        '(lupin_perception/yolo_detector) on the gripper cam. '
                        'Heavy (torch) — runs laptop-side here, not on the '
                        'Pi. Requires ultralytics + numpy<2 in this env '
                        '(see project_ultralytics_install_gotcha). Set false '
                        'if ultralytics is not installed; discovery still '
                        'works, flowers just stay unclassified.',
        ),
        DeclareLaunchArgument(
            'discovery_goal', default_value='5',
            description='ExplorationMission default: number of distinct tags '
                        'to discover before switching to the monitoring loop. '
                        'Overridable per /mission/start request.',
        ),
        DeclareLaunchArgument(
            'joystick', default_value='true',
            description='Bring up Xbox controller teleop (joy_node + '
                        'teleop_twist_joy + arm_teleop) on the laptop. '
                        'Default true — the robot-side joy_node path was '
                        'removed (BLE pad pairing on the Orange Pi image is '
                        'fragile, see project_xbox_ble_pairing_fix); the pad '
                        'now plugs into the laptop. Pass joystick:=false to '
                        'silence joy_node when no controller is connected.',
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

    # Tag layout — hardware and sim share the SAME committed snapshot
    # tag_locations_widened.json (the verbatim mdp-greenhouse 1.0.8 demo layout,
    # exact coords). That IS the official demo space, so the nav goals the
    # orchestrator builds match the real room. Per project_approach_pose_pipeline
    # the operator can still tune individual approach poses in
    # approach_overrides.yaml without rebuilding.
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
            ('discovery_goal', LaunchConfiguration('discovery_goal')),
        ],
        condition=IfCondition(LaunchConfiguration('mission')),
    )

    # require_visual_confirmation is left at its default (False): the mission
    # reads climate straight from the greenhouse bridge oracle and does NOT
    # require the camera to actually see the AprilTag first. The aggregator does
    # serve /perception/confirm_tag, so make the gap loud rather than implied by
    # the service merely existing.
    mission_confirm_banner = LogInfo(
        msg='[lupin_bringup] mission: camera tag-confirm is OFF '
            '(require_visual_confirmation=false) — readings come from the bridge '
            'oracle, not a visual AprilTag check.',
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

    # ── Lupin Web HMI — Vite + rosbridge (video proxied to robot) ─────
    # tls:=true + rosbridge:=true mirrors what the old laptop systemd unit
    # ran. Co-located rosbridge so the JSON-encoding load lives on the
    # laptop, not the Pi (Phase-2 split per project_offload_strategy).
    #
    # video:=false is the latency fix: web_video_server now runs on the
    # ROBOT (lupin-cameras.service) so raw frames never cross WiFi —
    # matching vendor mirte.local/ros-video/ behaviour. The Vite /_video
    # proxy is retargeted at the robot's 8091 endpoint via video_target.
    #
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
            ('video', 'false'),
            # Retarget the Vite /_video proxy at the robot's web_video_server.
            # robot_ip defaults to the discovery-server IP, so this follows the
            # link automatically (AP 192.168.42.1 or wired 10.42.0.1) — the old
            # hardcoded AP IP broke the camera tab over a wired-only connection.
            ('video_target', ['http://', LaunchConfiguration('robot_ip'), ':8091']),
            # The robot's lupin-onboard.service already runs light_strip_bridge.
            # Don't start a second one on the laptop — two instances collide on
            # the node name and the /lupin/leds/{set,auto} override services.
            # The HMI LightControl card still works: it reaches the robot's
            # bridge over DDS.
            ('leds', 'false'),
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

    # ── Perception (AprilTag detector + HMI overlay JSON) ──────────────
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'false')],
        condition=IfCondition(LaunchConfiguration('perception')),
    )

    # Perception aggregator — tag discovery (→ /perception/discovered_tags),
    # KIND_FLOWER observations, and the /perception/confirm_tag service.
    # Light (no torch); gated with tag_annotator under `perception`.
    perception_aggregator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception_aggregator.launch.py'),
        ),
        launch_arguments=[
            ('use_sim_time', 'false'),
            # Real planter rectangles for the HMI box overlay + bloom placement.
            ('tag_locations_file', tag_locations),
        ],
        condition=IfCondition(LaunchConfiguration('perception')),
    )

    # YOLO flower/anomaly detector on the gripper cam. Heavy (torch) — runs
    # here laptop-side, not on the Pi. start_viewer=false so no rqt window
    # pops up during a mission.
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'yolo_detector.launch.py'),
        ),
        launch_arguments=[('start_viewer', 'false')],
        condition=IfCondition(LaunchConfiguration('yolo')),
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

    # HMI URL banner — printed before any process starts so the user can
    # scroll up to find it later. Modern terminals (gnome-terminal, kitty,
    # iTerm, VS Code) auto-detect https:// strings and make them
    # ctrl/cmd-clickable. We compute the LAN IP at launch-time (single
    # call) and substitute it; the port and scheme are hardcoded because
    # the web include below pins mode:=preview tls:=true rosbridge:=true,
    # so the HMI always lands at https://<lan>:8090. Only shown when
    # web:=true since otherwise the URL would 404.
    lan_ip = _get_lan_ip()
    hmi_banner = LogInfo(
        msg=[
            '\n',
            '╔══════════════════════════════════════════════════════════════╗\n',
            '║  Lupin HMI                                                   ║\n',
            '║    local:     https://localhost:8090\n',
            '║    LAN:       https://', lan_ip, ':8090\n',
            '║    rosbridge: wss://', lan_ip, ':8090/_ros (same-origin proxy)\n',
            '╚══════════════════════════════════════════════════════════════╝',
        ],
        condition=IfCondition(LaunchConfiguration('web')),
    )

    extra_banners = []
    if dds_env_banner:
        extra_banners.append(LogInfo(msg=dds_env_banner))

    return LaunchDescription([
        *args,
        hmi_banner,
        *extra_banners,
        LogInfo(msg=['[lupin_bringup] hardware: laptop-side bring-up. ',
                     'web=', LaunchConfiguration('web'),
                     ' slam=', LaunchConfiguration('slam'),
                     ' nav2=', LaunchConfiguration('nav2'),
                     ' twin=', LaunchConfiguration('twin'),
                     ' mission=', LaunchConfiguration('mission'),
                     ' rviz=', LaunchConfiguration('rviz'),
                     ' joystick=', LaunchConfiguration('joystick'),
                     ' perception=', LaunchConfiguration('perception'),
                     ' yolo=', LaunchConfiguration('yolo')]),
        # Phase 1 — fire-and-forget at t=0 (each gated by its own flag):
        web,
        slam_reset_node,
        bridge,
        mission,
        mission_confirm_banner,
        twin,
        seed,
        xbox_teleop,
        perception,
        perception_aggregator,
        yolo,
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


def _get_lan_ip() -> str:
    """Best-effort primary IPv4 address for the host.

    Opens a UDP socket toward 8.8.8.8 (no traffic is actually sent —
    connect() on UDP just picks the route's source address) and reads
    getsockname(). Falls back to 127.0.0.1 if there's no route. Used
    purely to print a LAN-reachable Lupin HMI URL in the bringup banner —
    modern terminals auto-link https:// strings.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 1))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()
