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

Chassis command bus (twist_mux owns priority arbitration):

    Xbox dead-man (LT held)  → /cmd_vel_joy     (prio 100)  ┐
    Web HMI joystick widget  → /cmd_vel_manual  (prio  50)  ├─► twist_mux
    Nav2 velocity_smoother   → /cmd_vel_auto    (prio  10)  ┘    │
                                                                 ▼
                                            /mirte_base_controller/cmd_vel_unstamped
                                                                 │
                                                  vendor twist_mux (prio 200)
                                                                 ▼
                                                         /cmd_vel → Gazebo

After bringup:
    1. Open http://localhost:8090
    2. (one-time) In Settings → Drive, set cmd_vel topic to ``/cmd_vel_manual``
       so the web joystick feeds twist_mux instead of racing Nav2 on the
       controller topic.
    3. Drive a small loop with the Xbox controller (hold LB, push left
       stick) or the web Teleop tab so slam_toolbox has scan context.
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
    AppendEnvironmentVariable
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
    pkg_twin = get_package_share_directory('lupin_twin')
    pkg_web = get_package_share_directory('lupin_web')
    pkg_hmi = get_package_share_directory('lupin_hmi')
    pkg_rosbridge = get_package_share_directory('rosbridge_server')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_perception = get_package_share_directory('lupin_perception')
    
    # The Gazebo ROS plugins look for models in GAZEBO_MODEL_PATH, which doesn't
    lupin_models_path = os.path.join(pkg_bringup, 'models')

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
        # joystick
        DeclareLaunchArgument(
            'joystick', default_value='true',
            description='Bring up Xbox controller teleop (joy_node + '
                        'teleop_twist_joy + arm_teleop). Drive with the left '
                        'stick while holding LT (dead-man); right stick + '
                        'D-pad drives the arm. Set false on hosts without a '
                        'controller — joy_node logs noisily otherwise.',
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
    # The world generator is run with --aisle-expand-y 1.5 so that Nav2
    # can thread the E-W aisles between table rows. The bridge and the
    # orchestrator must therefore consume the widened tag_locations JSON
    # (not the upstream one inside mdp-greenhouse) or their tag coords
    # disagree with the SDF. The widened file is committed under
    # lupin_bringup/config/ and installed into share/.
    widened_tag_locations = os.path.join(
        get_package_share_directory('lupin_bringup'),
        'config',
        'tag_locations_widened.json',
    )
    # Optional per-tag approach-pose overrides — empty stub in sim, hardware
    # operators populate it when geometry doesn't match the physical layout.
    approach_overrides = os.path.join(
        get_package_share_directory('lupin_bringup'),
        'config',
        'approach_overrides.yaml',
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', widened_tag_locations)],
    )

    # ── 4b. Perception pipeline (vision tag detection → map discovery) ──
    # tag_annotator: OpenCV-ArUco detector on the Gazebo RGB stream. The
    # Astra depth-camera plugin publishes /camera/image_raw +
    # /camera/camera_info with frame_id `camera_depth_optical_frame`; its
    # 60° HFOV gives K≈554. fallback_intrinsics is armed with that K in case
    # the plugin's CameraInfo ships a zero K (it hard-zeros Cx/Cy/focalLength).
    # tag_size_m = 0.036 = the rendered tag plate (0.04, real 4 cm tag) × 0.9
    # texture plane, so the PnP distance — and the broadcast tag→camera TF —
    # comes out metric.
    tag_annotator = Node(
        package='lupin_perception', executable='tag_annotator',
        name='tag_annotator',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'detections_topic': '/camera/tag_detections_json',
            'tag_size_m': 0.036,
            'tf_frame_prefix': 'tag_',
            'image_qos': 'reliable',
            'fallback_intrinsics': [554.254691191187, 554.254691191187, 320.5, 240.5],
        }],
        output='screen',
    )

    # perception_aggregator: debounces tag sightings (min_sightings TF
    # lookups map→tag_<id>) and publishes /perception/discovered_tags
    # (map-frame poses) — the feed ExplorationMission consumes to discover
    # tags during frontier exploration — plus the /perception/confirm_tag
    # service. Torch-free: the heavy yolo_detector is NOT run in sim, so
    # flower readings come from the greenhouse bridge oracle instead.
    perception_aggregator = Node(
        package='lupin_perception', executable='perception_aggregator',
        name='perception_aggregator',
        parameters=[{
            'use_sim_time': True,
            'tag_detections_topic': '/camera/tag_detections_json',
            'mission_state_topic': '/mission/state',
            'discovered_tags_topic': '/perception/discovered_tags',
            'observations_topic': '/floranova/observations',
            'confirm_service': '/perception/confirm_tag',
            'map_frame': 'map',
            'tf_frame_prefix': 'tag_',
            'min_sightings': 3,
            'max_tag_distance_m': 2.5,
        }],
        output='screen',
    )

    # sim flower detector: HSV colour stand-in for the real YOLO (best.pt is
    # trained on real dahlias, fires on nothing in Gazebo). Publishes the SAME
    # /yolo/detections contract, so perception_aggregator/twin/HMI are identical
    # sim vs hardware — the detector is the only swap. On the robot, run
    # perception_stack.launch.py (real yolo_detector) instead of this node.
    sim_flower_detector = Node(
        package='lupin_perception', executable='sim_flower_detector',
        name='sim_flower_detector',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/gripper_camera/image_raw',
            'detections_topic': '/yolo/detections',
            'image_qos': 'reliable',
            'publish_overlay': True,
        }],
        output='screen',
    )

    # ── 5. Mission orchestrator ─────────────────────────────────────────
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mission, 'launch', 'mission.launch.py'),
        ),
        launch_arguments=[
            # The whole sim stack runs on /clock; the orchestrator MUST too, or
            # its observation/nav-goal/MissionState stamps are wall-clock and the
            # twin treats every map pin as maximally stale (and Nav2/TF may reject
            # the goal stamps). mission.launch.py defaults this to 'false'.
            ('use_sim_time', 'true'),
            ('dependency_timeout_s', LaunchConfiguration('dependency_timeout_s')),
            ('tag_locations_file', widened_tag_locations),
            ('approach_overrides_file', approach_overrides),
            # Per-pot arm patrol: at each pot the orchestrator strikes the
            # `inspect` pose (gripper cam down on the bloom) for the flower
            # detector, then the travel pose between pots. Sim demo of the scan.
            ('arm_patrol_enabled', 'true'),
            # 'home' is arm-horizontal-forward (~0.28 m reach) and clips the pots
            # while driving; 'tuck' folds the arm over the base. settle 3.2 s lets
            # the fold finish before the base drives off. (Matches sim_autonomy.)
            ('arm_travel_preset', 'tuck'),
            ('arm_travel_settle_s', '3.2'),
            # Dwell ~4 s in SCANNING so the arm (3 s travel) reaches the inspect
            # pose and the flower detector reads the bloom before advancing.
            ('flower_scan_dwell_s', '4.0'),
        ],
    )

    # ── 5b. Digital twin — aggregates /floranova/observations into a live
    # world snapshot for the HMI (and any FloraNova consumer that wires in
    # later). Pure listener; no upstream dependency on Nav2 / Gazebo.
    twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_twin, 'launch', 'twin.launch.py'),
        ),
        # On /clock like the rest of the sim, so the twin's staleness
        # (now - last_seen) matches the orchestrator's sim-time observation stamps.
        launch_arguments=[('use_sim_time', 'true')],
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
        launch_arguments=[
            ('port', LaunchConfiguration('web_port')),
            ('rosbridge', 'false'),  # we start rosbridge above; avoid :9090 clash
        ],
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

    # ── 8b. twist_mux — priority arbitration on the chassis bus ─────────
    # Three Twist sources are arbitrated by priority + timeout:
    #
    #   /cmd_vel_joy    Xbox dead-man teleop      (prio 100) ── operator
    #   /cmd_vel_manual web HMI joystick widget   (prio  50) ── remote operator
    #   /cmd_vel_auto   Nav2 velocity_smoother    (prio  10) ── autonomy
    #
    # Output goes to /mirte_base_controller/cmd_vel_unstamped, which is
    # the input of the *vendor* twist_mux (priority 200), which then
    # publishes /cmd_vel → gazebo_planar_move. We sit upstream of the
    # vendor mux on purpose: it only de-conflicts the controller bus
    # against /zero_cmd_vel; our mux is where Lupin's own arbitration
    # lives.
    #
    # Replaces the old custom cmd_vel_mux node. Behaviour difference:
    # twist_mux uses pure priority+timeout (no "manual takeover" window),
    # so when LT is released or the web stops sending, the next priority
    # gets the bus immediately after timeout.
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[
            os.path.join(pkg_hmi, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('cmd_vel_out', '/mirte_base_controller/cmd_vel_unstamped'),
        ],
        output='log',
    )

    # ── 8c. arm_sim_shim (HMI Hiwonder services → ros2_control) ────────
    # The web HMI calls /io/servo/hiwonder/<joint>/set_angle_with_speed and
    # /io/servo/hiwonder/enable_all_servos — Hiwonder serial-bus services
    # that only exist on the real Mirte. In sim the arm and gripper are
    # exposed via ros2_control instead (mirte_master_arm_controller and
    # mirte_master_gripper_controller). This shim translates the HMI's
    # calls onto the controllers and republishes /joint_states as
    # ServoPosition feedback so the slider read-outs work. Sim-only — the
    # real robot serves these names natively via mirte_telemetrix_cpp.
    arm_sim_shim = Node(
        package='lupin_hmi', executable='arm_sim_shim', name='arm_sim_shim',
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # ── 8c2. arm_preset_server — named arm-pose service ────────────────
    # Backs the voice agent's `arm_preset` tool and the lupin_msgs
    # SetArmPreset service. Holds a static dict of named poses (home /
    # tuck / pick / place) and emits a JointTrajectory on
    # /mirte_master_arm_controller/joint_trajectory — same topic the
    # arm_sim_shim publishes to in sim and the real Mirte controller
    # listens on, so this node ships unchanged across both targets.
    arm_preset_server = Node(
        package='lupin_hmi', executable='arm_preset_server',
        name='arm_preset_server',
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # ── 8c2b. arm_library_server — saved arm poses/sequences ───────────
    # Laptop-side owner of ~/.config/lupin/arm_library.json. Backs the HMI
    # Arm tab's Pose Library + Sequence Recorder cards and the voice agent's
    # arm-library tools via /lupin/arm/library/*. Records off /joint_states
    # and replays onto /mirte_master_arm_controller/joint_trajectory — the
    # same topic arm_preset_server uses — so it ships unchanged across sim
    # and hardware. The built-in presets stay in arm_preset_server.
    arm_library_server = Node(
        package='lupin_hmi', executable='arm_library_server',
        name='arm_library_server',
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # ── 8c3. gripper_action_bridge — HMI gripper service → controller ──
    # Owns /lupin/gripper/set_angle_with_speed on both sim and hardware.
    # Translates HMI degree commands into a GripperCommand action goal so
    # the controller's commanded state stays aligned with the HMI request
    # (without this, the hardware-side ros2_control HW interface re-asserts
    # its stale 0 setpoint on every tick — see node docstring).
    gripper_action_bridge = Node(
        package='lupin_hmi', executable='gripper_action_bridge',
        name='gripper_action_bridge',
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    # ── 8d. Xbox controller teleop ─────────────────────────────────────
    # Joy → teleop_twist_joy → /cmd_vel_joy (twist_mux input, priority 100).
    # Arm joints driven directly from /joy by lupin_hmi.arm_teleop.
    #
    # Button layout (mecanum / omni — both sticks used for drive):
    #   Drive — hold LB (dead-man):
    #     Left stick     → translation (fwd/back + strafe)
    #     Right stick X  → rotation
    #     RB             → turbo (~2× scale)
    #   Arm — LB released (mutually exclusive with drive):
    #     Right stick    → shoulder pan / lift
    #     D-pad ←/→      → elbow ±   (always live)
    #     D-pad ↑/↓      → wrist ±   (always live)
    #
    # When LB is released, /cmd_vel_joy goes silent and twist_mux times
    # it out (0.3 s), handing the bus back to the web HMI or Nav2.
    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'true')],
        condition=_when('joystick'),
    )

    # ── 8. AMCL pose seed (one-shot, fires alongside the orchestrator) ─
    # The orchestrator subscribes to /amcl_pose at startup, so the seed
    # message lands the moment it's published — no need to chain on
    # downstream readiness. The orchestrator caches the latest pose and
    # uses it whenever PREPARE.LOCALIZING is entered, including for
    # multi-mission re-runs.
    seed = ExecuteProcess(
        # On /clock too, so the seeded /amcl_pose stamps match the sim-time stack.
        cmd=['ros2', 'run', 'lupin_bringup', 'seed_amcl_pose',
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='log',
        condition=_when('seed_amcl'),
    )

    # ── 9. Sim battery publisher (sim-only) ────────────────────────────────
    # Publishes a linearly draining sensor_msgs/BatteryState on
    # /io/power/power_watcher so the BatteryMonitor in the mission
    # orchestrator has data to work with. Hardware doesn't need this —
    # the real MIRTE power watcher publishes on the same topic natively.
    # Tune drain_rate_per_sec: 0.001 ≈ 17 min to empty (demo-safe);
    # use 0.01 for fast testing (~90 s to the 20% low-battery threshold).
    sim_battery_publisher = Node(
        package='lupin_bringup',
        executable='sim_battery_publisher',
        name='sim_battery_publisher',
        parameters=[{
            'use_sim_time': True,
            'initial_charge': 1.0,
            'drain_rate_per_sec': 0.001,
        }],
        output='log',
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
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', lupin_models_path),
        hmi_banner,
        LogInfo(msg='[lupin_bringup] sim_full: starting full sim chain '
                    '(Gazebo + bridge + orchestrator + web + rviz; '
                    'slam_toolbox waits for /scan, Nav2 waits for /map)'),
        # Phase 1 — fire-and-forget at t=0:
        greenhouse_sim,
        bridge,
        tag_annotator,
        perception_aggregator,
        sim_flower_detector,
        mission,
        twin,
        rosbridge,
        web,
        seed,
        sim_battery_publisher,
        twist_mux,
        xbox_teleop,
        arm_sim_shim,
        arm_preset_server,
        arm_library_server,
        gripper_action_bridge,
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


