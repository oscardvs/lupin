"""onboard.launch.py — Lupin nodes that must run on the robot itself.

Everything in here is what the operator expects to be alive the moment the
robot boots, with no laptop-side launch needed:

    Robot (mirte-ros.service):
        telemetrix, controllers, lidar, cameras, rosbridge :9090

    Robot (lupin-onboard.service ← THIS launch):
        twist_mux              — arbitrates /cmd_vel_{joy,manual,auto} →
                                 /mirte_base_controller/cmd_vel
        arm_preset_server      — /lupin/arm/preset (named arm poses)
        gripper_action_bridge  — /lupin/gripper/set_angle_with_speed →
                                 mirte_master_gripper_controller/gripper_cmd
        xbox_teleop            — joy_node + teleop_twist_joy + arm_teleop,
                                 reads /dev/input/jsN on the robot itself.
                                 If no controller is plugged in, joy_node
                                 logs an open error and the process idles
                                 — twist_mux still arbitrates the other
                                 two inputs, so this is non-blocking.

    Robot (lupin-web.service):
        Vite preview on :8090 (HTTPS), web_video_server on :8091

    Robot (lupin-cameras-throttle.service):
        topic_tools throttle pipeline → /lupin/camera/...

After power-on, the HMI at https://<robot-ip>:8090 can drive the chassis,
move the arm, and operate the gripper without anyone running a `ros2 launch`
on a laptop. An Xbox controller plugged into the robot's USB works on boot
too, no laptop needed. The laptop-side `hardware.launch.py` only adds Nav2 +
slam + RViz on top of this baseline and pushes its goals through twist_mux
at priority 10 (auto), so the operator override at priority 50/100 still
wins. (`hardware.launch.py joystick:=true` still works as an escape hatch
for bench-testing teleop on the laptop side.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_hmi = get_package_share_directory('lupin_hmi')

    # ── twist_mux — chassis arbitration ────────────────────────────────
    # Output goes to /mirte_base_controller/cmd_vel (Twist, the topic the
    # real Mirte's mecanum controller listens on — see project_drive_topic
    # memory; sim uses _unstamped, hardware uses the stamped sink).
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[
            os.path.join(pkg_hmi, 'config', 'twist_mux.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[('cmd_vel_out', '/mirte_base_controller/cmd_vel')],
        output='log',
    )

    # ── arm_preset_server — named arm pose service ─────────────────────
    arm_preset_server = Node(
        package='lupin_hmi', executable='arm_preset_server',
        name='arm_preset_server',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ── gripper_action_bridge ──────────────────────────────────────────
    # HMI gripper goes /lupin/gripper/set_angle_with_speed → this bridge →
    # mirte_master_gripper_controller/gripper_cmd. See
    # project_gripper_command_path memory: don't shortcut to the raw
    # Hiwonder service on hardware; it fights the GripperActionController.
    gripper_action_bridge = Node(
        package='lupin_hmi', executable='gripper_action_bridge',
        name='gripper_action_bridge',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ── xbox_teleop — joystick plugged directly into the robot ─────────
    # Pulls in joy_node + teleop_twist_joy + arm_teleop (all configured
    # in lupin_hmi/launch/xbox_teleop.launch.py). Always-on: when no
    # controller is plugged in, joy_node just logs a device-open error
    # and idles, which doesn't affect the other two nodes above.
    # use_sim_time:=false because this launch only ever runs on the real
    # robot (lupin-onboard.service).
    xbox_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hmi, 'launch', 'xbox_teleop.launch.py'),
        ),
        launch_arguments=[('use_sim_time', 'false')],
    )

    return LaunchDescription([
        LogInfo(msg='[lupin_bringup] onboard: twist_mux + arm_preset_server '
                    '+ gripper_action_bridge + xbox_teleop — operator can '
                    'drive on boot (web HMI or Xbox controller)'),
        twist_mux,
        arm_preset_server,
        gripper_action_bridge,
        xbox_teleop,
    ])
