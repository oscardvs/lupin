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

    Robot (lupin-web.service):
        Vite preview on :8090 (HTTPS), web_video_server on :8091

    Robot (lupin-cameras-throttle.service):
        topic_tools throttle pipeline → /lupin/camera/...

After power-on, the HMI at https://<robot-ip>:8090 can drive the chassis,
move the arm, and operate the gripper without anyone running a `ros2 launch`
on a laptop. The laptop-side `hardware.launch.py` only adds Nav2 + slam +
RViz on top of this baseline and pushes its goals through twist_mux at
priority 10 (auto), so the operator override at priority 50/100 still wins.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
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

    return LaunchDescription([
        LogInfo(msg='[lupin_bringup] onboard: twist_mux + arm_preset_server '
                    '+ gripper_action_bridge — operator can drive on boot'),
        twist_mux,
        arm_preset_server,
        gripper_action_bridge,
    ])
