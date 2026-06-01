"""onboard.launch.py — Lupin nodes that must run on the robot itself.

Everything in here is what the operator expects to be alive the moment the
robot boots, with no laptop-side launch needed:

    Robot (mirte-ros.service):
        telemetrix, controllers, lidar, cameras, rosbridge :9090

    Robot (lupin-onboard.service ← THIS launch):
        twist_mux              — arbitrates /cmd_vel_{joy,manual,auto} →
                                 /mirte_base_controller/cmd_vel
        arm_preset_server      — /lupin/arm/preset (named arm poses)
        arm_calibrate_server   — /lupin/arm/calibrate (Hiwonder zero-offset
                                 calibration, operator-in-the-loop)
        gripper_action_bridge  — /lupin/gripper/set_angle_with_speed →
                                 mirte_master_gripper_controller/gripper_cmd

    Robot (lupin-cameras.service):
        kills vendor camera nodes, relaunches at config-driven low FPS
        on the same /camera/* and /gripper_camera/* topic names

    Robot (lupin-auto-home.service):
        oneshot — heals controllers (vendor first-boot wedge) then calls
        /lupin/arm/preset {home} so the arm rests in a low-load pose on boot.
        See lupin_hmi/auto_home.py.

Xbox controller teleop lives on the LAPTOP now (hardware.launch.py
joystick:=true). The robot-side joy_node path was removed because BLE pad
pairing on this Orange Pi image is fragile (BlueZ 5.64 + Xbox Series X|S
firmware — see project_xbox_ble_pairing_fix) and the joy_node respawn
race added boot-time complexity for no gain over plugging the pad into
the laptop. The web HMI on https://<laptop-ip>:8090 still drives the
robot without the laptop launch, just no joystick option.
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

    # ── arm_calibrate_server — Hiwonder zero-offset calibration ────────
    # Hardware-only. Drives the operator-in-the-loop calibration flow via
    # /lupin/arm/calibrate {start|commit|cancel|status}. Bus services
    # (/io/servo/hiwonder/<j>/_set_offset, /enable_arm_control) only exist
    # on the real Mirte, so on sim the start action will return an error.
    arm_calibrate_server = Node(
        package='lupin_hmi', executable='arm_calibrate_server',
        name='arm_calibrate_server',
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ── light_strip_bridge ─────────────────────────────────────────────
    # Mirrors the latched /mission/state snapshot onto the robot's status
    # neopixel strip via the MIRTE LED service. Keep robot-local so the
    # status indication works without any laptop-side launch.
    light_strip_bridge = Node(
        package='lupin_hmi', executable='light_strip_bridge',
        name='light_strip_bridge',
        parameters=[{
            'use_sim_time': False,
            'mission_state_topic': '/mission/state',
            'led_service': '/io/leds/leds/set_color',
        }],
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
                    '+ arm_calibrate_server + light_strip_bridge + gripper_action_bridge — '
                    'operator drives via web HMI (laptop-side joystick lives '
                    'in hardware.launch.py joystick:=true)'),
        twist_mux,
        arm_preset_server,
        arm_calibrate_server,
        light_strip_bridge,
        gripper_action_bridge,
    ])
