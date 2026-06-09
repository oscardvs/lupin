"""xbox_teleop.launch.py — joystick-side teleop only.

Brings up:
  - joy_node           publishing /joy from the Xbox controller
  - teleop_twist_joy   publishing /cmd_vel_joy when LB (dead-man) is held
  - arm_teleop         driving the 4-DOF arm from the face buttons + D-pad

USB vs Bluetooth auto-detect
----------------------------
The button/axis index order an Xbox pad presents depends on the kernel driver,
which depends on how the pad is connected: a USB cable binds `xpad` (the
"standard" layout) while Bluetooth binds the Microsoft-HID path (a shifted
layout — LB=6/RB=7, face buttons A=0/B=1/X=3/Y=4 with index 2 skipped). The two
mappings therefore live in two files:

    config/xbox_config.usb.yaml          config/xbox_config.bluetooth.yaml

At launch we read the transport bus (lupin_hmi.pad_detect, via
/proc/bus/input/devices) and load the matching file. Override with
`controller_mode:=usb|bluetooth` (default `auto`); falls back to USB if no pad
is detected. Each file carries the joy_node, teleop_twist_joy_node and
arm_teleop parameter sections.

Re-probe a pad after a kernel/controller swap with the interactive calibrator,
which detects the mode and rewrites the matching file in place:

    ros2 run lupin_hmi calibrate_xbox

Control scheme (held in the yaml; calibrate_xbox preserves it):
    Drive (hold LB dead-man): left stick → translate, right stick X → yaw,
        RB → turbo.
    Arm (LB released): Y/A → shoulder lift ±, B/X → shoulder pan ±,
        D-pad ←/→ → wrist ±, D-pad ↑/↓ → elbow ±, RT/LT → gripper open/close.
    Face buttons (not the right stick) carry shoulder pan/lift so brushing a
    stick can never flick the arm; LB also silences the shoulder buttons while
    driving. LT/RT can't be the dead-man — teleop_twist_joy's enable_button
    takes a *button* index and the triggers are axes.

Does NOT include twist_mux — priority arbitration is owned by the bringup
launch (sim_full.launch.py / hardware.launch.py). Run this standalone to bench
a controller binding; the three bringup launches include it and pass only
use_sim_time, so the auto-detect and the new controller_mode arg are
transparent to them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_mode(context) -> tuple:
    """Return (mode, human_readable_detection_note)."""
    forced = LaunchConfiguration('controller_mode').perform(context).strip().lower()
    if forced in ('usb', 'bluetooth'):
        return forced, f'forced via controller_mode:={forced}'
    # auto (or anything unexpected) → detect from the transport bus.
    try:
        from lupin_hmi.pad_detect import detect_pad_mode, describe_pads
        detected = detect_pad_mode()
        note = describe_pads()
        if detected is None:
            return 'usb', f'no Xbox pad detected ({note}) → defaulting to USB'
        return detected, note
    except Exception as exc:  # pragma: no cover - detection must never crash launch
        return 'usb', f'pad detection failed ({exc!r}) → defaulting to USB'


def _make_nodes(context, *args, **kwargs):
    pkg_hmi = get_package_share_directory('lupin_hmi')
    mode, note = _resolve_mode(context)
    joy_config = os.path.join(pkg_hmi, 'config', f'xbox_config.{mode}.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    device_name = LaunchConfiguration('device_name').perform(context)
    device_id = LaunchConfiguration('device_id').perform(context).strip()

    # joy_node params: the yaml carries device_name/device_id/deadzone. The
    # launch args override only when explicitly set (non-empty), so a normal
    # launch uses the per-mode yaml values.
    joy_overrides = {
        'autorepeat_rate': 20.0,
        'use_sim_time': use_sim_time,
    }
    if device_name != '':
        joy_overrides['device_name'] = device_name
    if device_id != '':
        try:
            joy_overrides['device_id'] = int(device_id)
        except ValueError:
            pass

    return [
        LogInfo(msg=f'[xbox_teleop] controller_mode={mode}  '
                    f'config=xbox_config.{mode}.yaml  ({note})'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='screen',
            parameters=[joy_config, joy_overrides],
            # joy_node only enumerates SDL2 gamepads at startup. If the pad is
            # off at boot it binds nothing and stays dead until restarted; the
            # lupin_bringup udev rule (99-lupin-xbox-rebind.rules) pkills this
            # process when an Xbox pad appears so respawn picks it up.
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config, {'use_sim_time': use_sim_time}],
            # twist_mux arbitrates this against /cmd_vel_manual + /cmd_vel_auto.
            remappings=[('cmd_vel', '/cmd_vel_joy')],
        ),
        Node(
            package='lupin_hmi', executable='arm_teleop', name='arm_teleop',
            output='screen',
            # Index/sign params come from the yaml (calibrate_xbox owns them).
            # The motion-tuning params stay here — they're controller-agnostic.
            #   step_rad=0.15 + URDF velocity limit 2.0 rad/s
            #   → full stick → 1.5 rad/s; tiny push 0.10 → 0.15 rad/s.
            # arm-stick deadzone 0.05: analog sticks rest at exactly 0.0, so a
            # 0.05 floor only filters genuine noise.
            parameters=[joy_config, {
                'use_sim_time': use_sim_time,
                'update_rate_hz': 10.0,
                'step_rad': 0.15,
                'time_from_start_s': 0.20,
                'joint_velocity': 2.0,
                'deadzone': 0.05,
                'gripper_step_rad': 0.04,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_mode', default_value='auto',
            choices=['auto', 'usb', 'bluetooth'],
            description='Which mapping to load. "auto" reads the transport bus '
                        '(USB cable vs Bluetooth) and picks the matching '
                        'xbox_config.<mode>.yaml; falls back to usb if no pad '
                        'is found.',
        ),
        DeclareLaunchArgument(
            'device_name', default_value='',
            description='Override the SDL2 gamepad name. Empty (default) uses '
                        'the value baked into the selected config file. List '
                        'names with `ros2 run joy joy_enumerate_devices`.',
        ),
        DeclareLaunchArgument(
            'device_id', default_value='',
            description='Override the SDL2 device index. Empty (default) uses '
                        'the config-file value (0).',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Forwarded to joy + teleop_twist_joy + arm_teleop.',
        ),
        OpaqueFunction(function=_make_nodes),
    ])
