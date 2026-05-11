"""xbox_teleop.launch.py — joystick-side teleop only.

Brings up:
  - joy_node           publishing /joy from the Xbox controller
  - teleop_twist_joy   publishing /cmd_vel_joy when LT (dead-man) is held
  - arm_teleop         driving the 4-DOF arm from the right stick + D-pad

Button layout (Xbox Wireless Controller — chassis is mecanum / omni,
verified live with /tmp/joy_probe.py):

    Drive (hold LB as dead-man):
        Left stick     → translation  (forward/back + strafe)
        Right stick X  → rotation
        RB (turbo)     → ~2× the linear/angular scale

    Arm (LB released — modes are mutually exclusive by construction):
        Right stick    → shoulder pan / lift  (X = pan, Y = lift)
        D-pad ←/→      → elbow ±   (always live)
        D-pad ↑/↓      → wrist ±   (always live)

LT/RT can't be the dead-man — teleop_twist_joy's enable_button only
takes a *button* index and triggers on this controller are axes (4, 5).
LB is the closest button equivalent.

Does NOT include twist_mux — priority arbitration is owned by the bringup
launch (sim_full.launch.py / hardware.launch.py) so all sources of
/cmd_vel_* are arbitrated in one place. Run this standalone if you only
want joystick output (e.g. for bench-testing a new controller binding).

device_name matches SDL2's reported gamepad string (NOT the kernel
name — SDL2 has its own controller DB). Verify available names with:

    ros2 run joy joy_enumerate_devices

Default 'Xbox Series X Controller' is what an Xbox Wireless Controller
(045e:0b13) reports under SDL2 on Linux. SDL2 filters out non-gamepad
input devices, so the laptop accelerometer doesn't shift the index.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_hmi = get_package_share_directory('lupin_hmi')
    joy_config = os.path.join(pkg_hmi, 'config', 'xbox_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name', default_value='Xbox Series X Controller',
            description='SDL2 gamepad name (run `ros2 run joy '
                        'joy_enumerate_devices` to list). Falls back to '
                        'device_id when no match.',
        ),
        DeclareLaunchArgument(
            'device_id', default_value='0',
            description='Fallback /dev/input/jsN index if device_name is '
                        'empty or no match is found.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Forwarded to joy + teleop_twist_joy. Joy stamps are '
                        'always wall-clock from the kernel; only matters for '
                        'the published Twist headers (which are unstamped '
                        'anyway, but consistency is cheap).',
        ),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='screen',
            parameters=[{
                'device_name': LaunchConfiguration('device_name'),
                'device_id': LaunchConfiguration('device_id'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # Publish to a dedicated topic — twist_mux arbitrates between
            # this, /cmd_vel_manual (web HMI) and /cmd_vel_auto (Nav2).
            remappings=[('cmd_vel', '/cmd_vel_joy')],
        ),
        Node(
            package='lupin_hmi', executable='arm_teleop', name='arm_teleop',
            output='screen',
            # The vendor controller_manager ticks at 10 Hz, so we publish at
            # 10 Hz too — faster just thrashes the JTC with goals it never
            # gets to interpolate. Step + time_from_start = 1 controller
            # cycle's worth of motion, sized so even partial-stick deflection
            # produces visible movement:
            #
            #   step_rad=0.15 + URDF velocity limit 2.0 rad/s
            #   → full stick (1.0)  → 1.5 rad/s   (close to URDF max)
            #   → typical push 0.6 → 0.9 rad/s
            #   → tiny push 0.10   → 0.15 rad/s   (barely visible)
            #
            # deadzone shrunk to 0.05 — analog sticks rest at exactly 0.0
            # (probed cleanly), so a 0.05 floor only filters genuine noise.
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_rate_hz': 10.0,
                'step_rad': 0.15,
                'time_from_start_s': 0.20,
                'joint_velocity': 2.0,
                'deadzone': 0.05,
                'gripper_step_rad': 0.04,
            }],
        ),
    ])
