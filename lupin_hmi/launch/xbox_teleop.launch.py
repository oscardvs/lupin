"""xbox_teleop.launch.py — joystick-side teleop only.

Brings up:
  - joy_node           publishing /joy from the Xbox controller
  - teleop_twist_joy   publishing /cmd_vel_joy when LT (dead-man) is held
  - arm_teleop         driving the 4-DOF arm from the right stick + D-pad

Button layout (Xbox Wireless Controller — chassis is mecanum / omni):

    Drive (hold LT as dead-man):
        Left stick        → translation  (forward/back + strafe)
        Right stick X     → rotation
        RT (turbo)        → ~2× the linear/angular scale

    Arm (D-pad always live):
        LB + Right stick  → shoulder pan / lift
        D-pad ←/→         → elbow ±
        D-pad ↑/↓         → wrist ±

LB is the shoulder-enable button — without it, every chassis yaw on the
right stick would also try to swing the arm. Arm and drive modes are
ergonomically exclusive (one finger on LT vs LB) but the system does NOT
prevent you from holding both at once; if you do, the right stick
commands BOTH yaw and shoulder.

Does NOT include twist_mux — priority arbitration is owned by the bringup
launch (sim_full.launch.py / hardware_full.launch.py) so all sources of
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
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
