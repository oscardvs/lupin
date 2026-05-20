"""xbox_teleop.launch.py — joystick-side teleop only.

Brings up:
  - joy_node           publishing /joy from the Xbox controller
  - teleop_twist_joy   publishing /cmd_vel_joy when LT (dead-man) is held
  - arm_teleop         driving the 4-DOF arm from the right stick + D-pad

Button layout (Xbox Wireless Controller — chassis is mecanum / omni,
verified live with /tmp/joy_probe.py):

    Drive (hold LB as dead-man):
        Left stick     → translation  (forward/back + strafe)
        Right stick X  → rotation (turn in place)
        RB (turbo)     → ~2× the linear/angular scale

    Arm (LB released — modes are mutually exclusive by construction):
        Y (top)        → shoulder_lift +
        A (bottom)     → shoulder_lift -
        B (right)      → shoulder_pan +
        X (left)       → shoulder_pan -
        D-pad ←/→      → wrist ±
        D-pad ↑/↓      → elbow ±

Shoulder pan/lift moved off the right stick to the face buttons because
the right stick used to be dual-use — chassis yaw while LB was held,
shoulder pan while LB was released. Brushing the stick during an
LB-release flicked the arm. Face buttons are physically separate from
any chassis input, so the two modes can never bleed into each other.
Right stick Y is now unbound. Shoulder gating via shoulder_disable_button
is kept (LB-held silences shoulder buttons too) as a "don't move the
arm while driving" safety.

LT/RT can't be the dead-man — teleop_twist_joy's enable_button only
takes a *button* index and triggers on this controller are axes (4, 5).
LB is the closest button equivalent.

Face-button indices assume the SDL2 standard (A=0, B=1, X=2, Y=3). On
this Series X|S BT mapping, LB/RB landed at 6/7 instead of the expected
4/5, so re-probe with `ros2 topic echo /joy` before trusting the face
indices. They're plain launch params, so a swap is one-line.

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
                # 0.15 covers the Series X|S BT-mode resting drift we see
                # on Mirte-247264's pad (probed 2026-05-12: right-stick X
                # idled ~0.5% off centre and made the chassis spin in
                # place when LB was held). 0.05 (the joy_node default) was
                # too tight; with cmd_vel_joy ≈ drift × scale_angular(-0.7),
                # any drift >0.05 yielded a constant angular.z.
                'deadzone': 0.15,
                'autorepeat_rate': 20.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # joy_node only enumerates SDL2 gamepads at startup. If the pad
            # is off at boot, the node binds nothing and stays dead until
            # restarted. Pair with the lupin_bringup udev rule
            # (99-lupin-xbox-rebind.rules) which pkills this process when an
            # Xbox Wireless Controller appears — respawn brings it back, the
            # fresh scan picks up the device, and the operator doesn't have
            # to `systemctl restart lupin-onboard` after powering the pad.
            respawn=True,
            respawn_delay=2.0,
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
                # Shoulder pan/lift on face buttons; right stick is now
                # chassis-yaw only. -1 disables the stick fallback.
                'shoulder_pan_axis': -1,
                'shoulder_lift_axis': -1,
                'shoulder_pan_plus_button': 1,    # B  (right)  → pan +
                'shoulder_pan_minus_button': 2,   # X  (left)   → pan -
                'shoulder_lift_plus_button': 3,   # Y  (top)    → lift +
                'shoulder_lift_minus_button': 0,  # A  (bottom) → lift -
            }],
        ),
    ])
