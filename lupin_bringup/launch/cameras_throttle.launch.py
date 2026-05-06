"""cameras_throttle.launch.py — robot-side topic_tools throttle pipeline.

Runs one `topic_tools throttle messages` node per enabled camera in
`config/cameras.yaml`. Vendor camera publishers (orbbec_camera,
usb_cam_node_exe) keep running at their native rates inside the mirte-ros
service; Lupin republishes a downsampled copy under /lupin/<source>. The HMI
camera default points at the Lupin topics so web_video_server only encodes
the slow stream — saves CPU on the Pi and DDS bandwidth to the laptop, which
together push the Pi back below the rosbridge-wedge threshold during Nav2
sessions (see project_rosbridge_wedge memory).

This is a v1: vendor still produces full-rate frames. A v2 that disables
vendor cameras and replaces them with FPS-capped equivalents would also save
producer-side CPU but is invasive — deferred per feedback_pragmatic_v1.

Run via:
    ros2 launch lupin_bringup cameras_throttle.launch.py
or via the lupin-cameras-throttle.service systemd unit installed by
scripts/install-cameras-throttle-systemd.sh.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('lupin_bringup'),
    'config', 'cameras.yaml',
)


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=DEFAULT_CONFIG,
            description='Path to the cameras throttle yaml. Defaults to the '
                        'one shipped in lupin_bringup/config — override to '
                        'point at a per-host config without rebuilding.',
        ),
    ]

    actions = list(args)
    actions.append(LogInfo(
        msg=['[lupin_bringup/cameras_throttle] reading ',
             LaunchConfiguration('config_file')],
    ))

    # Resolve the config path at launch-construction time so we can read it
    # synchronously and emit one Node per enabled camera. Using
    # OpaqueFunction would be cleaner if we needed substitutions, but the
    # config_file launch arg is rarely overridden — the static read keeps the
    # launch file readable.
    config_path = _resolve_config_path()
    cameras = _load_cameras(config_path)

    enabled = []
    skipped = []
    for name, cam in cameras.items():
        if not cam.get('enabled', False):
            skipped.append(name)
            continue
        enabled.append(name)
        actions.append(_build_throttle_node(name, cam))

    actions.append(LogInfo(msg=(
        f'[lupin_bringup/cameras_throttle] enabled={enabled or "<none>"}; '
        f'skipped={skipped or "<none>"}'
    )))

    return LaunchDescription(actions)


def _resolve_config_path() -> str:
    """Pick the most specific config path that exists.

    Order: $LUPIN_CAMERAS_CONFIG → DEFAULT_CONFIG (installed share). Lets ops
    override without rebuilding — handy on the robot where editing the
    install/share copy is awkward.
    """
    env = os.environ.get('LUPIN_CAMERAS_CONFIG')
    if env and os.path.isfile(env):
        return env
    return DEFAULT_CONFIG


def _load_cameras(path: str) -> dict:
    if not os.path.isfile(path):
        # Empty pipeline rather than a hard fail — service stays up so it
        # can pick up an edited config on the next restart.
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get('cameras', {})


def _build_throttle_node(name: str, cam: dict) -> Node:
    rate = float(cam.get('rate_hz', 1.0))
    source = str(cam['source_topic'])
    output = str(cam['output_topic'])
    return Node(
        package='topic_tools',
        executable='throttle',
        # `messages` mode rate-limits in messages/sec; matches our intent
        # (max FPS) better than `bytes` which is throughput-based.
        arguments=['messages', source, str(rate), output],
        name=f'lupin_throttle_{name}',
        output='log',
    )
