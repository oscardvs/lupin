"""cameras.launch.py — robot-side native-FPS camera pipeline (v2).

Replaces the v1 lupin-cameras-throttle pipeline. Where v1 left the vendor
cameras running at native rates and downsampled the output via
`topic_tools/throttle`, v2 runs the cameras at the rate we want at the
source. The vendor `/camera` and `/gripper_camera` nodes started by
`mirte-ros.service` are killed by `lupin-cameras.service`'s
ExecStartPre (`scripts/kill-vendor-cameras.sh`); this launch then brings up
our replacements with identical topic names but reduced FPS and
depth/pointcloud off by default.

CPU win vs v1 (4-core Cortex-A55 — see project_robot_deployment_state):
    usb_cam_node_exe         ~31% → ~5%
    orbbec component_container  ~30% → ~10%
    eliminates both topic_tools/throttle nodes (~24% combined).

Reads `config/cameras.yaml`. To override the file without rebuilding, set
the `LUPIN_CAMERAS_CONFIG` env var to an absolute path before launching.

Run via:
    ros2 launch lupin_bringup cameras.launch.py
or via the `lupin-cameras.service` systemd unit installed by
`scripts/install-cameras-systemd.sh`.
"""

import os
import re
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('lupin_bringup'),
    'config', 'cameras.yaml',
)

# v4l device names of the USB gripper cameras shipped with Mirte Master, copied
# verbatim from mirte_bringup/gripper_camera.launch.py so we replicate vendor
# detection. If the vendor changes its accepted cam list we'll need to update.
_GRIPPER_CAM_NAMES = {'HD Camera: HD Camera', 'USB 2.0 PC Cam'}
_VIDEO_RE = re.compile(r'video\d+')


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=DEFAULT_CONFIG,
            description='Path to the cameras yaml. Defaults to the one shipped '
                        'in lupin_bringup/config — override to point at a '
                        'per-host config without rebuilding.',
        ),
    ]

    actions: list = list(args)
    actions.append(LogInfo(
        msg=['[lupin_bringup/cameras] reading ', LaunchConfiguration('config_file')],
    ))

    config_path = _resolve_config_path()
    cfg = _load_config(config_path)

    color_cfg = cfg.get('color', {})
    depth_cfg = cfg.get('depth', {})
    pc_cfg = cfg.get('point_cloud', {})
    gripper_cfg = cfg.get('gripper', {})

    color_on = bool(color_cfg.get('enabled', True))
    depth_on = bool(depth_cfg.get('enabled', False))
    pc_on = bool(pc_cfg.get('enabled', False))
    gripper_on = bool(gripper_cfg.get('enabled', True))

    if color_on or depth_on or pc_on:
        actions.append(_orbbec_launch(color_cfg, depth_cfg, pc_cfg,
                                      color_on, depth_on, pc_on))
        actions.append(LogInfo(msg=(
            f'[lupin_bringup/cameras] orbbec: color={color_on} '
            f'depth={depth_on} pointcloud={pc_on}'
        )))
    else:
        actions.append(LogInfo(msg='[lupin_bringup/cameras] orbbec: disabled'))

    gripper_nodes = _gripper_nodes(gripper_cfg) if gripper_on else []
    actions.extend(gripper_nodes)
    actions.append(LogInfo(msg=(
        f'[lupin_bringup/cameras] gripper: enabled={gripper_on} '
        f'devices={len(gripper_nodes)}'
    )))

    return LaunchDescription(actions)


def _resolve_config_path() -> str:
    env = os.environ.get('LUPIN_CAMERAS_CONFIG')
    if env and os.path.isfile(env):
        return env
    return DEFAULT_CONFIG


def _load_config(path: str) -> dict:
    if not os.path.isfile(path):
        # Empty config — service stays up and the next restart picks up edits.
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get('cameras', {})


def _orbbec_launch(color_cfg: dict, depth_cfg: dict, pc_cfg: dict,
                   color_on: bool, depth_on: bool, pc_on: bool) -> IncludeLaunchDescription:
    """Include vendor astra.launch.py with our overrides.

    The Orbbec driver wants `enable_color: true` for camera_info/tf to flow,
    so we keep colour on whenever any of the three streams is requested and
    let the FPS knob do the work for "RGB-disabled" scenarios. Pointcloud
    needs depth — we couple them here so a typo in the yaml doesn't yield a
    dead driver.
    """
    pc_on = pc_on and depth_on  # pointcloud requires depth
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([
                FindPackageShare('orbbec_camera'), 'launch', 'astra.launch.py',
            ])]
        ),
        launch_arguments={
            'enable_color': str(color_on).lower(),
            'color_fps': str(int(color_cfg.get('fps', 5))),
            'color_width': str(int(color_cfg.get('width', 640))),
            'color_height': str(int(color_cfg.get('height', 480))),
            'enable_depth': str(depth_on).lower(),
            'depth_fps': str(int(depth_cfg.get('fps', 5))),
            'enable_point_cloud': str(pc_on).lower(),
            # Belt-and-braces: turn off IR (we never use it) and skip the
            # depth↔colour soft filter when depth is off — the driver still
            # spawns the filter thread otherwise.
            'enable_ir': 'false',
            'enable_soft_filter': str(depth_on).lower(),
        }.items(),
    )


def _gripper_nodes(gripper_cfg: dict) -> list:
    """One usb_cam_node_exe per detected gripper-camera v4l device.

    Mirrors mirte_bringup/gripper_camera.launch.py's auto-detection so the
    swap is transparent. Extra knobs (framerate, pixel_format) come from yaml.
    """
    framerate = float(gripper_cfg.get('framerate', 5.0))
    pixel_format = str(gripper_cfg.get('pixel_format', 'yuyv2rgb'))
    image_width = int(gripper_cfg.get('width', 640))
    image_height = int(gripper_cfg.get('height', 480))

    devices = _detect_gripper_devices()
    nodes = []
    for i, device in enumerate(devices):
        ns = 'gripper_camera' if i == 0 else f'gripper_camera_{i}'
        nodes.append(Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=ns,
            namespace=ns,
            parameters=[{
                'pixel_format': pixel_format,
                'video_device': str(device),
                'framerate': framerate,
                'image_width': image_width,
                'image_height': image_height,
            }],
            output='log',
        ))
    return nodes


def _detect_gripper_devices() -> list:
    """Return /dev/videoN paths whose v4l friendly-name matches a known cam.

    Identical filter to mirte_bringup's gripper_camera.launch.py. We need
    `v4l2-ctl --device=<dev> --all` to succeed (i.e. the device is currently
    free) — when called from our service, vendor cameras have already been
    killed by ExecStartPre so the devices are released.
    """
    candidates = sorted(
        d for d in Path('/dev').glob('video*') if _VIDEO_RE.match(d.name)
    )
    out = []
    for device in candidates:
        name_path = Path('/sys/class/video4linux') / device.name / 'name'
        try:
            v4l_name = name_path.read_text().strip()
        except OSError:
            continue
        if v4l_name not in _GRIPPER_CAM_NAMES:
            continue
        probe = subprocess.run(
            ['bash', '-c', f'v4l2-ctl --device={device} --all '
                           f'| grep "Format Video Capture"'],
            check=False, capture_output=True,
        )
        if probe.returncode != 0:
            continue
        out.append(device)
    return out
