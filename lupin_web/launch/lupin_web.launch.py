"""Launch the Lupin Web HMI as a static file server on :8090, plus a
LAN-reachable web_video_server on :8091 for the camera tab, plus (optionally)
a co-located rosbridge_websocket on :9090.

Brings up a built `dist/` artifact via `npm run preview` (which also handles
SPA fallback). Coexists with the course web stack — never binds :80, :8080.

Why a second web_video_server: the vendor MIRTE setup already runs one on
[::1]:8181, but it's bound to localhost only — browsers on the LAN can't
reach it. We launch our own on 0.0.0.0:8091 alongside the UI so the camera
tab works from any device. The vendor instance is left untouched.

Why co-locate rosbridge: when the HMI runs on the operator's laptop (the
Phase-2 split per `project_offload_strategy`), serving rosbridge from the
same host as the Vite preview lets the same-origin /_ros proxy point at
localhost:9090. The Pi is no longer doing JSON encoding for the HMI flood
(/tf, /scan, /joint_states) — that load moves to the laptop, which has
plenty of CPU. The vendor rosbridge on the Pi stays running but idle
(nobody connects to it). On the robot bringup, set rosbridge:=false so we
don't bind-conflict with the vendor one on :9090.

TLS: with tls:=true, Vite serves https on the UI port using a self-signed
cert (@vitejs/plugin-basic-ssl). Same-origin proxies /_ros and /_video then
let rosbridge (:9090) and web_video_server (:8091) reach the browser over
wss/https without each needing its own cert. Required for the voice tab —
browsers gate getUserMedia (mic) to secure contexts and http://<host>:8090
is not one. Sim/dev usually leave tls:=false because they hit
http://localhost (a secure context already).

Usage:
    ros2 launch lupin_web lupin_web.launch.py                     # default
    ros2 launch lupin_web lupin_web.launch.py port:=8091          # override UI port
    ros2 launch lupin_web lupin_web.launch.py mode:=dev           # vite dev (HMR)
    ros2 launch lupin_web lupin_web.launch.py video:=false        # skip web_video_server
    ros2 launch lupin_web lupin_web.launch.py rosbridge:=false    # skip rosbridge (use vendor)
    ros2 launch lupin_web lupin_web.launch.py tls:=true           # https + wss/_ros (mic)

Requirements:
    - Node 20+ and npm available on PATH
    - `npm install` already run inside lupin_web/web/
    - For mode:=preview (default): `npm run build` already run (so dist/ exists)
    - `web_video_server` + `rosbridge_server` ROS packages installed

The Vite project is NOT installed to share/ (node_modules + dist are too large
and the tooling expects a writeable working tree). The launch file resolves
the source-tree `web/` directory in this order:
    1. $LUPIN_WEB_SRC if set
    2. ../web relative to this launch file (works from the source tree)
    3. ~/ros2_ws/src/lupin/lupin_web/web (the documented robot layout)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_web_dir() -> str:
    """Resolve the lupin_web/web/ source directory at launch time."""
    candidates = [
        os.environ.get('LUPIN_WEB_SRC'),
        os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'web')),
        os.path.expanduser('~/ros2_ws/src/lupin/lupin_web/web'),
    ]
    for c in candidates:
        if c and os.path.isdir(c) and os.path.isfile(os.path.join(c, 'package.json')):
            return c
    raise FileNotFoundError(
        'Could not locate lupin_web/web/. Set LUPIN_WEB_SRC or check the repo '
        'is at ~/ros2_ws/src/lupin.'
    )


_REPO_WEB_DIR = _resolve_web_dir()


def generate_launch_description():
    port = LaunchConfiguration('port')
    mode = LaunchConfiguration('mode')
    video = LaunchConfiguration('video')
    video_target = LaunchConfiguration('video_target')
    rosbridge = LaunchConfiguration('rosbridge')
    tls = LaunchConfiguration('tls')

    # Resolve the npm script at launch time based on `mode`.
    npm_script = PythonExpression([
        "'dev' if '", mode, "'.lower() == 'dev' else 'preview'"
    ])

    spawn_video = PythonExpression([
        "'", video, "'.lower() in ('true', '1', 'yes')"
    ])

    spawn_rosbridge = PythonExpression([
        "'", rosbridge, "'.lower() in ('true', '1', 'yes')"
    ])

    # vite.config.ts reads LUPIN_TLS=1 to enable @vitejs/plugin-basic-ssl.
    tls_env = PythonExpression([
        "'1' if '", tls, "'.lower() in ('true', '1', 'yes') else '0'"
    ])

    pkg_rosbridge = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='8090',
            description='HTTP port for the UI. Defaults to 8090 to avoid conflicts '
                        'with :80 (course UI) / :8080 (wifi-connect AP) / :9090 (rosbridge).',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='preview',
            description="'preview' serves the built dist/ (production); "
                        "'dev' runs the Vite dev server with HMR (development).",
            choices=['preview', 'dev'],
        ),
        DeclareLaunchArgument(
            'video',
            default_value='true',
            description='Spawn a LAN-reachable web_video_server (0.0.0.0:8091) '
                        'alongside the UI. Set false when the robot already '
                        'runs one (hardware path — lupin-cameras.service exposes '
                        '8091 on the Pi so raw frames stay on-host).',
        ),
        DeclareLaunchArgument(
            'video_target',
            default_value='http://localhost:8091',
            description='Where Vite forwards /_video proxy requests. Default '
                        'localhost matches the sim/dev path (laptop also runs '
                        'web_video_server). On hardware, set this to the robot '
                        '(http://192.168.42.1:8091) AND pass video:=false so '
                        'raw frames never cross WiFi.',
        ),
        DeclareLaunchArgument(
            'rosbridge',
            default_value='true',
            description='Spawn rosbridge_websocket on :9090 co-located with the '
                        'UI. Default true so the HMI is self-sufficient on '
                        'whichever host runs it (laptop or robot). Set false '
                        'on the robot to defer to the vendor rosbridge in '
                        'mirte-ros.service, or anywhere :9090 is already '
                        'bound.',
        ),
        DeclareLaunchArgument(
            'tls',
            default_value='false',
            description='Serve the UI over https with a self-signed cert. Required '
                        'on the robot for the voice tab (browsers gate the mic API '
                        'to secure contexts). Sim/dev keep http on localhost.',
        ),

        LogInfo(msg=['Lupin Web HMI · serving from ', _REPO_WEB_DIR, ' on :', port]),

        # 1. Vite (preview by default, dev with HMR if requested). LUPIN_TLS=1
        # tells vite.config.ts to enable @vitejs/plugin-basic-ssl so the same
        # port serves https (with same-origin /_ros + /_video proxies).
        # LUPIN_VIDEO_TARGET retargets the /_video proxy at runtime — see the
        # `video_target` launch arg above.
        ExecuteProcess(
            cmd=['npm', 'run', npm_script, '--', '--port', port, '--host', '0.0.0.0'],
            cwd=_REPO_WEB_DIR,
            additional_env={
                'LUPIN_TLS': tls_env,
                'LUPIN_VIDEO_TARGET': video_target,
            },
            output='screen',
            shell=False,
            # When this launch goes down (Ctrl-C), kill the npm + child vite cleanly.
            sigterm_timeout='5',
            sigkill_timeout='2',
        ),

        # 2. LAN-reachable web_video_server for the camera tab. Bound to 0.0.0.0
        # so any device on the same network can pull MJPEG; vendor's localhost-only
        # instance on [::1]:8181 is left untouched. Port 8091 is hardcoded — the
        # web app's default URL points at it; if you ever need to change it, edit
        # both this file and src/lib/settings.ts:defaultWebVideoUrl().
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='lupin_web_video_server',
            parameters=[{'port': 8091, 'address': '0.0.0.0'}],
            output='screen',
            condition=IfCondition(spawn_video),
        ),

        # 3. rosbridge_websocket on :9090 — JSON broker for the HMI. Default
        # on so the HMI is self-sufficient. When this launch runs on the
        # laptop alongside Nav2/SLAM, JSON encoding of the /tf flood lives
        # here instead of on the Pi (the Phase-2 split in
        # project_offload_strategy). Robot-side bringup should set
        # rosbridge:=false to avoid bind-conflict with mirte-ros's vendor
        # instance.
        #
        # NOTE: explicit port:=9090 — without it, launch's arg propagation
        # carries this file's outer `port` (8090, the Vite UI) into
        # rosbridge_websocket_launch.xml's same-named `port` arg, causing
        # rosbridge to try binding 8090 and crash-loop on EADDRINUSE.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(pkg_rosbridge, 'launch',
                             'rosbridge_websocket_launch.xml'),
            ),
            launch_arguments=[('port', '9090')],
            condition=IfCondition(spawn_rosbridge),
        ),
    ])
