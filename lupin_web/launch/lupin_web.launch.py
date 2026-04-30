"""Launch the Lupin Web HMI as a static file server on :8090, plus a
LAN-reachable web_video_server on :8091 for the camera tab.

Brings up a built `dist/` artifact via `npm run preview` (which also handles
SPA fallback). Coexists with the course web stack — never binds :80, :8080,
or :9090.

Why a second web_video_server: the vendor MIRTE setup already runs one on
[::1]:8181, but it's bound to localhost only — browsers on the LAN can't
reach it. We launch our own on 0.0.0.0:8091 alongside the UI so the camera
tab works from any device. The vendor instance is left untouched.

Usage:
    ros2 launch lupin_web lupin_web.launch.py                     # default
    ros2 launch lupin_web lupin_web.launch.py port:=8091          # override UI port
    ros2 launch lupin_web lupin_web.launch.py mode:=dev           # vite dev (HMR)
    ros2 launch lupin_web lupin_web.launch.py video:=false        # skip web_video_server

Requirements:
    - Node 20+ and npm available on PATH
    - `npm install` already run inside lupin_web/web/
    - For mode:=preview (default): `npm run build` already run (so dist/ exists)
    - `web_video_server` ROS package installed (it ships with the MIRTE image)

The Vite project is NOT installed to share/ (node_modules + dist are too large
and the tooling expects a writeable working tree). The launch file resolves
the source-tree `web/` directory in this order:
    1. $LUPIN_WEB_SRC if set
    2. ../web relative to this launch file (works from the source tree)
    3. ~/ros2_ws/src/lupin/lupin_web/web (the documented robot layout)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
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

    # Resolve the npm script at launch time based on `mode`.
    npm_script = PythonExpression([
        "'dev' if '", mode, "'.lower() == 'dev' else 'preview'"
    ])

    spawn_video = PythonExpression([
        "'", video, "'.lower() in ('true', '1', 'yes')"
    ])

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
                        'alongside the UI. Set false if you bring your own.',
        ),

        LogInfo(msg=['Lupin Web HMI · serving from ', _REPO_WEB_DIR, ' on :', port]),

        # 1. Vite (preview by default, dev with HMR if requested).
        ExecuteProcess(
            cmd=['npm', 'run', npm_script, '--', '--port', port, '--host', '0.0.0.0'],
            cwd=_REPO_WEB_DIR,
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
    ])
