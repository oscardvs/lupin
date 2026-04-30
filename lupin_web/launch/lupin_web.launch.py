"""Launch the Lupin Web HMI as a static file server on :8090.

Brings up a built `dist/` artifact via `npm run preview` (which also handles
SPA fallback). Coexists with the course web stack — never binds :80, :8080,
or :9090.

Usage:
    ros2 launch lupin_web lupin_web.launch.py            # serve dist/ on :8090
    ros2 launch lupin_web lupin_web.launch.py port:=8091 # override port
    ros2 launch lupin_web lupin_web.launch.py mode:=dev  # vite dev server (HMR)

Requirements:
    - Node 20+ and npm available on PATH
    - `npm install` already run inside lupin_web/web/
    - For mode:=preview (default): `npm run build` already run (so dist/ exists)

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
from launch.substitutions import LaunchConfiguration, PythonExpression


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

    # Resolve the npm script at launch time based on `mode`.
    npm_script = PythonExpression([
        "'dev' if '", mode, "'.lower() == 'dev' else 'preview'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='8090',
            description='HTTP port. Defaults to 8090 to avoid conflicts with '
                        ':80 (course UI) / :8080 (wifi-connect AP) / :9090 (rosbridge).',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='preview',
            description="'preview' serves the built dist/ (production); "
                        "'dev' runs the Vite dev server with HMR (development).",
            choices=['preview', 'dev'],
        ),

        LogInfo(msg=['Lupin Web HMI · serving from ', _REPO_WEB_DIR, ' on :', port]),

        ExecuteProcess(
            cmd=['npm', 'run', npm_script, '--', '--port', port, '--host', '0.0.0.0'],
            cwd=_REPO_WEB_DIR,
            output='screen',
            shell=False,
            # When this launch goes down (Ctrl-C), kill the npm + child vite cleanly.
            sigterm_timeout='5',
            sigkill_timeout='2',
        ),
    ])
