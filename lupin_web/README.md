# lupin_web

Browser-based HMI for the MIRTE Master. Vite + React + Tailwind + shadcn/ui
frontend served on port `8090`, talking to the rosbridge_websocket already
running on port `9090` of the course image. **Coexists with the course web
interface — does not replace it.** If `lupin_web` crashes, the course UI is
still up.

## Surfaces

A persistent top bar (logo · rosbridge pill · battery pill · clock · settings
gear · permanent E-STOP) wraps five tabs:

- **Teleop** — two virtual joysticks (left = linear x/y, right = angular z),
  live Twist readout, speed-scale slider, mirrored stop button.
- **Cameras** — MJPEG stream from `web_video_server` with FPS counter,
  reload, fullscreen, friendly placeholder when no stream.
- **Telemetry** — Lidar canvas (top-down), IMU (roll/pitch/yaw + ω bars),
  Odometry pose+twist, Battery + voltage sparkline, Arm joints, System
  placeholder.
- **Logs** — `/rosout` live tail with severity filter, node filter, search,
  pause/clear, auto-scroll toggle, capped at 500 rows.
- **Map** — live SLAM occupancy grid (subscribes `/map`), robot pose via
  `ROSLIB.TFClient` against the `map → base_link` transform, latest Nav2
  plan (`/plan`) as a chartreuse polyline, and click-and-drag to publish a
  `geometry_msgs/PoseStamped` to `/goal_pose`. AprilTag overlay still TODO.

## Engineering notes

- **One ROS context.** A single `RosProvider` owns the rosbridge connection,
  reconnects with exponential backoff, exposes `useTopic` / `usePublisher`
  hooks. No component creates its own `ROSLIB.Ros`.
- **Mock mode.** `?mock=1` swaps the provider for a synthetic-data source —
  every surface works without a robot. The bar for "demoable" is a clean
  walkthrough in mock mode.
- **E-STOP is permanent furniture, not a button.** A red full-height bar on
  every screen. Triggers on press, `visibilitychange`, `blur`, `beforeunload`,
  and rosbridge disconnect. While active, gates all `useCmdVel` publishes
  and heartbeats zero-Twist at 10 Hz. Reset is manual.
- **No sign hacks.** The app publishes vanilla REP-103 Twists to whatever
  topic Settings declares (default `/cmd_vel`). The hardware quirk where
  `+linear.x` moves real Mirte backward is fixed downstream by a separate
  ROS adapter node, not in here.
- **TypeScript message contracts.** `src/types/ros.ts` defines the message
  layouts we subscribe to; rosbridge JSON is cast directly.
- **Throttled rendering.** High-frequency feeds (Lidar, IMU) write to refs
  and use `useThrottledRender` / `useAnimationLoop` so we don't re-render
  the whole tree on every message.

## Layout

```
lupin_web/
├── package.xml          ament_python package
├── setup.py             installs only ROS metadata, never node_modules/dist
├── lupin_web/           Python package (room for future ROS nodes)
└── web/                 Vite + React + TS + Tailwind + shadcn frontend
    ├── COLCON_IGNORE    keeps colcon out of this directory entirely
    └── src/
        ├── App.tsx           layout shell + tabs + providers
        ├── main.tsx
        ├── index.css         dark-default theme tokens
        ├── types/ros.ts      message-layout contracts
        ├── lib/
        │   ├── ros.tsx       RosProvider, useTopic, usePublisher
        │   ├── estop.tsx     EStopProvider, useCmdVel, auto-stop triggers
        │   ├── settings.ts   localStorage-backed settings store
        │   ├── mock.ts       synthetic-data generators
        │   ├── throttle.ts   useThrottledRender / useAnimationLoop / Ring
        │   └── utils.ts      cn helper
        ├── components/
        │   ├── TopBar.tsx · ConnectionPill · BatteryPill · ClockPill
        │   ├── EStopButton.tsx · SettingsDrawer.tsx
        │   ├── views/        Teleop / Cameras / Telemetry / Logs / Map
        │   ├── widgets/      Joystick / TwistReadout / LidarCanvas /
        │   │                 ImuCard / OdometryCard / ArmJointsCard /
        │   │                 BatteryCard / Sparkline / CameraStream /
        │   │                 SystemCard
        │   └── ui/            inlined shadcn primitives
        └── ...
```

`web/node_modules/`, `web/dist/`, and `web/dist-node-ts/` are gitignored at
the package root.

## Dev workflow

On any machine with Node 20+ (laptop or robot):

```bash
cd web/
npm install                       # first time + after dep changes
npm run dev                       # Vite dev server on 0.0.0.0:8090
npm run dev -- --mode mock        # (or just append ?mock=1 to the URL)
```

Open `http://<host>:8090` from any device on the same network. With
`?mock=1` the app runs against synthetic data and is fully demoable
offline.

The page picks up its rosbridge URL from `ws://<window.location.hostname>:9090`
by default; the Settings drawer (gear icon) lets you override that and every
topic name. All overrides persist to `localStorage`.

## Diagnostic logging

Settings → Diagnostics → "Log outgoing publishes" (or set
`debugPublish: true` in localStorage) prints every published message as
JSON to the console with the topic name. Use this when debugging
rosbridge-side wire-format issues end to end.

## Deploying on the robot (hardware branch)

Two ways to bring the UI up; pick whichever fits the moment.

### Manual via ros2 launch

```bash
ros2 launch lupin_web lupin_web.launch.py             # serves dist/ on :8090
ros2 launch lupin_web lupin_web.launch.py mode:=dev   # vite dev server (HMR)
ros2 launch lupin_web lupin_web.launch.py port:=8091  # alternative port
```

The `preview` mode (default) requires `npm run build` to have produced a
`dist/` artefact. The launch file just wraps `npm run preview` / `npm run dev`
with the flags pinned, so you still need `npm install` to have been run once.

### Auto-start at boot via systemd

A system-level unit lives at `systemd/lupin-web.service` and is installed
through `scripts/install-systemd.sh`. Idempotent — re-run any time:

```bash
# on the robot, from anywhere on the repo
sudo ~/ros2_ws/src/lupin/lupin_web/scripts/install-systemd.sh
```

The unit runs `npm run preview` against `dist/` as the `mirte` user. After
each `git pull` you must rebuild and restart:

```bash
cd ~/ros2_ws/src/lupin/lupin_web/web
npm install      # only if package-lock.json changed
npm run build
sudo systemctl restart lupin-web
```

Inspect with `journalctl -u lupin-web -f`. Uninstall with
`sudo .../install-systemd.sh --uninstall`.

The unit binds `:8090` only — never `:80` (course UI), `:8080` (wifi-connect
AP captive portal), or `:9090` (rosbridge). Failure of the course web stack
does not bring this down and vice versa.

## What's not in here yet

- AprilTag overlay on the camera stream
- Authentication, PWA / service worker, multi-user awareness — all deferred
