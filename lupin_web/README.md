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
- **Map / Nav** — placeholder pane for upcoming SLAM + AprilTag work.

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

## What's not in here yet

- Actual SLAM / Nav2 integration (Map view is a placeholder)
- AprilTag overlay on the camera stream
- Authentication, PWA / service worker, multi-user awareness — all deferred
- Hardware launch file + systemd unit — those land in a follow-up MR into
  the `hardware` branch
