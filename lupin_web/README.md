# lupin_web

Browser-based HMI for the MIRTE Master. A Vite + React + shadcn/ui frontend
served on port `8090`, talking to the rosbridge_websocket already running on
port `9090` of the course image.

This package coexists with the course web interface (rosboard, nginx, the
Node backend) — it does not replace any of it. If `lupin_web` crashes the
course UI is still up.

## Layout

```
lupin_web/
├── package.xml          ament_python package
├── setup.py             installs only ROS metadata, never node_modules/dist
├── lupin_web/           Python package (room for a future ROS node)
└── web/                 Vite + React + TS + Tailwind + shadcn frontend
    ├── COLCON_IGNORE    keeps colcon out of this directory entirely
    ├── src/
    │   ├── App.tsx
    │   ├── components/  StatusDot + shadcn ui
    │   └── lib/         rosbridge connection + Twist helpers
    └── package.json
```

`web/node_modules/` and `web/dist/` are gitignored at the package root.

## Dev workflow

On the robot (or any machine with Node 20+):

```bash
cd web/
npm install        # only needed once and after dependency changes
npm run dev        # Vite dev server on 0.0.0.0:8090
```

Then from any device on the same network: open `http://<robot-ip>:8090`.

Vite serves on all interfaces (`server.host: true`) so phones on the same
hotspot, on ZeroTier, or connected to the robot's own AP all reach it via
the same URL.

The page connects to `ws://<window.location.hostname>:9090` automatically.
Override with `?ros=ws://otherhost:9090` for cross-machine debugging.

## What's in the first vertical

- A connection-status dot showing rosbridge state (connecting / connected /
  closed / error) with the URL it's targeting.
- A single "Nudge forward" button that publishes
  `geometry_msgs/Twist {linear.x = 0.1}` at 20 Hz for 500 ms on
  `/mirte_base_controller/cmd_vel`, then a zero Twist to stop.

This vertical exists to prove the rosbridge plumbing end-to-end before we
layer the actual joystick UI on top.
