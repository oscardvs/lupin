# lupin_greenhouse_bridge

ROS 2 wrapper around the course-provided
[`mdp-greenhouse`](https://pypi.org/project/mdp-greenhouse/) Python
simulator. Holds a long-lived `GreenhouseSimulator` instance and
exposes it as a service so the rest of the Lupin stack can consume
sensor readings through a stable contract.

Environmental sensing in this project is **not** done in Gazebo — the
robot navigates to a tag, identifies it, and asks this bridge for the
tag's temperature / humidity / CO₂ / light / soil-moisture readings.

## Service

| Name | Type | Purpose |
| --- | --- | --- |
| `/greenhouse_bridge/get_tag_reading` | `lupin_msgs/srv/GetTagReading` | Returns a `TagReading` (sim-time-of-day + per-sensor `SensorReading[]`) for the requested `tag_id`. Returns `STATUS_UNKNOWN_TAG` for IDs not in the loaded greenhouse. |

This is intentionally the *only* interface — see the message
definitions in `lupin_msgs/{msg,srv}/` for the full schema. A
publisher loop was rejected during design because the mission node is
the only consumer that knows when a reading is meaningful (it's
on-demand, not at a fixed rate).

## Launch

```bash
ros2 launch lupin_greenhouse_bridge greenhouse_bridge.launch.py
```

Brings up one node, `/greenhouse_bridge`, advertising the service
above. No Gazebo dependency — runs standalone.

Smoke test from a second shell:

```bash
ros2 service call /greenhouse_bridge/get_tag_reading \
  lupin_msgs/srv/GetTagReading "{tag_id: '1'}"
```

### Launch args

All forward to the underlying `GreenhouseSimulator` and the sim-time
config; sentinels mean "leave the upstream default alone". Setting
`debug_time_of_day` or `debug_seed` flips the sim's `debug_mode` on.

| Arg | Default | Meaning |
| --- | --- | --- |
| `tag_file` | `''` | Path to a custom `tag_locations.json` (else use `mdp-greenhouse`'s shipped one). |
| `sim_config_file` | `''` | Path to a custom `greenhouse_config.yaml`. |
| `debug_time_of_day` | `-1.0` | Hours (0–24); freezes sim time for deterministic tests. |
| `speedup_factor` | `0.0` | Sim seconds per real second (default config: 1800 → full 24 h cycle in 48 s). |
| `debug_seed` | `-1` | RNG seed; deterministic noise for assertion-based tests. |

## Upstream gotchas (mdp-greenhouse, as of 1.0.6)

- **Missing entry point.** The wheel ships no console script, so the
  upstream `mdp-greenhouse` CLI in the README is **not** on PATH.
  Invoke as a module: `python -m greenhouse_sim.cli ...`.
- **Undeclared deps.** Bare `pip install mdp-greenhouse` crashes on
  first import (`ModuleNotFoundError: yaml`). Top-level README's
  prerequisites step pins the missing deps (`pyyaml`, `matplotlib`,
  `numpy`, `python3-tk` for GUIs).
- **`current_time()` returns wrong values (partial fix in 1.0.6).**
  1.0.6 wraps the result with `% 86400`, so the *range* is now
  guaranteed. The hour-to-seconds conversion underneath is still
  wrong in both branches:
  - debug uses `t_h * 24 * 60 * 60` — that's seconds-per-day, ×24
    too big. Effect: every integer `debug_time_of_day` collapses to
    `0` after the wrap.
  - live uses `24 * 60 * now.hour` — that's 1440 sec/h instead of
    3600, ×2.5 too small.

  Effect: `sim_time_of_day_seconds` does not correspond to wall-clock
  time — treat it as opaque. The bridge pins
  `mdp-greenhouse>=1.0.6` for the range guarantee and keeps its own
  modulo wrap as defense-in-depth. `debug_seed` is unaffected and
  works correctly. Reported to C.Pek@tudelft.nl; partial fix landed
  in 1.0.6, the conversion itself is still pending.

## Tests

```bash
colcon test --packages-select lupin_greenhouse_bridge
colcon test-result --verbose
```

Contract tests live in `test/test_bridge_node.py`. They use deterministic
mode (`debug_seed`) so assertions on sensor noise are meaningful.

## Where to fit in the stack

```
[mission node]                                    [Nav2 / mission planner]
       │                                                       │
       │ get_tag_reading(tag_id)                               │
       ▼                                                       ▼
[lupin_greenhouse_bridge] ────── (no shared state) ────── [robot pose]
       │
       ▼
[GreenhouseSimulator]  (mdp-greenhouse, in-process Python)
```

Tag IDs match the IDs baked into the generated greenhouse SDF
(`lupin_bringup/worlds/greenhouse.world`), so once an AprilTag
detector lands and produces real IDs, the same call works without
re-mapping.

## Maintainer

Oscar Devos — `o.a.e.devos@student.tudelft.nl`
