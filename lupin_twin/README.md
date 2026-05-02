# lupin_twin

Digital-twin node for Team Lupin's MIRTE Master. Aggregates per-tag
observations from `lupin_mission` into a live world snapshot and exposes
two contracts to downstream consumers (the web HMI today; FloraNova export
and anomaly detection later):

- `/twin/state` — periodic latched snapshot of every observed tag.
- `/twin/get_field` — on-demand IDW interpolation of a sensor field over
  a bbox. NaN-gates beyond `max_distance_to_nearest_tag` so the heat map
  only paints where data supports it.

## Architecture

```
                                    +------------------------+
                                    |     lupin_twin         |
                                    |  (TwinStateStore +     |
                                    |   IDW + cache + node)  |
                                    +-----------+------------+
                                                |
       /floranova/observations  --------------> | (subscribe)
       (Observation incl.                       |
        tag_pose_in_map from AMCL)              |
                                                +--> /twin/state (1 Hz, latched)
                                                +--> /twin/get_field (srv)
                                                |
                                                v
                                        +-------+--------+
                                        |   HMI / export |
                                        +----------------+
```

- **Pure listener.** No upstream dependency on Nav2 or Gazebo. The twin
  starts immediately and waits for observations.
- **Single-threaded.** `MutuallyExclusiveCallbackGroup` +
  `SingleThreadedExecutor`. The store and field cache aren't thread-safe
  by design; serialisation is at the executor layer.
- **Sim and hardware identical.** The twin only consumes
  `/floranova/observations`; whether the orchestrator filled it from
  `mdp-greenhouse` (sim) or real perception + sensor modules (hardware)
  is invisible here. Swapping the bridge is a bridge-implementation
  concern, not a twin contract change.

## ROS interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| Subscribe | `/floranova/observations` | `lupin_msgs/Observation` | RELIABLE+TRANSIENT_LOCAL depth 50 — late subscribers get the orchestrator's backlog. Frame-checked against `frame_id` param; mismatched frames are dropped with a warning. |
| Publish | `/twin/state` | `lupin_msgs/TwinState` | RELIABLE+TRANSIENT_LOCAL depth 1 (latched) at `state_publish_rate_hz`. Empty `tags` is the legitimate startup state — nothing observed yet. |
| Service | `/twin/get_field` | `lupin_msgs/srv/GetField` | On-demand IDW field. Cached by `(sensor, observation_count, bbox, resolution)` with LRU bound (64 entries). Cells beyond `idw_max_distance_m` from any sample are encoded as NaN. |

### QoS notes for native subscribers

Both publish topics are TRANSIENT_LOCAL durability. A native ROS 2
subscriber that defaults to VOLATILE — e.g.
`ros2 topic echo /twin/state` without `--qos-durability transient_local`
— will get nothing on first connect and only see future updates at the
publish rate. The HMI consumes via rosbridge_websocket, which doesn't
expose the QoS wrinkle, so this only bites CLI debugging.

## Per-tag observation pipeline

1. Orchestrator scans tag *N*, calls bridge, builds an `Observation`
   with `STATUS_OK` plus `tag_pose_in_map` populated from the latest
   AMCL snapshot. v1 = robot's standoff pose; v2 hardware =
   AprilTag's pose from `apriltag_ros`.
2. Twin's subscriber thread records the observation in
   `TwinStateStore` — pose cached from the *first* OK observation per
   tag (tags don't move; re-stamping would jitter the HMI marker).
3. `samples_for_sensor()` filters to tags with both a pose AND a
   finite reading for the requested sensor. NaN/inf readings are
   dropped here so they can't poison the IDW math downstream.
4. `compute_idw_field()` builds the heat-map grid: weighted sum
   within `idw_falloff_radius_m`, NaN beyond `idw_max_distance_m`
   (defaults equal — see "Why no donut" below).
5. Result cached as a frozen `_CachedField`, copied into each rclpy
   `GetField.Response` on the way out — no Response-object aliasing.

## IDW design notes

- **Why no donut.** `idw_falloff_radius_m` and `idw_max_distance_m`
  default equal (1.5 m). When they differ — falloff < max_distance —
  cells in the gap pass the honesty gate but find no neighbours within
  falloff and fall through to the nearest-value branch, producing a
  hard "constant ring" between the two. Operators can still configure
  them differently if they want that behaviour explicitly; the default
  avoids it.
- **Why pin coincident cells.** Cells closer than `COINCIDENT_EPS_M`
  (1 mm) to a tag get pinned to that tag's value rather than dividing
  by ≈0 in the weighted sum. Operators expect heat-map values *at*
  the tag locations to read exactly the tag's reading.
- **Why `power=2`.** Textbook IDW default; visually pleasant gradient.
  Lower → too smooth, no per-tag definition. Higher (3+) → Voronoi-like
  cells. Stay in [1, 4] in practice.
- **Why no Kriging / GP.** Sample sizes are small (≤30 tags), the
  visualisation latency budget is tight (sub-100 ms for a 200×200
  grid), and IDW is "lousy interpolator for prediction, fine
  interpolator for visualisation" — visualisation is the brief.

## Parameters

```yaml
lupin_twin:
  ros__parameters:
    observations_topic: /floranova/observations
    state_topic: /twin/state
    field_service_name: /twin/get_field
    state_publish_rate_hz: 1.0
    frame_id: map               # observations in any other frame are dropped + warned
    buffer_len: 32              # per-tag history depth (forward-compat — not exposed yet)

    # IDW tuning. Defaults are the brief's; override per-mission if tag
    # density or sensor footprint changes. Falloff and max-distance are
    # deliberately equal — see the "Why no donut" note above.
    idw_power: 2.0
    idw_falloff_radius_m: 1.5
    idw_max_distance_m: 1.5
```

## Tests

```bash
colcon test --packages-select lupin_twin --event-handlers console_direct+
```

26 unit tests covering:
- `idw.py` (17): grid sizing, empty-input all-NaN, single-sample disc,
  max-distance independence from falloff, coincident-cell pinning,
  multi-sample weighting, explored-mask gating, value-min/max bookkeeping
  with NaN cells, sample-count propagation, immutability.
- `state.py` (8): observation-record, pose-cache-on-first, observation
  count, malformed input rejection, history cap, lazy pose-set,
  NaN/inf reading filter, insertion-order preservation.

End-to-end smoke (manual): publish a fake `Observation` to
`/floranova/observations` with `--qos-durability transient_local`, check
that `/twin/state` echoes the tag and `/twin/get_field` returns a sensible
single-source field. See `lupin_twin/launch/twin.launch.py` for the
standalone bring-up.

## What's NOT in scope this MR

- **Disk persistence** — orchestrator's JSONL log already covers replay.
  When FloraNova actually wires up an export consumer, the twin gets a
  bag-recorder sidecar, not a re-implementation.
- **/map gating** — `compute_idw_field` accepts an `explored_mask` arg
  but the node passes `None`. When the brief tightens to "only paint
  inside the SLAM-explored area", subscribe to `/map`, threshold to a
  bool grid, pass through. Code-comment in `node.py:_handle_get_field`
  flags the hook.
- **History service** — per-tag time series for trend lines on the HMI.
  Store already keeps a 32-deep ring buffer; service interface is the
  remaining work.
- **Anomaly detection** — domain-specific; lives in a separate node
  that subscribes to `/twin/state` (or `/floranova/observations` for
  raw values).
