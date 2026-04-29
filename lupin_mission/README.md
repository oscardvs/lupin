# lupin_mission

Mission orchestrator for the Lupin / FloraNova digital twin.

A single ROS 2 node, `mission_orchestrator`, that drives the MIRTE Master
through a tag-scanning routine end-to-end:

```
IDLE -> NAVIGATING -> SCANNING -> LOGGING -> NAVIGATING -> ... -> DONE
```

For each tag in the configured sequence the orchestrator

1. sends a `nav2_msgs/action/NavigateToPose` goal to the tag's known
   `(x, y)` from `mdp-greenhouse`'s `tag_locations.json`,
2. on `STATUS_SUCCEEDED`, calls `/greenhouse_bridge/get_tag_reading` for
   that tag's sensor readings,
3. logs a one-line summary, appends to an in-memory mission log, and
   moves to the next tag.

There is no AprilTag perception in v1 — the robot navigates to known map
coordinates and trusts that it's at the right station. Closing the loop
with a visual confirmation lives outside this package.

## Quickstart

The orchestrator reads tag coordinates from the `mdp-greenhouse` Python
package, which is **not** in rosdep. Install it once per workspace:

```bash
pip install 'mdp-greenhouse>=1.0.3,<2'
```

Then bring up the four pieces in any order before launching the
orchestrator:

```bash
# Terminal 1 — Greenhouse Gazebo world (sim only)
ros2 launch lupin_bringup greenhouse.launch.py

# Terminal 2 — Greenhouse bridge (the sensor service)
ros2 launch lupin_greenhouse_bridge greenhouse_bridge.launch.py

# Terminal 3 — Nav2 (localisation + navigation lifecycle stack)
ros2 launch lupin_navigation nav2.launch.py

# Terminal 4 — The orchestrator
ros2 launch lupin_mission mission.launch.py
```

To visit a specific subset of tags instead of all 22:

```bash
ros2 launch lupin_mission mission.launch.py tag_sequence:='[1, 5, 12]'
```

## Parameters

| Parameter                  | Type           | Default | Notes |
|----------------------------|----------------|---------|-------|
| `tag_sequence`             | `string[]`     | `[]`    | Tag IDs in visit order. Empty = all tags from `tag_locations.json` in numeric-string order. |
| `approach_yaw`             | `double`       | `0.0`   | Yaw (rad) sent in the `NavigateToPose` goal. Single value for v1; per-tag yaw later. |
| `nav_timeout_sec`          | `double`       | `60.0`  | Per-attempt nav timeout. On expiry the goal is cancelled and counted as a failure. |
| `service_timeout_sec`      | `double`       | `5.0`   | Bridge service-call timeout. |
| `dependency_timeout_sec`   | `double`       | `30.0`  | How long IDLE waits for Nav2 + bridge to appear. |
| `nav_retry_limit`          | `int`          | `1`     | Re-issues per tag on nav failure. `1` = one retry (so up to 2 attempts total). |
| `frame_id`                 | `string`       | `map`   | Frame used in the `PoseStamped` goal. |

## What a happy run looks like

State transitions log at `INFO` with `[state] EVENT: from->to (tag_id=X)`:

```
[INFO] [mission_orchestrator]: Mission orchestrator created: 3 tags in sequence, frame_id=map, approach_yaw=0.0
[INFO] [mission_orchestrator]: Waiting up to 30.0s for nav2 + bridge...
[INFO] [mission_orchestrator]: Dependencies up. Starting mission.
[INFO] [mission_orchestrator]: [IDLE] NEXT_TAG: IDLE->NAVIGATING (tag_id=1)
[INFO] [mission_orchestrator]: [NAVIGATING] NAV_SUCCEEDED: NAVIGATING->SCANNING (tag_id=1)
[INFO] [mission_orchestrator]: [SCANNING] BRIDGE_OK: SCANNING->LOGGING (tag_id=1)
[INFO] [mission_orchestrator]: [tag 1] temperature=22.4 humidity=58.1 co2=441.0 light=796.1 (sim_time=43200s)
[INFO] [mission_orchestrator]: [LOGGING] NEXT_TAG: LOGGING->NAVIGATING (tag_id=2)
...
[INFO] [mission_orchestrator]: Mission DONE: attempted=3 succeeded=3 failed=0
```

## Topics / services / actions

| Direction | Name                                    | Type                                     |
|-----------|-----------------------------------------|------------------------------------------|
| Action    | `navigate_to_pose`                      | `nav2_msgs/action/NavigateToPose` (client) |
| Service   | `/greenhouse_bridge/get_tag_reading`    | `lupin_msgs/srv/GetTagReading` (client)  |

The orchestrator publishes nothing in v1. A future digital-twin publisher
and a `start_mission` service are flagged with `TODO` markers in the
source.

## Tests

```bash
cd ~/ros2_ws
colcon test --packages-select lupin_mission --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/lupin_mission
```

The tests spawn an in-process fake `NavigateToPose` action server and a
fake `GetTagReading` service; they don't pull in Nav2 or the real bridge.
