# Stream A — Mission Logic Pass — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the monitoring mission finite (one nearest-neighbour sweep then return), survive a failed battery dock without silently ending, and label mission events honestly in the HMI.

**Architecture:** Pure-Python, ROS-free decision logic (new `return_policy.py`, `events.py`, plus functions in `exploration_mission.py`) unit-tested like `approach.py`; the orchestrator node (`node.py`) wires them in; one e2e harness test proves the exploration→monitoring→DONE path. Message contract gains two fields (`last_event`, `last_event_severity`).

**Tech Stack:** ROS 2 Humble (rclpy, `transitions` HSM), pytest, React/TypeScript HMI.

**Scope:** Stream A of the greenhouse mission redesign (see `docs/superpowers/specs/2026-06-03-greenhouse-mission-redesign-design.md`). Streams B/C/D are separate plans.

---

## Environment & conventions

- All work happens in the worktree `/home/oskrt/worktrees/mission-redesign` on branch `feat/mission-perception-redesign`. Integrate into `feat/sim-port-mission-2026-06-02` via MR when the phase is verified.
- **One-time setup** (builds the worktree's packages as a symlinked overlay so Python edits are picked up live; only `.msg` changes need a rebuild):
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/oskrt/ros2_ws/install/setup.bash        # underlay: all deps + sibling lupin pkgs
  cd /home/oskrt/worktrees/mission-redesign
  colcon build --symlink-install --packages-select lupin_msgs lupin_mission
  source install/setup.bash                            # overlay: worktree lupin_msgs + lupin_mission
  ```
- **Run pure tests** (no rebuild needed for `.py`-only changes):
  ```bash
  cd /home/oskrt/worktrees/mission-redesign/lupin_mission
  python -m pytest test/<file>::<test> -v
  ```
- Commit messages: conventional style, **no `Co-Authored-By` trailer**.

---

## Task 1: MonitoringMission — one sweep then complete

**Files:**
- Modify: `lupin_mission/lupin_mission/exploration_mission.py` (`MonitoringMission.__init__`, `is_complete`)
- Test: `lupin_mission/test/test_monitoring_mission.py`

- [ ] **Step 1: Replace the obsolete "never completes" test and add completion tests.**

In `test/test_monitoring_mission.py`, DELETE the whole `test_cursor_wraps_and_never_completes` function and ADD:

```python
def test_one_sweep_then_complete():
    m = _mission(('1', '2', '3'))  # target_cycles defaults to 1
    assert m.current_tag_id() == '1'
    assert not m.is_complete()
    m.advance()                      # closed tag 1 -> cursor 2
    assert m.current_tag_id() == '2'
    assert not m.is_complete()
    m.advance()                      # closed tag 2 -> cursor 3
    assert m.current_tag_id() == '3'
    assert not m.is_complete()
    m.advance()                      # closed tag 3 -> wrap, cycle 1
    assert m.cycles == 1
    assert m.is_complete()


def test_target_cycles_allows_multiple_sweeps():
    discovered = {'1': _pose(0.0, 0.0), '2': _pose(1.0, 0.0)}
    m = MonitoringMission(
        mission_id='m', discovered=discovered,
        nav_max_attempts=1, approach_yaw=0.0, standoff_m=0.5,
        target_cycles=2,
    )
    m.advance(); m.advance()         # one full sweep -> cycle 1
    assert m.cycles == 1
    assert not m.is_complete()
    m.advance(); m.advance()         # second sweep -> cycle 2
    assert m.cycles == 2
    assert m.is_complete()
```

- [ ] **Step 2: Run the new tests, verify they FAIL.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_monitoring_mission.py::test_one_sweep_then_complete test/test_monitoring_mission.py::test_target_cycles_allows_multiple_sweeps -v
```
Expected: FAIL — `test_one_sweep_then_complete` fails at `assert m.is_complete()` (currently always False); `test_target_cycles_allows_multiple_sweeps` fails with `TypeError: __init__() got an unexpected keyword argument 'target_cycles'`.

- [ ] **Step 3: Add `target_cycles` and finite completion.**

In `exploration_mission.py`, `MonitoringMission.__init__` signature — add the param after `standoff_m`:
```python
    def __init__(
        self,
        *,
        mission_id: str,
        discovered: dict[str, object],
        nav_max_attempts: int,
        approach_yaw: float,
        standoff_m: float,
        target_cycles: int = 1,
    ):
```
In the body, after `self.standoff_m = float(standoff_m)`:
```python
        # Number of full sweeps over the discovered set before the loop
        # completes (→ RETURNING). 1 = visit every tag once. Operator can
        # raise it for repeated monitoring.
        self._target_cycles = max(1, int(target_cycles))
```
Replace `is_complete`:
```python
    def is_complete(self) -> bool:
        # Complete once the cursor has wrapped target_cycles times (one full
        # sweep per cycle), or immediately if there are no tags to visit.
        return (not self._tags) or (self.cycles >= self._target_cycles)
```

- [ ] **Step 4: Run the full file, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_monitoring_mission.py -v
```
Expected: PASS (all tests, including the unchanged `test_empty_discovered_is_complete`, `test_fresh_result_each_visit_and_tally`, `test_tags_sorted_and_cursor_starts_at_first`).

- [ ] **Step 5: Commit.**

```bash
cd /home/oskrt/worktrees/mission-redesign
git add lupin_mission/lupin_mission/exploration_mission.py lupin_mission/test/test_monitoring_mission.py
git commit -m "feat(mission): monitoring completes after target_cycles sweeps (default 1)"
```

---

## Task 2: Nearest-neighbour sweep order

**Files:**
- Modify: `lupin_mission/lupin_mission/exploration_mission.py` (import, new `order_tags_nearest_first`, `MonitoringMission.__init__`)
- Test: `lupin_mission/test/test_monitoring_mission.py`

- [ ] **Step 1: Write failing tests.**

In `test/test_monitoring_mission.py`, update the import line to also import the new function:
```python
from lupin_mission.exploration_mission import (
    ExplorationMission, MonitoringMission, order_tags_nearest_first,
)
```
Add:
```python
def test_order_numeric_when_no_start():
    discovered = {'10': _pose(0.0, 0.0), '2': _pose(9.0, 0.0), '1': _pose(8.0, 0.0)}
    assert order_tags_nearest_first(discovered, None) == ['1', '2', '10']


def test_order_nearest_neighbour_from_start():
    discovered = {'A': _pose(0.0, 0.0), 'B': _pose(5.0, 0.0), 'C': _pose(10.0, 0.0)}
    # Start near x=11 -> visit C(10), B(5), A(0).
    assert order_tags_nearest_first(discovered, (11.0, 0.0)) == ['C', 'B', 'A']


def test_monitoring_uses_nearest_order():
    discovered = {'A': _pose(0.0, 0.0), 'B': _pose(5.0, 0.0), 'C': _pose(10.0, 0.0)}
    m = MonitoringMission(
        mission_id='m', discovered=discovered, nav_max_attempts=1,
        approach_yaw=0.0, standoff_m=0.5, start_xy=(11.0, 0.0),
    )
    assert m.current_tag_id() == 'C'
```

- [ ] **Step 2: Run, verify FAIL.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_monitoring_mission.py::test_order_numeric_when_no_start test/test_monitoring_mission.py::test_order_nearest_neighbour_from_start test/test_monitoring_mission.py::test_monitoring_uses_nearest_order -v
```
Expected: FAIL — `ImportError: cannot import name 'order_tags_nearest_first'`.

- [ ] **Step 3: Implement the ordering function and use it.**

In `exploration_mission.py`, add to the imports near the top (after `from .inspection_mission import Mission, TagResult`):
```python
from .tag_locations import numeric_string_sort_key
```
Add this module-level function above `class ExplorationMission`:
```python
def order_tags_nearest_first(discovered, start_xy):
    """Visiting order over the discovered tags.

    Greedy nearest-neighbour from ``start_xy`` (the robot's map position) when
    every discovered pose exposes an (x, y); otherwise a deterministic numeric
    tag-id sort. Pure (no ROS) so it is unit-testable.
    """
    ids = list(discovered.keys())

    def _xy(pose):
        try:
            return (float(pose.position.x), float(pose.position.y))
        except AttributeError:
            return None

    if start_xy is None or any(_xy(discovered[t]) is None for t in ids):
        return sorted(ids, key=numeric_string_sort_key)

    remaining = list(ids)
    order: list[str] = []
    cx, cy = float(start_xy[0]), float(start_xy[1])
    while remaining:
        def _key(t, _cx=cx, _cy=cy):
            x, y = _xy(discovered[t])
            return ((x - _cx) ** 2 + (y - _cy) ** 2, numeric_string_sort_key(t))
        nxt = min(remaining, key=_key)
        order.append(nxt)
        remaining.remove(nxt)
        cx, cy = _xy(discovered[nxt])
    return order
```
In `MonitoringMission.__init__`, add `start_xy` to the signature (after `target_cycles`):
```python
        target_cycles: int = 1,
        start_xy: Optional[tuple[float, float]] = None,
```
Replace the tag-ordering line:
```python
        # Deterministic order so the loop is predictable run-to-run.
        self._tags: list[str] = sorted(discovered.keys())
```
with:
```python
        # Nearest-neighbour from the robot's start pose (falls back to a
        # numeric tag-id sort) so the sweep is a coherent path, not a zig-zag.
        self._tags: list[str] = order_tags_nearest_first(discovered, start_xy)
```

- [ ] **Step 4: Run the full file, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_monitoring_mission.py -v
```
Expected: PASS. (`test_tags_sorted_and_cursor_starts_at_first` still passes: no `start_xy` → numeric sort of `('3','1','2')` → `['1','2','3']`.)

- [ ] **Step 5: Commit.**

```bash
cd /home/oskrt/worktrees/mission-redesign
git add lupin_mission/lupin_mission/exploration_mission.py lupin_mission/test/test_monitoring_mission.py
git commit -m "feat(mission): nearest-neighbour monitoring sweep order"
```

---

## Task 3: Wire completion + ordering into the orchestrator

**Files:**
- Modify: `lupin_mission/lupin_mission/node.py` (HSM transition ~157-161; `monitoring_sweeps` param ~411 and ~665; `on_enter_MONITORING` ~1751-1758)
- Test: `lupin_mission/test/test_orchestrator_v2.py` (`test_exploration_discovers_then_monitors`)

- [ ] **Step 1: Update the e2e test to expect one sweep then DONE.**

In `test_orchestrator_v2.py`, REPLACE the body of `test_exploration_discovers_then_monitors` with:

```python
    def test_exploration_discovers_then_monitors(self):
        # No /map is published, so EXPLORING just waits on frontiers; the
        # discovery feed (faked) drives the EXPLORING→MONITORING switch.
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50)

        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=2,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0),
                        f'state={self.orch.state}')

        # Publish the discovered tags meeting the goal → switch to MONITORING.
        self.discovered.publish(['1', '2'])
        self.assertTrue(_wait_until(
            lambda: self.orch.state.startswith('MONITORING'), 10.0,
        ), f'state={self.orch.state}')

        # One full sweep over the 2 discovered tags, then RETURNING → DONE.
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 20.0),
                        f'state={self.orch.state}')

        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok_obs), 2,
                         'expected exactly one monitoring sweep of 2 tags')
        # mission_type stayed ExplorationMission across the model swap.
        seen_types = {s.mission_type for s in self.collector.states if s.mission_type}
        self.assertIn('ExplorationMission', seen_types)
```

- [ ] **Step 2: Run it, verify FAIL.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_orchestrator_v2.py::TestOrchestratorV2::test_exploration_discovers_then_monitors -v
```
Expected: FAIL — the orchestrator never reaches `DONE` (monitoring loops forever), so the `_wait_until(... == 'DONE')` assertion times out.

- [ ] **Step 3: Add `MONITORING_PUBLISHING` to the completion transition.**

In `node.py`, the `inspection_complete` transition (currently lines ~157-161):
```python
        {
            "trigger": "inspection_complete",
            "source": "INSPECTING_PUBLISHING",
            "dest": "RETURNING",
        },
```
becomes:
```python
        {
            "trigger": "inspection_complete",
            "source": ["INSPECTING_PUBLISHING", "MONITORING_PUBLISHING"],
            "dest": "RETURNING",
        },
```

- [ ] **Step 4: Declare and read the `monitoring_sweeps` parameter.**

In `node.py`, next to the other `declare_parameter` calls (e.g. right after `self.declare_parameter('arm_travel_settle_s', 0.0)` ~line 411), add:
```python
        self.declare_parameter('monitoring_sweeps', 1)
```
Where the arm presets are read (right after `self._arm_travel_preset = str(self.get_parameter('arm_travel_preset').value)` ~line 665), add:
```python
        self._monitoring_sweeps = max(1, int(self.get_parameter('monitoring_sweeps').value))
```

- [ ] **Step 5: Pass `target_cycles` and `start_xy` when building the monitoring loop.**

In `on_enter_MONITORING` (~line 1751), the `MonitoringMission(...)` call gains two args:
```python
        self._mission = MonitoringMission(
            mission_id=self._mission.mission_id if self._mission else
            f'{self._mission_id_prefix}-{uuid.uuid4().hex[:8]}',
            discovered=discovered_poses,
            nav_max_attempts=self._nav_max_attempts,
            approach_yaw=self._approach_yaw,
            standoff_m=self._approach_standoff,
            target_cycles=self._monitoring_sweeps,
            start_xy=self._robot_xy(),
        )
```

- [ ] **Step 6: Run the e2e test + the full mission suite, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_orchestrator_v2.py::TestOrchestratorV2::test_exploration_discovers_then_monitors -v
python -m pytest test/ -v
```
Expected: PASS for the updated test and the whole suite (the InspectionMission path is unchanged; `test_exploration_no_tags_returns_home` still passes).

- [ ] **Step 7: Commit.**

```bash
cd /home/oskrt/worktrees/mission-redesign
git add lupin_mission/lupin_mission/node.py lupin_mission/test/test_orchestrator_v2.py
git commit -m "feat(mission): monitoring does one nearest-neighbour sweep then returns"
```

---

## Task 4: Battery dock — retry then pause, never silently DONE

**Files:**
- Create: `lupin_mission/lupin_mission/return_policy.py`
- Create: `lupin_mission/test/test_return_policy.py`
- Modify: `lupin_mission/lupin_mission/node.py` (`_on_return_nav_result`; dock-retry state + param; reset in `on_battery_low`)

- [ ] **Step 1: Write the failing unit test for the decision.**

Create `lupin_mission/test/test_return_policy.py`:
```python
"""Unit tests for the dock-return failure policy (pure, no ROS)."""

from lupin_mission.return_policy import decide_failed_dock


def test_battery_dock_retries_until_max():
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=0, retry_max=3) == 'retry'
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=2, retry_max=3) == 'retry'


def test_battery_dock_pauses_after_max():
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=3, retry_max=3) == 'pause'


def test_non_battery_dock_failure_is_done():
    # Normal completion / abort return: a failed dock ends the mission.
    assert decide_failed_dock(
        battery_low=False, docked_for_battery=False, resumable=False,
        retry_count=0, retry_max=3) == 'done'
    # Battery flag but not resumable -> still done.
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=False,
        retry_count=0, retry_max=3) == 'done'
```

- [ ] **Step 2: Run it, verify FAIL.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_return_policy.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'lupin_mission.return_policy'`.

- [ ] **Step 3: Implement the pure policy.**

Create `lupin_mission/lupin_mission/return_policy.py`:
```python
"""Dock-return failure policy — pure, ROS-free, unit-testable.

A battery-triggered return must never end the mission: its whole purpose is
to preserve the run for resume after charge. So a non-SUCCEEDED dock goal is
retried, then the robot pauses in place (still resumable). Every other return
(normal completion, operator abort) treats a failed dock as non-fatal and
ends the mission rather than stranding the robot in RETURNING.
"""

from __future__ import annotations


def decide_failed_dock(
    *,
    battery_low: bool,
    docked_for_battery: bool,
    resumable: bool,
    retry_count: int,
    retry_max: int,
) -> str:
    """Return 'retry' | 'pause' | 'done' for a non-SUCCEEDED dock result."""
    if battery_low and docked_for_battery and resumable:
        if retry_count < retry_max:
            return 'retry'
        return 'pause'
    return 'done'
```

- [ ] **Step 4: Run it, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_return_policy.py -v
```
Expected: PASS.

- [ ] **Step 5: Wire the policy into the orchestrator.**

In `node.py`, add the import near the other `from .` imports (e.g. after `from .approach import ...`):
```python
from .return_policy import decide_failed_dock
```
Declare the param next to the others (~line 411):
```python
        self.declare_parameter('dock_retry_max', 3)
```
Read it and init the counter where `self._docked_for_battery` is initialised (~line 768):
```python
        self._dock_retry_max = max(0, int(self.get_parameter('dock_retry_max').value))
        self._dock_retry_count = 0
```
In `on_battery_low`, reset the counter when it commits to a battery return — right after `self._docked_for_battery = True` (~line 923):
```python
        self._dock_retry_count = 0
```
In `_on_return_nav_result`, REPLACE the non-SUCCEEDED block. Current:
```python
        if status != GoalStatus.STATUS_SUCCEEDED:
            if status == GoalStatus.STATUS_CANCELED and not self._battery_low:
                self.get_logger().warn(
                    "RETURN: got CANCELED while returning without battery_low; "
                    "re-sending dock goal once."
                )
                self._send_return_nav_goal()
                return

            # Per spec: failures while returning to dock are non-fatal. End the
            # mission so the operator can start a fresh one instead of getting
            # stuck forever in RETURNING.
            self.get_logger().warn(
                f"RETURN: dock goal ended with status {status}; transitioning to DONE"
            )
            self._docked_for_battery = False
            self._manual_dock_requested = False
            self._paused = False
            self.returned()  # type: ignore[attr-defined]
            return
```
Replace with:
```python
        if status != GoalStatus.STATUS_SUCCEEDED:
            if status == GoalStatus.STATUS_CANCELED and not self._battery_low:
                self.get_logger().warn(
                    "RETURN: got CANCELED while returning without battery_low; "
                    "re-sending dock goal once."
                )
                self._send_return_nav_goal()
                return

            decision = decide_failed_dock(
                battery_low=self._battery_low,
                docked_for_battery=self._docked_for_battery,
                resumable=self._return_resumable,
                retry_count=self._dock_retry_count,
                retry_max=self._dock_retry_max,
            )
            if decision == 'retry':
                self._dock_retry_count += 1
                self.get_logger().warn(
                    f"RETURN: battery dock status {status}; "
                    f"retry {self._dock_retry_count}/{self._dock_retry_max}."
                )
                self._send_return_nav_goal()
                return
            if decision == 'pause':
                self.get_logger().warn(
                    f"RETURN: battery dock failed {self._dock_retry_count} time(s); "
                    "pausing in place, awaiting charge + /mission/resume."
                )
                self._paused = True
                self._last_error = "dock_unreachable"
                return

            # 'done': non-battery return — failed dock is non-fatal, end the
            # mission so the operator isn't stuck in RETURNING.
            self.get_logger().warn(
                f"RETURN: dock goal ended with status {status}; transitioning to DONE"
            )
            self._docked_for_battery = False
            self._manual_dock_requested = False
            self._paused = False
            self.returned()  # type: ignore[attr-defined]
            return
```
In the SUCCESS path of the same method, reset the counter — in the block that handles a successful battery dock (where `self._paused = True` / "Docked due to low battery"), add before its `return`:
```python
            self._dock_retry_count = 0
```

- [ ] **Step 6: Run the mission suite, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/ -v
```
Expected: PASS (existing battery/abort/return tests unaffected; the policy change only alters the battery-dock-FAILURE path, which no current test exercises).

- [ ] **Step 7: Commit.**

```bash
cd /home/oskrt/worktrees/mission-redesign
git add lupin_mission/lupin_mission/return_policy.py lupin_mission/test/test_return_policy.py lupin_mission/lupin_mission/node.py
git commit -m "fix(mission): battery dock retries then pauses, never silently DONE"
```

> **Manual sim check (do at end of phase, before MR):** run a sim mission, let the battery cross the low threshold, and force the dock goal to fail (e.g. block the dock pose). Confirm the mission RETRIES the dock and then ends up `paused` in `RETURNING` with `last_error=dock_unreachable` — NOT `DONE`. This is the regression for the reported "battery_low → DONE" bug; it is not automated because it needs the battery publisher + a reliably-failing dock.

---

## Task 5: Honest mission-event labelling

**Files:**
- Create: `lupin_mission/lupin_mission/events.py`
- Create: `lupin_mission/test/test_events.py`
- Modify: `lupin_msgs/msg/MissionState.msg`
- Modify: `lupin_mission/lupin_mission/node.py` (`_publish_state`)
- Modify: `lupin_web/web/src/types/ros.ts` (`MissionState`)
- Modify: `lupin_web/web/src/lib/mission.ts` (event log + `MissionEvent.kind`)

- [ ] **Step 1: Write the failing classifier unit test.**

Create `lupin_mission/test/test_events.py`:
```python
"""Unit tests for mission-event severity classification (pure, no ROS)."""

from lupin_mission.events import (
    SEVERITY_INFO, SEVERITY_WARN, SEVERITY_ERROR, classify_event,
)


def test_empty_is_info_no_label():
    assert classify_event('') == (SEVERITY_INFO, '')
    assert classify_event(None) == (SEVERITY_INFO, '')


def test_known_notices_are_warn_and_friendly():
    assert classify_event('battery_low') == (SEVERITY_WARN, 'battery low')
    assert classify_event('dock_unreachable') == (SEVERITY_WARN, 'dock unreachable — paused')
    assert classify_event('exploration_timeout') == (SEVERITY_WARN, 'exploration timed out')


def test_nav_status_prefix_is_warn():
    sev, label = classify_event('nav_status_6')
    assert sev == SEVERITY_WARN
    assert label == 'navigation retry'


def test_startup_failures_are_error():
    assert classify_event('localization_failed (0.42)')[0] == SEVERITY_ERROR
    assert classify_event('dependency_timeout: amcl')[0] == SEVERITY_ERROR


def test_unknown_code_is_warn_and_passes_through():
    assert classify_event('something_new') == (SEVERITY_WARN, 'something_new')
```

- [ ] **Step 2: Run it, verify FAIL.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_events.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'lupin_mission.events'`.

- [ ] **Step 3: Implement the classifier.**

Create `lupin_mission/lupin_mission/events.py`:
```python
"""Mission-event severity classification — pure, ROS-free, unit-testable.

The orchestrator surfaces a single ``last_error`` string for a mix of genuine
faults and routine notices (battery, timeouts, nav retries). This maps a code
to a severity + a human label so the HMI can show the right tone instead of
flagging every event as a red "orchestrator error".
"""

from __future__ import annotations

SEVERITY_INFO = 0
SEVERITY_WARN = 1
SEVERITY_ERROR = 2

# Codes that indicate a real, blocking failure (vs. a routine notice).
_ERROR_PREFIXES = ('localization_failed', 'dependency_timeout')

# Friendly labels for known codes (matched on the prefix before any detail).
_FRIENDLY = {
    'battery_low': 'battery low',
    'dock_unreachable': 'dock unreachable — paused',
    'exploration_timeout': 'exploration timed out',
    'estop_engaged': 'e-stop engaged',
    'aborted_in_prepare': 'aborted during prepare',
    'localization_failed': 'localization failed',
    'dependency_timeout': 'dependencies not ready',
    'scan_failed': 'scan failed',
}


def classify_event(code):
    """Map a last_error/event code to (severity, friendly_label).

    Empty/None code -> (SEVERITY_INFO, ''). Unknown codes pass through as
    SEVERITY_WARN with the raw code as the label.
    """
    code = (code or '').strip()
    if not code:
        return (SEVERITY_INFO, '')
    if code.startswith('nav_status_'):
        return (SEVERITY_WARN, 'navigation retry')
    # Strip a ' (...)' or ': ...' detail suffix to match the base code.
    base = code.split(':', 1)[0].split(' ', 1)[0]
    severity = SEVERITY_ERROR if base in _ERROR_PREFIXES else SEVERITY_WARN
    return (severity, _FRIENDLY.get(base, code))
```

- [ ] **Step 4: Run it, verify PASS.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_mission
python -m pytest test/test_events.py -v
```
Expected: PASS.

- [ ] **Step 5: Add the two fields to the message contract.**

In `lupin_msgs/msg/MissionState.msg`, replace the existing `last_error` line:
```
string last_error                # last non-fatal error surfaced by the orchestrator
```
with:
```
string last_error                # raw last-event code (battery_low, nav_status_6, ...)

# Human-facing event surfaced alongside last_error, classified by severity so
# the HMI shows the right tone instead of labelling everything a red fault.
string last_event                # friendly label, "" when no event
uint8  SEVERITY_INFO=0
uint8  SEVERITY_WARN=1
uint8  SEVERITY_ERROR=2
uint8  last_event_severity
```

- [ ] **Step 6: Populate the fields in `_publish_state`.**

In `node.py`, add the import near the other `from .` imports:
```python
from .events import classify_event
```
In `_publish_state`, replace:
```python
        msg.last_error = self._last_error
```
with:
```python
        msg.last_error = self._last_error
        severity, label = classify_event(self._last_error)
        msg.last_event = label
        msg.last_event_severity = severity
```

- [ ] **Step 7: Rebuild the message package and re-run the suite.**

```bash
source /home/oskrt/ros2_ws/install/setup.bash
cd /home/oskrt/worktrees/mission-redesign
colcon build --symlink-install --packages-select lupin_msgs lupin_mission
source install/setup.bash
cd lupin_mission && python -m pytest test/ -v
```
Expected: build succeeds; all mission tests PASS.

- [ ] **Step 8: Add the fields to the HMI MissionState type.**

In `lupin_web/web/src/types/ros.ts`, in `interface MissionState`, replace:
```typescript
  last_error: string
```
with:
```typescript
  last_error: string
  /** Friendly event label classified by the orchestrator; "" when none. */
  last_event: string
  /** 0 = info, 1 = warn, 2 = error. */
  last_event_severity: number
```

- [ ] **Step 9: Replace the "orchestrator error" event-log block.**

In `lupin_web/web/src/lib/mission.ts`, add `'notice'` to the `MissionEvent.kind` union (line ~145):
```typescript
  kind: 'lifecycle' | 'phase' | 'target' | 'pause' | 'estop' | 'fault' | 'observation' | 'notice'
```
Replace the `last_error` block (~lines 258-266):
```typescript
      if (state.last_error && state.last_error !== prev.last_error) {
        append({
          at,
          kind: 'fault',
          label: 'orchestrator error',
          detail: state.last_error,
          tone: 'err',
        })
      }
```
with:
```typescript
      if (state.last_error && state.last_error !== prev.last_error) {
        const sev = state.last_event_severity ?? 1
        const isFault = state.lifecycle_state === 'FAULT' || sev >= 2
        append({
          at,
          kind: isFault ? 'fault' : 'notice',
          label: state.last_event || state.last_error,
          detail: state.last_error,
          tone: isFault ? 'err' : sev === 0 ? 'info' : 'warn',
        })
      }
```

- [ ] **Step 10: Type-check the HMI.**

```bash
cd /home/oskrt/worktrees/mission-redesign/lupin_web/web
npm run build
```
Expected: build/tsc passes with no type errors. (If `npm` deps are not installed in the worktree, run `npm ci` first.)

- [ ] **Step 11: Commit.**

```bash
cd /home/oskrt/worktrees/mission-redesign
git add lupin_mission/lupin_mission/events.py lupin_mission/test/test_events.py \
        lupin_msgs/msg/MissionState.msg lupin_mission/lupin_mission/node.py \
        lupin_web/web/src/types/ros.ts lupin_web/web/src/lib/mission.ts
git commit -m "feat(mission): classify mission events; HMI shows tone not 'orchestrator error'"
```

---

## Verification (end of phase)

- [ ] Full mission test suite green: `cd /home/oskrt/worktrees/mission-redesign/lupin_mission && python -m pytest test/ -v`
- [ ] HMI builds: `cd /home/oskrt/worktrees/mission-redesign/lupin_web/web && npm run build`
- [ ] Manual sim run: exploration → monitoring does exactly one nearest-neighbour sweep → RETURNING → DONE (no infinite loop); HMI event log shows "battery low" (warn), not "orchestrator error" (err).
- [ ] Manual sim run (battery): battery low + failed dock → retry then `paused` in RETURNING, not DONE (Task 4 note).
- [ ] Open MR from `feat/mission-perception-redesign` into `feat/sim-port-mission-2026-06-02` for Stream A, or hold until later phases land — operator's call.

## Self-review (filled in by plan author)

- **Spec coverage (Stream A):** one-sweep completion ✓ (T1,T3); nearest-neighbour order ✓ (T2,T3); battery dock-and-pause surviving a failed dock ✓ (T4); honest event labels / kill "orchestrator error" ✓ (T5). `discovery_goal` cap unchanged (operator bumps in HMI) — no task needed.
- **Placeholders:** none — every step has concrete code + exact commands + expected output.
- **Type consistency:** `target_cycles`/`start_xy` params match between `exploration_mission.py` and the `node.py` call site; `decide_failed_dock(...)` kwargs match between `return_policy.py`, its test, and the node; `classify_event` return shape `(severity, label)` matches `events.py`, its test, and `_publish_state`; `last_event`/`last_event_severity` names match across `.msg`, `node.py`, `ros.ts`, `mission.ts`.
