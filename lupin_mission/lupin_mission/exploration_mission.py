"""ExplorationMission + MonitoringMission — the two phases of an
autonomous "discover N tags, then monitor them" run.

Like InspectionMission these are pure-Python models the orchestrator's HSM
drives; no ROS imports. ExplorationMission tracks the discovered-tag set and
the goal N (the EXPLORING lifecycle state). When N is reached the orchestrator
swaps in a MonitoringMission built from the discovered set — a wrap-around
cursor over those tags that completes after ``target_cycles`` full sweeps
(default 1: visit each tag once), or early on pause/abort.
"""

from __future__ import annotations

from typing import Optional

from lupin_msgs.msg import Observation

from .inspection_mission import Mission, TagResult


class ExplorationMission(Mission):
    """Frontier exploration until ``discovery_goal`` distinct tags are found.

    The discovered-tag registry itself lives on the orchestrator (fed by the
    /perception/discovered_tags subscription); this model holds the goal and
    a mirror of the latest count/poses so ``is_complete()`` and the published
    counters work without reaching back into the node.
    """

    name = 'ExplorationMission'

    def __init__(self, *, mission_id: str, discovery_goal: int):
        self.mission_id = mission_id
        self.discovery_goal = max(1, int(discovery_goal))
        # tag_id -> map Pose, updated from the discovery feed.
        self.discovered: dict[str, object] = {}
        self.no_frontier = False

    def update_discovered(self, discovered: dict[str, object]) -> None:
        self.discovered = dict(discovered)

    def discovered_count(self) -> int:
        return len(self.discovered)

    def is_complete(self) -> bool:
        return self.discovered_count() >= self.discovery_goal

    # ── Mission surface (no per-tag nav during EXPLORING) ───────────────
    def current_tag_id(self) -> str:
        return ''

    def current_result(self) -> TagResult:  # pragma: no cover - not driven
        return TagResult(tag_id='')

    def counters(self) -> dict[str, int]:
        return {
            'total': self.discovery_goal,
            'completed': self.discovered_count(),
            'failed': 0,
            'unreachable': 0,
            'skipped': 0,
        }


class MonitoringMission(Mission):
    """Re-scan loop over the discovered tags.

    Mirrors InspectionMission's per-tag surface so the orchestrator's
    NAVIGATING/SCANNING callbacks drive it unchanged. The cursor *wraps*
    (``advance()`` modulo len, bumping ``cycles``) and the loop completes
    after ``target_cycles`` full sweeps (default 1). The orchestrator can
    also stop it early via pause/abort.
    """

    name = 'MonitoringMission'

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
        self.mission_id = mission_id
        self.nav_max_attempts = max(1, int(nav_max_attempts))
        self.approach_yaw = float(approach_yaw)
        self.standoff_m = float(standoff_m)
        # Number of full sweeps over the discovered set before the loop
        # completes (→ RETURNING). 1 = visit every tag once. Operator can
        # raise it for repeated monitoring.
        self._target_cycles = max(1, int(target_cycles))
        # Deterministic order so the loop is predictable run-to-run.
        self._tags: list[str] = sorted(discovered.keys())
        self._poses: dict[str, object] = dict(discovered)
        self._index = 0
        self.cycles = 0
        # cumulative per-status tallies across the whole monitoring run
        self._tally = {'completed': 0, 'failed': 0, 'unreachable': 0}
        self._result: TagResult = TagResult(
            tag_id=self._tags[0] if self._tags else ''
        )

    # ── current-leg accessors ──────────────────────────────────────────
    def is_complete(self) -> bool:
        # Complete once the cursor has wrapped target_cycles times (one full
        # sweep per cycle), or immediately if there are no tags to visit.
        return (not self._tags) or (self.cycles >= self._target_cycles)

    def current_tag_id(self) -> str:
        if not self._tags:
            return ''
        return self._tags[self._index]

    def current_result(self) -> TagResult:
        return self._result

    def pose_for(self, tag_id: str) -> Optional[object]:
        return self._poses.get(tag_id)

    def current_pose(self) -> Optional[object]:
        return self._poses.get(self.current_tag_id())

    # ── advancement (wrap-around) ──────────────────────────────────────
    def advance(self) -> None:
        # Tally the leg we just finished.
        st = self._result.status
        if self._result.closed:
            if st == Observation.STATUS_OK:
                self._tally['completed'] += 1
            elif st == Observation.STATUS_UNREACHABLE:
                self._tally['unreachable'] += 1
            elif st == Observation.STATUS_SCAN_FAILED:
                self._tally['failed'] += 1
        if not self._tags:
            return
        self._index += 1
        if self._index >= len(self._tags):
            self._index = 0
            self.cycles += 1
        # Fresh result for the (re)visited tag.
        self._result = TagResult(tag_id=self._tags[self._index])

    def counters(self) -> dict[str, int]:
        return {
            'total': len(self._tags),
            'completed': self._tally['completed'],
            'failed': self._tally['failed'],
            'unreachable': self._tally['unreachable'],
            'skipped': 0,
        }
