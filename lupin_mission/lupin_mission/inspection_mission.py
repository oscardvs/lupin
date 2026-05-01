"""InspectionMission: the per-tag controller that owns the data the
INSPECTING sub-states operate on.

The state machine itself lives on the orchestrator node (so the HSM diagram
covers the full lifecycle in one graph). This class is the *model* the
sub-machine drives — tag list, current index, attempt counter, per-tag
result, mission-level counters. Future MappingMission will be a sibling
class with the same shape but different innards; ``Mission`` is the common
base.

No ROS imports here on purpose — keeps it unit-testable and keeps the
node-vs-mission concerns separated.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from lupin_msgs.msg import Observation, TagReading


@dataclass
class TagResult:
    tag_id: str
    status: int = Observation.STATUS_OK   # final status; mutable until tag is closed
    closed: bool = False                  # set when the tag's observation has been emitted
    nav_attempts: int = 0                 # total NavigateToPose goals issued for this tag
    detail: str = ''                      # status_detail for the emitted observation
    tag_reading: Optional[TagReading] = None


class Mission:
    """Common surface so the orchestrator can drive any mission type."""
    name: str = ''

    def is_complete(self) -> bool:
        raise NotImplementedError


class InspectionMission(Mission):
    """Walks ``tag_sequence`` once: navigate → scan → publish per tag.

    Intentionally narrow surface — the orchestrator drives the lifecycle
    sub-machine; this object just bookkeeps so the orchestrator can ask
    "what tag are we on, what's its pose, mark this attempt failed, are
    we done?" without juggling its own indices.
    """

    name = 'InspectionMission'

    def __init__(
        self,
        *,
        mission_id: str,
        tag_sequence: list[str],
        tag_locations: dict,
        nav_max_attempts: int,
        approach_yaw: float,
    ):
        self.mission_id = mission_id
        self.tag_locations = tag_locations
        self.nav_max_attempts = max(1, int(nav_max_attempts))
        self.approach_yaw = float(approach_yaw)
        self.results: list[TagResult] = [TagResult(tag_id=t) for t in tag_sequence]
        self._index: int = 0

    # ── current-tag accessors ──────────────────────────────────────────
    def is_complete(self) -> bool:
        return self._index >= len(self.results)

    def current_index(self) -> int:
        return self._index

    def current_result(self) -> TagResult:
        return self.results[self._index]

    def current_tag_id(self) -> str:
        if self.is_complete():
            return ''
        return self.results[self._index].tag_id

    def current_target_xy(self) -> tuple[float, float]:
        """(x, y) from tag_locations.json for the active tag.

        Raises KeyError if the active tag isn't in the loaded locations —
        the orchestrator should have validated tag_sequence at start.
        """
        tag_id = self.current_tag_id()
        loc = self.tag_locations[tag_id]
        return float(loc['x']), float(loc['y'])

    # ── nav outcome ────────────────────────────────────────────────────
    def register_nav_attempt(self) -> int:
        """Increment the nav attempts counter; return the new count.

        Called each time a NavigateToPose goal is issued for the current tag.
        """
        result = self.current_result()
        result.nav_attempts += 1
        return result.nav_attempts

    def can_retry_nav(self) -> bool:
        """True iff another NavigateToPose attempt is still allowed."""
        return self.current_result().nav_attempts < self.nav_max_attempts

    def mark_unreachable(self, detail: str) -> TagResult:
        result = self.current_result()
        result.status = Observation.STATUS_UNREACHABLE
        result.detail = detail
        result.closed = True
        return result

    # ── scan outcome ───────────────────────────────────────────────────
    def mark_scan_ok(self, tag_reading: TagReading) -> TagResult:
        result = self.current_result()
        result.status = Observation.STATUS_OK
        result.tag_reading = tag_reading
        result.closed = True
        return result

    def mark_scan_failed(self, detail: str) -> TagResult:
        result = self.current_result()
        result.status = Observation.STATUS_SCAN_FAILED
        result.detail = detail
        result.closed = True
        return result

    # ── operator overrides ─────────────────────────────────────────────
    def mark_skipped(self, detail: str) -> TagResult:
        """Used by /mission/skip_current and by /mission/abort for remaining tags."""
        result = self.current_result()
        result.status = Observation.STATUS_SKIPPED
        result.detail = detail
        result.closed = True
        return result

    def remaining_indices(self) -> list[int]:
        """Indices of all not-yet-closed tags from current onward.

        Used by /mission/abort to emit one SKIPPED observation per
        outstanding tag in a single sweep.
        """
        return [
            i for i in range(self._index, len(self.results))
            if not self.results[i].closed
        ]

    def force_skip(self, index: int, detail: str) -> TagResult:
        """Mark an arbitrary not-yet-closed index as SKIPPED. Helper for abort."""
        result = self.results[index]
        result.status = Observation.STATUS_SKIPPED
        result.detail = detail
        result.closed = True
        return result

    # ── advancement ────────────────────────────────────────────────────
    def advance(self) -> None:
        """Move the cursor forward. Call after the current tag's observation
        has been emitted (i.e. on PUBLISHING → next)."""
        self._index += 1

    # ── summary ────────────────────────────────────────────────────────
    def counters(self) -> dict[str, int]:
        ok = sum(1 for r in self.results if r.closed and r.status == Observation.STATUS_OK)
        unreachable = sum(
            1 for r in self.results if r.closed and r.status == Observation.STATUS_UNREACHABLE
        )
        scan_failed = sum(
            1 for r in self.results if r.closed and r.status == Observation.STATUS_SCAN_FAILED
        )
        skipped = sum(
            1 for r in self.results if r.closed and r.status == Observation.STATUS_SKIPPED
        )
        return {
            'total': len(self.results),
            'completed': ok,
            'failed': scan_failed,
            'unreachable': unreachable,
            'skipped': skipped,
        }

    def summary_lines(self) -> list[str]:
        lines = []
        for r in self.results:
            if r.status == Observation.STATUS_OK and r.tag_reading is not None:
                names = [sr.name for sr in r.tag_reading.readings]
                lines.append(
                    f'  tag {r.tag_id}: OK (attempts={r.nav_attempts}, '
                    f'sensors={names})'
                )
            elif r.status == Observation.STATUS_UNREACHABLE:
                lines.append(
                    f'  tag {r.tag_id}: UNREACHABLE (attempts={r.nav_attempts}, '
                    f'reason={r.detail or "?"})'
                )
            elif r.status == Observation.STATUS_SCAN_FAILED:
                lines.append(
                    f'  tag {r.tag_id}: SCAN_FAILED (reason={r.detail or "?"})'
                )
            elif r.status == Observation.STATUS_SKIPPED:
                lines.append(
                    f'  tag {r.tag_id}: SKIPPED (reason={r.detail or "?"})'
                )
            else:
                lines.append(f'  tag {r.tag_id}: status={r.status} closed={r.closed}')
        return lines
