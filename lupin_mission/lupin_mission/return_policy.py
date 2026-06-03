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
