"""arm_traj — one place to build the arm JointTrajectory.

Before this, the bridge, the preset server, the teleop node and the sim shim
each constructed a `JointTrajectory` slightly differently (different velocity
hints, different time_from_start flooring, some with no velocities at all). The
JTC consumes the per-point velocity to SHAPE its interpolated per-tick position
setpoints, and the Hiwonder Telemetrix HW interface was reported to reject
points with velocity 0.0 — so an inconsistent velocity hint changed how a move
executed depending on which surface issued it. All four publishers now go
through `build_arm_trajectory()`.

NOTE on the 0.0-velocity claim: it is asserted by the original teleop comment
but NOT yet confirmed empirically against the JTC→HW path. The non-zero default
is kept as the conservative choice; the live check is tracked in the audit doc.
"""

from __future__ import annotations

from typing import Sequence

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Floor on time_from_start: zero is invalid for the JTC; very small values race
# with the controller's ~10 Hz update period.
MIN_TRAJECTORY_TIME_S = 0.1
# Per-joint velocity hint (rad/s). Non-zero by default — see module docstring.
TRAJECTORY_VELOCITY_RAD_S = 1.0


def duration_from_seconds(seconds: float) -> Duration:
    sec = int(seconds)
    nsec = int(round((seconds - sec) * 1e9))
    d = Duration()
    d.sec = sec
    d.nanosec = nsec
    return d


def build_arm_trajectory(
    joint_names: Sequence[str],
    positions: Sequence[float],
    time_s: float,
    velocity: float = TRAJECTORY_VELOCITY_RAD_S,
) -> JointTrajectory:
    """Build a single-point JointTrajectory holding `positions` for all
    `joint_names`, reached over `max(MIN_TRAJECTORY_TIME_S, time_s)`."""
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = [float(p) for p in positions]
    point.velocities = [float(velocity)] * len(joint_names)
    point.time_from_start = duration_from_seconds(max(MIN_TRAJECTORY_TIME_S, time_s))
    traj.points = [point]
    return traj
