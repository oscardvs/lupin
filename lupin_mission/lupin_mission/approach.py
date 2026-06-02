"""Per-tag approach-pose computation.

Tag locations come as ``(x, y)`` only — they don't say how the robot should
park itself to scan the tag. For sim that's fine because the bridge is a
sim oracle keyed by tag id, but on hardware the camera must actually see
the AprilTag, so the *yaw* of the goal pose matters as much as ``(x, y)``.

This module owns the geometry. It is pure Python — no ROS imports — so the
unit tests don't need rclpy and the orchestrator can call into it directly
without indirection. The orchestrator builds its NavigateToPose goal from a
:class:`TagApproach`, hardware operators tweak edge cases via an overrides
yaml, and a future perception MR can layer visual servoing on top of the
result without touching this module.

Geometry rule
-------------
A tag belongs to the *table* whose bbox-centre it sits closest to (Euclidean,
ties broken by lexicographic table id). The tag's outward normal is the unit
vector from that table's centre toward the tag — i.e. it points away from the
plant bed. The robot's goal pose is ``tag + standoff * normal`` and its yaw
points back at the tag, so the front-facing camera frames the AprilTag
head-on.

Degenerate inputs (no tables in scope, tag exactly at a table centre, both
position and orientation overridden) fall back to the orchestrator's global
``approach_yaw`` parameter and ``derived_from='fallback'``. A loud-but-survive
posture: the mission still runs and the operator can read what happened
from the log line.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional


# Sanity clamps. A standoff of 0.2 m keeps a 100 mm AprilTag on the Orbbec at a
# usable size; 1.5 m starts to lose detection reliability. The user-facing
# default lives on the orchestrator parameter — these are floor and ceiling.
MIN_STANDOFF_M = 0.2
MAX_STANDOFF_M = 1.5

# Below this distance from the picked table centre the normal is ill-defined,
# so we punt to the fallback yaw rather than divide by ~zero.
_DEGENERATE_NORMAL_EPS = 1e-6


@dataclass(frozen=True)
class TagApproach:
    """Result of :func:`compute_approach`. The orchestrator drops the first
    three fields straight into a NavigateToPose goal; the rest are diagnostic
    and surface in the log line so an operator can tell which path won."""

    tag_id: str
    goal_x: float
    goal_y: float
    goal_yaw: float
    standoff_m: float
    derived_from: str  # 'geometry' | 'override' | 'fallback'
    table_id: Optional[str] = None  # which table the geometry attached to


def compute_approach(
    tag_id: str,
    tags: Mapping[str, Mapping],
    tables: Mapping[str, Mapping],
    *,
    standoff_m: float,
    fallback_yaw: float,
    overrides: Optional[Mapping[str, Mapping]] = None,
) -> TagApproach:
    """Compute the NavigateToPose goal for ``tag_id``.

    Parameters
    ----------
    tag_id:
        Key into ``tags``. Must be present.
    tags:
        ``{tag_id: {'x': float, 'y': float, ...}}`` — the same dict the
        orchestrator already loads via :mod:`lupin_mission.tag_locations`.
    tables:
        ``{table_id: {'x0': float, 'x1': float, 'y0': float, 'y1': float}}``
        from the ``tables`` key of the same JSON. May be empty (e.g. an
        unrelated mission type) — that just forces ``derived_from='fallback'``.
    standoff_m:
        How far in front of the tag to park, before clamping to
        :data:`MIN_STANDOFF_M`/:data:`MAX_STANDOFF_M`.
    fallback_yaw:
        Used when geometry can't be derived. The orchestrator passes its
        ``approach_yaw`` parameter so legacy behaviour is preserved exactly
        for anything geometry can't handle.
    overrides:
        Optional ``{tag_id: {<subset of approach fields>}}`` — partial
        overrides merge with the geometry result, full overrides (all four
        of ``goal_x``, ``goal_y``, ``goal_yaw``, ``standoff``) skip the
        geometry path entirely.

    Raises
    ------
    KeyError
        If ``tag_id`` is not in ``tags``. The orchestrator validates the
        tag sequence at ``/mission/start`` time so this should never reach
        a real mission, but raising is kinder than silently mis-driving.
    """
    if tag_id not in tags:
        raise KeyError(f'unknown tag_id: {tag_id!r}')

    tag_x = float(tags[tag_id]['x'])
    tag_y = float(tags[tag_id]['y'])
    override = (overrides or {}).get(tag_id) or {}

    # Full-pose override short-circuits geometry entirely. Useful on hardware
    # for tags whose physical placement breaks the "outward from table centre"
    # heuristic (mounting on a shelf, glare from a window, etc.).
    if all(k in override for k in ('goal_x', 'goal_y', 'yaw')):
        return TagApproach(
            tag_id=tag_id,
            goal_x=float(override['goal_x']),
            goal_y=float(override['goal_y']),
            goal_yaw=float(override['yaw']),
            standoff_m=_clamp_standoff(float(override.get('standoff', standoff_m))),
            derived_from='override',
            table_id=None,
        )

    # Geometry path. Pick the nearest table by bbox-centre distance.
    picked = _nearest_table(tag_x, tag_y, tables)
    standoff_used = _clamp_standoff(float(override.get('standoff', standoff_m)))

    if picked is None:
        # No tables loaded — only path here in normal operation is misconfig.
        # Honour an explicit yaw override, otherwise fall back to the global.
        return TagApproach(
            tag_id=tag_id,
            goal_x=tag_x,
            goal_y=tag_y,
            goal_yaw=float(override.get('yaw', fallback_yaw)),
            standoff_m=0.0,
            derived_from='fallback',
            table_id=None,
        )

    table_id, (cx, cy) = picked
    nx = tag_x - cx
    ny = tag_y - cy
    norm = math.hypot(nx, ny)
    if norm < _DEGENERATE_NORMAL_EPS:
        # Tag at table centre — normal is undefined. Same fallback shape as
        # "no tables".
        return TagApproach(
            tag_id=tag_id,
            goal_x=tag_x,
            goal_y=tag_y,
            goal_yaw=float(override.get('yaw', fallback_yaw)),
            standoff_m=0.0,
            derived_from='fallback',
            table_id=table_id,
        )
    nx /= norm
    ny /= norm

    # Goal pose: standoff metres along the outward normal from the tag.
    # Yaw: the robot's +X (front camera) points at the tag, i.e. opposite
    # the outward normal.
    geom_x = tag_x + nx * standoff_used
    geom_y = tag_y + ny * standoff_used
    geom_yaw = math.atan2(tag_y - geom_y, tag_x - geom_x)

    # Partial overrides merge field-by-field.
    return TagApproach(
        tag_id=tag_id,
        goal_x=float(override['goal_x']) if 'goal_x' in override else geom_x,
        goal_y=float(override['goal_y']) if 'goal_y' in override else geom_y,
        goal_yaw=float(override['yaw']) if 'yaw' in override else geom_yaw,
        standoff_m=standoff_used,
        derived_from='override' if override else 'geometry',
        table_id=table_id,
    )


def compute_discovered_approach(
    tag_id: str,
    pose_in_map: Mapping[str, float] | object,
    *,
    standoff_m: float,
    fallback_yaw: float,
    robot_xy: Optional[tuple[float, float]] = None,
) -> TagApproach:
    """Approach pose for a tag discovered live (no a-priori table geometry).

    A discovered tag carries its own map pose (position + orientation) from
    the detector's solvePnP + TF lookup, so — unlike :func:`compute_approach`,
    which needs the table bbox — we derive the standoff from the tag's own
    facing normal.

    Geometry rule
    -------------
    The AprilTag's surface normal is its local +Z axis (OpenCV/ArUco marker
    convention: +Z points out of the marker toward the camera that saw it,
    i.e. into the aisle). The robot parks ``standoff`` metres along that
    normal, projected onto the map's XY plane, yaw pointing back at the tag.

    When the normal is near-vertical (tag facing up/down, or noisy
    orientation), the projection degenerates; we fall back to approaching
    along the vector from ``robot_xy`` to the tag (a side we can already
    reach), and finally to ``fallback_yaw`` if no robot pose is available.

    Parameters
    ----------
    pose_in_map:
        A ``geometry_msgs/Pose`` (or any object with ``.position.{x,y}`` and
        ``.orientation.{x,y,z,w}``). The tag's location in the map frame.
    standoff_m, fallback_yaw, robot_xy:
        As above. ``robot_xy`` is the robot's current map position, used only
        for the degenerate-normal fallback.
    """
    tag_x = float(pose_in_map.position.x)
    tag_y = float(pose_in_map.position.y)
    standoff_used = _clamp_standoff(float(standoff_m))

    # Tag local +Z axis expressed in the map frame (third column of the
    # rotation matrix for quaternion (x, y, z, w)).
    qx = float(pose_in_map.orientation.x)
    qy = float(pose_in_map.orientation.y)
    qz = float(pose_in_map.orientation.z)
    qw = float(pose_in_map.orientation.w)
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    norm = math.hypot(nx, ny)

    if norm >= _DEGENERATE_NORMAL_EPS:
        nx /= norm
        ny /= norm
        derived = 'discovered_normal'
    elif robot_xy is not None:
        # Degenerate normal — approach from where the robot already is.
        rx, ry = robot_xy
        vx, vy = rx - tag_x, ry - tag_y
        vnorm = math.hypot(vx, vy)
        if vnorm >= _DEGENERATE_NORMAL_EPS:
            nx, ny = vx / vnorm, vy / vnorm
            derived = 'discovered_robot'
        else:
            return TagApproach(
                tag_id=tag_id, goal_x=tag_x, goal_y=tag_y,
                goal_yaw=float(fallback_yaw), standoff_m=0.0,
                derived_from='fallback', table_id=None,
            )
    else:
        return TagApproach(
            tag_id=tag_id, goal_x=tag_x, goal_y=tag_y,
            goal_yaw=float(fallback_yaw), standoff_m=0.0,
            derived_from='fallback', table_id=None,
        )

    goal_x = tag_x + nx * standoff_used
    goal_y = tag_y + ny * standoff_used
    goal_yaw = math.atan2(tag_y - goal_y, tag_x - goal_x)
    return TagApproach(
        tag_id=tag_id, goal_x=goal_x, goal_y=goal_y, goal_yaw=goal_yaw,
        standoff_m=standoff_used, derived_from=derived, table_id=None,
    )


def load_approach_overrides(path: str) -> dict:
    """Load the per-tag overrides yaml. Empty path → empty dict (no error).

    Schema::

        overrides:
          "12":
            yaw: 1.5708
            standoff: 0.35
          "17":
            goal_x: 4.10
            goal_y: 5.83
            yaw: 3.14

    Any subset of ``goal_x`` / ``goal_y`` / ``yaw`` / ``standoff`` is allowed;
    missing fields fall back to the geometry computation. Unknown keys are
    silently dropped (forwards-compat). Unknown tag ids are kept — they just
    never match anything at lookup time, which is the right behaviour if the
    operator pre-stages overrides before adding the corresponding tag.
    """
    if not path:
        return {}
    import yaml  # local import — avoids the ROS package gaining a yaml dep

    p = Path(path).expanduser()
    if not p.is_file():
        raise FileNotFoundError(f'approach overrides file not found: {p}')
    raw = yaml.safe_load(p.read_text()) or {}
    table = raw.get('overrides') or {}
    if not isinstance(table, dict):
        raise ValueError(
            f'{p}: top-level "overrides:" must be a mapping, got {type(table).__name__}'
        )

    allowed = {'yaw', 'standoff', 'goal_x', 'goal_y'}
    cleaned: dict[str, dict] = {}
    for tag_id, entry in table.items():
        if not isinstance(entry, dict):
            continue
        cleaned[str(tag_id)] = {k: float(v) for k, v in entry.items() if k in allowed}
    return cleaned


# ---------------------------------------------------------------------------
# helpers


def _clamp_standoff(s: float) -> float:
    if not math.isfinite(s):
        return MIN_STANDOFF_M
    return max(MIN_STANDOFF_M, min(MAX_STANDOFF_M, s))


def _nearest_table(
    x: float, y: float, tables: Mapping[str, Mapping],
) -> Optional[tuple[str, tuple[float, float]]]:
    """Return (table_id, (cx, cy)) of the nearest table centre, or None.

    Ties broken by lexicographic table id so the result is deterministic
    across runs and unit tests.
    """
    best: Optional[tuple[float, str, tuple[float, float]]] = None
    for tid in sorted(tables.keys()):
        t = tables[tid]
        try:
            cx = (float(t['x0']) + float(t['x1'])) / 2.0
            cy = (float(t['y0']) + float(t['y1'])) / 2.0
        except (KeyError, TypeError, ValueError):
            continue
        d = math.hypot(x - cx, y - cy)
        if best is None or d < best[0]:
            best = (d, tid, (cx, cy))
    if best is None:
        return None
    return best[1], best[2]
