"""Unit tests for :mod:`lupin_mission.frontier`.

Pure-Python — synthetic occupancy grids built from a tiny ASCII helper so
the frontier intent is obvious from the fixture. 'u'=unknown(-1), '.'=free(0),
'#'=occupied(100).
"""

from __future__ import annotations

import pytest

from lupin_mission.frontier import select_frontier_goal

_CELL = {'u': -1, '.': 0, '#': 100}


def _grid(ascii_rows):
    rows = [r.split() for r in ascii_rows]
    height = len(rows)
    width = len(rows[0])
    data = [_CELL[c] for row in rows for c in row]
    return data, width, height


def _select(ascii_rows, **kw):
    data, w, h = _grid(ascii_rows)
    return select_frontier_goal(data, w, h, 1.0, 0.0, 0.0, **kw)


def test_open_frontier_found():
    # 3x3 free block surrounded by unknown: the 8 border-free cells all
    # border unknown → one cluster of 8.
    goal = _select(
        [
            'u u u u u',
            'u . . . u',
            'u . . . u',
            'u . . . u',
            'u u u u u',
        ],
        min_cluster_cells=4,
        robot_radius_cells=0,
    )
    assert goal is not None
    assert goal.cluster_size == 8
    assert goal.num_clusters == 1
    # Returned point sits inside the free block (cols/rows 1..3 → world 1.5..3.5).
    assert 1.0 <= goal.x <= 4.0 and 1.0 <= goal.y <= 4.0


def test_all_unknown_returns_none():
    assert _select(['u u u', 'u u u', 'u u u'], robot_radius_cells=0) is None


def test_fully_explored_no_frontier():
    # Free everywhere, no unknown to border → no frontier.
    assert _select(['. . .', '. . .', '. . .'], robot_radius_cells=0) is None


def test_noise_cluster_rejected():
    # A single free cell touching unknown is a 1-cell frontier; with a higher
    # min_cluster_cells it's discarded as noise.
    rows = [
        '# # # # #',
        '# # . # #',
        '# # u # #',
        '# # # # #',
    ]
    assert _select(rows, min_cluster_cells=4, robot_radius_cells=0) is None


def test_clearance_drops_frontier_near_obstacle():
    # Free cell borders unknown but sits right next to an obstacle; a robot
    # radius of 1 cell removes it, leaving no frontier.
    rows = [
        '# . u',
        '# . #',
        '# # #',
    ]
    assert _select(rows, min_cluster_cells=1, robot_radius_cells=1) is None
    # With zero clearance the same frontier is acceptable.
    assert _select(rows, min_cluster_cells=1, robot_radius_cells=0) is not None


def test_nearest_cluster_preferred_with_robot_xy():
    # Two separate free blocks bordering unknown; robot near the left one.
    rows = [
        'u u u u u u u u u',
        'u . . u u u . . u',
        'u . . u u u . . u',
        'u u u u u u u u u',
    ]
    data, w, h = _grid(rows)
    near_left = select_frontier_goal(
        data, w, h, 1.0, 0.0, 0.0, robot_xy=(1.5, 1.5),
        min_cluster_cells=2, robot_radius_cells=0,
    )
    near_right = select_frontier_goal(
        data, w, h, 1.0, 0.0, 0.0, robot_xy=(7.5, 1.5),
        min_cluster_cells=2, robot_radius_cells=0,
    )
    assert near_left is not None and near_right is not None
    assert near_left.x < near_right.x  # each picks its own side


def test_degenerate_inputs():
    assert select_frontier_goal([], 0, 0, 1.0, 0.0, 0.0) is None
    assert select_frontier_goal([0, 0], 5, 5, 1.0, 0.0, 0.0) is None  # short data
    assert select_frontier_goal([0], 1, 1, 0.0, 0.0, 0.0) is None  # zero resolution


# ── metric clearance (robot_clearance_m) ────────────────────────────────────
# The clearance pass must reject frontier goals that fall inside Nav2's
# inflation halo. At the real 0.05 m/cell map resolution the legacy 4-cell
# default is only 0.20 m — *smaller* than the 0.25 m inflation_radius — so a
# goal 0.25 m from a wall would be accepted and then rejected by the planner.
# robot_clearance_m fixes that by working in metres regardless of resolution.

# col0 occupied, cols1-5 free, col6 unknown → the only frontier is col5,
# whose Chebyshev distance to the obstacle is 5 cells (= 0.25 m at res 0.05).
_INFLATION_ROWS = [
    '# . . . . . u',
    '# . . . . . u',
    '# . . . . . u',
]


def test_clearance_metres_drops_goal_inside_inflation():
    data, w, h = _grid(_INFLATION_ROWS)
    res = 0.05
    # Legacy 4-cell clearance (0.20 m) keeps the goal — it lands 0.25 m from
    # the wall, *inside* Nav2's 0.25 m inflation (the bug we're fixing).
    kept = select_frontier_goal(
        data, w, h, res, 0.0, 0.0,
        min_cluster_cells=2, robot_radius_cells=4,
    )
    assert kept is not None
    # The metric default (0.35 m → ceil(0.35/0.05) = 7 cells) drops it.
    dropped = select_frontier_goal(
        data, w, h, res, 0.0, 0.0,
        min_cluster_cells=2, robot_clearance_m=0.35,
    )
    assert dropped is None


def test_cell_count_override_takes_precedence_over_metres():
    # An explicit robot_radius_cells overrides robot_clearance_m, so callers
    # can still pin the old cell-count behaviour for tests / tuning.
    data, w, h = _grid(_INFLATION_ROWS)
    goal = select_frontier_goal(
        data, w, h, 0.05, 0.0, 0.0,
        min_cluster_cells=2,
        robot_radius_cells=4,      # 4 cells wins...
        robot_clearance_m=0.35,    # ...even though 0.35 m would drop the goal
    )
    assert goal is not None


def test_metric_clearance_scales_with_resolution():
    # Same 0.35 m clearance, coarser 0.5 m/cell map → ceil(0.35/0.5) = 1 cell,
    # so a frontier two cells from the wall survives. Proves the conversion
    # tracks resolution rather than a fixed cell count.
    data, w, h = _grid(_INFLATION_ROWS)
    goal = select_frontier_goal(
        data, w, h, 0.5, 0.0, 0.0,
        min_cluster_cells=2, robot_clearance_m=0.35,
    )
    assert goal is not None


def test_soft_obstacle_blocks_only_with_lowered_threshold():
    # 'o' = a soft/inflated obstacle (occupancy 50, below the 65 occupied
    # threshold). By default it blocks nothing; clearance_occupied_thresh=50
    # makes the clearance pass treat it as solid.
    cell = {'u': -1, '.': 0, 'o': 50}
    rows = [r.split() for r in _INFLATION_ROWS]  # same layout, swap '#'→'o'
    rows = [['o' if c == '#' else c for c in row] for row in rows]
    h, w = len(rows), len(rows[0])
    data = [cell[c] for row in rows for c in row]
    res = 0.05
    # Default 65 threshold: 50 is not "occupied" → frontier survives.
    kept = select_frontier_goal(
        data, w, h, res, 0.0, 0.0,
        min_cluster_cells=2, robot_clearance_m=0.35,
    )
    assert kept is not None
    # Lowered clearance threshold: 50 now counts as solid → frontier dropped.
    dropped = select_frontier_goal(
        data, w, h, res, 0.0, 0.0,
        min_cluster_cells=2, robot_clearance_m=0.35,
        clearance_occupied_thresh=50,
    )
    assert dropped is None
