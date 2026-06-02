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
