"""Unit tests for :mod:`lupin_mission.approach`.

Pure-Python tests — no rclpy, no fixtures spinning up nodes. Tag/table
fixtures are kept tiny and table-driven so the geometry intent is obvious
from the test names.
"""

from __future__ import annotations

import math
import textwrap

import pytest

from types import SimpleNamespace

from lupin_mission.approach import (
    MAX_STANDOFF_M,
    MIN_STANDOFF_M,
    TagApproach,
    compute_approach,
    compute_discovered_approach,
    load_approach_overrides,
)


def _pose(x, y, *, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_discovered_uses_tag_normal():
    # Quaternion rotating +Z onto +X (90° about Y): tag faces +X, so the
    # robot parks standoff metres east of the tag, yaw pointing back (−X = π).
    pose = _pose(2.0, 0.0, qy=0.70710678, qw=0.70710678)
    a = compute_discovered_approach(
        '7', pose, standoff_m=0.5, fallback_yaw=0.0, robot_xy=(0.0, 0.0),
    )
    assert a.derived_from == 'discovered_normal'
    assert a.goal_x == pytest.approx(2.5, abs=1e-6)
    assert a.goal_y == pytest.approx(0.0, abs=1e-6)
    assert _approx_yaw(a.goal_yaw, math.pi)


def test_discovered_degenerate_normal_falls_back_to_robot_vector():
    # Identity orientation → +Z is vertical → degenerate; approach from the
    # robot's side. Robot at origin, tag at (2,0) → park at (1.5, 0) facing +X.
    pose = _pose(2.0, 0.0)
    a = compute_discovered_approach(
        '7', pose, standoff_m=0.5, fallback_yaw=1.23, robot_xy=(0.0, 0.0),
    )
    assert a.derived_from == 'discovered_robot'
    assert a.goal_x == pytest.approx(1.5, abs=1e-6)
    assert a.goal_y == pytest.approx(0.0, abs=1e-6)
    assert _approx_yaw(a.goal_yaw, 0.0)


def test_discovered_degenerate_no_robot_uses_fallback_yaw():
    pose = _pose(2.0, 0.0)
    a = compute_discovered_approach(
        '7', pose, standoff_m=0.5, fallback_yaw=1.23, robot_xy=None,
    )
    assert a.derived_from == 'fallback'
    assert a.goal_x == pytest.approx(2.0)
    assert a.goal_y == pytest.approx(0.0)
    assert a.goal_yaw == pytest.approx(1.23)


def test_discovered_clamps_standoff():
    pose = _pose(2.0, 0.0, qy=0.70710678, qw=0.70710678)
    a = compute_discovered_approach(
        '7', pose, standoff_m=99.0, fallback_yaw=0.0, robot_xy=(0.0, 0.0),
    )
    assert a.standoff_m == pytest.approx(MAX_STANDOFF_M)


# A single 1m × 0.2m table centred at (1.0, 0.0). Tags placed on each
# cardinal face one metre out from the centre.
ONE_TABLE = {
    'T': {'x0': 0.5, 'x1': 1.5, 'y0': -0.1, 'y1': 0.1},
}
CARDINAL_TAGS = {
    'east':  {'x': 2.0, 'y': 0.0},
    'west':  {'x': 0.0, 'y': 0.0},
    'north': {'x': 1.0, 'y': 1.0},
    'south': {'x': 1.0, 'y': -1.0},
}


def _approx_yaw(actual: float, expected: float) -> bool:
    """Compare yaws modulo 2π so π and -π count as equal."""
    diff = (actual - expected + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) < 1e-6


# ── geometry: cardinal-face placements ─────────────────────────────────────


@pytest.mark.parametrize(
    'tag_id, expected_yaw_deg, expected_goal',
    [
        # East face: outward normal = +X. Goal sits +0.5 m past the tag in +X
        # (i.e. (2.5, 0.0)). Robot yaw faces back at the tag = +π (west).
        ('east',  180.0, (2.5,  0.0)),
        # West face: outward normal = -X. Goal at (-0.5, 0). Yaw = 0 (east).
        ('west',    0.0, (-0.5, 0.0)),
        # North face: outward normal = +Y. Goal at (1.0, 1.5). Yaw = -π/2 (south).
        ('north', -90.0, (1.0,  1.5)),
        # South face: outward normal = -Y. Goal at (1.0, -1.5). Yaw = +π/2 (north).
        ('south',  90.0, (1.0, -1.5)),
    ],
)
def test_cardinal_faces_compute_correct_pose(tag_id, expected_yaw_deg, expected_goal):
    out = compute_approach(
        tag_id, CARDINAL_TAGS, ONE_TABLE,
        standoff_m=0.5, fallback_yaw=0.0,
    )
    assert out.derived_from == 'geometry'
    assert out.table_id == 'T'
    assert out.goal_x == pytest.approx(expected_goal[0], abs=1e-9)
    assert out.goal_y == pytest.approx(expected_goal[1], abs=1e-9)
    assert _approx_yaw(out.goal_yaw, math.radians(expected_yaw_deg))
    assert out.standoff_m == 0.5


def test_geometry_picks_nearest_table_with_lex_tiebreak():
    # Two equidistant tables — tag at the midpoint between them. Lex tiebreak
    # picks "A". We then expect the normal to point from "A"'s centre toward
    # the tag, i.e. +X (since A is at x=-1, tag at x=0).
    tables = {
        'A': {'x0': -1.5, 'x1': -0.5, 'y0': -0.1, 'y1': 0.1},
        'B': {'x0': 0.5, 'x1': 1.5, 'y0': -0.1, 'y1': 0.1},
    }
    tags = {'mid': {'x': 0.0, 'y': 0.0}}
    out = compute_approach(
        'mid', tags, tables, standoff_m=0.4, fallback_yaw=0.0,
    )
    assert out.table_id == 'A'
    assert out.goal_x == pytest.approx(0.4, abs=1e-9)
    assert out.goal_y == pytest.approx(0.0, abs=1e-9)
    assert _approx_yaw(out.goal_yaw, math.pi)  # face -X back at tag


# ── degenerate / fallback paths ────────────────────────────────────────────


def test_no_tables_falls_back_to_global_yaw():
    tags = {'1': {'x': 1.0, 'y': 1.0}}
    out = compute_approach(
        '1', tags, {}, standoff_m=0.5, fallback_yaw=1.234,
    )
    assert out.derived_from == 'fallback'
    assert out.table_id is None
    assert out.goal_x == 1.0  # no offset when geometry can't run
    assert out.goal_y == 1.0
    assert out.goal_yaw == 1.234
    assert out.standoff_m == 0.0


def test_tag_exactly_at_table_centre_falls_back():
    tables = {'T': {'x0': 0.0, 'x1': 2.0, 'y0': 0.0, 'y1': 1.0}}
    tags = {'1': {'x': 1.0, 'y': 0.5}}  # exactly the centre
    out = compute_approach(
        '1', tags, tables, standoff_m=0.5, fallback_yaw=0.7,
    )
    assert out.derived_from == 'fallback'
    assert out.table_id == 'T'
    assert out.goal_yaw == 0.7  # honours the fallback yaw


def test_unknown_tag_id_raises():
    with pytest.raises(KeyError):
        compute_approach(
            'nope', CARDINAL_TAGS, ONE_TABLE, standoff_m=0.5, fallback_yaw=0.0,
        )


# ── overrides ──────────────────────────────────────────────────────────────


def test_partial_override_yaw_only_keeps_geometry_position():
    out = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE,
        standoff_m=0.5, fallback_yaw=0.0,
        overrides={'east': {'yaw': 0.123}},
    )
    assert out.derived_from == 'override'
    assert out.goal_x == pytest.approx(2.5, abs=1e-9)
    assert out.goal_y == pytest.approx(0.0, abs=1e-9)
    assert out.goal_yaw == 0.123


def test_partial_override_standoff_only_changes_offset():
    out = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE,
        standoff_m=0.5, fallback_yaw=0.0,
        overrides={'east': {'standoff': 0.7}},
    )
    assert out.derived_from == 'override'
    assert out.standoff_m == 0.7
    assert out.goal_x == pytest.approx(2.7, abs=1e-9)
    # yaw is geometry-derived (face the tag), unchanged by standoff
    assert _approx_yaw(out.goal_yaw, math.pi)


def test_full_override_skips_geometry_entirely():
    # Position completely off the table-centred normal — geometry-only would
    # never produce this. Full override means we trust the operator.
    out = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE,
        standoff_m=0.5, fallback_yaw=0.0,
        overrides={'east': {'goal_x': 9.0, 'goal_y': 9.0, 'yaw': -1.0, 'standoff': 0.4}},
    )
    assert out.derived_from == 'override'
    assert out.goal_x == 9.0
    assert out.goal_y == 9.0
    assert out.goal_yaw == -1.0
    assert out.standoff_m == 0.4


def test_standoff_clamped_to_safe_range():
    out_lo = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE,
        standoff_m=0.05, fallback_yaw=0.0,
    )
    assert out_lo.standoff_m == MIN_STANDOFF_M
    out_hi = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE,
        standoff_m=10.0, fallback_yaw=0.0,
    )
    assert out_hi.standoff_m == MAX_STANDOFF_M


# ── overrides loader ───────────────────────────────────────────────────────


def test_load_overrides_empty_path_returns_empty_dict():
    assert load_approach_overrides('') == {}


def test_load_overrides_parses_partial_and_full(tmp_path):
    p = tmp_path / 'approach.yaml'
    p.write_text(textwrap.dedent("""
        overrides:
          "12":
            yaw: 1.5708
            standoff: 0.35
          "17":
            goal_x: 4.10
            goal_y: 5.83
            yaw: 3.14
            unknown_key: 99   # silently dropped
    """).strip())
    loaded = load_approach_overrides(str(p))
    assert loaded == {
        '12': {'yaw': 1.5708, 'standoff': 0.35},
        '17': {'goal_x': 4.10, 'goal_y': 5.83, 'yaw': 3.14},
    }


def test_load_overrides_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_approach_overrides(str(tmp_path / 'nope.yaml'))


def test_load_overrides_missing_top_level_key_returns_empty(tmp_path):
    p = tmp_path / 'a.yaml'
    p.write_text('# comment-only file, no overrides key')
    assert load_approach_overrides(str(p)) == {}


def test_load_overrides_wrong_top_type_raises(tmp_path):
    p = tmp_path / 'a.yaml'
    p.write_text('overrides: "not-a-mapping"')
    with pytest.raises(ValueError):
        load_approach_overrides(str(p))


# ── shape sanity ───────────────────────────────────────────────────────────


def test_returned_object_is_immutable():
    out = compute_approach(
        'east', CARDINAL_TAGS, ONE_TABLE, standoff_m=0.5, fallback_yaw=0.0,
    )
    assert isinstance(out, TagApproach)
    with pytest.raises(Exception):
        out.goal_x = 0.0  # type: ignore[misc]


# ── footprint guard ─────────────────────────────────────────────────────────


def test_geometry_goal_never_lands_inside_a_table():
    # Tag near table A's top edge; the naive centre-outward goal at 0.8 m would
    # overshoot to (1.0, 2.6) — inside table B across the aisle. The result must
    # not sit inside any table footprint (standoff pulled back, else fallback).
    tags = {'6': {'x': 1.0, 'y': 1.8}}
    tables = {
        'A': {'x0': 0.0, 'y0': 0.0, 'x1': 2.0, 'y1': 2.0},
        'B': {'x0': 0.0, 'y0': 2.5, 'x1': 2.0, 'y1': 4.5},
    }
    out = compute_approach('6', tags, tables, standoff_m=0.8, fallback_yaw=0.0)
    for tid, t in tables.items():
        inside = (t['x0'] <= out.goal_x <= t['x1']
                  and t['y0'] <= out.goal_y <= t['y1'])
        assert not inside, f'goal ({out.goal_x:.2f},{out.goal_y:.2f}) inside {tid}'
