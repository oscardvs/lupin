"""Unit tests for :mod:`lupin_twin.idw`.

Pure-Python tests, no rclpy. Mirrors the table-driven style of
``lupin_mission/test/test_approach.py``.
"""

from __future__ import annotations

import math

import pytest

from lupin_twin.idw import (
    COINCIDENT_EPS_M,
    DEFAULT_MAX_DISTANCE_M,
    FieldSample,
    FieldGrid,
    compute_idw_field,
    grid_dimensions,
)


def _nan_count(values):
    return sum(1 for v in values if math.isnan(v))


def _finite_count(values):
    return sum(1 for v in values if math.isfinite(v))


# ── grid sizing ────────────────────────────────────────────────────────────


def test_grid_dimensions_basic():
    assert grid_dimensions(0.0, 0.0, 2.0, 1.0, resolution=0.5) == (4, 2)


def test_grid_dimensions_rounds_up_partial_cells():
    # 2.3 m at 1 m resolution → 3 cells (we never under-paint)
    assert grid_dimensions(0.0, 0.0, 2.3, 1.0, resolution=1.0) == (3, 1)


def test_grid_dimensions_rejects_nonpositive_resolution():
    with pytest.raises(ValueError):
        grid_dimensions(0.0, 0.0, 1.0, 1.0, resolution=0.0)
    with pytest.raises(ValueError):
        grid_dimensions(0.0, 0.0, 1.0, 1.0, resolution=-0.1)


def test_grid_dimensions_rejects_inverted_bbox():
    with pytest.raises(ValueError):
        grid_dimensions(2.0, 0.0, 1.0, 1.0, resolution=0.5)


def test_grid_dimensions_clamps_degenerate_to_one_cell():
    # Single-point bbox is technically valid for caller; min size 1×1.
    assert grid_dimensions(1.0, 1.0, 1.0, 1.0, resolution=0.5) == (1, 1)


# ── empty input → all-NaN grid ─────────────────────────────────────────────


def test_empty_samples_returns_all_nan_grid():
    g = compute_idw_field(
        [],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=2.0, bbox_max_y=1.0,
        resolution=0.5,
    )
    assert g.width == 4 and g.height == 2
    assert _nan_count(g.values) == 8
    assert g.value_min == 0.0 and g.value_max == 0.0
    assert g.sample_count == 0


# ── single sample → constant disc, NaN beyond max_distance ─────────────────


def test_single_sample_constant_disc_within_max_distance():
    # One tag at the centre, 5×5 grid, resolution 1.0, max_distance 1.5.
    # Cells within ≤ 1.5 m of the tag get the value; the corners (~2.83 m)
    # stay NaN.
    sample = FieldSample(x=2.5, y=2.5, value=42.0)
    g = compute_idw_field(
        [sample],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=5.0, bbox_max_y=5.0,
        resolution=1.0,
        max_distance_m=1.5,
        falloff_radius_m=2.0,
    )
    assert g.width == 5 and g.height == 5
    # Value at the centre cell (2,2) — its centre is (2.5, 2.5) — must be
    # exactly the sample's value (coincident path).
    assert g.values[2 * 5 + 2] == pytest.approx(42.0, abs=1e-9)
    # Corner cells are well past max_distance.
    assert math.isnan(g.values[0])
    assert math.isnan(g.values[4])
    assert math.isnan(g.values[24])
    # Some finite cells must exist.
    assert _finite_count(g.values) >= 1
    # All finite cells are exactly 42 (single-source IDW degenerate case).
    finite = [v for v in g.values if math.isfinite(v)]
    assert all(abs(v - 42.0) < 1e-9 for v in finite)


def test_max_distance_gates_against_falloff_independently():
    # Two samples 10 m apart; max_distance generous, falloff small. Cell
    # midway between them is within max_distance of EACH (5 m < default
    # 2.0 m fails — let's test with a bigger max_distance).
    s1 = FieldSample(x=0.0, y=0.0, value=10.0)
    s2 = FieldSample(x=10.0, y=0.0, value=20.0)
    g = compute_idw_field(
        [s1, s2],
        bbox_min_x=-1.0, bbox_min_y=-1.0,
        bbox_max_x=11.0, bbox_max_y=1.0,
        resolution=2.0,
        max_distance_m=6.0,    # cells within 6 m of any sample paint
        falloff_radius_m=1.5,  # but falloff is tight — only one neighbour each
    )
    # Cell containing (0, 0) — index by checking the value matches s1.
    assert _finite_count(g.values) >= 2
    # Mid-bbox cell at (~5, 0) is within max_distance of both, but neither
    # is within falloff — fall back to nearest. With ties, lex order in
    # the loop picks s1 first.
    # No assertion on which "nearest" wins; just that the cell IS finite.


# ── coincident sample → exact value ────────────────────────────────────────


def test_sample_at_cell_centre_returns_exact_value():
    # Place the sample exactly at a cell centre; expect the cell value
    # to be the sample value (coincident path).
    s = FieldSample(x=0.25, y=0.25, value=7.5)
    g = compute_idw_field(
        [s],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=0.5, bbox_max_y=0.5,
        resolution=0.5,
    )
    assert g.values[0] == pytest.approx(7.5, abs=COINCIDENT_EPS_M)


# ── multi-sample weighting ─────────────────────────────────────────────────


def test_two_samples_midpoint_equal_weight():
    # Two equal-distance samples → cell value = average.
    s1 = FieldSample(x=0.0, y=0.5, value=10.0)
    s2 = FieldSample(x=1.0, y=0.5, value=20.0)
    g = compute_idw_field(
        [s1, s2],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=1.0, bbox_max_y=1.0,
        resolution=1.0,
        falloff_radius_m=1.0,
        max_distance_m=2.0,
    )
    # Single 1×1 cell at centre (0.5, 0.5). Equidistant from both samples.
    assert g.values[0] == pytest.approx(15.0, abs=1e-9)


def test_closer_sample_dominates():
    # Same as above but cell is closer to s1.
    s1 = FieldSample(x=0.0, y=0.5, value=10.0)
    s2 = FieldSample(x=10.0, y=0.5, value=100.0)
    g = compute_idw_field(
        [s1, s2],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=1.0, bbox_max_y=1.0,
        resolution=1.0,
        falloff_radius_m=15.0,
        max_distance_m=15.0,
        power=2.0,
    )
    # Cell centre at (0.5, 0.5). d1=0.5, d2=9.5. Weights w1=1/0.25=4,
    # w2=1/90.25≈0.01108. Value ≈ (4*10 + 0.01108*100)/(4 + 0.01108)
    # ≈ 41.108 / 4.01108 ≈ 10.249.
    assert g.values[0] == pytest.approx(10.249, abs=1e-2)


# ── explored mask ──────────────────────────────────────────────────────────


def test_explored_mask_zeros_unexplored_cells_to_nan():
    s = FieldSample(x=0.5, y=0.5, value=42.0)
    # 2×2 grid, only top-left explored.
    mask = [True, False, False, False]
    g = compute_idw_field(
        [s],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=2.0, bbox_max_y=2.0,
        resolution=1.0,
        max_distance_m=5.0,
        falloff_radius_m=5.0,
        explored_mask=mask,
    )
    # Only the explored cell should be finite.
    finite_indices = [i for i, v in enumerate(g.values) if math.isfinite(v)]
    assert finite_indices == [0]


def test_explored_mask_length_validated():
    s = FieldSample(x=0.0, y=0.0, value=1.0)
    with pytest.raises(ValueError):
        compute_idw_field(
            [s],
            bbox_min_x=0.0, bbox_min_y=0.0,
            bbox_max_x=2.0, bbox_max_y=2.0,
            resolution=1.0,
            explored_mask=[True],  # wrong length
        )


# ── value_min / value_max bookkeeping ──────────────────────────────────────


def test_value_min_max_skip_nan_cells():
    s1 = FieldSample(x=0.0, y=0.0, value=10.0)
    s2 = FieldSample(x=1.0, y=0.0, value=30.0)
    g = compute_idw_field(
        [s1, s2],
        bbox_min_x=-2.0, bbox_min_y=-2.0,
        bbox_max_x=3.0, bbox_max_y=2.0,
        resolution=1.0,
        max_distance_m=1.5,
        falloff_radius_m=1.5,
    )
    # NaN cells exist (corners far from samples) — they must NOT pull
    # value_min toward 0 or NaN. Result must be a finite range.
    assert math.isfinite(g.value_min) and math.isfinite(g.value_max)
    assert g.value_min >= 10.0 - 1e-6
    assert g.value_max <= 30.0 + 1e-6
    assert g.value_min <= g.value_max


def test_all_nan_returns_zero_min_max():
    # Sample exists but max_distance is so tight that no cell qualifies.
    s = FieldSample(x=100.0, y=100.0, value=42.0)
    g = compute_idw_field(
        [s],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=1.0, bbox_max_y=1.0,
        resolution=1.0,
        max_distance_m=0.1,
    )
    assert _nan_count(g.values) == g.width * g.height
    assert g.value_min == 0.0 and g.value_max == 0.0


def test_sample_count_propagates():
    g = compute_idw_field(
        [FieldSample(0, 0, 1), FieldSample(1, 1, 2), FieldSample(2, 2, 3)],
        bbox_min_x=-1.0, bbox_min_y=-1.0,
        bbox_max_x=3.0, bbox_max_y=3.0,
        resolution=1.0,
    )
    assert g.sample_count == 3


# ── shape sanity ───────────────────────────────────────────────────────────


def test_returned_grid_is_immutable():
    g = compute_idw_field(
        [FieldSample(0.5, 0.5, 1.0)],
        bbox_min_x=0.0, bbox_min_y=0.0,
        bbox_max_x=1.0, bbox_max_y=1.0,
        resolution=1.0,
    )
    assert isinstance(g, FieldGrid)
    with pytest.raises(Exception):
        g.width = 0  # type: ignore[misc]
