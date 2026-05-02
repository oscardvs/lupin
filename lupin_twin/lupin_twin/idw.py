"""Inverse-distance-weighted (IDW) sensor-field reconstruction.

Pure Python — no ROS imports. The twin node calls into this module to
build the heat-map response for ``/twin/get_field``; the HMI's mock
generator can call into it too if we ever want a sim-side preview that
isn't a robot.

The algorithm is deliberately simple. Each cell ``c`` evaluates to::

    field(c) = sum_i (w_i * v_i) / sum_i w_i

where the sum is over observed tags within ``falloff_radius_m``, and
``w_i = 1 / max(d_i, eps) ** power``. ``d_i`` is the Euclidean distance
from the cell to tag ``i``; cells closer than ``eps`` (default 1 mm)
are pinned to the nearest tag's value to avoid weight blow-up.

Cells whose nearest tag is farther than ``max_distance_to_nearest_tag``
are encoded as NaN. That's what keeps the field honest: only paint
where we have data. The HMI renders NaN as transparent so unexplored
greenhouse stays the colour of the SLAM map underneath.

Why not Kriging / GP regression? Overkill for the sample sizes we
have (≤ 30 tags) and the visual-feedback latency we want (sub-100 ms
for a 200×200 grid). IDW is a lousy interpolator for *prediction*
work but a good *visualisation* — and visualisation is the brief.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


# Default tunings — match what the twin node declares as ROS params, so
# unit tests against this module reflect the defaults the operator sees.
DEFAULT_POWER = 2.0
# Falloff and max-distance are deliberately equal at the defaults: a cell
# whose nearest tag is in (falloff, max_distance] would otherwise pass the
# honesty gate but find no neighbours within falloff, falling through to
# the nearest-value branch and producing a hard "constant ring" between
# falloff and max_distance — visually a donut. Operators can still
# configure them differently via launch params if they want that behaviour
# explicitly; the default avoids the donut.
DEFAULT_FALLOFF_RADIUS_M = 1.5
DEFAULT_MAX_DISTANCE_M = 1.5
# Cells closer than this to a tag get pinned to that tag's value rather
# than risking a 1/0 in the weighted sum.
COINCIDENT_EPS_M = 1e-3


@dataclass(frozen=True)
class FieldSample:
    """One per-tag reading the field is reconstructed from."""
    x: float
    y: float
    value: float


@dataclass(frozen=True)
class FieldGrid:
    """Result of an IDW evaluation. Cells with no nearby data are NaN."""
    width: int
    height: int
    origin_x: float
    origin_y: float
    resolution: float
    values: list  # row-major width*height; floats with NaN for missing
    value_min: float
    value_max: float
    sample_count: int


def grid_dimensions(
    bbox_min_x: float, bbox_min_y: float,
    bbox_max_x: float, bbox_max_y: float,
    resolution: float,
) -> tuple[int, int]:
    """Compute (width, height) cells from a bbox + resolution.

    Returns at least 1×1 for degenerate inputs — caller decides whether
    that's an error worth surfacing or just a tiny preview.
    """
    if resolution <= 0:
        raise ValueError(f'resolution must be positive, got {resolution}')
    if bbox_max_x < bbox_min_x or bbox_max_y < bbox_min_y:
        raise ValueError(
            f'bbox max must be ≥ min: '
            f'x=[{bbox_min_x},{bbox_max_x}], y=[{bbox_min_y},{bbox_max_y}]'
        )
    w = max(1, int(math.ceil((bbox_max_x - bbox_min_x) / resolution)))
    h = max(1, int(math.ceil((bbox_max_y - bbox_min_y) / resolution)))
    return w, h


def compute_idw_field(
    samples: Sequence[FieldSample],
    *,
    bbox_min_x: float,
    bbox_min_y: float,
    bbox_max_x: float,
    bbox_max_y: float,
    resolution: float,
    falloff_radius_m: float = DEFAULT_FALLOFF_RADIUS_M,
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
    power: float = DEFAULT_POWER,
    explored_mask: Optional[Iterable[bool]] = None,
) -> FieldGrid:
    """Render an IDW field over the bbox.

    Parameters
    ----------
    samples:
        Per-tag observations. Empty → an all-NaN grid (with min=max=0).
    bbox_*:
        Map-frame extent.
    resolution:
        Cell size in metres.
    falloff_radius_m:
        Tags farther than this from a cell don't contribute. Caps how
        much "smearing" any one observation does — without it a single
        tag would paint the whole grid.
    max_distance_m:
        If the *nearest* tag is farther than this, the cell is NaN.
        Different from ``falloff_radius_m``: falloff caps neighbour
        contribution; max-distance is the honesty gate that prevents
        painting in regions with no nearby data at all. Defaults make
        sense when ``max_distance_m >= falloff_radius_m``; the function
        doesn't enforce that.
    power:
        IDW exponent. p=2 is the textbook default and gives a soft,
        visually pleasant gradient; p=1 is too smooth, p=4+ becomes
        Voronoi-like. Stay in [1, 4] in practice.
    explored_mask:
        Optional row-major bool iterable, width*height. False cells are
        NaN regardless of sample distance — used to clip to SLAM-explored
        area. None means "treat the whole bbox as explored".

    Returns
    -------
    FieldGrid with row-major NaN-aware values + value_min/value_max
    over the non-NaN cells.
    """
    width, height = grid_dimensions(
        bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, resolution,
    )
    n_cells = width * height
    values: list[float] = [math.nan] * n_cells

    sample_list = list(samples)
    if not sample_list:
        return FieldGrid(
            width=width, height=height,
            origin_x=bbox_min_x, origin_y=bbox_min_y,
            resolution=resolution,
            values=values,
            value_min=0.0, value_max=0.0,
            sample_count=0,
        )

    if explored_mask is not None:
        explored = list(explored_mask)
        if len(explored) != n_cells:
            raise ValueError(
                f'explored_mask length {len(explored)} != width*height {n_cells}'
            )
    else:
        explored = None

    inv_p = float(power)
    falloff_sq = falloff_radius_m * falloff_radius_m
    max_dist_sq = max_distance_m * max_distance_m

    val_min = math.inf
    val_max = -math.inf

    for j in range(height):
        cy = bbox_min_y + (j + 0.5) * resolution
        for i in range(width):
            idx = j * width + i
            if explored is not None and not explored[idx]:
                continue  # leave NaN
            cx = bbox_min_x + (i + 0.5) * resolution

            # Find nearest sample first — gates max-distance and short-
            # circuits the coincident case. O(N) per cell; samples ≤ 30
            # so a KD-tree isn't worth it.
            nearest_d_sq = math.inf
            nearest_value = 0.0
            for s in sample_list:
                dx = cx - s.x
                dy = cy - s.y
                d_sq = dx * dx + dy * dy
                if d_sq < nearest_d_sq:
                    nearest_d_sq = d_sq
                    nearest_value = s.value
            if nearest_d_sq > max_dist_sq:
                continue  # leave NaN — too far from any data

            # Coincident-ish: pin to nearest sample's value to avoid
            # 1/0 weights and to keep the heat-map exactly right at the
            # tag locations themselves (operators expect this).
            if nearest_d_sq < COINCIDENT_EPS_M * COINCIDENT_EPS_M:
                values[idx] = nearest_value
                if nearest_value < val_min: val_min = nearest_value
                if nearest_value > val_max: val_max = nearest_value
                continue

            # Weighted sum over samples within falloff_radius.
            num = 0.0
            den = 0.0
            for s in sample_list:
                dx = cx - s.x
                dy = cy - s.y
                d_sq = dx * dx + dy * dy
                if d_sq > falloff_sq:
                    continue
                d = math.sqrt(d_sq)
                w = 1.0 / (d ** inv_p)
                num += w * s.value
                den += w
            if den <= 0:
                # No neighbours within falloff — fall back to nearest
                # rather than NaN, since max_distance already passed.
                # Avoids hard "donut" rings around isolated samples.
                v = nearest_value
            else:
                v = num / den
            values[idx] = v
            if v < val_min: val_min = v
            if v > val_max: val_max = v

    if not math.isfinite(val_min):
        val_min = 0.0
        val_max = 0.0

    return FieldGrid(
        width=width, height=height,
        origin_x=bbox_min_x, origin_y=bbox_min_y,
        resolution=resolution,
        values=values,
        value_min=val_min, value_max=val_max,
        sample_count=len(sample_list),
    )
