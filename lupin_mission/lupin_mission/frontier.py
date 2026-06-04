"""Frontier selection for autonomous exploration.

Pure Python (no rclpy / numpy) so it unit-tests against synthetic grids and
the orchestrator can call it directly. Reads an occupancy grid (the live
slam_toolbox ``/map``), finds the boundary between explored-free and unknown
space, and returns the map-frame ``(x, y)`` of the best frontier to drive to.
The orchestrator turns that into a NavigateToPose goal on the existing nav
client — Nav2's planner already has ``allow_unknown: true`` and the global
costmap ``track_unknown_space: true``, so this is the only missing piece.

Occupancy-grid convention (nav_msgs/OccupancyGrid.data, row-major):
    -1        unknown
    0..100    free→occupied probability
Cell (col, row) maps to world via
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence

UNKNOWN = -1


@dataclass(frozen=True)
class FrontierGoal:
    x: float
    y: float
    cluster_size: int
    num_clusters: int


def select_frontier_goal(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    *,
    robot_xy: Optional[tuple[float, float]] = None,
    free_thresh: int = 20,
    occupied_thresh: int = 65,
    min_cluster_cells: int = 6,
    robot_radius_cells: Optional[int] = None,
    robot_clearance_m: float = 0.35,
    clearance_occupied_thresh: Optional[int] = None,
) -> Optional[FrontierGoal]:
    """Pick the best frontier to explore next, or None if there are none.

    A *frontier cell* is FREE (``0 <= v <= free_thresh``) and 4-adjacent to at
    least one UNKNOWN cell. Frontier cells too close to an OCCUPIED cell are
    dropped (the robot wouldn't fit). The clearance is ``robot_clearance_m``
    metres, converted to cells via ``ceil(m / resolution)`` so it stays correct
    regardless of map resolution — keep it >= Nav2's ``inflation_radius`` or
    goals land in the inflation halo and the planner rejects them. Pass
    ``robot_radius_cells`` to override with a raw cell count (e.g. in tests).
    ``clearance_occupied_thresh`` optionally lowers the occupancy treated as
    solid *for the clearance pass only*, so soft inflated cells (below
    ``occupied_thresh``) also push frontiers away. Remaining cells are
    flood-filled into clusters (8-connectivity); clusters smaller than
    ``min_cluster_cells`` are discarded as SLAM noise. Each surviving cluster
    is scored ``size / (1 + distance_to_robot_m)`` (size only when
    ``robot_xy`` is None), and the cell nearest the winning cluster's centroid
    — guaranteed free and adjacent to unknown — is returned in map coords.
    """
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    if len(data) < width * height:
        return None

    def idx(c: int, r: int) -> int:
        return r * width + c

    def is_free(v: int) -> bool:
        return 0 <= v <= free_thresh

    # Pass 1: frontier mask (free + 4-neighbour unknown).
    frontier = bytearray(width * height)
    for r in range(height):
        for c in range(width):
            v = data[idx(c, r)]
            if not is_free(v):
                continue
            adj_unknown = (
                (c > 0 and data[idx(c - 1, r)] == UNKNOWN)
                or (c < width - 1 and data[idx(c + 1, r)] == UNKNOWN)
                or (r > 0 and data[idx(c, r - 1)] == UNKNOWN)
                or (r < height - 1 and data[idx(c, r + 1)] == UNKNOWN)
            )
            if adj_unknown:
                frontier[idx(c, r)] = 1

    if not any(frontier):
        return None

    # Pass 2: clearance — drop frontier cells too close to an obstacle. Work in
    # metres (converted to cells via the resolution) so the clearance tracks
    # Nav2's inflation_radius regardless of map resolution; an explicit
    # robot_radius_cells overrides with a raw cell count.
    if robot_radius_cells is not None:
        rad = max(0, int(robot_radius_cells))
    else:
        rad = max(0, math.ceil(robot_clearance_m / resolution))
    clear_occ = (
        occupied_thresh if clearance_occupied_thresh is None
        else clearance_occupied_thresh
    )
    if rad > 0:
        safe = bytearray(frontier)
        for r in range(height):
            for c in range(width):
                if not frontier[idx(c, r)]:
                    continue
                blocked = False
                for dr in range(-rad, rad + 1):
                    rr = r + dr
                    if rr < 0 or rr >= height:
                        continue
                    for dc in range(-rad, rad + 1):
                        cc = c + dc
                        if cc < 0 or cc >= width:
                            continue
                        if data[idx(cc, rr)] >= clear_occ:
                            blocked = True
                            break
                    if blocked:
                        break
                if blocked:
                    safe[idx(c, r)] = 0
        frontier = safe
        if not any(frontier):
            return None

    # Pass 3: cluster (8-connectivity flood fill).
    visited = bytearray(width * height)
    clusters: list[list[tuple[int, int]]] = []
    for r in range(height):
        for c in range(width):
            if not frontier[idx(c, r)] or visited[idx(c, r)]:
                continue
            cells: list[tuple[int, int]] = []
            q = deque([(c, r)])
            visited[idx(c, r)] = 1
            while q:
                cc, rr = q.popleft()
                cells.append((cc, rr))
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        if dc == 0 and dr == 0:
                            continue
                        nc, nr = cc + dc, rr + dr
                        if 0 <= nc < width and 0 <= nr < height:
                            j = idx(nc, nr)
                            if frontier[j] and not visited[j]:
                                visited[j] = 1
                                q.append((nc, nr))
            if len(cells) >= min_cluster_cells:
                clusters.append(cells)

    if not clusters:
        return None

    def cell_to_world(c: int, r: int) -> tuple[float, float]:
        return (origin_x + (c + 0.5) * resolution,
                origin_y + (r + 0.5) * resolution)

    # Pass 4: score + pick. Centroid → nearest member cell (so the returned
    # point is a real free, unknown-adjacent cell, not an averaged hole).
    best_score = -math.inf
    best: Optional[FrontierGoal] = None
    for cells in clusters:
        mean_c = sum(c for c, _ in cells) / len(cells)
        mean_r = sum(r for _, r in cells) / len(cells)
        # cell closest to the centroid
        cc, cr = min(cells, key=lambda cr_: (cr_[0] - mean_c) ** 2 + (cr_[1] - mean_r) ** 2)
        wx, wy = cell_to_world(cc, cr)
        if robot_xy is not None:
            dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            score = len(cells) / (1.0 + dist)
        else:
            score = float(len(cells))
        if score > best_score:
            best_score = score
            best = FrontierGoal(x=wx, y=wy, cluster_size=len(cells),
                                num_clusters=len(clusters))
    return best
