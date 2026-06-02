#!/usr/bin/env python3
"""Generate a Gazebo SDF world from mdp-greenhouse's tag_locations.json.

The course-supplied ``mdp-greenhouse`` Python package defines the canonical
greenhouse layout: a set of AprilTag positions and table rectangles. The
``lupin_greenhouse_bridge`` already consumes this layout for sensor readings,
so the Gazebo world MUST share the same coordinate frame (metres, origin at
JSON (0,0)) — otherwise nav2, the bridge, and AprilTag detection will all
disagree about where things are.

Rather than hand-author SDF, this script reads ``tag_locations.json`` from
the installed package and emits a deterministic world file. Re-run it after
upstream layout changes to keep the sim in sync.

Tag visuals are placeholder coloured planes — real ``tag36h11`` textures are
the perception teammate's territory (see the TODO inside ``_render_tag``).
What this script DOES guarantee: every tag's *pose* and *integer ID* matches
the JSON, which is what the detector will publish.

Usage::

    python3 generate_greenhouse_world.py \
        --output ../worlds/greenhouse.world

By default the input JSON is resolved from the installed ``greenhouse_sim``
package via ``importlib.resources``; use ``--input`` to override.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from importlib import resources
from pathlib import Path
from typing import Any

# --- Defaults ----------------------------------------------------------------

DEFAULT_TAG_SIZE_M = 0.04          # real demo tags are 4×4 cm
DEFAULT_TAG_THICKNESS_M = 0.005    # plane thickness (so depth camera sees it)
DEFAULT_TAG_HEIGHT_M = 0.10        # on the low planter side, near the rim
# Low planter so the short Mirte arm + gripper camera can actually reach and
# see the blooms (the real pots are ≤15 cm). The "table" box stands in for the
# planter body; flowers rise out of the trough on top of it.
DEFAULT_TABLE_HEIGHT_M = 0.10      # planter body height (was a 0.70 m bench)
DEFAULT_WALL_HEIGHT_M = 2.0
DEFAULT_WALL_THICKNESS_M = 0.10
DEFAULT_WALL_MARGIN_M = 0.50       # padding from outer tag/table extent
DEFAULT_AISLE_EXPAND_Y = 1.0       # >1 widens E-W aisles (see _expand_layout_y)
DEFAULT_WORLD_NAME = "greenhouse"


# --- Layout transforms -------------------------------------------------------


def _expand_layout_y(layout: dict[str, Any], factor: float) -> dict[str, Any]:
    """Scale y-coordinates of tables and tags about the layout's y-midpoint.

    The upstream JSON packs greenhouse rows with ~0.75 m E-W aisles between
    them; with a 0.22 m robot and any sensible Nav2 inflation_radius those
    aisles fully fill with cost and the planner cannot thread them. Pushing
    rows apart in y is the simplest unblock.

    Table SIZES are preserved — only their y-centers move. Tag (x, y) move
    the same way so each tag follows its table. x is untouched (N-S aisles
    are already wide enough). ``factor=1.0`` is identity.
    """
    if factor == 1.0:
        return layout
    if factor <= 0:
        raise ValueError(f"aisle_expand_y must be positive, got {factor}")

    y_min, y_max = _layout_y_bounds(layout)
    y_center = (y_min + y_max) / 2.0

    def scale(y: float) -> float:
        return y_center + (y - y_center) * factor

    out = {**layout}  # shallow copy; rebuild tables/tags below
    out["tables"] = {}
    for name, rect in layout.get("tables", {}).items():
        cy_old = (rect["y0"] + rect["y1"]) / 2.0
        sy = rect["y1"] - rect["y0"]
        cy_new = scale(cy_old)
        out["tables"][name] = {
            "x0": rect["x0"],
            "x1": rect["x1"],
            "y0": cy_new - sy / 2.0,
            "y1": cy_new + sy / 2.0,
        }
    out["tags"] = {}
    for tag_id, tag in layout.get("tags", {}).items():
        out["tags"][tag_id] = {**tag, "y": scale(float(tag["y"]))}
    return out


def _layout_y_bounds(layout: dict[str, Any]) -> tuple[float, float]:
    ys: list[float] = []
    for tag in layout.get("tags", {}).values():
        ys.append(float(tag["y"]))
    for tbl in layout.get("tables", {}).values():
        ys.extend([float(tbl["y0"]), float(tbl["y1"])])
    if not ys:
        raise ValueError("tag_locations.json has neither tags nor tables")
    return min(ys), max(ys)


# --- SDF rendering helpers ---------------------------------------------------


def _render_table(name: str, rect: dict[str, float], height: float) -> str:
    cx = (rect["x0"] + rect["x1"]) / 2.0
    cy = (rect["y0"] + rect["y1"]) / 2.0
    sx = abs(rect["x1"] - rect["x0"])
    sy = abs(rect["y1"] - rect["y0"])
    cz = height / 2.0
    safe = name.replace(" ", "_").lower()
    return f"""    <model name="{safe}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
          <material>
            <ambient>0.45 0.30 0.20 1</ambient>
            <diffuse>0.55 0.38 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _place_on_table_face(
    tx: float,
    ty: float,
    tables: dict[str, dict[str, float]],
    *,
    eps: float = 0.003,
    plate_half_width: float = 0.0,
) -> tuple[float, float, float]:
    """Snap a tag's (tx, ty) onto the nearest vertical face of the nearest table.

    Returns ``(x, y, yaw)`` where ``yaw`` orients the tag's local +X (its
    plate normal direction) outward from the table — so a robot driving
    down the aisle sees the tag face-on.

    The JSON has no per-tag orientation, but the real greenhouse mounts
    tags on the table sides. We pick the closest of the four edges of the
    nearest table, project the tag's coordinate onto that edge, and offset
    the tag plate by ``eps`` so it sits just outside the table's collision
    box rather than intersecting it.

    ``plate_half_width`` is the half-width of the plate along the chosen
    edge (i.e. ``size / 2`` for a square plate). The projection is clamped
    so the *whole* plate stays inside the edge length — without this, a
    tag whose JSON coord falls past the edge endpoint snaps to the corner
    and pokes half its width past it. If the edge is shorter than the
    plate (rare in practice — bench legs are >0.16 m), centre the plate.
    """
    if not tables:
        return tx, ty, 0.0

    def rect_dist(rect: dict[str, float]) -> float:
        dx = max(rect["x0"] - tx, 0.0, tx - rect["x1"])
        dy = max(rect["y0"] - ty, 0.0, ty - rect["y1"])
        return dx * dx + dy * dy

    best = min(tables.values(), key=rect_dist)

    edges = {
        "south": abs(ty - best["y0"]),
        "north": abs(ty - best["y1"]),
        "west": abs(tx - best["x0"]),
        "east": abs(tx - best["x1"]),
    }
    side = min(edges, key=edges.get)

    def clamp_inside(val: float, lo: float, hi: float) -> float:
        # Reserve plate_half_width at each end so the whole plate fits.
        margin_lo = lo + plate_half_width
        margin_hi = hi - plate_half_width
        if margin_lo > margin_hi:
            return (lo + hi) / 2.0
        return max(margin_lo, min(margin_hi, val))

    if side in ("south", "north"):
        x = clamp_inside(tx, best["x0"], best["x1"])
        if side == "south":
            return x, best["y0"] - eps, -math.pi / 2.0  # face -Y
        return x, best["y1"] + eps, math.pi / 2.0       # face +Y
    y = clamp_inside(ty, best["y0"], best["y1"])
    if side == "west":
        return best["x0"] - eps, y, math.pi              # face -X
    return best["x1"] + eps, y, 0.0                      # face +X


def _render_tag(
    tag_id: int,
    x: float,
    y: float,
    yaw: float,
    *,
    size: float,
    thickness: float,
    height: float,
    use_placeholder: bool = False, # toggle to see if stuff works 
) -> str:
    # The plate is a thin box with its thin axis along the model's local +X.
    pose = f"{x:.4f} {y:.4f} {height:.4f} 0 0 {yaw:.6f}"

    if use_placeholder:
        # ORIGINAL MAGENTA PLACEHOLDER (Safe and sound)
        return f"""    <model name="apriltag_{tag_id}_PLACEHOLDER">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{thickness:.4f} {size:.4f} {size:.4f}</size></box></geometry>
        </collision>
        <visual name="TAG_{tag_id}_PLACEHOLDER_NEEDS_TEXTURE">
          <geometry><box><size>{thickness:.4f} {size:.4f} {size:.4f}</size></box></geometry>
          <material>
            <ambient>1.0 0.0 1.0 1</ambient>
            <diffuse>1.0 0.0 1.0 1</diffuse>
            <emissive>1.0 0.0 1.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""
    
    else:
        # NEW REAL APRILTAG TEXTURE
        return f"""    <model name="apriltag_{tag_id}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{thickness:.4f} {size:.4f} {size:.4f}</size></box></geometry>
        </collision>
        
        <!-- White background plate to give the tag a border -->
        <visual name="bg_visual">
          <geometry><box><size>{thickness:.4f} {size:.4f} {size:.4f}</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>

        <!-- The actual AprilTag texture, mapped to a plane on the +X face -->
        <visual name="tag_visual">
          <!-- Shifted slightly forward on X to prevent Z-fighting, rotated to face outward -->
          <pose>{thickness/2 + 0.001} 0 0 0 1.57079 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>{size*0.9:.4f} {size*0.9:.4f}</size></plane></geometry>
          <material>
            <script>
              <uri>model://apriltags/materials/scripts</uri>
              <uri>model://apriltags/materials/textures</uri>
              <name>AprilTag/Tag36h11_{tag_id}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _render_wall(name: str, cx: float, cy: float, sx: float, sy: float, sz: float) -> str:
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {sz / 2.0:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.90 0.90 0.90 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _bounds(layout: dict[str, Any]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for tag in layout.get("tags", {}).values():
        xs.append(float(tag["x"]))
        ys.append(float(tag["y"]))
    for tbl in layout.get("tables", {}).values():
        xs.extend([float(tbl["x0"]), float(tbl["x1"])])
        ys.extend([float(tbl["y0"]), float(tbl["y1"])])
    if not xs or not ys:
        raise ValueError("tag_locations.json has neither tags nor tables")
    return min(xs), max(xs), min(ys), max(ys)


# --- Top-level ---------------------------------------------------------------


# Flower blossom palette keyed by species (matches FlowerObservation.species +
# the sim HSV detector class ids 0/1/2). The physical demo uses dahlias in three
# colours, so we reproduce THOSE, not literal RGB primaries:
#   'red'   -> vivid MAGENTA / hot-pink  (HSV: pink-magenta hue, HIGH saturation)
#   'white' -> white                     (HSV: ~zero saturation, high value)
#   'pink'  -> pale / light pink         (HSV: pink hue, LOW saturation, high V)
# 'red' and 'pink' share a hue family (as the real magenta vs pale-pink dahlias
# do), so the detector separates them by SATURATION — magenta high, pale-pink
# low — and 'white' by ~zero saturation. All emissive so the colour the gripper
# camera sees stays stable under Gazebo lighting. Re-tune in lockstep with the
# sim flower detector's HSV bands.
_FLOWER_COLORS: dict[str, tuple] = {
    # species: (ambient_rgb, diffuse_rgb, emissive_rgb). Emissive is kept
    # CHROMATIC (low green) for red/pink so it doesn't wash the hue toward
    # white. Magenta sits at very high saturation, pale-pink at a clear mid
    # saturation, white at ~zero — a wide gap so the HSV detector separates
    # them cleanly even after Gazebo lighting desaturates a little.
    'red':   ((0.32, 0.02, 0.15), (0.86, 0.05, 0.40), (0.40, 0.00, 0.18)),  # magenta, very high S
    'white': ((0.42, 0.42, 0.42), (0.95, 0.95, 0.95), (0.40, 0.40, 0.40)),  # white, ~zero S
    'pink':  ((0.40, 0.26, 0.31), (0.93, 0.62, 0.74), (0.18, 0.06, 0.11)),  # pale pink, mid S
}
# Dominant blossom colour per table cycles through this order; accent blossoms
# of the other two colours fill the trough so each station is a CLUSTER of mixed
# flowers with more blossoms than tags — matching the hardware (no 1:1 tag↔flower
# mapping). Perception attributes the DOMINANT colour seen while scanning a tag.
_FLOWER_SPECIES_CYCLE = ('red', 'white', 'pink')


def _render_trough(name: str, rect: dict[str, float], table_height: float,
                   *, wall_h: float = 0.03, inset: float = 0.05) -> str:
    """A shallow soil-brown planter box on the table top — the flower trough.

    Stands in for the laser-cut wooden planter; the blossoms rise out of it.
    """
    cx = (rect["x0"] + rect["x1"]) / 2.0
    cy = (rect["y0"] + rect["y1"]) / 2.0
    sx = max(abs(rect["x1"] - rect["x0"]) - 2 * inset, 0.05)
    sy = max(abs(rect["y1"] - rect["y0"]) - 2 * inset, 0.05)
    cz = table_height + wall_h / 2.0
    safe = name.replace(" ", "_").lower()
    return f"""    <model name="trough_{safe}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {wall_h:.4f}</size></box></geometry>
          <material>
            <ambient>0.20 0.12 0.05 1</ambient>
            <diffuse>0.30 0.18 0.08 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _render_flower(idx: int, fx: float, fy: float, top_z: float, species: str,
                   *, stem_height: float = 0.05, blossom_radius: float = 0.03) -> str:
    """One flower: green stem + coloured emissive blossom, rising from the trough."""
    amb, dif, emi = _FLOWER_COLORS[species]
    stem_cz = top_z + stem_height / 2.0
    blossom_cz = top_z + stem_height + blossom_radius * 0.55
    return f"""    <model name="flower_{idx}_{species}">
      <static>true</static>
      <pose>{fx:.4f} {fy:.4f} 0 0 0 0</pose>
      <link name="link">
        <visual name="stem">
          <pose>0 0 {stem_cz:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>0.009</radius><length>{stem_height:.4f}</length></cylinder></geometry>
          <material><ambient>0.05 0.40 0.05 1</ambient><diffuse>0.05 0.52 0.05 1</diffuse></material>
        </visual>
        <visual name="blossom">
          <pose>0 0 {blossom_cz:.4f} 0 0 0</pose>
          <geometry><sphere><radius>{blossom_radius:.4f}</radius></sphere></geometry>
          <material>
            <ambient>{amb[0]:.2f} {amb[1]:.2f} {amb[2]:.2f} 1</ambient>
            <diffuse>{dif[0]:.2f} {dif[1]:.2f} {dif[2]:.2f} 1</diffuse>
            <emissive>{emi[0]:.2f} {emi[1]:.2f} {emi[2]:.2f} 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def _render_bug(idx: int, fx: float, fy: float, top_z: float, *, plate: float = 0.05) -> str:
    """A small black 'bug' anomaly marker among the flowers (the pest target,
    like the printed spider tags in the real planter). Detected as the dark
    low-value 'bug' class so the aggregator raises the anomaly flag."""
    cz = top_z + 0.015
    return f"""    <model name="bug_{idx}">
      <static>true</static>
      <pose>{fx:.4f} {fy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{plate:.4f} {plate:.4f} 0.008</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _render_table_planting(
    table_index: int, rect: dict[str, float], table_height: float,
    *, flower_start_idx: int, with_bug: bool,
) -> tuple[list[str], int]:
    """Trough + a cluster of mixed-colour flowers (+ optional bug) for one table.

    Lays a row of blossoms along the table's longer axis: a DOMINANT colour
    (cycling red/white/pink by table) with periodic accents of the other two,
    so the station reads as one colour to perception while still being a mixed
    cluster. Returns (sdf_blocks, next_flower_idx).
    """
    blocks = [_render_trough(f"table_{table_index}", rect, table_height)]
    top_z = table_height + 0.03  # trough top
    dom = _FLOWER_SPECIES_CYCLE[table_index % len(_FLOWER_SPECIES_CYCLE)]
    accents = [s for s in _FLOWER_SPECIES_CYCLE if s != dom]

    x0, x1, y0, y1 = rect["x0"], rect["x1"], rect["y0"], rect["y1"]
    span_x, span_y = abs(x1 - x0), abs(y1 - y0)
    along_x = span_x >= span_y
    length = max(span_x, span_y)
    n = max(3, min(8, int(length / 0.16)))
    margin = 0.07

    idx = flower_start_idx
    for k in range(n):
        t = (k + 0.5) / n
        if along_x:
            fx = x0 + margin + t * (span_x - 2 * margin)
            fy = (y0 + y1) / 2.0
        else:
            fy = y0 + margin + t * (span_y - 2 * margin)
            fx = (x0 + x1) / 2.0
        # ~1 in 3 blossoms is an accent colour; the rest are the dominant.
        species = accents[k % len(accents)] if (k % 3 == 1) else dom
        blocks.append(_render_flower(idx, fx, fy, top_z, species))
        idx += 1

    if with_bug:
        if along_x:
            bx, by = (x0 + x1) / 2.0, (y0 + y1) / 2.0 + 0.045
        else:
            bx, by = (x0 + x1) / 2.0 + 0.045, (y0 + y1) / 2.0
        blocks.append(_render_bug(idx, bx, by, top_z))
        idx += 1

    return blocks, idx


def build_world(
    layout: dict[str, Any],
    *,
    world_name: str = DEFAULT_WORLD_NAME,
    tag_size: float = DEFAULT_TAG_SIZE_M,
    tag_thickness: float = DEFAULT_TAG_THICKNESS_M,
    tag_height: float = DEFAULT_TAG_HEIGHT_M,
    table_height: float = DEFAULT_TABLE_HEIGHT_M,
    wall_height: float = DEFAULT_WALL_HEIGHT_M,
    wall_thickness: float = DEFAULT_WALL_THICKNESS_M,
    wall_margin: float = DEFAULT_WALL_MARGIN_M,
) -> str:
    units = layout.get("units", "m")
    if units != "m":
        raise ValueError(f"expected units=='m' in tag_locations.json, got {units!r}")

    tags: dict[str, Any] = layout.get("tags", {})
    tables: dict[str, Any] = layout.get("tables", {})

    x_min, x_max, y_min, y_max = _bounds(layout)
    wx0 = x_min - wall_margin
    wx1 = x_max + wall_margin
    wy0 = y_min - wall_margin
    wy1 = y_max + wall_margin
    width = wx1 - wx0
    depth = wy1 - wy0
    cx = (wx0 + wx1) / 2.0
    cy = (wy0 + wy1) / 2.0

    table_blocks = [
        _render_table(name, rect, table_height)
        for name, rect in sorted(tables.items())
    ]
    # Sort tag IDs as integers so the SDF is deterministic and human-scannable.
    tag_blocks = []
    for tag_id, tag in sorted(tags.items(), key=lambda kv: int(kv[0])):
        tx, ty = float(tag["x"]), float(tag["y"])
        sx, sy, yaw = _place_on_table_face(
            tx, ty, tables,
            eps=tag_thickness,
            plate_half_width=tag_size / 2.0,
        )
        tag_blocks.append(
            _render_tag(
                int(tag_id), sx, sy, yaw,
                size=tag_size,
                thickness=tag_thickness,
                height=tag_height,
            )
        )

    # Planting: one trough + a mixed-colour flower cluster per table (the
    # gripper-cam flower-scan targets), with a bug anomaly marker on every
    # 4th table. More blossoms than tags — no 1:1 mapping, like the hardware.
    flower_blocks = []
    _flower_idx = 0
    for ti, (tname, rect) in enumerate(sorted(tables.items())):
        blocks, _flower_idx = _render_table_planting(
            ti, rect, table_height,
            flower_start_idx=_flower_idx,
            with_bug=(ti % 4 == 3),
        )
        flower_blocks.extend(blocks)
    walls = [
        _render_wall("wall_south", cx, wy0, width, wall_thickness, wall_height),
        _render_wall("wall_north", cx, wy1, width, wall_thickness, wall_height),
        _render_wall("wall_west", wx0, cy, wall_thickness, depth, wall_height),
        _render_wall("wall_east", wx1, cy, wall_thickness, depth, wall_height),
    ]

    body = "\n".join(walls + table_blocks + flower_blocks + tag_blocks)
    return f"""<?xml version="1.0" ?>
<!--
  Generated by lupin_bringup/scripts/generate_greenhouse_world.py from
  mdp-greenhouse's tag_locations.json. DO NOT EDIT BY HAND — re-run the
  script if the JSON changes.

  Tag IDs follow the JSON (integers). Tag visuals are PLACEHOLDER bright
  magenta planes (self-lit, model name suffix _PLACEHOLDER) — they exist
  to give the AprilTag detector something physically present at the
  right pose. They will NOT be detected as AprilTags. The perception
  teammate must replace them with real tag36h11 textures before the
  detector can produce real detections. See the TODO in the generator.
-->
<sdf version="1.6">
  <world name="{world_name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

{body}
  </world>
</sdf>
"""


def _resolve_input(arg: str | None) -> Path:
    if arg:
        return Path(arg).expanduser().resolve()
    try:
        # importlib.resources.files returns a Traversable; cast to Path.
        with resources.as_file(
            resources.files("greenhouse_sim").joinpath("configs/tag_locations.json")
        ) as p:
            return Path(p).resolve()
    except (ModuleNotFoundError, FileNotFoundError) as err:
        raise SystemExit(
            "Could not locate greenhouse_sim/configs/tag_locations.json. "
            "Either install mdp-greenhouse (`pip install mdp-greenhouse`) "
            "or pass --input pointing at the JSON.\n"
            f"underlying error: {err}"
        )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--input",
        help="Path to tag_locations.json (default: from installed greenhouse_sim).",
    )
    p.add_argument(
        "--output",
        default=str(Path(__file__).resolve().parent.parent / "worlds" / "greenhouse.world"),
        help="Where to write the SDF world (default: ../worlds/greenhouse.world).",
    )
    p.add_argument("--world-name", default=DEFAULT_WORLD_NAME)
    p.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M)
    p.add_argument("--tag-height", type=float, default=DEFAULT_TAG_HEIGHT_M)
    p.add_argument("--table-height", type=float, default=DEFAULT_TABLE_HEIGHT_M)
    p.add_argument(
        "--aisle-expand-y",
        type=float,
        default=DEFAULT_AISLE_EXPAND_Y,
        help="Scale y-coords of tables and tags about the layout y-midpoint. "
             ">1 widens E-W aisles. Table sizes and x-coords are preserved.",
    )
    p.add_argument(
        "--wall-margin",
        type=float,
        default=DEFAULT_WALL_MARGIN_M,
        help="Padding from outermost tag/table to the surrounding walls (m). "
             "Bigger values widen the perimeter aisle (e.g. between rows of "
             "tables and the walls). Default 0.50.",
    )
    p.add_argument(
        "--write-layout-json",
        help="If set, also write the (possibly transformed) layout JSON here. "
             "Bridge + orchestrator should be pointed at this file when "
             "--aisle-expand-y != 1 so their tag coords match the world.",
    )
    args = p.parse_args(argv)

    src = _resolve_input(args.input)
    layout = json.loads(src.read_text())
    layout = _expand_layout_y(layout, args.aisle_expand_y)
    sdf = build_world(
        layout,
        world_name=args.world_name,
        tag_size=args.tag_size,
        tag_height=args.tag_height,
        table_height=args.table_height,
        wall_margin=args.wall_margin,
    )

    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)

    if args.write_layout_json:
        layout_out = Path(args.write_layout_json).expanduser().resolve()
        layout_out.parent.mkdir(parents=True, exist_ok=True)
        layout_out.write_text(json.dumps(layout, indent=2, sort_keys=True))
        print(f"wrote {layout_out}", file=sys.stderr)

    n_tags = len(layout.get("tags", {}))
    n_tables = len(layout.get("tables", {}))
    x_min, x_max, y_min, y_max = _bounds(layout)
    print(
        f"wrote {out} ({n_tags} tags, {n_tables} tables, "
        f"bounds x=[{x_min:.2f},{x_max:.2f}] y=[{y_min:.2f},{y_max:.2f}])",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
