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

DEFAULT_TAG_SIZE_M = 0.16          # square apriltag edge length
DEFAULT_TAG_THICKNESS_M = 0.005    # plane thickness (so depth camera sees it)
DEFAULT_TAG_HEIGHT_M = 0.40        # mid-height on the table side, below table top
DEFAULT_TABLE_HEIGHT_M = 0.70      # typical greenhouse bench top height
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
) -> str:
    # TODO(perception): replace this magenta placeholder with a real tag36h11
    # texture per ID once the perception teammate provides the asset pack.
    # The placeholder is INTENTIONALLY ugly (bright self-lit magenta) so it's
    # immediately obvious to anyone running an AprilTag detector against this
    # world that the visuals are stand-ins — without real textures the
    # detector will see nothing, which would otherwise be a silent footgun.
    #
    # The plate is a thin box with its thin axis along the model's local +X.
    # Yaw rotates the model so that local +X points outward from the nearest
    # table face — the tag's visible face is what the camera sees.
    pose = f"{x:.4f} {y:.4f} {height:.4f} 0 0 {yaw:.6f}"
    # Integer ID embedded in name: AprilTag detector will publish detections
    # using these IDs, and "apriltag_<int>" is parseable downstream.
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
    walls = [
        _render_wall("wall_south", cx, wy0, width, wall_thickness, wall_height),
        _render_wall("wall_north", cx, wy1, width, wall_thickness, wall_height),
        _render_wall("wall_west", wx0, cy, wall_thickness, depth, wall_height),
        _render_wall("wall_east", wx1, cy, wall_thickness, depth, wall_height),
    ]

    body = "\n".join(walls + table_blocks + tag_blocks)
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
