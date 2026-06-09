"""Publish per-tag planter-box geometry from the KNOWN greenhouse layout.

The ``perception_aggregator`` consumes ``/perception/box_geometry_json`` (one
:class:`~lupin_perception.box_geometry.MapBox` per tag) and otherwise falls back
to a tag-anchored ``StandardBox``. This node is the **producer**: it loads the
course ``tag_locations.json`` (the real bench rectangles), registers that layout
onto the live ``map`` frame from the discovered-tag constellation
(``solve_rigid_2d`` — positions only, robust unlike a single noisy tag
quaternion), and publishes one ``MapBox`` per discovered tag: that tag's own
bench rectangle, in map frame.

An OpenCV/occupancy-grid node could publish the same contract; this producer is
*exact* because the greenhouse layout is known, and drawing the full rectangle
fills the cells the lidar never sees behind the near face. Keeping the geometry
out of the aggregator preserves the aggregator's role as a pure consumer.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from lupin_msgs.msg import DiscoveredTags

from .box_geometry import (
    GridView,
    front_face_occupied_points,
    map_box_from_rect,
    nearest_table_rect,
    snap_front_face,
    solve_rigid_2d,
    table_rect_corners,
)


class BoxLayoutPublisher(Node):
    """Register the known layout to the map and publish per-tag MapBoxes."""

    def __init__(self) -> None:
        super().__init__('box_layout_publisher')

        self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')
        self.declare_parameter('box_geometry_topic', '/perception/box_geometry_json')
        # Real planter rectangles + tag (x, y). Empty -> bundled greenhouse_sim.
        self.declare_parameter('tag_locations_file', '')
        # Match the aggregator's discovery threshold so a tag's box appears in
        # lockstep with its discovery.
        self.declare_parameter('min_sightings', 3)
        self.declare_parameter('box_height', 0.30)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('snap_enable', True)
        self.declare_parameter('snap_occupied_threshold', 65)
        self.declare_parameter('snap_band_m', 0.12)
        self.declare_parameter('snap_min_points', 8)
        self.declare_parameter('snap_min_elongation', 3.0)
        self.declare_parameter('snap_max_yaw_deg', 12.0)
        self.declare_parameter('snap_max_shift_m', 0.15)

        self._snap_enable = bool(self.get_parameter('snap_enable').value)
        self._snap_threshold = int(self.get_parameter('snap_occupied_threshold').value)
        self._snap_band_m = float(self.get_parameter('snap_band_m').value)
        self._snap_min_points = int(self.get_parameter('snap_min_points').value)
        self._snap_min_elongation = float(self.get_parameter('snap_min_elongation').value)
        self._snap_max_yaw_deg = float(self.get_parameter('snap_max_yaw_deg').value)
        self._snap_max_shift_m = float(self.get_parameter('snap_max_shift_m').value)
        self._latest_grid: OccupancyGrid | None = None

        self._min_sightings = int(self.get_parameter('min_sightings').value)
        self._box_height = float(self.get_parameter('box_height').value)
        # Latest map-frame (x, y) per discovered tag, from the discovery feed.
        self._tag_map_xy: dict[str, tuple[float, float]] = {}
        self._load_layout(str(self.get_parameter('tag_locations_file').value))

        # Latched, like /perception/discovered_tags and /twin/state, so a late
        # aggregator subscriber gets the current boxes immediately.
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._box_pub = self.create_publisher(
            String, str(self.get_parameter('box_geometry_topic').value), latched,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._on_map, latched,
        )
        self.create_subscription(
            DiscoveredTags,
            str(self.get_parameter('discovered_tags_topic').value),
            self._on_discovered, latched,
        )
        rate = max(0.2, float(self.get_parameter('publish_rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._publish_boxes)

        self.get_logger().info(
            f'box_layout_publisher online — {len(self._tables)} tables, '
            f'{len(self._tag_json_xy)} tag coords; registering the known layout '
            'onto the map from discovered tags.'
        )

    # ─── greenhouse layout ──────────────────────────────────────────────
    def _load_layout(self, path: str) -> None:
        """Load planter rectangles + tag (x, y) from tag_locations.json.

        ``path`` (the committed snapshot the demo launches pass) wins; otherwise
        the bundled ``greenhouse_sim`` package is used. Failure is non-fatal: we
        log once and publish nothing (the aggregator keeps its legacy fallback).
        """
        self._tables: dict = {}
        self._tag_json_xy: dict[str, tuple[float, float]] = {}
        try:
            if path:
                data = json.loads(Path(path).expanduser().read_text())
            else:
                from importlib.resources import files
                cfg = files('greenhouse_sim').joinpath('configs/tag_locations.json')
                data = json.loads(cfg.read_text())
        except Exception as exc:  # noqa: BLE001 — any load failure → no boxes
            self.get_logger().warn(
                f'planter-box layout unavailable ({exc}); publishing no boxes, '
                'the aggregator keeps its tag-anchored fallback.'
            )
            return
        self._tables = data.get('tables', {}) or {}
        for tid, t in (data.get('tags', {}) or {}).items():
            try:
                self._tag_json_xy[str(tid)] = (float(t['x']), float(t['y']))
            except (KeyError, TypeError, ValueError):
                continue

    # ─── discovery feed ─────────────────────────────────────────────────
    def _on_discovered(self, msg: DiscoveredTags) -> None:
        for tag in msg.tags:
            if int(tag.sightings) < self._min_sightings:
                continue
            p = tag.pose_in_map.position
            self._tag_map_xy[str(tag.tag_id)] = (float(p.x), float(p.y))

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_grid = msg

    def _snap_box(self, mb):
        g = self._latest_grid
        if g is None:
            return mb
        grid = GridView(
            width=g.info.width, height=g.info.height, resolution=g.info.resolution,
            origin_x=g.info.origin.position.x, origin_y=g.info.origin.position.y,
            data=g.data,
        )
        pts = front_face_occupied_points(
            mb, grid, band_m=self._snap_band_m, threshold=self._snap_threshold,
        )
        return snap_front_face(
            mb, pts,
            min_points=self._snap_min_points,
            min_elongation=self._snap_min_elongation,
            max_yaw_rad=math.radians(self._snap_max_yaw_deg),
            max_shift_m=self._snap_max_shift_m,
        )

    def _layout_transform(self):
        """Rigid JSON→map transform from the discovered-tag constellation.

        Identity until ≥ 2 tags with JSON coords are seen (correct where the map
        is already layout-aligned, e.g. sim); the orientation comes from many
        tag positions, not one quaternion.
        """
        src: list[tuple[float, float]] = []
        dst: list[tuple[float, float]] = []
        for tid, mxy in self._tag_map_xy.items():
            j = self._tag_json_xy.get(tid)
            if j is None:
                continue
            src.append(j)
            dst.append(mxy)
        return solve_rigid_2d(src, dst)

    # ─── publish ────────────────────────────────────────────────────────
    def _publish_boxes(self) -> None:
        if not self._tables or not self._tag_map_xy:
            return
        tf = self._layout_transform()
        boxes: list[dict] = []
        for tid, mxy in self._tag_map_xy.items():
            j = self._tag_json_xy.get(tid)
            if j is None:
                continue
            rect = nearest_table_rect(j, self._tables)
            if rect is None:
                continue
            corners = [tf.apply(x, y) for (x, y) in table_rect_corners(rect)]
            mb = map_box_from_rect(tid, corners, toward=mxy, height=self._box_height)
            if mb is None:
                continue
            if self._snap_enable:
                mb = self._snap_box(mb)
            boxes.append({
                'id': mb.box_id, 'x': mb.x, 'y': mb.y, 'yaw': mb.yaw,
                'width': mb.width, 'depth': mb.depth, 'height': mb.height,
            })
        if boxes:
            self._box_pub.publish(String(data=json.dumps(boxes)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxLayoutPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
