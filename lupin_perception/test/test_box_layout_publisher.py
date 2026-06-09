"""Node tests for ``box_layout_publisher``, the known-layout producer that feeds
the aggregator's ``/perception/box_geometry_json`` contract.

Covers the producer in isolation (a real bench rectangle per discovered tag) and
the full graft end-to-end: the producer's JSON parsed by the *existing*
``perception_aggregator`` consumer yields the real Table1 rectangle, not the
fallback 0.80×0.40 box.
"""

import json

import pytest
import rclpy
from geometry_msgs.msg import Pose

from lupin_msgs.msg import DiscoveredTag, DiscoveredTags
from lupin_perception.box_geometry import MapBox, box_from_map_box
from lupin_perception.box_layout_publisher import BoxLayoutPublisher
from lupin_perception.perception_aggregator import PerceptionAggregator

# Tags 6 and 29 sit on Table1 = {x0:1.5, y0:7.3, x1:2.6, y1:7.55} in the bundled
# greenhouse_sim layout the node auto-loads.
_TABLE1 = {(1.5, 7.3), (2.6, 7.3), (2.6, 7.55), (1.5, 7.55)}
_TAG_JSON = {'6': (1.5, 7.3), '29': (2.4, 7.3)}


@pytest.fixture
def pub():
    rclpy.init()
    n = BoxLayoutPublisher()
    n.published = []
    n._box_pub.publish = n.published.append  # type: ignore[method-assign]
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _pose(x, y):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.w = 1.0
    return p


def _discovered(*ids_xy, sightings=3):
    msg = DiscoveredTags()
    for tid, (x, y) in ids_xy:
        t = DiscoveredTag()
        t.tag_id = tid
        t.pose_in_map = _pose(x, y)
        t.sightings = sightings
        msg.tags.append(t)
    return msg


def _boxes_by_id(node):
    assert node.published, 'expected a box_geometry_json publication'
    return {b['id']: b for b in json.loads(node.published[-1].data)}


def _footprint_of(box):
    mb = MapBox(box_id=box['id'], x=box['x'], y=box['y'], yaw=box['yaw'],
                width=box['width'], depth=box['depth'], height=box['height'])
    return {(round(x, 4), round(y, 4)) for x, y in box_from_map_box(mb).footprint()}


def test_publishes_real_rectangle_per_tag(pub):
    # Map frame == JSON frame: each tag localises to its JSON (x, y).
    pub._on_discovered(_discovered(('6', _TAG_JSON['6']), ('29', _TAG_JSON['29'])))
    pub._publish_boxes()
    boxes = _boxes_by_id(pub)
    assert {'6', '29'} <= set(boxes), 'a box keyed by each scanned tag id'
    assert _footprint_of(boxes['6']) == _TABLE1


def test_below_sighting_threshold_is_ignored(pub):
    pub._on_discovered(_discovered(('6', _TAG_JSON['6']), sightings=1))
    pub._publish_boxes()
    assert pub.published == []


def test_no_tags_no_publication(pub):
    pub._publish_boxes()
    assert pub.published == []


def test_graft_end_to_end_aggregator_consumes_producer(pub):
    # The producer's JSON, fed to the EXISTING aggregator consumer, must yield
    # the real Table1 rectangle (not the fallback box_from_tag 0.80×0.40).
    pub._on_discovered(_discovered(('6', _TAG_JSON['6']), ('29', _TAG_JSON['29'])))
    pub._publish_boxes()
    payload = pub.published[-1]

    agg = PerceptionAggregator()
    try:
        agg._on_box_geometry(payload)
        geom = agg._box_geometry_for('6', None)
        assert geom is not None, 'aggregator should build geometry from the box'
        footprint = {(round(x, 4), round(y, 4)) for x, y in geom.footprint()}
        assert footprint == _TABLE1
    finally:
        agg.destroy_node()
