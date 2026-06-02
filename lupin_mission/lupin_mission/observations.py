"""Helpers for building lupin_msgs/Observation messages.

The orchestrator emits observations from several places (SCANNING,
UNREACHABLE on nav exhaustion, SKIPPED on abort/skip_current). Centralising
the construction here keeps the message shape consistent and the call sites
short.
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header

from lupin_msgs.msg import Observation, TagReading


def make_tag_observation(
    *,
    mission_id: str,
    source: str,
    stamp,                          # builtin_interfaces/Time
    status: int,
    tag_reading: Optional[TagReading] = None,
    status_detail: str = '',
    frame_id: str = 'map',
    amcl_pose: Optional[PoseWithCovarianceStamped] = None,
    tag_map_pose: Optional[Pose] = None,
) -> Observation:
    """Build an Observation with KIND_TAG_READING.

    Used for OK (full reading), UNREACHABLE (no reading), SCAN_FAILED
    (bridge error), and SKIPPED (abort / skip_current). The TagReading
    sub-message is populated only when status==STATUS_OK.

    ``amcl_pose`` is the orchestrator's latest AMCL snapshot at publish
    time. When supplied, its inner ``pose.pose`` is copied into the
    Observation's ``tag_pose_in_map`` so downstream consumers (the digital
    twin in particular) know where the robot was when it scanned the tag.
    Leave None on UNREACHABLE/SKIPPED — the robot's actual pose at that
    point isn't a useful proxy for the tag's position. The default zero
    Pose has ``orientation.w == 0``, which the twin treats as "missing".
    """
    msg = Observation()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    msg.header = header
    msg.mission_id = mission_id
    msg.source = source
    msg.kind = Observation.KIND_TAG_READING
    msg.status = status
    msg.status_detail = status_detail
    if tag_reading is not None:
        msg.tag_reading = tag_reading
    # Prefer the tag's OWN map-frame pose (from the discovered-tags feed, which
    # perception_aggregator resolves via TF) so the operator map pins the tag
    # where it physically is — not where the robot stood. Fall back to the
    # robot's AMCL pose only if the tag pose is unknown.
    pose = tag_map_pose if tag_map_pose is not None else (
        amcl_pose.pose.pose if amcl_pose is not None else None
    )
    if pose is not None:
        msg.tag_pose_in_map = Pose()
        msg.tag_pose_in_map.position = pose.position
        msg.tag_pose_in_map.orientation = pose.orientation
        # The twin treats orientation.w==0 as "no pose" and drops the pin; if
        # the source left the quaternion unset, normalise to identity so a
        # valid position still renders.
        q = msg.tag_pose_in_map.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            q.w = 1.0
    return msg
