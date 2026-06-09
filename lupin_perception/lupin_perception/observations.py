"""Build lupin_msgs/Observation messages for passive (no-mission) perception.

Mirrors the KIND_TAG_READING shape lupin_mission.observations.make_tag_observation
emits, so the digital twin treats passive and mission readings identically. Kept
local to lupin_perception to avoid depending on lupin_mission (wrong direction);
extract to a shared util only if a third consumer appears.
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from lupin_msgs.msg import Observation, TagReading


def make_tag_reading_observation(
    *,
    source: str,
    stamp,                              # builtin_interfaces/Time
    tag_reading: TagReading,
    tag_map_pose: Optional[Pose] = None,
    frame_id: str = 'map',
    mission_id: str = '',
) -> Observation:
    """A STATUS_OK KIND_TAG_READING Observation for /floranova/observations.

    Pins the tag at its own map pose (from the discovered-tags feed). The twin
    treats orientation.w == 0 as "no pose", so normalise an all-zero quaternion
    to identity when a position is present.
    """
    msg = Observation()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.mission_id = mission_id
    msg.source = source
    msg.kind = Observation.KIND_TAG_READING
    msg.status = Observation.STATUS_OK
    msg.tag_reading = tag_reading
    if tag_map_pose is not None:
        pose = Pose()
        pose.position = tag_map_pose.position
        pose.orientation = tag_map_pose.orientation
        q = pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            q.w = 1.0
        msg.tag_pose_in_map = pose
    return msg
