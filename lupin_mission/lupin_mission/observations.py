"""Helpers for building lupin_msgs/Observation messages.

The orchestrator emits observations from several places (SCANNING,
UNREACHABLE on nav exhaustion, SKIPPED on abort/skip_current). Centralising
the construction here keeps the message shape consistent and the call sites
short.
"""

from __future__ import annotations

from typing import Optional

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
) -> Observation:
    """Build an Observation with KIND_TAG_READING.

    Used for OK (full reading), UNREACHABLE (no reading), SCAN_FAILED
    (bridge error), and SKIPPED (abort / skip_current). The TagReading
    sub-message is populated only when status==STATUS_OK.
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
    return msg
