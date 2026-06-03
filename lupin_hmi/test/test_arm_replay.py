from __future__ import annotations

import math

import pytest

from lupin_hmi.arm_library import Waypoint, extract_gripper_events
from lupin_hmi.arm_limits import gripper_rad_to_deg, gripper_deg_to_rad
from lupin_hmi.arm_traj import build_arm_trajectory_multi


def test_gripper_rad_to_deg_is_inverse_of_deg_to_rad():
    for deg in (-30.0, -10.0, 0.0, 15.0, 30.0):
        assert gripper_rad_to_deg(gripper_deg_to_rad(deg)) == pytest.approx(deg, abs=1e-6)


def test_build_multi_scales_time_and_keeps_all_joints():
    names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"]
    wps = [(0.0, [0, 0, 0, 0]), (2.0, [0.1, 0.1, 0.1, 0.1])]
    traj = build_arm_trajectory_multi(names, wps, speed=2.0)
    assert traj.joint_names == names
    assert len(traj.points) == 2
    # 2.0 s recorded, replayed at 2x -> 1.0 s for the last point
    last = traj.points[-1].time_from_start
    assert last.sec + last.nanosec / 1e9 == pytest.approx(1.0, abs=1e-3)
    assert len(traj.points[0].positions) == 4


def test_extract_gripper_events_emits_transitions_only():
    wps = [
        Waypoint(0.0, [0, 0, 0, 0], gripper=-0.2),   # open
        Waypoint(1.0, [0, 0, 0, 0], gripper=-0.2),   # same -> no event
        Waypoint(2.0, [0, 0, 0, 0], gripper=0.2),    # close -> event
        Waypoint(3.0, [0, 0, 0, 0], gripper=0.2),    # same -> no event
    ]
    events = extract_gripper_events(wps)
    times = [t for t, _ in events]
    assert times == [0.0, 2.0]
