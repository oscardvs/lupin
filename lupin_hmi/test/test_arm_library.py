"""Unit tests for :mod:`lupin_hmi.arm_library`. Pure-Python, no rclpy.
Mirrors the table-driven style of lupin_twin/test/test_idw.py.
"""
from __future__ import annotations

import json

import pytest

from lupin_hmi.arm_library import (
    ArmLibraryError,
    ArmLibraryStore,
    normalize_name,
    validate_name,
)


@pytest.mark.parametrize("raw,expected", [
    ("  Home ", "home"),
    ("Inspect_Left", "inspect_left"),
    ("PICK-2", "pick-2"),
])
def test_normalize_name(raw, expected):
    assert normalize_name(raw) == expected


@pytest.mark.parametrize("name,ok", [
    ("home", True), ("inspect_left", True), ("p1", True), ("a" * 32, True),
    ("", False), ("a" * 33, False), ("_x", False), ("has space", False), ("UP", False),
])
def test_validate_name(name, ok):
    assert validate_name(name) is ok


def test_load_missing_file_is_empty(tmp_path):
    store = ArmLibraryStore(tmp_path / "lib.json")
    store.load()
    assert store.list_metadata() == {"poses": [], "sequences": []}


def test_atomic_write_roundtrip(tmp_path):
    path = tmp_path / "lib.json"
    store = ArmLibraryStore(path)
    store.load()
    store.save_pose("home", [0.0, -0.1, 0.2, -1.5], gripper=-0.2, overwrite=False)
    # A fresh store sees the persisted data.
    store2 = ArmLibraryStore(path)
    store2.load()
    pose = store2.get_pose("home")
    assert pose.arm == [0.0, -0.1, 0.2, -1.5]
    assert pose.gripper == -0.2
    on_disk = json.loads(path.read_text())
    assert on_disk["version"] == 1


def test_corrupt_file_is_quarantined(tmp_path):
    path = tmp_path / "lib.json"
    path.write_text("{ this is not json")
    store = ArmLibraryStore(path)
    store.load()  # must not raise
    assert store.list_metadata() == {"poses": [], "sequences": []}
    quarantined = list(tmp_path.glob("lib.corrupt-*.json"))
    assert len(quarantined) == 1


def test_overwrite_guard(tmp_path):
    store = ArmLibraryStore(tmp_path / "lib.json")
    store.load()
    store.save_pose("home", [0, 0, 0, 0], gripper=None, overwrite=False)
    with pytest.raises(ArmLibraryError):
        store.save_pose("home", [1, 1, 1, 1], gripper=None, overwrite=False)
    store.save_pose("home", [1, 1, 1, 1], gripper=None, overwrite=True)
    assert store.get_pose("home").arm == [1, 1, 1, 1]


def test_invalid_name_rejected(tmp_path):
    store = ArmLibraryStore(tmp_path / "lib.json")
    store.load()
    with pytest.raises(ArmLibraryError):
        store.save_pose("bad name", [0, 0, 0, 0], gripper=None, overwrite=False)


def test_delete_and_rename(tmp_path):
    store = ArmLibraryStore(tmp_path / "lib.json")
    store.load()
    store.save_pose("a", [0, 0, 0, 0], gripper=None, overwrite=False)
    store.rename("pose", "a", "b")
    assert "b" in store.poses and "a" not in store.poses
    store.delete("pose", "b")
    assert store.list_metadata()["poses"] == []
    with pytest.raises(ArmLibraryError):
        store.delete("pose", "missing")
