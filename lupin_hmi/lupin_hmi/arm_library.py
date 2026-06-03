"""arm_library — pure, rclpy-free logic for the arm pose/sequence library.

The node (arm_library_server) is a thin wiring layer over this module so the
storage, validation, capture-gating and replay-building logic stay unit-
testable without spinning ROS (mirrors lupin_twin.idw / lupin_twin.state).

Units: radians throughout. Joint order is arm_limits.ARM_JOINTS
(shoulder_pan, shoulder_lift, elbow, wrist). Gripper is gripper_joint rad.
"""
from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

LIBRARY_VERSION = 1
_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")

# Capture gating + caps (see plan "Shared contracts").
CAPTURE_MIN_DT_S = 0.05
CAPTURE_MIN_DELTA_RAD = math.radians(0.5)
MAX_SEQUENCE_S = 60.0
MAX_WAYPOINTS = 800


class ArmLibraryError(Exception):
    """Raised for user-facing library errors (bad name, exists, not found)."""


def normalize_name(raw: str) -> str:
    return (raw or "").strip().lower()


def validate_name(name: str) -> bool:
    return bool(_NAME_RE.match(name or ""))


@dataclass
class Pose:
    arm: List[float]
    gripper: Optional[float]
    created: str = ""
    note: str = ""


@dataclass
class Waypoint:
    t: float
    arm: List[float]
    gripper: Optional[float]


@dataclass
class Sequence:
    mode: str
    include_gripper: bool
    duration_s: float
    waypoints: List[Waypoint] = field(default_factory=list)
    created: str = ""
    note: str = ""


class ArmLibraryStore:
    """Owns the on-disk JSON. All mutations persist immediately + atomically."""

    def __init__(self, path: os.PathLike | str) -> None:
        self.path = Path(path)
        self._poses: Dict[str, Pose] = {}
        self._sequences: Dict[str, Sequence] = {}

    # ── load / persist ──────────────────────────────────────────────────
    def load(self) -> None:
        self._poses = {}
        self._sequences = {}
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text())
        except (ValueError, OSError):
            self._quarantine()
            return
        for name, p in (data.get("poses") or {}).items():
            self._poses[name] = Pose(
                arm=[float(x) for x in p["arm"]],
                gripper=(None if p.get("gripper") is None else float(p["gripper"])),
                created=p.get("created", ""), note=p.get("note", ""),
            )
        for name, s in (data.get("sequences") or {}).items():
            self._sequences[name] = Sequence(
                mode=s.get("mode", "teleop"),
                include_gripper=bool(s.get("include_gripper", False)),
                duration_s=float(s.get("duration_s", 0.0)),
                waypoints=[
                    Waypoint(t=float(w["t"]), arm=[float(x) for x in w["arm"]],
                             gripper=(None if w.get("gripper") is None else float(w["gripper"])))
                    for w in (s.get("waypoints") or [])
                ],
                created=s.get("created", ""), note=s.get("note", ""),
            )

    def _quarantine(self) -> None:
        n = 0
        while True:
            dest = self.path.with_name(f"{self.path.stem}.corrupt-{n}.json")
            if not dest.exists():
                break
            n += 1
        try:
            self.path.replace(dest)
        except OSError:
            pass

    def _serialize(self) -> dict:
        return {
            "version": LIBRARY_VERSION,
            "poses": {
                n: {"arm": p.arm, "gripper": p.gripper, "created": p.created, "note": p.note}
                for n, p in self._poses.items()
            },
            "sequences": {
                n: {"mode": s.mode, "include_gripper": s.include_gripper,
                    "duration_s": s.duration_s, "created": s.created, "note": s.note,
                    "waypoints": [{"t": w.t, "arm": w.arm, "gripper": w.gripper} for w in s.waypoints]}
                for n, s in self._sequences.items()
            },
        }

    def _atomic_write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            try:
                self.path.replace(self.path.with_suffix(self.path.suffix + ".bak"))
            except OSError:
                pass
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(json.dumps(self._serialize(), indent=2))
        tmp.replace(self.path)

    # ── pose CRUD ───────────────────────────────────────────────────────
    def save_pose(self, name: str, arm: List[float], gripper: Optional[float],
                  *, overwrite: bool, created: str = "") -> None:
        key = normalize_name(name)
        if not validate_name(key):
            raise ArmLibraryError(f'invalid name "{name}" (allowed: a-z 0-9 _ -, max 32)')
        if len(arm) != 4:
            raise ArmLibraryError(f"pose needs 4 arm joints, got {len(arm)}")
        if key in self._poses and not overwrite:
            raise ArmLibraryError(f'pose "{key}" already exists (pass overwrite to replace)')
        self._poses[key] = Pose(arm=[float(x) for x in arm],
                                gripper=(None if gripper is None else float(gripper)),
                                created=created, note=self._poses.get(key, Pose([], None)).note)
        self._atomic_write()

    def get_pose(self, name: str) -> Pose:
        key = normalize_name(name)
        if key not in self._poses:
            raise ArmLibraryError(f'unknown pose "{key}"')
        return self._poses[key]

    # ── sequence CRUD ───────────────────────────────────────────────────
    def save_sequence(self, name: str, seq: Sequence, *, overwrite: bool) -> None:
        key = normalize_name(name)
        if not validate_name(key):
            raise ArmLibraryError(f'invalid name "{name}" (allowed: a-z 0-9 _ -, max 32)')
        if key in self._sequences and not overwrite:
            raise ArmLibraryError(f'sequence "{key}" already exists (pass overwrite to replace)')
        self._sequences[key] = seq
        self._atomic_write()

    def get_sequence(self, name: str) -> Sequence:
        key = normalize_name(name)
        if key not in self._sequences:
            raise ArmLibraryError(f'unknown sequence "{key}"')
        return self._sequences[key]

    # ── delete / rename ─────────────────────────────────────────────────
    def delete(self, kind: str, name: str) -> None:
        d = self._poses if kind == "pose" else self._sequences if kind == "sequence" else None
        if d is None:
            raise ArmLibraryError(f'unknown kind "{kind}" (expected pose|sequence)')
        key = normalize_name(name)
        if key not in d:
            raise ArmLibraryError(f'unknown {kind} "{key}"')
        del d[key]
        self._atomic_write()

    def rename(self, kind: str, name: str, new_name: str) -> None:
        d = self._poses if kind == "pose" else self._sequences if kind == "sequence" else None
        if d is None:
            raise ArmLibraryError(f'unknown kind "{kind}" (expected pose|sequence)')
        old = normalize_name(name)
        new = normalize_name(new_name)
        if old not in d:
            raise ArmLibraryError(f'unknown {kind} "{old}"')
        if not validate_name(new):
            raise ArmLibraryError(f'invalid new name "{new_name}"')
        if new in d:
            raise ArmLibraryError(f'{kind} "{new}" already exists')
        d[new] = d.pop(old)
        self._atomic_write()

    # ── read views ──────────────────────────────────────────────────────
    @property
    def poses(self) -> Dict[str, Pose]:
        return self._poses

    @property
    def sequences(self) -> Dict[str, Sequence]:
        return self._sequences

    def list_metadata(self) -> dict:
        return {
            "poses": [
                {"name": n, "note": p.note, "has_gripper": p.gripper is not None}
                for n, p in sorted(self._poses.items())
            ],
            "sequences": [
                {"name": n, "mode": s.mode, "duration_s": s.duration_s,
                 "n_waypoints": len(s.waypoints), "include_gripper": s.include_gripper}
                for n, s in sorted(self._sequences.items())
            ],
        }
