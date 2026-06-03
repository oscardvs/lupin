"""Unit tests for :mod:`lupin_twin.state` — pure-Python store of per-tag
observations behind the twin node."""

from __future__ import annotations

import pytest

from lupin_twin.state import (
    FlowerUpdate,
    TagSensorEntry,
    TwinFlower,
    TwinObservation,
    TwinStateStore,
)


def _obs(tag_id, t, *, pose=None, readings=()):
    return TwinObservation(
        tag_id=tag_id,
        monotonic_at=t,
        pose_x=pose[0] if pose else None,
        pose_y=pose[1] if pose else None,
        pose_qz=pose[2] if pose and len(pose) >= 3 else None,
        pose_qw=pose[3] if pose and len(pose) >= 4 else None,
        readings=[TagSensorEntry(name=n, value=v) for n, v in readings],
    )


def test_record_returns_true_for_first_observation():
    store = TwinStateStore()
    ok = store.record(_obs('1', 0.0, pose=(1.0, 2.0), readings=[('temperature', 22.5)]))
    assert ok is True
    assert store.observation_count == 1
    buf = store.tag('1')
    assert buf is not None
    assert buf.has_pose()
    assert buf.pose_x == 1.0 and buf.pose_y == 2.0
    assert buf.latest_readings == {'temperature': 22.5}


def test_pose_caches_from_first_observation_does_not_update():
    # Tag's first OK observation pins its pose; later observations update
    # readings but NOT pose, so the HMI marker doesn't jitter on AMCL drift.
    store = TwinStateStore()
    store.record(_obs('1', 0.0, pose=(1.0, 2.0), readings=[('temperature', 22.0)]))
    store.record(_obs('1', 1.0, pose=(1.05, 2.05), readings=[('temperature', 22.5)]))
    buf = store.tag('1')
    assert buf.pose_x == 1.0
    assert buf.pose_y == 2.0
    assert buf.latest_readings == {'temperature': 22.5}  # value updated


def test_observation_count_increments_per_record():
    store = TwinStateStore()
    for i in range(5):
        store.record(_obs('1', float(i), pose=(0, 0), readings=[('co2', 400.0 + i)]))
    assert store.observation_count == 5


def test_record_rejects_empty_tag_id():
    store = TwinStateStore()
    ok = store.record(_obs('', 0.0, pose=(0, 0)))
    assert ok is False
    assert store.observation_count == 0


def test_history_is_capped_at_buffer_len():
    store = TwinStateStore(buffer_len=4)
    for i in range(10):
        store.record(_obs('1', float(i), pose=(0, 0), readings=[('co2', float(i))]))
    buf = store.tag('1')
    assert len(buf.history) == 4
    # Newest preserved.
    assert buf.history[-1].readings[0].value == 9.0


def test_pose_only_set_when_first_pose_supplied():
    # First observation has no pose (e.g. SCAN_FAILED before AMCL gates) —
    # tag is still recorded but has no pose. A later observation WITH a
    # pose then pins it.
    store = TwinStateStore()
    store.record(_obs('1', 0.0, pose=None, readings=[('temperature', 22.0)]))
    assert store.tag('1').has_pose() is False
    store.record(_obs('1', 1.0, pose=(3.0, 4.0), readings=[('temperature', 23.0)]))
    assert store.tag('1').has_pose() is True
    assert store.tag('1').pose_x == 3.0


def test_samples_for_sensor_skips_pose_less_and_missing_sensor():
    store = TwinStateStore()
    # Tag with pose + temperature
    store.record(_obs('1', 0.0, pose=(1, 1), readings=[('temperature', 20.0)]))
    # Tag with pose but only humidity
    store.record(_obs('2', 0.0, pose=(2, 2), readings=[('humidity', 55.0)]))
    # Tag with no pose, has temperature
    store.record(_obs('3', 0.0, pose=None, readings=[('temperature', 19.0)]))

    samples = store.samples_for_sensor('temperature')
    assert samples == [(1, 1, 20.0)]


def test_samples_for_sensor_skips_nan_and_inf_readings():
    # Bridge stub returning NaN/inf must not poison the IDW field — the
    # store filters non-finite values before the IDW math sees them.
    store = TwinStateStore()
    store.record(_obs('1', 0.0, pose=(1, 1), readings=[('temperature', 20.0)]))
    store.record(_obs('2', 0.0, pose=(2, 2), readings=[('temperature', float('nan'))]))
    store.record(_obs('3', 0.0, pose=(3, 3), readings=[('temperature', float('inf'))]))
    store.record(_obs('4', 0.0, pose=(4, 4), readings=[('temperature', float('-inf'))]))
    samples = store.samples_for_sensor('temperature')
    assert samples == [(1, 1, 20.0)]


def test_tag_ids_preserve_insertion_order():
    store = TwinStateStore()
    for tid in ['a', 'b', 'c']:
        store.record(_obs(tid, 0.0, pose=(0, 0), readings=[('temperature', 1.0)]))
    assert store.tag_ids() == ['a', 'b', 'c']


# ── flower ingestion (KIND_FLOWER) ─────────────────────────────────────────

def _flower(tag_id, t, *, species='', conf=0.0, anomaly=False, pose=None):
    from lupin_twin.state import FlowerUpdate
    return FlowerUpdate(
        tag_id=tag_id,
        monotonic_at=t,
        species=species,
        species_confidence=conf,
        anomaly=anomaly,
        pose_x=pose[0] if pose else None,
        pose_y=pose[1] if pose else None,
        pose_qz=pose[2] if pose and len(pose) >= 3 else None,
        pose_qw=pose[3] if pose and len(pose) >= 4 else None,
    )


def test_record_flower_merges_species_onto_tag():
    store = TwinStateStore()
    store.record(_obs('5', 0.0, pose=(1.0, 2.0), readings=[('temperature', 21.0)]))
    assert store.record_flower(
        _flower('5', 1.0, species='tulip_red', conf=0.9)
    ) is True
    buf = store.tag('5')
    assert buf.species == 'tulip_red'
    assert buf.species_confidence == pytest.approx(0.9)
    assert buf.anomaly is False
    # Sensor readings untouched by the flower update.
    assert buf.latest_readings['temperature'] == pytest.approx(21.0)


def test_record_flower_pins_unpinned_tag():
    store = TwinStateStore()
    # Flower arrives before any sensor reading → it should pin the tag.
    store.record_flower(_flower('7', 0.0, species='tulip_pink', conf=0.8,
                                pose=(3.0, 4.0, 0.0, 1.0)))
    buf = store.tag('7')
    assert buf.has_pose()
    assert (buf.pose_x, buf.pose_y) == (3.0, 4.0)
    assert buf.species == 'tulip_pink'


def test_record_flower_latest_wins_and_clears_anomaly():
    store = TwinStateStore()
    store.record_flower(_flower('1', 0.0, species='tulip_white', conf=0.7, anomaly=True))
    assert store.tag('1').anomaly is True
    # A clean re-scan clears the anomaly (latest-wins).
    store.record_flower(_flower('1', 1.0, species='tulip_white', conf=0.7, anomaly=False))
    assert store.tag('1').anomaly is False


def test_record_flower_empty_tag_rejected():
    store = TwinStateStore()
    assert store.record_flower(_flower('', 0.0, species='tulip_red')) is False


def test_record_flower_stores_flowers_and_footprint():
    store = TwinStateStore()
    upd = FlowerUpdate(
        tag_id='1', monotonic_at=0.0, species='tulip_red',
        species_confidence=0.9, anomaly=False,
        flowers=[TwinFlower(x=1.8, y=0.1, species='tulip_red',
                            confidence=0.9, anomaly=False)],
        box_footprint=[(2.0, 0.4), (2.0, -0.4), (1.6, -0.4), (1.6, 0.4)],
    )
    assert store.record_flower(upd) is True
    buf = store.tag('1')
    assert len(buf.flowers) == 1
    assert buf.flowers[0].species == 'tulip_red'
    assert buf.box_footprint == [(2.0, 0.4), (2.0, -0.4), (1.6, -0.4), (1.6, 0.4)]


def test_record_flower_latest_wins_for_flowers():
    store = TwinStateStore()
    store.record_flower(FlowerUpdate(
        tag_id='1', monotonic_at=0.0, species='tulip_red',
        species_confidence=0.9, anomaly=False,
        flowers=[TwinFlower(1.8, 0.1, 'tulip_red', 0.9, False),
                 TwinFlower(1.8, -0.1, 'tulip_red', 0.8, False)],
        box_footprint=[(2.0, 0.4)],
    ))
    store.record_flower(FlowerUpdate(
        tag_id='1', monotonic_at=1.0, species='tulip_white',
        species_confidence=0.7, anomaly=False,
        flowers=[TwinFlower(1.7, 0.0, 'tulip_white', 0.7, False)],
        box_footprint=[(2.1, 0.4)],
    ))
    buf = store.tag('1')
    assert len(buf.flowers) == 1
    assert buf.flowers[0].species == 'tulip_white'
    assert buf.box_footprint == [(2.1, 0.4)]


def test_record_flower_without_flowers_keeps_defaults():
    # Old-style flower update (no flowers/footprint) still works: summary
    # fields set, flowers/footprint default to empty.
    store = TwinStateStore()
    store.record_flower(FlowerUpdate(
        tag_id='1', monotonic_at=0.0, species='tulip_pink',
        species_confidence=0.5, anomaly=True,
    ))
    buf = store.tag('1')
    assert buf.species == 'tulip_pink'
    assert buf.flowers == []
    assert buf.box_footprint == []


def test_record_flower_blooms_without_footprint():
    # Degenerate-tag fallback: the aggregator can emit blooms with no box
    # polygon. The store keeps the two decoupled — blooms stored, footprint
    # stays empty (the HMI still renders the dots without a box rectangle).
    store = TwinStateStore()
    store.record_flower(FlowerUpdate(
        tag_id='1', monotonic_at=0.0, species='tulip_red',
        species_confidence=0.9, anomaly=False,
        flowers=[TwinFlower(1.0, 2.0, 'tulip_red', 0.9, False)],
        box_footprint=[],
    ))
    buf = store.tag('1')
    assert len(buf.flowers) == 1
    assert buf.box_footprint == []
