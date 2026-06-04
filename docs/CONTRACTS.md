# Data-contract notes — deliberate dead / asymmetric wiring

Three message contracts in this repo look like broken or half-finished wiring
but are intentional. Documented here so a future reader (or an audit) doesn't
"fix" them.

## 1. `Observation.KIND_ANOMALY` / `AnomalyReport` — deliberate dead stub

`AnomalyReport` and the `KIND_ANOMALY` observation kind exist to lock the
contract, but **nothing produces them.** No code path emits an `Observation`
with `kind == KIND_ANOMALY`, and the digital twin logs-and-drops unmodelled
kinds if one ever arrives.

Pest/anomaly detection today rides the **flower** path instead: the gripper
YOLO classifier raises a `bug` flag that surfaces as `DiscoveredTag.anomaly`
and `FlowerObservation.anomaly` during SCANNING. Standalone anomalies (a fallen
plant, an obstruction) have no producer yet.

To wire it up later: emit `Observation(kind=KIND_ANOMALY, anomaly=AnomalyReport(...))`
and add a `KIND_ANOMALY` branch in `lupin_twin` (it currently drops it).

## 2. `DiscoveredTag.species` / `DiscoveredTag.anomaly` — producer-set, consumer-unread (for logic)

`lupin_perception`'s `perception_aggregator` fills `species`,
`species_confidence`, and `anomaly` on every `DiscoveredTag`. **No consumer uses
them for routing, filtering, or any mission decision.** They exist only to be
mirrored into `TwinTagState` for HMI display (the TYPE column, the pest icon,
the flower-layer colour). Changing them affects rendering only, never the state
machine — so don't go looking for the "missing" logic that reads them.

## 3. Web subscriptions are VOLATILE; most publishers are TRANSIENT_LOCAL (latched)

`roslib.js` exposes **no QoS API**, so every HMI subscription in
`lupin_web/web/src/lib/ros.tsx` is effectively `VOLATILE`. Several Lupin
publishers are `RELIABLE + TRANSIENT_LOCAL` (latched): `/twin/state`,
`/mission/state`, `/perception/discovered_tags`, `/tf_static`.

This mismatch is benign here: the latched state topics also republish
periodically (`/twin/state` and `/mission/state` at ~1 Hz), so a freshly
connected or reloaded HMI fills within a publish cycle. The only thing the web
client must **not** rely on is receiving the single latched backlog sample at
connect time the way a native `transient_local` subscriber would. Anything that
needs the current value immediately should pull it via a service call, not lean
on subscription durability.
