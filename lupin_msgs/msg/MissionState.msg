# Periodic snapshot of the mission orchestrator. Published at a fixed rate
# (default 5 Hz) any time the node is alive — fields just go empty when no
# mission is running.

std_msgs/Header header
string mission_id                # empty when no mission active
string mission_type              # "InspectionMission" | "ExplorationMission" | ""

# Lifecycle states (top-level FSM):
#   "BOOT" | "READY" | "PREPARE" | "EXPLORING" | "INSPECTING" | "MONITORING"
#   | "RETURNING" | "DONE" | "FAULT"
# EXPLORING (frontier search to discover tags) and MONITORING (continuous
# re-scan loop over discovered tags) are entered only by an ExplorationMission.
string lifecycle_state

# Internal sub-state, e.g. "NAVIGATING", "SCANNING", "PUBLISHING", "LOCALIZING".
# EXPLORING has no sub-machine (phase empty); MONITORING reuses the
# NAVIGATING/SCANNING/PUBLISHING phases. Empty when the active lifecycle
# state has no sub-machine.
string mission_phase

string current_target            # tag_id being processed, "" otherwise
uint16 targets_total
uint16 targets_completed         # status=OK
uint16 targets_failed            # status=SCAN_FAILED
uint16 targets_unreachable       # status=UNREACHABLE
uint16 targets_skipped           # status=SKIPPED (abort + skip_current)

string last_error                # last non-fatal error surfaced by the orchestrator

# ─── Exploration progress (ExplorationMission only) ──────────────────────
# Distinct AprilTags discovered so far during EXPLORING, and the target N
# the robot is searching for before it switches to MONITORING. Both 0 for
# an InspectionMission (which knows its targets a priori — see targets_*).
uint16 tags_discovered
uint16 discovery_goal

bool   estop_engaged
bool   paused

builtin_interfaces/Time started_at  # zero stamp when no mission has started yet
