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

string last_error                # raw last-event code (battery_low, nav_status_6, ...)

# Human-facing event surfaced alongside last_error, classified by severity so
# the HMI shows the right tone instead of labelling everything a red fault.
string last_event                # friendly label, "" when no event
uint8  SEVERITY_INFO=0
uint8  SEVERITY_WARN=1
uint8  SEVERITY_ERROR=2
uint8  last_event_severity

# ─── Exploration progress (ExplorationMission only) ──────────────────────
# Distinct AprilTags discovered so far during EXPLORING, and the target N
# the robot is searching for before it switches to MONITORING. Both 0 for
# an InspectionMission (which knows its targets a priori — see targets_*).
uint16 tags_discovered
uint16 discovery_goal

bool   estop_engaged
bool   paused

float32 battery_percentage
bool    battery_low


builtin_interfaces/Time started_at  # zero stamp when no mission has started yet
