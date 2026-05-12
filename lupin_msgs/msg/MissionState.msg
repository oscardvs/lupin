# Periodic snapshot of the mission orchestrator. Published at a fixed rate
# (default 5 Hz) any time the node is alive — fields just go empty when no
# mission is running.

std_msgs/Header header
string mission_id                # empty when no mission active
string mission_type              # "InspectionMission" | "" — extend as new mission types ship

# Lifecycle states (top-level FSM):
#   "BOOT" | "READY" | "PREPARE" | "INSPECTING" | "RETURNING" | "DONE" | "FAULT"
string lifecycle_state

# Internal sub-state, e.g. "NAVIGATING", "SCANNING", "PUBLISHING", "LOCALIZING".
# Empty when the active lifecycle state has no sub-machine.
string mission_phase

string current_target            # tag_id being processed, "" otherwise
uint16 targets_total
uint16 targets_completed         # status=OK
uint16 targets_failed            # status=SCAN_FAILED
uint16 targets_unreachable       # status=UNREACHABLE
uint16 targets_skipped           # status=SKIPPED (abort + skip_current)

string last_error                # last non-fatal error surfaced by the orchestrator

bool   estop_engaged
bool   paused

float32 battery_percentage
bool    battery_low


builtin_interfaces/Time started_at  # zero stamp when no mission has started yet
