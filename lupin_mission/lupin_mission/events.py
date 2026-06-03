"""Mission-event severity classification — pure, ROS-free, unit-testable.

The orchestrator surfaces a single ``last_error`` string for a mix of genuine
faults and routine notices (battery, timeouts, nav retries). This maps a code
to a severity + a human label so the HMI can show the right tone instead of
flagging every event as a red "orchestrator error".
"""

from __future__ import annotations

SEVERITY_INFO = 0
SEVERITY_WARN = 1
SEVERITY_ERROR = 2

# Codes that indicate a real, blocking failure (vs. a routine notice).
_ERROR_PREFIXES = ('localization_failed', 'dependency_timeout')

# Friendly labels for known codes (matched on the prefix before any detail).
_FRIENDLY = {
    'battery_low': 'battery low',
    'dock_unreachable': 'dock unreachable — paused',
    'exploration_timeout': 'exploration timed out',
    'estop_engaged': 'e-stop engaged',
    'aborted_in_prepare': 'aborted during prepare',
    'localization_failed': 'localization failed',
    'dependency_timeout': 'dependencies not ready',
    'scan_failed': 'scan failed',
}


def classify_event(code):
    """Map a last_error/event code to (severity, friendly_label).

    Empty/None code -> (SEVERITY_INFO, ''). Unknown codes pass through as
    SEVERITY_WARN with the raw code as the label.
    """
    code = (code or '').strip()
    if not code:
        return (SEVERITY_INFO, '')
    if code.startswith('nav_status_'):
        return (SEVERITY_WARN, 'navigation retry')
    # Strip a ' (...)' or ': ...' detail suffix to match the base code.
    base = code.split(':', 1)[0].split(' ', 1)[0]
    severity = SEVERITY_ERROR if base in _ERROR_PREFIXES else SEVERITY_WARN
    return (severity, _FRIENDLY.get(base, code))
