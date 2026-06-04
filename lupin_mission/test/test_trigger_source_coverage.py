"""smi-5: every HSM trigger is declared from every state the code fires it from.

This mechanically guards the class of bug behind the original PREPARE-abort gap:
`abort_to_return` used to be fired from PREPARE_LOCALIZING but was NOT declared
with PREPARE as a source, so `ignore_invalid_triggers=True` silently swallowed
the abort and the mission dead-ended in the unrecoverable FAULT state.

``FIRES_FROM`` records, per trigger, the set of (leaf) states the orchestrator
code actually fires it from — derived by reading the guard around each
``self.<trigger>()`` call site. The test asserts each of those states is covered
by the trigger's declared ``source`` list in build_hsm_spec() (accounting for
HSM hierarchy: a parent source covers its children). A too-narrow source list,
or a new fire site from an undeclared state, fails here instead of silently
no-opping on the robot.

When you add/move a ``self.<trigger>()`` call, update FIRES_FROM to match —
the completeness checks below fail if it drifts from the code.
"""

from __future__ import annotations

import re
from pathlib import Path

import lupin_mission.node as node_mod
from lupin_mission.node import build_hsm_spec

# Trigger -> leaf states the code fires it from (verified against the guards at
# each self.<trigger>() call site in node.py).
FIRES_FROM: dict[str, set[str]] = {
    'deps_up': {'BOOT'},
    'start_mission': {'READY'},               # from DONE we reset_for_next → READY first
    'localized': {'PREPARE_LOCALIZING'},
    'tags_discovered': {'EXPLORING'},
    'no_frontiers': {'EXPLORING'},
    'nav_succeeded': {'INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'},
    'nav_unreachable': {'INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'},
    'scan_done': {'INSPECTING_SCANNING', 'MONITORING_SCANNING'},
    'next_tag': {'INSPECTING_PUBLISHING', 'MONITORING_PUBLISHING'},
    'inspection_complete': {'INSPECTING_PUBLISHING', 'MONITORING_PUBLISHING'},
    'abort_to_return': {
        'EXPLORING', 'PREPARE_LOCALIZING',
        'INSPECTING_NAVIGATING', 'INSPECTING_SCANNING', 'INSPECTING_PUBLISHING',
        'MONITORING_NAVIGATING', 'MONITORING_SCANNING', 'MONITORING_PUBLISHING',
    },
    'returned': {'RETURNING'},
    'resume_inspection': {'RETURNING'},
    'resume_monitoring': {'RETURNING'},
    'resume_exploration': {'RETURNING'},
    'reset_for_next': {'DONE'},
    'recover': {'FAULT'},
    'fault': {'BOOT', 'PREPARE_LOCALIZING'},
}


def _declared_sources() -> dict[str, set[str]]:
    declared: dict[str, set[str]] = {}
    for tr in build_hsm_spec()['transitions']:
        src = tr['source']
        srcs = src if isinstance(src, list) else [src]
        declared.setdefault(tr['trigger'], set()).update(srcs)
    return declared


def _covered(state: str, sources: set[str]) -> bool:
    # A declared parent source covers its child leaf states (HSM inheritance):
    # source 'PREPARE' covers 'PREPARE_LOCALIZING'.
    return any(state == s or state.startswith(s + '_') for s in sources)


def test_every_fired_state_is_a_declared_source():
    declared = _declared_sources()
    gaps = []
    for trigger, states in FIRES_FROM.items():
        assert trigger in declared, f'{trigger} is fired in code but not an HSM trigger'
        for state in states:
            if not _covered(state, declared[trigger]):
                gaps.append(
                    f'  {trigger} fired from {state!r} but declared only from '
                    f'{sorted(declared[trigger])}'
                )
    assert not gaps, 'trigger fired from an undeclared source (would silently no-op):\n' + '\n'.join(gaps)


def test_fires_from_covers_exactly_the_triggers_fired_in_code():
    # Keep FIRES_FROM honest: every trigger called as self.<name>() in node.py
    # must be documented here, and vice-versa. Drift here means a fire site was
    # added/removed without updating the coverage map above.
    declared = _declared_sources()
    source = Path(node_mod.__file__).read_text()
    fired_in_code = {t for t in declared if re.search(rf'self\.{t}\(', source)}
    assert fired_in_code == set(FIRES_FROM), (
        'FIRES_FROM is out of sync with the self.<trigger>() call sites; '
        f'only-in-code={sorted(fired_in_code - set(FIRES_FROM))}, '
        f'only-in-map={sorted(set(FIRES_FROM) - fired_in_code)}'
    )
