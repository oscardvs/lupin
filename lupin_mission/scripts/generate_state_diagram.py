#!/usr/bin/env python3
"""Render the v2 HSM diagrams to docs/.

Produces two PNGs:
    docs/state_machine.png             — top-level lifecycle (PREPARE /
                                          INSPECTING shown as opaque
                                          composites for legibility)
    docs/state_machine_inspection.png  — INSPECTING sub-machine zoom-in

Each one is a separate transitions HierarchicalGraphMachine on a dummy
model, so neither needs ROS or the orchestrator instance.

Run from the package root:

    python3 scripts/generate_state_diagram.py [docs/]

Requires:
    pip install 'transitions[diagrams]>=0.9'  (or transitions + pygraphviz)
    apt install graphviz                       (for the `dot` CLI)
"""

import sys
from pathlib import Path

from transitions.extensions import HierarchicalGraphMachine


class _Dummy:
    pass


# ─── top-level lifecycle (composites kept opaque) ────────────────────────


def _top_level_spec() -> dict:
    """Flat lifecycle — PREPARE/INSPECTING are leaf names so the diagram
    doesn't unfold their children. Mirrors the spec's lifecycle picture."""
    return {
        'states': ['BOOT', 'READY', 'PREPARE', 'EXPLORING', 'INSPECTING',
                   'MONITORING', 'RETURNING', 'DONE', 'FAULT'],
        'initial': 'BOOT',
        'transitions': [
            {'trigger': 'deps_up', 'source': 'BOOT', 'dest': 'READY'},
            {'trigger': 'start_mission', 'source': 'READY', 'dest': 'PREPARE'},
            # PREPARE branches by mission type (condition shown opaque here).
            {'trigger': 'localized', 'source': 'PREPARE', 'dest': 'INSPECTING'},
            {'trigger': 'localized', 'source': 'PREPARE', 'dest': 'EXPLORING'},
            # ExplorationMission: explore → monitor (or home if nothing found).
            {'trigger': 'tags_discovered', 'source': 'EXPLORING', 'dest': 'MONITORING'},
            {'trigger': 'no_frontiers', 'source': 'EXPLORING', 'dest': 'MONITORING'},
            {'trigger': 'no_frontiers', 'source': 'EXPLORING', 'dest': 'RETURNING'},
            {'trigger': 'inspection_complete', 'source': 'INSPECTING', 'dest': 'RETURNING'},
            {'trigger': 'abort_to_return', 'source': 'INSPECTING', 'dest': 'RETURNING'},
            {'trigger': 'abort_to_return', 'source': 'EXPLORING', 'dest': 'RETURNING'},
            {'trigger': 'abort_to_return', 'source': 'MONITORING', 'dest': 'RETURNING'},
            {'trigger': 'returned', 'source': 'RETURNING', 'dest': 'DONE'},
            {'trigger': 'reset_for_next', 'source': 'DONE', 'dest': 'READY'},
            {
                'trigger': 'fault',
                'source': ['BOOT', 'READY', 'PREPARE', 'EXPLORING', 'INSPECTING',
                           'MONITORING', 'RETURNING'],
                'dest': 'FAULT',
            },
        ],
    }


# ─── INSPECTING sub-machine zoom-in ──────────────────────────────────────


def _inspecting_spec() -> dict:
    return {
        'states': ['NAVIGATING', 'SCANNING', 'PUBLISHING'],
        'initial': 'NAVIGATING',
        'transitions': [
            {'trigger': 'nav_succeeded', 'source': 'NAVIGATING', 'dest': 'SCANNING'},
            {'trigger': 'nav_unreachable', 'source': 'NAVIGATING', 'dest': 'PUBLISHING'},
            {'trigger': 'scan_done', 'source': 'SCANNING', 'dest': 'PUBLISHING'},
            {'trigger': 'next_tag', 'source': 'PUBLISHING', 'dest': 'NAVIGATING'},
        ],
    }


# ─── styling ─────────────────────────────────────────────────────────────


def _style_graph(graph, *, terminal_red=None, terminal_green=None) -> None:
    graph.graph_attr.update({
        'rankdir': 'LR',
        'nodesep': '0.4',
        'ranksep': '0.8',
        'splines': 'true',
        'pad': '0.3',
        'bgcolor': 'white',
    })
    graph.node_attr.update({
        'fontname': 'Helvetica',
        'fontsize': '11',
    })
    graph.edge_attr.update({
        'fontname': 'Helvetica',
        'fontsize': '9',
    })
    for state, attrs in (
        (terminal_red, {'fillcolor': '#fde2e2', 'style': 'filled', 'color': '#a83232'}),
        (terminal_green, {'fillcolor': '#e2f3e2', 'style': 'filled', 'color': '#2e7d32'}),
    ):
        if state is None:
            continue
        if state in graph.nodes():
            graph.get_node(state).attr.update(attrs)
    # transitions paints the initial state in a coral colour by default,
    # which collides with FAULT's red on the same canvas. Recolour it to
    # a neutral blue so "start here" reads visually distinct from "error".
    for node in graph.nodes():
        if (node.attr.get('fillcolor', '') or '').startswith('#'):
            # Skip nodes we explicitly styled above.
            if node in {terminal_red, terminal_green}:
                continue
        # The initial-state node gets its own peripheries/style; recolour
        # only those that weren't explicitly themed.
        if node not in {terminal_red, terminal_green}:
            current = (node.attr.get('fillcolor', '') or '').lower()
            if current and current not in {'white', '#ffffff', ''}:
                node.attr.update({
                    'fillcolor': '#dde7f5',
                    'color': '#2c5da0',
                })
    # Soften the catch-all `fault` edges so they don't dominate the eye.
    for edge in graph.edges():
        label = (edge.attr.get('label', '') or '').strip()
        if label == 'fault':
            edge.attr.update({
                'color': '#888888',
                'style': 'dashed',
                'fontcolor': '#888888',
            })


def _render(spec: dict, title: str, out: Path,
            *, terminal_red=None, terminal_green=None) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    machine = HierarchicalGraphMachine(
        model=_Dummy(),
        states=spec['states'],
        transitions=spec['transitions'],
        initial=spec['initial'],
        show_conditions=False,
        show_state_attributes=False,
        title=title,
    )
    graph = machine.get_graph()
    _style_graph(graph, terminal_red=terminal_red, terminal_green=terminal_green)
    graph.draw(out, prog='dot')
    print(f'wrote {out}')


def main() -> int:
    docs = Path(sys.argv[1] if len(sys.argv) > 1 else 'docs')
    _render(
        _top_level_spec(),
        'lupin_mission orchestrator v2 — lifecycle',
        docs / 'state_machine.png',
        terminal_red='FAULT',
        terminal_green='DONE',
    )
    _render(
        _inspecting_spec(),
        'INSPECTING sub-machine (per tag)',
        docs / 'state_machine_inspection.png',
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
