"""Tag-location loader, factored out of the orchestrator so the node and
tests can both import it.

Coordinates come from the mdp-greenhouse package's bundled
configs/tag_locations.json by default. The package is pip-installed
(declared in setup.py install_requires); colcon won't install it.

The demo launches pass an explicit ``tag_locations_file`` pointing at the
committed snapshot ``lupin_bringup/config/tag_locations_widened.json`` (a
verbatim copy of the 1.0.8 layout — the name is historical; it is no longer
y-stretched). Always pass that file so the bridge, orchestrator and the
generated SDF agree on tag coords regardless of the installed package version.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional


def load_default_tag_locations(path: Optional[str] = None) -> dict:
    """Return the ``tags`` sub-dict from a tag_locations.json file.

    If ``path`` is given (and non-empty), load from that filesystem path.
    Otherwise resolve the bundled JSON inside the installed
    ``greenhouse_sim`` package via ``importlib.resources`` so we don't
    bake a site-packages path into the source.
    """
    if path:
        data = json.loads(Path(path).expanduser().read_text())
        return data['tags']

    try:
        from importlib.resources import files
    except ImportError:  # pragma: no cover - we target py3.10
        from importlib_resources import files  # type: ignore

    try:
        cfg_path = files('greenhouse_sim').joinpath('configs/tag_locations.json')
    except (ModuleNotFoundError, ImportError) as exc:
        raise RuntimeError(
            "Could not import 'greenhouse_sim'. The mdp-greenhouse package is "
            "a runtime dependency of lupin_mission and is not in rosdep — "
            "install it with: pip install 'mdp-greenhouse>=1.0.3,<2'"
        ) from exc
    data = json.loads(cfg_path.read_text())
    return data['tags']


def load_default_tables(path: Optional[str] = None) -> dict:
    """Return the ``tables`` sub-dict from the same tag_locations.json file.

    The orchestrator's per-tag approach-pose computation needs the table
    bounding boxes alongside the tags, so we expose a sibling loader rather
    than widening :func:`load_default_tag_locations`'s return type and
    breaking older callers.

    Returns an empty dict (rather than raising) when the JSON has no
    ``tables`` key — keeps mission-types that don't need geometry working
    against legacy fixtures.
    """
    if path:
        data = json.loads(Path(path).expanduser().read_text())
        return data.get('tables', {})

    try:
        from importlib.resources import files
    except ImportError:  # pragma: no cover - we target py3.10
        from importlib_resources import files  # type: ignore

    try:
        cfg_path = files('greenhouse_sim').joinpath('configs/tag_locations.json')
    except (ModuleNotFoundError, ImportError) as exc:
        raise RuntimeError(
            "Could not import 'greenhouse_sim'. The mdp-greenhouse package is "
            "a runtime dependency of lupin_mission and is not in rosdep — "
            "install it with: pip install 'mdp-greenhouse>=1.0.3,<2'"
        ) from exc
    data = json.loads(cfg_path.read_text())
    return data.get('tables', {})


def numeric_string_sort_key(tag_id: str):
    """Sort numeric tag IDs as ints (numeric IDs first), alpha IDs after."""
    try:
        return (0, int(tag_id))
    except ValueError:
        return (1, tag_id)
