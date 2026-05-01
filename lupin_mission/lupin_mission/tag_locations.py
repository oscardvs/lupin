"""Tag-location loader, factored out of the orchestrator so the node and
tests can both import it.

Coordinates come from the mdp-greenhouse package's bundled
configs/tag_locations.json. The package is pip-installed (declared in
setup.py install_requires); colcon won't install it.
"""

from __future__ import annotations

import json


def load_default_tag_locations() -> dict:
    """Return the tags sub-dict from the installed mdp-greenhouse package.

    Path is resolved via importlib.resources so we don't bake a
    site-packages path into the source.
    """
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


def numeric_string_sort_key(tag_id: str):
    """Sort numeric tag IDs as ints (numeric IDs first), alpha IDs after."""
    try:
        return (0, int(tag_id))
    except ValueError:
        return (1, tag_id)
