"""Regression guard for the HMI "Erase map" button in simulation.

The erase-map button calls ``/lupin/nav/clear_map`` (std_srvs/srv/Trigger),
owned by ``slam_reset_node``. That node SIGTERMs the slam_toolbox process; the
launch file must pair it with ``respawn=True`` so slam_toolbox comes back up
with an empty pose graph. ``hardware.launch.py`` and the standalone
``slam_{sim,hardware}.launch.py`` wrappers do this via the shared
``_slam_core.launch.py``.

The one-shot sim bundles (``sim_full`` / ``sim_autonomy``) historically
included the *upstream* slam_toolbox ``online_async_launch.py`` instead, which
ships neither ``respawn=True`` nor ``slam_reset_node`` — so the erase button was
dead in sim while working on hardware. This test pins the bundles to the shared
core so that asymmetry cannot silently come back.
"""

import os

from ament_index_python.packages import get_package_share_directory

# Bundled sim entrypoints that bring up slam_toolbox themselves.
SIM_SLAM_BUNDLES = ('sim_full.launch.py', 'sim_autonomy.launch.py')


def _read(package: str, launch_file: str) -> str:
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file,
    )
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def test_sim_bundles_route_slam_through_shared_core() -> None:
    """sim_full / sim_autonomy must use _slam_core, not upstream online_async."""
    for name in SIM_SLAM_BUNDLES:
        src = _read('lupin_bringup', name)
        assert '_slam_core.launch.py' in src, (
            f'{name} must include _slam_core.launch.py so /lupin/nav/clear_map '
            '(the HMI erase-map service) and respawn exist in sim'
        )
        assert 'online_async_launch.py' not in src, (
            f'{name} still includes upstream online_async_launch.py, which has '
            'no respawn=True and no slam_reset_node — erase-map is dead in sim'
        )


def test_slam_core_provides_erase_map_machinery() -> None:
    """The shared core both bundles rely on must carry the erase-map pieces."""
    src = _read('lupin_navigation', '_slam_core.launch.py')
    assert 'respawn=True' in src
    assert 'slam_reset_node' in src
    assert '/lupin/nav/clear_map' in src
