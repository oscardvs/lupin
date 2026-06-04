"""conftest.py — ensures the lupin_hmi package source is importable in pytest.

The launch_testing pytest hook (pytest < 7) runs pyimport() on every test file
during collection.  pyimport() walks up the directory tree looking for
__init__.py files to determine the module name.  Because lupin_hmi/ has a
top-level __init__.py, pyimport() resolves the import as lupin_hmi.arm_library
but finds the stray top-level package instead of the inner
lupin_hmi/lupin_hmi/ package.

Fix: pre-load the inner lupin_hmi package into sys.modules using an explicit
SourceFileLoader so that by the time pyimport() / __import__ is called the
correct package and submodules are already cached.
"""
import importlib.util
import sys
from pathlib import Path

_inner = Path(__file__).parent / "lupin_hmi"


def _preload(name: str, path: Path) -> None:
    if name in sys.modules:
        return
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)


# Load the inner package first, then any submodule new tests may import early.
_preload("lupin_hmi", _inner / "__init__.py")
for _mod in ("arm_library", "arm_limits", "arm_traj", "estop_bridge", "light_strip_bridge"):
    _path = _inner / f"{_mod}.py"
    if _path.exists():
        _preload(f"lupin_hmi.{_mod}", _path)
