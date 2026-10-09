"""``gym`` shim: re-export the gymnasium API under the names DRL-VO imports.

The 2021-era DRL-VO code does ``import gym`` and annotates arguments with
``gym.spaces.Box``, and its checkpoint's pickled observation space references
``gym.spaces.box.Box``. Re-exporting the gymnasium equivalents keeps that import
working without installing the abandoned ``gym`` package beside gymnasium.
"""

import importlib as _importlib
import sys as _sys

import gymnasium as _gymnasium
from gymnasium import Env, Space, Wrapper, error, spaces, utils  # noqa: F401

__version__ = getattr(_gymnasium, "__version__", "0.0.0-shim")

# The submodule names the pickled classes were stored under. Aliasing them to the
# gymnasium modules keeps one class object per type, which is what unpickling a
# checkpoint compares against.
for _name in (
    "box",
    "discrete",
    "dict",
    "multi_binary",
    "multi_discrete",
    "tuple",
    "space",
    "utils",
):
    try:
        _sys.modules[f"{__name__}.spaces.{_name}"] = _importlib.import_module(
            f"gymnasium.spaces.{_name}"
        )
    except ModuleNotFoundError:
        pass
