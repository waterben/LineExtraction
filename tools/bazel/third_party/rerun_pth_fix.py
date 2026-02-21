"""Activate rerun-sdk .pth-based import path for Bazel.

The ``rerun-sdk`` wheel installs its ``rerun`` Python package inside a
``rerun_sdk/`` subdirectory and relies on a ``.pth`` file
(``rerun_sdk.pth``) to add that subdirectory to ``sys.path`` so that
``import rerun`` works.

Bazel's ``rules_python`` does **not** process ``.pth`` files, so
``import rerun`` would fail at runtime.  Importing this module first
patches ``sys.path`` to include the ``rerun_sdk`` subdirectory, making
``import rerun`` work transparently.

This module is imported automatically as a ``deps`` dependency via the
``//tools/bazel/third_party:rerun_sdk`` wrapper target.
"""

from __future__ import annotations

import importlib
import importlib.util
import pathlib
import sys


def _activate_rerun_pth() -> None:
    """Add the ``rerun_sdk`` wheel subdirectory to ``sys.path``.

    Locates the installed ``rerun_sdk`` package directory and adds it to
    ``sys.path`` so that ``import rerun`` resolves correctly — equivalent
    to what the ``rerun_sdk.pth`` file would do in a normal pip install.
    """
    try:
        spec = importlib.util.find_spec("rerun_sdk")
    except (ModuleNotFoundError, ValueError):
        return

    if spec is None or spec.origin is None:
        return

    # spec.origin points to rerun_sdk/__init__.py
    rerun_sdk_dir = str(pathlib.Path(spec.origin).parent)
    if rerun_sdk_dir not in sys.path:
        sys.path.insert(0, rerun_sdk_dir)


_activate_rerun_pth()
