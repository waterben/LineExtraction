"""lsfm — Python package for the LineExtraction project.

Provides shared utilities (dataset access, path resolution) and re-exports
the pybind11-based C++ extension modules (le_imgproc, le_edge, le_lsd, …).

Usage::

    # Import via namespace (preferred for new code)
    from lsfm import le_imgproc, le_edge, le_lsd

    # Import native modules directly (backward compatible)
    import le_imgproc

    # Utilities
    from lsfm.data import TestImages
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = [
    "le_imgproc",
    "le_edge",
    "le_geometry",
    "le_eval",
    "le_lsd",
    "data",
]

_NATIVE_MODULES = frozenset(__all__[:-1])  # All except 'data'


def __getattr__(name: str) -> object:
    """Lazily import native extension modules on first access.

    Allows ``from lsfm import le_imgproc`` without requiring the C++
    extensions to be loaded at package-import time.
    """
    if name in _NATIVE_MODULES:
        import importlib  # noqa: C0415

        module = importlib.import_module(name)
        globals()[name] = module
        return module
    raise AttributeError(f"module 'lsfm' has no attribute {name!r}")
