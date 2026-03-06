"""Shared utilities for Jupyter notebooks in the LineExtraction project.

Provides workspace discovery, ``sys.path`` setup for Bazel-built native
extension modules, and common visualization helpers used across tutorial
and demo notebooks.

.. note::

   The bootstrap (workspace discovery + ``sys.path`` setup) **must** be
   done inline in the notebook's first cell because ``lsfm`` itself is
   not importable until the paths are configured.  The helpers below
   (e.g. :func:`show_images`) can be imported *after* that bootstrap.

Typical inline bootstrap in a notebook cell::

    try:
        import lsfm
    except ModuleNotFoundError:
        import sys
        from pathlib import Path
        _p = Path.cwd().resolve()
        for _c in [_p, *_p.parents]:
            if (_c / 'MODULE.bazel').exists():
                WS = _c; break
        for _d in ['python',
                    *[f'bazel-bin/libs/{lib}/python'
                      for lib in ('algorithm', 'edge', 'eval', 'geometry',
                                  'imgproc', 'lfd', 'lsd')]]:
            _sp = str(WS / _d)
            if _sp not in sys.path:
                sys.path.insert(0, _sp)

    # Now lsfm is importable
    from lsfm.notebook import show_images
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    import numpy as np


def find_workspace_root(start: Path | None = None) -> Path:
    """Find the LineExtraction workspace root by searching for ``MODULE.bazel``.

    :param start: Directory to start searching from.  Defaults to
        :func:`Path.cwd`.
    :type start: Path, optional
    :return: Absolute path to the workspace root.
    :rtype: Path
    :raises RuntimeError: If no ``MODULE.bazel`` is found.
    """
    p = (start or Path.cwd()).resolve()
    for candidate in [p, *p.parents]:
        if (candidate / "MODULE.bazel").exists():
            return candidate
    msg = "Cannot find LineExtraction workspace root (MODULE.bazel)"
    raise RuntimeError(msg)


_NATIVE_LIBS = (
    "algorithm",
    "edge",
    "eval",
    "geometry",
    "imgproc",
    "lfd",
    "lsd",
)


def setup_sys_path(workspace: Path) -> None:
    """Add Bazel-built native extension directories to ``sys.path``.

    :param workspace: The workspace root (containing ``MODULE.bazel``).
    :type workspace: Path
    """
    # Main Python package
    py_dir = str(workspace / "python")
    if py_dir not in sys.path:
        sys.path.insert(0, py_dir)

    # Bazel-generated native extension modules
    for lib in _NATIVE_LIBS:
        p = str(workspace / "bazel-bin" / "libs" / lib / "python")
        if p not in sys.path:
            sys.path.insert(0, p)


def setup_notebook(start: Path | None = None) -> Path:
    """One-call notebook bootstrap: find workspace and configure imports.

    :param start: Directory to start searching from.  Defaults to
        :func:`Path.cwd`.
    :type start: Path, optional
    :return: Absolute path to the workspace root.
    :rtype: Path
    """
    ws = find_workspace_root(start)
    setup_sys_path(ws)
    return ws


def show_images(
    images: Sequence[np.ndarray],
    titles: Sequence[str],
    *,
    cmap: str = "gray",
    figsize: tuple[float, float] | None = None,
) -> None:
    """Display multiple images side by side with matplotlib.

    :param images: List of images (2-D grayscale or 3-D BGR/RGB).
    :type images: Sequence[numpy.ndarray]
    :param titles: Per-image title strings.
    :type titles: Sequence[str]
    :param cmap: Colormap for grayscale images.
    :type cmap: str
    :param figsize: Figure size ``(width, height)`` in inches.
        Defaults to ``(4*n, 4)``.
    :type figsize: tuple[float, float], optional
    """
    import matplotlib.pyplot as plt

    n = len(images)
    if figsize is None:
        figsize = (4.0 * n, 4.0)
    fig, axes = plt.subplots(1, n, figsize=figsize)
    if n == 1:
        axes = [axes]
    for ax, img, title in zip(axes, images, titles):
        ax.imshow(img, cmap=cmap if img.ndim == 2 else None)
        ax.set_title(title, fontsize=10)
        ax.axis("off")
    plt.tight_layout()
    plt.show()
