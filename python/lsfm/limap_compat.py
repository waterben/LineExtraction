"""LIMAP forward-compatibility helpers.

Thin conversion layer between the ``le_lfd`` / ``le_lsd`` Python
bindings and the data formats expected by
`LIMAP <https://github.com/cvg/limap>`_ (Line Mapping).

LIMAP operates on plain NumPy arrays and Python dicts at its
Python-level API boundary, so interop is straightforward:

============ ===================== =======================================
Our type     LIMAP equivalent      Shape / Format
============ ===================== =======================================
LineSegment  segments array        ``(N, 5)`` float64 — x1 y1 x2 y2 score
FdcLBD       descinfo dict         ``{ms_lines, line_descriptors}``
Match list   match pairs           ``(M, 2)`` int32 — query_idx match_idx
============ ===================== =======================================

Usage example::

    import le_lsd, le_lfd
    from lsfm.limap_compat import (
        segments_to_limap,
        descriptors_to_limap,
        matches_to_limap,
    )

    det = le_lsd.LsdCC()
    det.detect(image)
    segs = det.line_segments()

    limap_segs = segments_to_limap(segs)        # (N, 5) ndarray
    limap_desc = descriptors_to_limap(creator, segs)  # dict
    limap_matches = matches_to_limap(matches)   # (M, 2) ndarray
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    # Avoid hard import so module works even when bindings are not on
    # sys.path (e.g. during static analysis or documentation builds).
    import le_lfd  # noqa: F401


def segments_to_limap(
    segments: Sequence[object],
    *,
    score: float = 1.0,
) -> NDArray[np.float64]:
    """Convert a list of ``LineSegment`` objects to LIMAP ``(N, 5)`` format.

    Each row is ``[x1, y1, x2, y2, score]``.

    :param segments: Line segments returned by an LSD detector
        (e.g. ``le_lsd.LsdCC.line_segments()``).
    :type segments: Sequence[LineSegment]
    :param score: Constant confidence score appended to every row.
        LIMAP stores the LSD line width here; use 1.0 as a safe default.
    :type score: float
    :return: Segments in LIMAP format.
    :rtype: numpy.ndarray of shape ``(N, 5)``
    """
    if not segments:
        return np.zeros((0, 5), dtype=np.float64)

    rows = np.empty((len(segments), 5), dtype=np.float64)
    for i, seg in enumerate(segments):
        x1, y1, x2, y2 = seg.end_points()  # type: ignore[attr-defined]
        rows[i] = (x1, y1, x2, y2, score)
    return rows


def _build_multiscale_lines(
    segs_np: NDArray[np.float64],
    num_octaves: int = 5,
) -> list[list[tuple[int, NDArray[np.float64]]]]:
    """Build a single-scale approximation of LIMAP's multiscale line structure.

    LIMAP's LBD extractor stores a ``ms_lines`` list where each segment
    is represented at multiple image-pyramid levels.  Since our LBD
    descriptor is already computed from the original resolution, we create
    a single-scale (octave 0) entry and approximate the remaining octaves
    by scaling.

    :param segs_np: Segment array of shape ``(N, 5)`` (output of
        :func:`segments_to_limap`).
    :type segs_np: numpy.ndarray
    :param num_octaves: Number of pyramid octaves to generate.
    :type num_octaves: int
    :return: Per-segment multiscale line representation.
    :rtype: list[list[tuple[int, numpy.ndarray]]]
    """
    ms_lines: list[list[tuple[int, NDArray[np.float64]]]] = []
    for row in segs_np:
        x1, y1, x2, y2 = row[:4]
        length = float(np.hypot(x2 - x1, y2 - y1))
        base = np.array([x1, y1, x2, y2, 0.0, length], dtype=np.float64)
        octaves: list[tuple[int, NDArray[np.float64]]] = [(0, base)]
        for o in range(1, num_octaves):
            scale = 2.0**o
            scaled = base.copy()
            scaled[:4] /= scale
            scaled[5] /= scale
            octaves.append((o, scaled))
        ms_lines.append(octaves)
    return ms_lines


def descriptors_to_limap(
    creator: object,
    segments: Sequence[object],
    *,
    num_octaves: int = 5,
) -> dict[str, object]:
    """Convert LBD descriptors to LIMAP ``descinfo`` dict.

    LIMAP's ``LBD/extractor.py`` produces a dict with keys
    ``"ms_lines"`` (multiscale segment geometry) and
    ``"line_descriptors"`` (the actual descriptor matrix).

    :param creator: An ``FdcLBD`` descriptor creator that has already been
        constructed with gradient images.
    :type creator: le_lfd.FdcLBD
    :param segments: Line segments for which to compute descriptors.
    :type segments: Sequence[LineSegment]
    :param num_octaves: Number of pyramid octaves for ``ms_lines``.
    :type num_octaves: int
    :return: LIMAP-compatible descriptor info dict.
    :rtype: dict with keys ``"ms_lines"`` and ``"line_descriptors"``
    """
    # Get the (N, D) descriptor matrix directly from the creator
    desc_mat: NDArray[np.floating] = creator.create_mat(segments)  # type: ignore[attr-defined]

    segs_np = segments_to_limap(segments)
    ms_lines = _build_multiscale_lines(segs_np, num_octaves=num_octaves)

    return {
        "ms_lines": ms_lines,
        "line_descriptors": np.asarray(desc_mat, dtype=np.float64),
    }


def matches_to_limap(
    matches: Sequence[object],
    *,
    drop_nan: bool = True,
) -> NDArray[np.int32]:
    """Convert ``DescriptorMatch`` objects to LIMAP ``(M, 2)`` index pairs.

    :param matches: Match list from a brute-force matcher (e.g.
        ``le_lfd.BruteForceLBD.best()``).
    :type matches: Sequence[DescriptorMatch]
    :param drop_nan: If *True*, silently skip matches with NaN distance
        (caused by degenerate descriptors).
    :type drop_nan: bool
    :return: Match index pairs in LIMAP format.
    :rtype: numpy.ndarray of shape ``(M, 2)``
    """
    if not matches:
        return np.zeros((0, 2), dtype=np.int32)

    pairs: list[list[int]] = []
    for m in matches:
        if drop_nan and np.isnan(m.distance):  # type: ignore[attr-defined]
            continue
        pairs.append([m.query_idx, m.match_idx])  # type: ignore[attr-defined]

    if not pairs:
        return np.zeros((0, 2), dtype=np.int32)
    return np.array(pairs, dtype=np.int32)
