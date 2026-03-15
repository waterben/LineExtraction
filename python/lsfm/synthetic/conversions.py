"""Shared segment-numpy conversion utilities.

These functions are the single source of truth for converting between
``le_geometry`` segment types and numpy arrays.  Used by rendering,
evaluation, and visualization modules.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import le_geometry


def segments_to_array(
    segments: list[le_geometry.LineSegment_f64],
) -> NDArray[np.float64]:
    """Convert 2D line segments to a numpy array.

    :param segments: List of 2D line segments.
    :type segments: list[le_geometry.LineSegment_f64]
    :return: Array of shape ``(N, 2, 2)`` where ``[i, 0]`` is the start
        point and ``[i, 1]`` is the end point.
    :rtype: numpy.ndarray
    """
    if not segments:
        return np.zeros((0, 2, 2), dtype=np.float64)
    arr = np.empty((len(segments), 2, 2), dtype=np.float64)
    for i, seg in enumerate(segments):
        sp = seg.start_point()
        ep = seg.end_point()
        arr[i, 0, 0] = sp[0]
        arr[i, 0, 1] = sp[1]
        arr[i, 1, 0] = ep[0]
        arr[i, 1, 1] = ep[1]
    return arr


def segments3d_to_array(
    segments: list[le_geometry.LineSegment3_f64],
) -> NDArray[np.float64]:
    """Convert 3D line segments to a numpy array.

    :param segments: List of 3D line segments.
    :type segments: list[le_geometry.LineSegment3_f64]
    :return: Array of shape ``(N, 2, 3)`` where ``[i, 0]`` is the start
        point and ``[i, 1]`` is the end point.
    :rtype: numpy.ndarray
    """
    if not segments:
        return np.zeros((0, 2, 3), dtype=np.float64)
    arr = np.empty((len(segments), 2, 3), dtype=np.float64)
    for i, seg in enumerate(segments):
        sp = seg.start_point()
        ep = seg.end_point()
        arr[i, 0, 0] = sp[0]
        arr[i, 0, 1] = sp[1]
        arr[i, 0, 2] = sp[2]
        arr[i, 1, 0] = ep[0]
        arr[i, 1, 1] = ep[1]
        arr[i, 1, 2] = ep[2]
    return arr


def build_line_segments_3d(
    vertices: NDArray[np.float64],
    edge_indices: list[tuple[int, int]],
    pose: le_geometry.Pose_f64 | None = None,
) -> list[le_geometry.LineSegment3_f64]:
    """Construct 3D line segments from topology and an optional pose.

    :param vertices: Local-space vertices ``(N, 3)`` float64.
    :type vertices: numpy.ndarray
    :param edge_indices: Edge index pairs ``[(i, j), ...]``.
    :type edge_indices: list[tuple[int, int]]
    :param pose: Optional world-space pose to transform vertices.
    :type pose: le_geometry.Pose_f64, optional
    :return: List of ``LineSegment3_f64`` in world space.
    :rtype: list[le_geometry.LineSegment3_f64]
    """
    import le_geometry

    if pose is not None:
        vertices = _transform_vertices(vertices, pose)

    segments: list[le_geometry.LineSegment3_f64] = []
    for i, j in edge_indices:
        v0 = vertices[i]
        v1 = vertices[j]
        seg = le_geometry.LineSegment3_f64.from_endpoints(
            float(v0[0]),
            float(v0[1]),
            float(v0[2]),
            float(v1[0]),
            float(v1[1]),
            float(v1[2]),
        )
        segments.append(seg)
    return segments


def _transform_vertices(
    vertices: NDArray[np.float64],
    pose: le_geometry.Pose_f64,
) -> NDArray[np.float64]:
    """Apply a Pose_f64 transform to an array of vertices.

    :param vertices: Vertices ``(N, 3)`` in local space.
    :type vertices: numpy.ndarray
    :param pose: Pose transform to apply.
    :type pose: le_geometry.Pose_f64
    :return: Transformed vertices ``(N, 3)`` in world space.
    :rtype: numpy.ndarray
    """
    hom = pose.hom_matrix()
    rot = np.array([[hom[r][c] for c in range(3)] for r in range(3)], dtype=np.float64)
    trans = np.array([hom[0][3], hom[1][3], hom[2][3]], dtype=np.float64)
    return (vertices @ rot.T) + trans
