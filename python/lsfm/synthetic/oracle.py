"""Oracle matching and shared detected-to-GT assignment.

:func:`assign_detected_to_gt` is the single source of truth for mapping
detected 2D segments to ground truth segments.  Used by both oracle
matching and evaluation.

:func:`oracle_match` derives cross-view correspondences from GT
knowledge, providing a perfect-matching baseline.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import le_geometry

    from lsfm.synthetic.ground_truth import SceneGroundTruth


def assign_detected_to_gt(
    detected: list[le_geometry.LineSegment_f64],
    gt_segments: list[le_geometry.LineSegment_f64],
    max_distance: float = 20.0,
    max_angle_deg: float = 30.0,
) -> dict[int, int]:
    """Greedy 1:1 assignment of detected segments to GT segments.

    Uses a combined cost metric: midpoint distance + angular difference.
    Matches with distance > ``max_distance`` or angular difference >
    ``max_angle_deg`` are rejected.

    :param detected: Detected 2D line segments.
    :type detected: list[le_geometry.LineSegment_f64]
    :param gt_segments: Ground truth 2D line segments.
    :type gt_segments: list[le_geometry.LineSegment_f64]
    :param max_distance: Maximum midpoint distance in pixels.
    :type max_distance: float
    :param max_angle_deg: Maximum angular difference in degrees.
    :type max_angle_deg: float
    :return: Mapping ``{detected_idx: gt_idx}`` for assigned pairs.
    :rtype: dict[int, int]
    """
    if not detected or not gt_segments:
        return {}

    max_angle_rad = math.radians(max_angle_deg)

    # Precompute midpoints and angles
    det_mids = np.array([_midpoint(s) for s in detected], dtype=np.float64)
    gt_mids = np.array([_midpoint(s) for s in gt_segments], dtype=np.float64)
    det_angles = np.array([_segment_angle(s) for s in detected], dtype=np.float64)
    gt_angles = np.array([_segment_angle(s) for s in gt_segments], dtype=np.float64)

    # Compute pairwise costs
    candidates: list[tuple[float, int, int]] = []
    for di in range(len(detected)):
        for gi in range(len(gt_segments)):
            dist = float(np.linalg.norm(det_mids[di] - gt_mids[gi]))
            if dist > max_distance:
                continue
            angle_diff = abs(_angle_diff(det_angles[di], gt_angles[gi]))
            if angle_diff > max_angle_rad:
                continue
            # Combined cost: distance + weighted angular penalty
            cost = dist + (angle_diff / max_angle_rad) * max_distance
            candidates.append((cost, di, gi))

    # Greedy assignment: sort by cost, assign best first
    candidates.sort()
    assigned_det: set[int] = set()
    assigned_gt: set[int] = set()
    result: dict[int, int] = {}

    for cost, di, gi in candidates:
        if di in assigned_det or gi in assigned_gt:
            continue
        result[di] = gi
        assigned_det.add(di)
        assigned_gt.add(gi)

    return result


def oracle_match(
    detected_views: list[list[le_geometry.LineSegment_f64]]
    | dict[int, list[le_geometry.LineSegment_f64]],
    gt: SceneGroundTruth,
    view_i: int,
    view_j: int,
    max_distance: float = 20.0,
    max_angle_deg: float = 30.0,
) -> list[tuple[int, int]]:
    """Oracle matching using GT correspondences.

    Assigns detected segments to GT in each view via
    :func:`assign_detected_to_gt`, then joins assignments through
    the shared 3D parent line index.

    :param detected_views: Detected segments per view.
    :type detected_views: list[list[le_geometry.LineSegment_f64]]
    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param view_i: First view index.
    :type view_i: int
    :param view_j: Second view index.
    :type view_j: int
    :param max_distance: Maximum midpoint distance for assignment.
    :type max_distance: float
    :param max_angle_deg: Maximum angular difference for assignment.
    :type max_angle_deg: float
    :return: List of ``(det_idx_view_i, det_idx_view_j)`` pairs.
    :rtype: list[tuple[int, int]]
    """
    # Assign detected to GT in each view
    assign_i = assign_detected_to_gt(
        detected_views[view_i],
        gt.segments_2d(view_i),
        max_distance=max_distance,
        max_angle_deg=max_angle_deg,
    )
    assign_j = assign_detected_to_gt(
        detected_views[view_j],
        gt.segments_2d(view_j),
        max_distance=max_distance,
        max_angle_deg=max_angle_deg,
    )

    if not assign_i or not assign_j:
        return []

    # Get parent 3D indices
    parents_i = gt.parent_indices(view_i)
    parents_j = gt.parent_indices(view_j)

    # Build reverse map: 3D parent -> detected idx in view_j
    parent_to_det_j: dict[int, int] = {}
    for det_j, gt_j in assign_j.items():
        parent_3d = parents_j[gt_j]
        parent_to_det_j[parent_3d] = det_j

    # Join through shared 3D parent
    matches: list[tuple[int, int]] = []
    for det_i, gt_i in assign_i.items():
        parent_3d = parents_i[gt_i]
        if parent_3d in parent_to_det_j:
            matches.append((det_i, parent_to_det_j[parent_3d]))

    return matches


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _midpoint(seg: le_geometry.LineSegment_f64) -> np.ndarray:
    """Return the midpoint of a 2D segment as ``(2,)`` array."""
    cp = seg.center_point()
    return np.array([cp[0], cp[1]], dtype=np.float64)


def _segment_angle(seg: le_geometry.LineSegment_f64) -> float:
    """Return the angle of a 2D segment in radians ``[-pi, pi)``."""
    sp = seg.start_point()
    ep = seg.end_point()
    dx = ep[0] - sp[0]
    dy = ep[1] - sp[1]
    return math.atan2(dy, dx)


def _angle_diff(a: float, b: float) -> float:
    """Signed angular difference, accounting for line symmetry.

    Lines have 180-degree ambiguity, so we normalize the difference
    to ``[0, pi/2]``.
    """
    diff = a - b
    # Normalize to [-pi, pi]
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    # Line symmetry: angle and angle+pi are the same line
    return min(abs(diff), abs(abs(diff) - math.pi))


def oracle_tracks(
    detected_views: list[list[le_geometry.LineSegment_f64]],
    gt: SceneGroundTruth,
    view_indices: list[int] | None = None,
    max_distance: float = 20.0,
    max_angle_deg: float = 30.0,
) -> list[list[tuple[int, int]]]:
    """Build multiview line tracks from GT parent correspondences.

    For each 3D parent line, collects all detected segments across
    the specified views that are assigned to a GT segment originating
    from that parent.  The result is a list of tracks where each track
    is a list of ``(view_index, detected_segment_index)`` tuples.

    :param detected_views: Detected segments per view (indexed by view).
    :type detected_views: list[list[le_geometry.LineSegment_f64]]
    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param view_indices: Which views to include (default: all).
    :type view_indices: list[int] | None
    :param max_distance: Maximum midpoint distance for assignment.
    :type max_distance: float
    :param max_angle_deg: Maximum angular difference for assignment.
    :type max_angle_deg: float
    :return: List of tracks.  Each track is a list of
        ``(view_index, detected_segment_index)`` with len >= 2.
    :rtype: list[list[tuple[int, int]]]
    """
    if view_indices is None:
        view_indices = list(range(gt.n_views))

    # parent_3d -> list of (view_idx, det_idx)
    parent_to_observations: dict[int, list[tuple[int, int]]] = {}

    for view_idx in view_indices:
        assignment = assign_detected_to_gt(
            detected_views[view_idx],
            gt.segments_2d(view_idx),
            max_distance=max_distance,
            max_angle_deg=max_angle_deg,
        )
        parents = gt.parent_indices(view_idx)
        for det_idx, gt_idx in assignment.items():
            parent_3d = parents[gt_idx]
            parent_to_observations.setdefault(parent_3d, []).append((view_idx, det_idx))

    # Keep only tracks with observations in at least 2 different views.
    tracks: list[list[tuple[int, int]]] = []
    for obs_list in parent_to_observations.values():
        views_seen = {v for v, _ in obs_list}
        if len(views_seen) >= 2:
            tracks.append(obs_list)

    return tracks
