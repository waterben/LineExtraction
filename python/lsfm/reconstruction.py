"""High-level 3D line reconstruction API.

Provides simple functions that combine line detection, descriptor
extraction, matching, and triangulation into one call.  The API is
designed for both quick prototyping and integration into larger
reconstruction pipelines.

Main entry point:

* :func:`reconstruct_lines_stereo`
    Stereo pair triangulation using the native C++ backend.

Usage::

    import cv2
    from lsfm.data import TestImages
    from lsfm.reconstruction import reconstruct_lines_stereo

    ds = TestImages()
    img_l = cv2.imread(str(ds.get("MDB/MiddEval3-H/Adirondack/im0.png")), 0)
    img_r = cv2.imread(str(ds.get("MDB/MiddEval3-H/Adirondack/im1.png")), 0)
    cam_l, cam_r = ds.stereo_camera_pair("Adirondack")

    result = reconstruct_lines_stereo(img_l, img_r, cam_l, cam_r)
    print(f"Reconstructed {result['n_lines_3d']} 3D lines")
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass
class StereoConfig:
    """Configuration for stereo reconstruction.

    All thresholds use sensible defaults; override as needed.

    :param min_length: Minimum 2D segment length in pixels.
    :type min_length: float
    :param ratio_threshold: Lowe's ratio test threshold for matching.
    :type ratio_threshold: float
    :param use_stereo_filter: Use epipolar stereo filter.
    :type use_stereo_filter: bool
    :param use_rotation_filter: Use global rotation consistency filter.
    :type use_rotation_filter: bool
    :param stereo_max_distance: ``StereoLineFilter`` max y-distance (px).
    :type stereo_max_distance: float
    :param stereo_angle_threshold: ``StereoLineFilter`` max angle (deg).
    :type stereo_angle_threshold: float
    :param stereo_min_overlap: ``StereoLineFilter`` min y-overlap ratio.
    :type stereo_min_overlap: float
    :param max_reproj_error: Maximum reprojection error for 3D segments.
    :type max_reproj_error: float
    :param triangulation_method: One of ``"midpoint"``, ``"plane"``,
        ``"opencv"``.
    :type triangulation_method: str
    :param merge_enabled: Enable colinear segment merging.
    :type merge_enabled: bool
    :param merge_angle_threshold: Max angle (deg) for merge candidacy.
    :type merge_angle_threshold: float
    :param merge_distance_threshold: Max perpendicular gap for merging.
    :type merge_distance_threshold: float
    """

    min_length: float = 25.0
    ratio_threshold: float = 0.75
    use_stereo_filter: bool = True
    use_rotation_filter: bool = True
    stereo_max_distance: float = 10000.0
    stereo_angle_threshold: float = 30.0
    stereo_min_overlap: float = 0.3
    max_reproj_error: float = 10.0
    triangulation_method: str = "midpoint"
    merge_enabled: bool = True
    merge_angle_threshold: float = 5.0
    merge_distance_threshold: float = 1.0

    _TRIANGULATION_METHODS = frozenset({"midpoint", "plane", "opencv"})

    def __post_init__(self) -> None:
        """Validate configuration values."""
        if self.min_length <= 0:
            raise ValueError(f"min_length must be positive, got {self.min_length}")
        if not (0.0 < self.ratio_threshold <= 1.0):
            raise ValueError(
                f"ratio_threshold must be in (0, 1], got {self.ratio_threshold}"
            )
        if self.triangulation_method not in self._TRIANGULATION_METHODS:
            raise ValueError(
                f"triangulation_method must be one of {sorted(self._TRIANGULATION_METHODS)},"
                f" got '{self.triangulation_method}'"
            )
        if not (0.0 <= self.stereo_min_overlap <= 1.0):
            raise ValueError(
                f"stereo_min_overlap must be in [0, 1], got {self.stereo_min_overlap}"
            )
        if self.stereo_max_distance <= 0:
            raise ValueError(
                f"stereo_max_distance must be positive, got {self.stereo_max_distance}"
            )
        if self.max_reproj_error <= 0:
            raise ValueError(
                f"max_reproj_error must be positive, got {self.max_reproj_error}"
            )
        if self.merge_angle_threshold < 0:
            raise ValueError(
                f"merge_angle_threshold must be non-negative, got {self.merge_angle_threshold}"
            )
        if self.merge_distance_threshold < 0:
            raise ValueError(
                f"merge_distance_threshold must be non-negative,"
                f" got {self.merge_distance_threshold}"
            )


@dataclass
class StereoResult:
    """Result container for stereo reconstruction.

    :param segments_3d: Reconstructed 3D line segments.
    :type segments_3d: list
    :param segments_left: Detected 2D segments in the left image.
    :type segments_left: list
    :param segments_right: Detected 2D segments in the right image.
    :type segments_right: list
    :param matches: Index pairs ``(left_idx, right_idx)``.
    :type matches: numpy.ndarray
    :param reproj_errors: Per-segment reprojection error.
    :type reproj_errors: numpy.ndarray
    :param n_lines_3d: Number of 3D lines after filtering.
    :type n_lines_3d: int
    :param timings: Timing breakdown in seconds.
    :type timings: dict[str, float]
    :param config: Configuration used.
    :type config: StereoConfig
    """

    segments_3d: list = field(default_factory=list)
    segments_left: list = field(default_factory=list)
    segments_right: list = field(default_factory=list)
    matches: NDArray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.int32))
    reproj_errors: NDArray = field(default_factory=lambda: np.zeros(0))
    n_lines_3d: int = 0
    timings: dict[str, float] = field(default_factory=dict)
    config: StereoConfig = field(default_factory=StereoConfig)


# ---------------------------------------------------------------------------
# Stereo reconstruction
# ---------------------------------------------------------------------------


def reconstruct_lines_stereo(
    img_left: NDArray[np.uint8],
    img_right: NDArray[np.uint8],
    cam_left: object,
    cam_right: object,
    config: StereoConfig | None = None,
) -> StereoResult:
    """End-to-end stereo line reconstruction.

    Runs the full pipeline: LSD detection → LBD descriptors → matching
    (with optional stereo / rotation filtering) → triangulation →
    reprojection error filtering → optional colinear merge.

    :param img_left: Grayscale left image (``uint8``).
    :type img_left: numpy.ndarray
    :param img_right: Grayscale right image (``uint8``).
    :type img_right: numpy.ndarray
    :param cam_left: Left camera (``le_geometry.Camera_f64``).
    :type cam_left: Camera_f64
    :param cam_right: Right camera (``le_geometry.Camera_f64``).
    :type cam_right: Camera_f64
    :param config: Pipeline configuration (uses defaults if ``None``).
    :type config: StereoConfig, optional
    :return: Reconstruction result with 3D segments and diagnostics.
    :rtype: StereoResult

    Example::

        result = reconstruct_lines_stereo(img_l, img_r, cam_l, cam_r)
        for seg3d in result.segments_3d:
            sp = seg3d.start_point()
            ep = seg3d.end_point()
            print(f"  ({sp[0]:.1f}, {sp[1]:.1f}, {sp[2]:.1f}) → "
                  f"({ep[0]:.1f}, {ep[1]:.1f}, {ep[2]:.1f})")
    """
    import le_geometry
    import le_lfd
    import le_lsd

    if config is None:
        config = StereoConfig()

    timings: dict[str, float] = {}
    result = StereoResult(config=config)

    # ---- 1. Detection ----
    t0 = time.perf_counter()
    det_l = le_lsd.LsdCC()
    det_l.detect(img_left)
    segs_l = list(det_l.line_segments())

    det_r = le_lsd.LsdCC()
    det_r.detect(img_right)
    segs_r = list(det_r.line_segments())
    timings["detection"] = time.perf_counter() - t0

    # Filter by minimum length
    segs_l = [s for s in segs_l if s.length > config.min_length]
    segs_r = [s for s in segs_r if s.length > config.min_length]

    result.segments_left = segs_l
    result.segments_right = segs_r

    if not segs_l or not segs_r:
        result.timings = timings
        return result

    # ---- 2. Descriptors ----
    t0 = time.perf_counter()
    gx_l, gy_l = (
        det_l.image_data()[0].astype(np.float32),
        det_l.image_data()[1].astype(np.float32),
    )
    gx_r, gy_r = (
        det_r.image_data()[0].astype(np.float32),
        det_r.image_data()[1].astype(np.float32),
    )

    lbd_l = le_lfd.FdcLBD(gx_l, gy_l)
    desc_l = lbd_l.create_list(segs_l)

    lbd_r = le_lfd.FdcLBD(gx_r, gy_r)
    desc_r = lbd_r.create_list(segs_r)
    timings["descriptors"] = time.perf_counter() - t0

    # ---- 3. Matching ----
    t0 = time.perf_counter()
    matcher = le_lfd.BruteForceLBD()

    if config.use_stereo_filter:
        sf = le_lfd.StereoLineFilter(
            height=img_left.shape[0],
            max_dis_px=config.stereo_max_distance,
            angle_th=config.stereo_angle_threshold,
            min_y_overlap=config.stereo_min_overlap,
        )
        sf.train(segs_l, segs_r)
        matcher.train_filtered_stereo(desc_l, desc_r, sf)
    else:
        matcher.train(desc_l, desc_r)

    best = matcher.best()

    rot_filter = None
    if config.use_rotation_filter:
        rot_filter = le_lfd.GlobalRotationFilter()
        rot_filter.train(segs_l, segs_r)

    pairs: list[tuple[int, int]] = []
    for i, m in enumerate(best):
        if np.isnan(m.distance):
            continue
        if rot_filter is not None and not rot_filter.filter(i, m.match_idx):
            continue
        pairs.append((i, m.match_idx))

    timings["matching"] = time.perf_counter() - t0

    if not pairs:
        result.timings = timings
        return result

    result.matches = np.array(pairs, dtype=np.int32)

    # ---- 4. Triangulation ----
    t0 = time.perf_counter()

    method_map = {
        "midpoint": le_geometry.Stereo_f64,
        "plane": le_geometry.StereoPlane_f64,
        "opencv": le_geometry.StereoCV_f64,
    }
    stereo_cls = method_map.get(config.triangulation_method)
    if stereo_cls is None:
        msg = (
            f"Unknown triangulation method: {config.triangulation_method!r}. "
            f"Choose from: {list(method_map)}"
        )
        raise ValueError(msg)

    stereo = stereo_cls(cam_left, cam_right)

    matched_l = _segments_to_f64([segs_l[il] for il, _ in pairs])
    matched_r = _segments_to_f64([segs_r[ir] for _, ir in pairs])
    segs3d = stereo.triangulate_segments(matched_l, matched_r)
    timings["triangulation"] = time.perf_counter() - t0

    # ---- 5. Reprojection error filter ----
    t0 = time.perf_counter()
    projector = le_geometry.CameraPluecker_f64(cam_left)

    errors: list[float] = []
    good_idx: list[int] = []
    for k, seg3d in enumerate(segs3d):
        proj2d = projector.project_line_segment(seg3d)
        orig2d = matched_l[k]
        # Use center-point distance as error metric
        pc = proj2d.center_point()
        oc = orig2d.center_point()
        err = float(np.sqrt((pc[0] - oc[0]) ** 2 + (pc[1] - oc[1]) ** 2))
        errors.append(err)
        if err < config.max_reproj_error:
            good_idx.append(k)

    result.reproj_errors = np.array(errors)
    filtered_3d = [segs3d[k] for k in good_idx]
    timings["filter"] = time.perf_counter() - t0

    # ---- 6. Optional colinear merge ----
    if config.merge_enabled and len(filtered_3d) > 1:
        t0 = time.perf_counter()
        filtered_3d = _merge_colinear_3d(
            filtered_3d,
            angle_th=config.merge_angle_threshold,
            dist_th=config.merge_distance_threshold,
        )
        timings["merge"] = time.perf_counter() - t0

    result.segments_3d = filtered_3d
    result.n_lines_3d = len(filtered_3d)
    result.timings = timings
    return result


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _segments_to_f64(segments: list) -> list:
    """Convert float-precision ``LineSegment`` to ``LineSegment_f64``.

    The LSD detector returns single-precision line segments while the
    stereo triangulators operate in double precision.  This helper
    bridges the type gap via endpoint conversion.

    :param segments: List of ``LineSegment`` (float) objects.
    :type segments: list
    :return: Equivalent ``LineSegment_f64`` objects.
    :rtype: list
    """
    import le_geometry

    return [
        le_geometry.LineSegment_f64.from_endpoints(*s.end_points()) for s in segments
    ]


def _merge_colinear_3d(
    segments: list,
    *,
    angle_th: float = 5.0,
    dist_th: float = 1.0,
) -> list:
    """Greedily merge near-colinear 3D segments.

    :param segments: List of ``LineSegment3`` objects.
    :type segments: list
    :param angle_th: Maximum angle (degrees) between direction vectors.
    :type angle_th: float
    :param dist_th: Maximum perpendicular distance for merging.
    :type dist_th: float
    :return: Merged list of ``LineSegment3`` objects.
    :rtype: list
    """
    import le_geometry

    if len(segments) <= 1:
        return segments

    cos_th = np.cos(np.radians(angle_th))

    # Extract direction vectors and midpoints
    dirs = []
    mids = []
    for seg in segments:
        sp = np.array(seg.start_point())
        ep = np.array(seg.end_point())
        d = ep - sp
        length = np.linalg.norm(d)
        if length < 1e-12:
            d = np.array([1.0, 0.0, 0.0])
        else:
            d = d / length
        dirs.append(d)
        mids.append((sp + ep) / 2.0)

    merged = list(range(len(segments)))  # union-find parent

    def find(x: int) -> int:
        while merged[x] != x:
            merged[x] = merged[merged[x]]
            x = merged[x]
        return x

    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            if find(i) == find(j):
                continue
            # Check angle
            cos_angle = abs(float(np.dot(dirs[i], dirs[j])))
            if cos_angle < cos_th:
                continue
            # Check perpendicular distance from midpoint j to line i
            diff = mids[j] - mids[i]
            proj = float(np.dot(diff, dirs[i]))
            perp = np.linalg.norm(diff - proj * dirs[i])
            if perp < dist_th:
                merged[find(j)] = find(i)

    # Build merged segments
    groups: dict[int, list[int]] = {}
    for idx in range(len(segments)):
        root = find(idx)
        groups.setdefault(root, []).append(idx)

    result = []
    for members in groups.values():
        if len(members) == 1:
            result.append(segments[members[0]])
            continue

        # Collect all endpoints, project onto principal direction,
        # take the two most extreme points
        all_pts = []
        for idx in members:
            all_pts.append(np.array(segments[idx].start_point()))
            all_pts.append(np.array(segments[idx].end_point()))

        principal = dirs[members[0]]
        projections = [float(np.dot(p, principal)) for p in all_pts]
        i_min = int(np.argmin(projections))
        i_max = int(np.argmax(projections))

        p1 = all_pts[i_min]
        p2 = all_pts[i_max]

        merged_seg = le_geometry.LineSegment3.from_endpoints(
            float(p1[0]),
            float(p1[1]),
            float(p1[2]),
            float(p2[0]),
            float(p2[1]),
            float(p2[2]),
        )
        result.append(merged_seg)

    return result
