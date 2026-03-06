"""High-level 3D line reconstruction API.

Provides simple functions that combine line detection, descriptor
extraction, matching, and triangulation into one call.  The API is
designed for both quick prototyping and integration into larger
reconstruction pipelines.

Two main entry points:

* :func:`reconstruct_lines_stereo`
    Stereo pair triangulation using the native C++ backend.

* :func:`reconstruct_lines_multiview`
    Multi-view reconstruction via LIMAP (optional dependency).

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
from typing import TYPE_CHECKING, Any, Sequence

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
# Multi-view reconstruction (LIMAP)
# ---------------------------------------------------------------------------


def reconstruct_lines_multiview(
    images: Sequence[NDArray[np.uint8]],
    cameras: Sequence[dict[str, Any]],
    *,
    output_dir: str | None = None,
    limap_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Multi-view 3D line reconstruction via LIMAP.

    Requires ``limap`` to be installed.  This function detects line
    segments with LIMAP's LSD detector, matches them across views
    using epipolar IoU, and triangulates matched pairs using LIMAP's
    two-view line triangulation.

    :param images: Sequence of grayscale images.
    :type images: Sequence[numpy.ndarray]
    :param cameras: Camera parameter dicts. Each must have ``"K"``
        (3x3 intrinsics), ``"R"`` (3x3 rotation), ``"t"`` (3-vector
        **camera center in world coordinates**), and optionally
        ``"width"``/``"height"``.
    :type cameras: Sequence[dict]
    :param output_dir: Directory for intermediate LIMAP results.  If
        *None*, a temporary directory is created and cleaned up.
    :type output_dir: str, optional
    :param limap_config: Override dict merged into the default config
        (controls detector settings, matching thresholds, etc.).
    :type limap_config: dict, optional
    :return: Dict with ``"lines_3d"`` (list of Nx6 arrays for start/end
        3D points), ``"n_lines_3d"`` (count), and ``"timings"``.
    :rtype: dict
    :raises ImportError: If ``limap`` is not installed.

    .. note::
        This is a thin wrapper around LIMAP's lower-level API.  For
        advanced usage (bundle adjustment, line-to-surface association),
        call LIMAP directly and use :mod:`lsfm.limap_compat` for format
        conversion.
    """
    import os
    import tempfile

    try:
        import limap  # noqa: F401
        import limap.line2d  # type: ignore[import-untyped]
        from limap.base import CameraView  # type: ignore[import-untyped]
        from limap.base import Line2d as LLine2d
        from limap.triangulation import (  # type: ignore[import-untyped]
            compute_epipolar_IoU,
            triangulate_line_by_endpoints,
        )
    except ImportError as exc:
        msg = (
            "CVG LIMAP is required for multi-view reconstruction but is "
            "not installed.  Install from source with:\n"
            "  uv sync --extra limap\n"
            "or manually:\n"
            "  git clone --recursive https://github.com/cvg/limap.git\n"
            "  pip install -Ive ./limap\n"
            "NOTE: The PyPI 'limap' package is an unrelated project. "
            "Requires Python 3.10/3.11 and CMake >= 3.17."
        )
        raise ImportError(msg) from exc

    # Verify this is the real CVG LIMAP, not the unrelated PyPI package.
    if not hasattr(limap, "base"):
        msg = (
            "The installed 'limap' package is not CVG LIMAP "
            "(https://github.com/cvg/limap). You may have the unrelated "
            "PyPI package installed. Uninstall it and install the real one:\n"
            "  pip uninstall limap\n"
            "  uv sync --extra limap"
        )
        raise ImportError(msg)

    import cv2
    from limap import runners  # type: ignore[import-untyped]

    timings: dict[str, float] = {}

    # ---- 1. Write images to disk (LIMAP detector reads from files) ----
    use_tmpdir = output_dir is None
    tmpdir = tempfile.mkdtemp(prefix="limap_") if use_tmpdir else None
    work_dir = tmpdir if use_tmpdir else output_dir
    assert work_dir is not None

    t0 = time.perf_counter()
    img_dir = os.path.join(work_dir, "images")
    os.makedirs(img_dir, exist_ok=True)
    img_paths: list[str] = []
    for idx, img in enumerate(images):
        p = os.path.join(img_dir, f"image_{idx:04d}.png")
        cv2.imwrite(p, img)
        img_paths.append(p)

    imagecols = _build_limap_imagecols(cameras, images, img_paths)
    timings["setup"] = time.perf_counter() - t0

    # ---- 2. Build config & detect segments ----
    cfg = _build_limap_default_config(work_dir)
    if limap_config:
        _deep_update(cfg, limap_config)
    cfg = runners.setup(cfg)

    if cfg["max_image_dim"] != -1 and cfg["max_image_dim"] is not None:
        imagecols.set_max_image_dim(cfg["max_image_dim"])

    t0 = time.perf_counter()
    detector = limap.line2d.get_detector(
        cfg["line2d"]["detector"],
        max_num_2d_segs=cfg["line2d"]["max_num_2d_segs"],
        do_merge_lines=cfg["line2d"]["do_merge_lines"],
        visualize=cfg["line2d"]["visualize"],
    )
    folder_save = os.path.join(
        cfg["dir_save"], "line_detections", cfg["line2d"]["detector"]["method"]
    )
    all_2d_segs = detector.detect_all_images(folder_save, imagecols, skip_exists=False)
    timings["detection"] = time.perf_counter() - t0

    # ---- 3. Build camera views for triangulation ----
    views: dict[int, CameraView] = {}
    for img_id in imagecols.get_img_ids():
        cam = imagecols.cam(img_id)
        pose = imagecols.campose(img_id)
        views[img_id] = CameraView(cam, pose)

    # ---- 4. Match + triangulate across all neighbor pairs ----
    t0 = time.perf_counter()
    iou_threshold = 0.3
    max_reproj_error = 5.0
    # Maximum reliable depth: beyond this the triangulation angle is
    # too shallow (< ~0.5 deg) for useful results.  Computed per-pair
    # from focal length and baseline.
    lines_3d_list: list[NDArray] = []
    img_ids = imagecols.get_img_ids()

    # Process all pairs (avoid duplicates)
    processed_pairs: set[tuple[int, int]] = set()
    for i in img_ids:
        for j in img_ids:
            if i >= j:
                continue
            pair = (i, j)
            if pair in processed_pairs:
                continue
            processed_pairs.add(pair)

            segs_i = all_2d_segs[i]  # (N_i, 4)
            segs_j = all_2d_segs[j]  # (N_j, 4)
            view_i = views[i]
            view_j = views[j]

            # Compute maximum reliable depth from camera geometry.
            # Require at least min_parallax_deg convergence angle.
            center_i = imagecols.campose(i).center()
            center_j = imagecols.campose(j).center()
            baseline = float(np.linalg.norm(np.array(center_i) - np.array(center_j)))
            min_parallax_deg = 0.5
            if baseline > 0:
                max_depth = baseline / np.tan(np.radians(min_parallax_deg))
            else:
                max_depth = float("inf")

            # Convert to Line2d objects
            lines_i = [
                LLine2d(np.array([s[0], s[1]]), np.array([s[2], s[3]])) for s in segs_i
            ]
            lines_j = [
                LLine2d(np.array([s[0], s[1]]), np.array([s[2], s[3]])) for s in segs_j
            ]

            n_i = len(lines_i)
            n_j = len(lines_j)

            # Build IoU matrix and find mutual best matches
            best_j_for_i: list[tuple[int, float]] = [(-1, 0.0)] * n_i
            best_i_for_j: list[tuple[int, float]] = [(-1, 0.0)] * n_j

            for li_idx, li in enumerate(lines_i):
                for lj_idx, lj in enumerate(lines_j):
                    iou = compute_epipolar_IoU(li, view_i, lj, view_j)
                    if iou > iou_threshold:
                        if iou > best_j_for_i[li_idx][1]:
                            best_j_for_i[li_idx] = (lj_idx, iou)
                        if iou > best_i_for_j[lj_idx][1]:
                            best_i_for_j[lj_idx] = (li_idx, iou)

            # Keep only mutual best matches (cross-check)
            for li_idx in range(n_i):
                lj_idx, iou_val = best_j_for_i[li_idx]
                if lj_idx < 0:
                    continue
                # Verify mutual: j's best match should be i
                if best_i_for_j[lj_idx][0] != li_idx:
                    continue

                li = lines_i[li_idx]
                lj = lines_j[lj_idx]

                line3d = triangulate_line_by_endpoints(li, view_i, lj, view_j)
                s = line3d.start
                e = line3d.end

                # Filter invalid results: default [0,0,0]->[1,1,1]
                # and lines with non-positive depth.
                if line3d.length() <= 0:
                    continue
                if np.allclose(s, [0, 0, 0]) and np.allclose(e, [1, 1, 1]):
                    continue
                if s[2] <= 0 or e[2] <= 0:
                    continue

                # Depth filter: reject if either endpoint is deeper
                # than the max reliable depth for this camera pair.
                if s[2] > max_depth or e[2] > max_depth:
                    continue

                # Reprojection error filter: project 3D endpoints back
                # and compare with original 2D segment endpoints.
                proj_i = line3d.projection(view_i)
                err_i_s = float(
                    np.linalg.norm(np.array(proj_i.start) - np.array(li.start))
                )
                err_i_e = float(np.linalg.norm(np.array(proj_i.end) - np.array(li.end)))

                proj_j = line3d.projection(view_j)
                err_j_s = float(
                    np.linalg.norm(np.array(proj_j.start) - np.array(lj.start))
                )
                err_j_e = float(np.linalg.norm(np.array(proj_j.end) - np.array(lj.end)))

                if max(err_i_s, err_i_e, err_j_s, err_j_e) > max_reproj_error:
                    continue

                lines_3d_list.append(np.array([[s[0], s[1], s[2]], [e[0], e[1], e[2]]]))
    timings["matching_and_triangulation"] = time.perf_counter() - t0

    # ---- 5. Cleanup ----
    if use_tmpdir and tmpdir is not None:
        import shutil

        shutil.rmtree(tmpdir, ignore_errors=True)

    return {
        "lines_3d": lines_3d_list,
        "n_lines_3d": len(lines_3d_list),
        "timings": timings,
    }


# ---------------------------------------------------------------------------
# Full multi-view reconstruction via LIMAP runner
# ---------------------------------------------------------------------------


def reconstruct_lines_multiview_full(
    imagecols: Any,
    neighbors: dict[int, list[int]] | None = None,
    ranges: Any | None = None,
    *,
    output_dir: str | None = None,
    max_image_dim: int = 1600,
    n_neighbors: int = 20,
    n_visible_views: int = 4,
    use_exhaustive_matcher: bool = True,
    detector_class: str | None = None,
    limap_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Full multi-view 3D line reconstruction via LIMAP's ``line_triangulation`` runner.

    Unlike :func:`reconstruct_lines_multiview` (which does pairwise
    triangulation), this function uses LIMAP's complete pipeline:
    multi-view triangulation with track building, reprojection
    filtering, remerging, and optional bundle adjustment.

    :param imagecols: A ``limap.base.ImageCollection`` with camera
        poses and image paths.  Use ``limap.pointsfm.read_infos_colmap()``
        or build manually.
    :type imagecols: limap.base.ImageCollection
    :param neighbors: Per-image neighbor lists.  If *None*, LIMAP
        computes them from COLMAP covisibility.
    :type neighbors: dict[int, list[int]], optional
    :param ranges: Scene bounding ranges ``(min_xyz, max_xyz)``.  If
        *None*, LIMAP computes them automatically.
    :type ranges: pair of numpy arrays, optional
    :param output_dir: Working directory for LIMAP outputs.  A
        temporary directory is used if *None*.
    :type output_dir: str, optional
    :param max_image_dim: Maximum image dimension (longest side).
        ETH3D images are ~6200px; 1600 is a good balance.
    :type max_image_dim: int
    :param n_neighbors: Number of visual neighbors per image.
    :type n_neighbors: int
    :param n_visible_views: Minimum views a track must appear in.
    :type n_visible_views: int
    :param use_exhaustive_matcher: Use exhaustive epipolar matching
        (no descriptor matching needed).
    :type use_exhaustive_matcher: bool
    :param detector_class: Detector to use.  ``None`` for LIMAP's
        built-in LSD (pytlsd), ``"lsfm_lsd"`` for our native detector.
    :type detector_class: str, optional
    :param limap_config: Override dict merged into the default config.
    :type limap_config: dict, optional
    :return: Dict with ``"linetracks"`` (list of ``LineTrack``),
        ``"n_tracks"`` (count), ``"track_lengths"`` (list of segment
        counts per track), ``"supporting_views"`` (list of view counts
        per track), and ``"timings"``.
    :rtype: dict
    :raises ImportError: If ``limap`` is not installed.
    """
    import tempfile

    from lsfm.limap_compat import check_limap_available

    check_limap_available()

    import limap.runners as runners  # type: ignore[import-untyped]
    from limap.runners import line_triangulation as _line_triangulation  # type: ignore[import-untyped]

    timings: dict[str, float] = {}

    # ---- 1. Build config ----
    use_tmpdir = output_dir is None
    tmpdir = tempfile.mkdtemp(prefix="limap_mv_") if use_tmpdir else None
    work_dir = tmpdir if use_tmpdir else output_dir
    assert work_dir is not None

    t0 = time.perf_counter()
    cfg = _build_limap_default_config(work_dir)
    cfg["max_image_dim"] = max_image_dim
    cfg["n_neighbors"] = n_neighbors
    cfg["n_visible_views"] = n_visible_views
    cfg["triangulation"]["use_exhaustive_matcher"] = use_exhaustive_matcher

    # Inject our detector if requested
    if detector_class == "lsfm_lsd":
        cfg["line2d"]["detector"]["method"] = "lsfm_lsd"

    if limap_config:
        _deep_update(cfg, limap_config)

    cfg = runners.setup(cfg)

    # ---- 2. Monkey-patch detector registry if using our detector ----
    if detector_class == "lsfm_lsd":
        import limap.line2d  # type: ignore[import-untyped]
        from lsfm.limap_compat import LsfmLimapDetector

        _orig_get_detector = limap.line2d.get_detector

        def _patched_get_detector(detector_cfg: dict[str, Any], **kwargs: Any) -> Any:
            if detector_cfg.get("method") == "lsfm_lsd":
                return LsfmLimapDetector(
                    max_num_2d_segs=kwargs.get("max_num_2d_segs", 3000),
                    do_merge_lines=kwargs.get("do_merge_lines", False),
                    visualize=kwargs.get("visualize", False),
                )
            return _orig_get_detector(detector_cfg, **kwargs)

        limap.line2d.get_detector = _patched_get_detector
    timings["setup"] = time.perf_counter() - t0

    # ---- 3. Run full pipeline ----
    t0 = time.perf_counter()
    try:
        linetracks = _line_triangulation(
            cfg, imagecols, neighbors=neighbors, ranges=ranges
        )
    finally:
        # Restore original detector registry
        if detector_class == "lsfm_lsd":
            limap.line2d.get_detector = _orig_get_detector  # type: ignore[possibly-undefined]
    timings["pipeline"] = time.perf_counter() - t0

    # ---- 4. Extract statistics ----
    track_lengths = [track.count_lines() for track in linetracks]
    supporting_views = [track.count_images() for track in linetracks]

    result: dict[str, Any] = {
        "linetracks": linetracks,
        "n_tracks": len(linetracks),
        "track_lengths": track_lengths,
        "supporting_views": supporting_views,
        "timings": timings,
    }

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


def _build_limap_imagecols(
    cameras: Sequence[dict[str, Any]],
    images: Sequence[NDArray[np.uint8]],
    img_paths: Sequence[str],
) -> Any:
    """Build a LIMAP ``ImageCollection`` from camera dicts and images.

    :param cameras: Per-image camera parameter dicts.  Each must have
        ``"K"`` (3x3 intrinsics) and may include ``"R"`` (3x3 rotation,
        default identity) and ``"t"`` (3-vector **camera center in world
        coordinates**, default origin).
    :type cameras: Sequence[dict]
    :param images: Images (used to infer dimensions).
    :type images: Sequence[numpy.ndarray]
    :param img_paths: Absolute file paths for each image (LIMAP reads
        images from disk via ``imagecols.read_image``).
    :type img_paths: Sequence[str]
    :return: ``limap.base.ImageCollection`` instance.
    :rtype: limap.base.ImageCollection
    """
    from limap.base import Camera as LCamera  # type: ignore[import-untyped]
    from limap.base import CameraImage as LImage  # type: ignore[import-untyped]
    from limap.base import CameraPose as LPose  # type: ignore[import-untyped]
    from limap.base import ImageCollection  # type: ignore[import-untyped]

    lcameras = []
    limages = []

    for idx, cam_dict in enumerate(cameras):
        K = np.asarray(cam_dict["K"], dtype=np.float64)
        h = cam_dict.get("height", images[idx].shape[0])
        w = cam_dict.get("width", images[idx].shape[1])

        lcam = LCamera("PINHOLE", K, cam_id=idx, hw=(h, w))
        lcameras.append(lcam)

        R = np.asarray(cam_dict.get("R", np.eye(3)), dtype=np.float64)
        # cam_dict["t"] is the camera center in world coordinates.
        # LIMAP CameraPose expects t_cam = -R @ center.
        center = np.asarray(cam_dict.get("t", np.zeros(3)), dtype=np.float64)
        t_cam = -R @ center
        pose = LPose(R, t_cam)
        limages.append(LImage(lcam, pose, img_paths[idx]))

    return ImageCollection(lcameras, limages)


def _build_limap_default_config(work_dir: str) -> dict[str, Any]:
    """Build a minimal default LIMAP config for LSD + LBD triangulation.

    This replicates the essential keys from LIMAP's
    ``cfgs/triangulation/default.yaml`` with conservative settings and
    classical (non-deep-learning) detector/descriptor/matcher.

    :param work_dir: Working directory for intermediate LIMAP outputs.
    :type work_dir: str
    :return: LIMAP configuration dict.
    :rtype: dict
    """
    import os

    return {
        "output_dir": os.path.join(work_dir, "limap_output"),
        "output_folder": "finaltracks",
        "load_dir": "",
        "use_tmp": False,
        "max_image_dim": -1,
        "skip_exists": False,
        "visualize": False,
        "n_neighbors": 20,
        "n_jobs": 1,
        "n_visible_views": 4,
        "load_det": False,
        "load_match": False,
        "load_undistort": False,
        "undistortion_output_dir": "undistorted",
        "var2d": {
            "sold2": 5.0,
            "lsd": 2.0,
            "deeplsd": 2.0,
            "tp_lsd": 2.0,
            "hatp": 5.0,
            "default": 5.0,
        },
        "line2d": {
            "max_num_2d_segs": 3000,
            "do_merge_lines": False,
            "visualize": False,
            "save_l3dpp": False,
            "compute_descinfo": False,
            "detector": {
                "method": "lsd",
                "skip_exists": False,
            },
            "extractor": {
                "method": "lbd",
                "skip_exists": False,
            },
            "matcher": {
                "method": "lbd",
                "n_jobs": 1,
                "topk": 10,
                "skip_exists": False,
                "superglue": {
                    "weights": "outdoor",
                },
            },
        },
        "triangulation": {
            "method": "triangulate_scoring",
            "var2d": -1,
            "add_halfpix": False,
            "fullscore_th": -1,
            "max_valid_conns": -1,
            "min_num_outer_edges": -1,
            "merging_strategy": "greedy",
            "num_outliers_aggregator": 2,
            "use_vp": False,
            "vpdet_config": {
                "method": "jlinkage",
                "n_jobs": -1,
            },
            "use_exhaustive_matcher": True,
            "use_pointsfm": {
                "enable": False,
                "colmap_folder": None,
                "reuse_sfminfos_colmap": True,
                "use_triangulated_points": False,
                "use_neighbors": True,
            },
            "linker2d": {
                "score_th": 0.5,
                "th_angle": 10.0,
                "th_perp": 5.0,
            },
            "linker3d": {
                "score_th": 0.5,
                "th_angle": 5.0,
                "th_overlap": 0.05,
                "th_smartangle": 10.0,
                "th_smartoverlap": 0.05,
                "th_perp": 10.0,
                "th_innerjunc": 10.0,
            },
            "filtering2d": {
                "th_angular_2d": 10.0,
                "th_perp_2d": 2.0,
                "th_sv_angular_3d": 2.0,
                "th_sv_num_supports": 4,
                "th_overlap": 0.1,
                "th_overlap_num_supports": 4,
            },
            "remerging": {
                "disable": False,
                "linker3d": {
                    "score_th": 0.5,
                    "th_angle": 3.0,
                    "th_overlap": 0.02,
                    "th_smartangle": 5.0,
                    "th_smartoverlap": 0.02,
                    "th_perp": 5.0,
                    "th_innerjunc": 5.0,
                },
            },
        },
        "refinement": {
            "disable": True,
        },
        "sfm": {},
        "structures": {
            "bpt2d": {},
        },
    }


def _deep_update(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* in place.

    :param base: Base dict to update.
    :type base: dict
    :param override: Values to merge in.
    :type override: dict
    :return: The updated *base*.
    :rtype: dict
    """
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base
