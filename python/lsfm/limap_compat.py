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


# ---------------------------------------------------------------------------
# LIMAP adapter classes
# ---------------------------------------------------------------------------
# These classes conform to LIMAP's expected interfaces for line detection,
# descriptor extraction, and matching.  They allow LIMAP's multi-view
# reconstruction pipeline (``limap.runners``) to use our native C++
# implementations transparently.
#
# LIMAP is an *optional* dependency.  The adapters work standalone —
# they only need ``le_lsd``, ``le_lfd``, and ``le_geometry`` on sys.path.
# ---------------------------------------------------------------------------

_LIMAP_INSTALL_MSG = (
    "CVG LIMAP (https://github.com/cvg/limap) is not installed.\n"
    "Install from source with:\n"
    "  uv sync --extra limap\n"
    "or manually:\n"
    "  git clone --recursive https://github.com/cvg/limap.git\n"
    "  pip install -Ive ./limap\n"
    "NOTE: The PyPI 'limap' package is an unrelated project. "
    "Requires Python 3.10/3.11 and CMake >= 3.17."
)


def check_limap_available() -> None:
    """Verify that CVG LIMAP is installed and is the correct package.

    :raises ImportError: If LIMAP is missing or the wrong package is
        installed (e.g. the unrelated PyPI ``limap``).
    """
    try:
        import limap  # noqa: F401
    except ImportError as exc:
        raise ImportError(_LIMAP_INSTALL_MSG) from exc

    if not hasattr(limap, "base"):
        msg = (
            "The installed 'limap' package is not CVG LIMAP "
            "(https://github.com/cvg/limap). You may have the unrelated "
            "PyPI package installed. Uninstall it and install the real one:\n"
            "  pip uninstall limap\n"
            "  uv sync --extra limap"
        )
        raise ImportError(msg)


class LsfmLineDetector:
    """LIMAP-compatible line segment detector backed by ``le_lsd``.

    Wraps any LSD detector variant (default: ``LsdCC``) so that it can
    be plugged into LIMAP's ``LineDetector`` interface.

    Usage::

        from lsfm.limap_compat import LsfmLineDetector
        det = LsfmLineDetector()          # uses LsdCC
        segs = det.detect(image_gray)     # → (N, 5) ndarray

    :param detector_cls: Name of the ``le_lsd`` detector class.
    :type detector_cls: str
    :param kwargs: Keyword arguments forwarded to the detector constructor.
    """

    def __init__(
        self,
        detector_cls: str = "LsdCC",
        **kwargs: object,
    ) -> None:
        import le_lsd as _le_lsd  # noqa: N812

        cls = getattr(_le_lsd, detector_cls)
        self._det = cls(**kwargs)
        self._le_lsd = _le_lsd

    def detect(self, image: NDArray[np.uint8]) -> NDArray[np.float64]:
        """Detect line segments and return them in LIMAP ``(N, 5)`` format.

        :param image: Grayscale ``uint8`` image.
        :type image: numpy.ndarray
        :return: Segments as ``[x1, y1, x2, y2, score]`` rows.
        :rtype: numpy.ndarray of shape ``(N, 5)``
        """
        self._det.detect(image)
        segs = self._det.line_segments()
        return segments_to_limap(segs)

    @property
    def line_segments(self) -> list[object]:
        """Return raw ``LineSegment`` objects from the last detection.

        :return: List of line segments.
        :rtype: list[LineSegment]
        """
        return list(self._det.line_segments())

    @property
    def image_data(self) -> list[NDArray]:
        """Return the detector's image data (gradients, maps).

        :return: List of image data arrays ``[gx, gy, mag, edge, seg]``.
        :rtype: list[numpy.ndarray]
        """
        return list(self._det.image_data())


class LsfmLineDescriptor:
    """LIMAP-compatible LBD descriptor extractor backed by ``le_lfd``.

    Wraps ``FdcLBD`` so that it can be used where LIMAP expects a
    descriptor extractor.

    Usage::

        from lsfm.limap_compat import LsfmLineDescriptor
        desc = LsfmLineDescriptor()
        desc_info = desc.compute(image_gray, segments)  # → dict

    :param num_band: Number of bands for the LBD descriptor.
    :type num_band: int
    :param width_band: Width of each band.
    :type width_band: int
    """

    def __init__(
        self,
        num_band: int = 9,
        width_band: int = 7,
    ) -> None:
        self._num_band = num_band
        self._width_band = width_band

    def compute(
        self,
        image: NDArray[np.uint8],
        segments: Sequence[object] | NDArray[np.float64],
        *,
        num_octaves: int = 5,
    ) -> dict[str, object]:
        """Compute LBD descriptors and return LIMAP ``descinfo`` dict.

        Can accept either raw ``LineSegment`` objects or an ``(N, 5)``
        LIMAP segment array (in which case segments are converted back
        to ``LineSegment`` objects internally).

        :param image: Grayscale ``uint8`` image.
        :type image: numpy.ndarray
        :param segments: Line segments — either ``LineSegment`` list or
            ``(N, 5)`` LIMAP array.
        :type segments: Sequence[LineSegment] or numpy.ndarray
        :param num_octaves: Pyramid octaves for ``ms_lines``.
        :type num_octaves: int
        :return: LIMAP-compatible descriptor info dict with keys
            ``"ms_lines"`` and ``"line_descriptors"``.
        :rtype: dict
        """
        import le_lfd as _le_lfd  # noqa: N812
        import le_geometry as _le_geometry  # noqa: N812
        import le_lsd as _le_lsd  # noqa: N812

        # Compute Sobel gradients for LBD
        det = _le_lsd.LsdCC()
        det.detect(image)
        gx = det.image_data()[0].astype(np.float32)
        gy = det.image_data()[1].astype(np.float32)

        # Convert LIMAP array to LineSegment objects if needed
        seg_objs: Sequence[object]
        if isinstance(segments, np.ndarray) and segments.ndim == 2:
            seg_objs = [
                _le_geometry.LineSegment.from_endpoints(
                    float(row[0]),
                    float(row[1]),
                    float(row[2]),
                    float(row[3]),
                )
                for row in segments
            ]
        else:
            seg_objs = segments

        creator = _le_lfd.FdcLBD(
            gx,
            gy,
            num_band=self._num_band,
            width_band=self._width_band,
        )
        return descriptors_to_limap(
            creator,
            seg_objs,
            num_octaves=num_octaves,
        )

    @property
    def dimension(self) -> int:
        """Descriptor dimension (depends on ``num_band``).

        :return: Descriptor vector length.
        :rtype: int
        """
        return self._num_band * 8


class LsfmLineMatcher:
    """LIMAP-compatible brute-force line matcher backed by ``le_lfd``.

    Wraps ``BruteForceLBD`` with optional stereo and rotation filtering.

    Usage::

        from lsfm.limap_compat import LsfmLineMatcher
        matcher = LsfmLineMatcher()
        match_pairs = matcher.match(desc_info_1, desc_info_2)  # (M, 2)

    :param use_stereo_filter: Enable ``StereoLineFilter``.
    :type use_stereo_filter: bool
    :param use_rotation_filter: Enable ``GlobalRotationFilter``.
    :type use_rotation_filter: bool
    :param stereo_kwargs: Additional keyword arguments for
        ``StereoLineFilter`` (e.g. ``angle_th``, ``min_y_overlap``).
    :type stereo_kwargs: dict
    """

    def __init__(
        self,
        *,
        use_stereo_filter: bool = False,
        use_rotation_filter: bool = False,
        **stereo_kwargs: object,
    ) -> None:
        self._use_stereo = use_stereo_filter
        self._use_rotation = use_rotation_filter
        self._stereo_kwargs = stereo_kwargs

    def match(
        self,
        desc_info_1: dict[str, object],
        desc_info_2: dict[str, object],
        *,
        segments_1: Sequence[object] | None = None,
        segments_2: Sequence[object] | None = None,
        image_height: int = 0,
    ) -> NDArray[np.int32]:
        """Match descriptors and return ``(M, 2)`` index pairs.

        :param desc_info_1: Descriptor info dict for the first image
            (from :meth:`LsfmLineDescriptor.compute`).
        :type desc_info_1: dict
        :param desc_info_2: Descriptor info dict for the second image.
        :type desc_info_2: dict
        :param segments_1: Line segments of the first image (required
            if stereo or rotation filter is enabled).
        :type segments_1: Sequence[LineSegment], optional
        :param segments_2: Line segments of the second image.
        :type segments_2: Sequence[LineSegment], optional
        :param image_height: Image height in pixels (for stereo filter).
        :type image_height: int
        :return: Match index pairs.
        :rtype: numpy.ndarray of shape ``(M, 2)``
        """
        import le_lfd as _le_lfd  # noqa: N812

        mat1 = np.asarray(
            desc_info_1["line_descriptors"],
            dtype=np.float32,
        )
        mat2 = np.asarray(
            desc_info_2["line_descriptors"],
            dtype=np.float32,
        )

        # Use the raw brute-force matching on descriptor matrices
        matcher = _le_lfd.BruteForceLBD()

        # If stereo filter requested + segments available, use filtered
        if (
            self._use_stereo
            and segments_1 is not None
            and segments_2 is not None
            and image_height > 0
        ):
            sf = _le_lfd.StereoLineFilter(
                height=image_height,
                **self._stereo_kwargs,
            )
            sf.train(segments_1, segments_2)
            # Need descriptor lists for filtered matching
            desc_list_1 = _list_from_mat(mat1, _le_lfd)
            desc_list_2 = _list_from_mat(mat2, _le_lfd)
            matcher.train_filtered_stereo(desc_list_1, desc_list_2, sf)
        else:
            desc_list_1 = _list_from_mat(mat1, _le_lfd)
            desc_list_2 = _list_from_mat(mat2, _le_lfd)
            matcher.train(desc_list_1, desc_list_2)

        best = matcher.best()

        # Convert to index pairs, apply rotation filter if requested
        rot_filter = None
        if self._use_rotation and segments_1 is not None and segments_2 is not None:
            rot_filter = _le_lfd.GlobalRotationFilter()
            rot_filter.train(segments_1, segments_2)

        pairs: list[list[int]] = []
        for i, m in enumerate(best):
            if np.isnan(m.distance):
                continue
            if rot_filter is not None and not rot_filter.filter(
                i,
                m.match_idx,
            ):
                continue
            pairs.append([i, m.match_idx])

        if not pairs:
            return np.zeros((0, 2), dtype=np.int32)
        return np.array(pairs, dtype=np.int32)


def _list_from_mat(
    mat: NDArray[np.floating],
    le_lfd_module: object,
) -> list[object]:
    """Create a list of FdLBD descriptors from a matrix.

    Constructs descriptors by creating a dummy FdcLBD and using
    create_list with synthetic segments, then replacing data.
    Falls back to raw matrix matching if not possible.

    :param mat: Descriptor matrix of shape ``(N, D)``.
    :type mat: numpy.ndarray
    :param le_lfd_module: The ``le_lfd`` module.
    :type le_lfd_module: module
    :return: List of descriptor objects.
    :rtype: list
    """
    # For BruteForceLBD.train(), we need FdLBD descriptor objects.
    # Use the static best_match_mat approach if available, otherwise
    # create descriptors via FdcLBD from a dummy gradient image.
    n = mat.shape[0]
    if n == 0:
        return []

    # Create matching-compatible descriptors from the matrix
    # by using a tiny gradient image and replacing descriptor data
    import le_geometry as _le_geometry  # noqa: N812

    h, w = 4, max(mat.shape[1] + 20, 100)
    dummy_gx = np.zeros((h, w), dtype=np.float32)
    dummy_gy = np.ones((h, w), dtype=np.float32)

    # Create one segment per descriptor
    segs = [
        _le_geometry.LineSegment.from_endpoints(0.0, 1.0, float(w - 1), 1.0)
        for _ in range(n)
    ]

    creator = le_lfd_module.FdcLBD(dummy_gx, dummy_gy)
    desc_list = creator.create_list(segs)

    # Replace descriptor data with actual values
    for i, desc in enumerate(desc_list):
        desc.data[:] = mat[i]

    return desc_list


def cameras_to_limap(
    cam_left: object,
    cam_right: object,
    baseline: float,
) -> dict[str, object]:
    """Convert our Camera_f64 pair to LIMAP camera/pose dicts.

    :param cam_left: Left camera (``le_geometry.Camera_f64``).
    :type cam_left: Camera_f64
    :param cam_right: Right camera (``le_geometry.Camera_f64``).
    :type cam_right: Camera_f64
    :param baseline: Stereo baseline in world units.
    :type baseline: float
    :return: Dict with keys ``"cameras"`` (list of cam dicts) and
        ``"poses"`` (list of ``[R, t]`` pairs as numpy arrays).
    :rtype: dict
    """
    fx_l, fy_l = cam_left.focal  # type: ignore[attr-defined]
    cx_l, cy_l = cam_left.offset  # type: ignore[attr-defined]
    w = cam_left.width  # type: ignore[attr-defined]
    h = cam_left.height  # type: ignore[attr-defined]

    K_left = np.array(
        [
            [fx_l, 0, cx_l],
            [0, fy_l, cy_l],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )

    fx_r, fy_r = cam_right.focal  # type: ignore[attr-defined]
    cx_r, cy_r = cam_right.offset  # type: ignore[attr-defined]

    K_right = np.array(
        [
            [fx_r, 0, cx_r],
            [0, fy_r, cy_r],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )

    R_left = np.eye(3, dtype=np.float64)
    t_left = np.zeros(3, dtype=np.float64)
    R_right = np.eye(3, dtype=np.float64)
    t_right = np.array([baseline, 0.0, 0.0], dtype=np.float64)

    return {
        "cameras": [
            {"K": K_left, "width": w, "height": h},
            {"K": K_right, "width": w, "height": h},
        ],
        "poses": [
            [R_left, t_left],
            [R_right, t_right],
        ],
    }


def segments3d_to_limap(
    segs3d: Sequence[object],
) -> NDArray[np.float64]:
    """Convert 3D line segments to LIMAP ``(N, 6)`` endpoint format.

    Each row is ``[x1, y1, z1, x2, y2, z2]``.

    :param segs3d: 3D line segments (``LineSegment3`` objects).
    :type segs3d: Sequence[LineSegment3]
    :return: Segment endpoint array.
    :rtype: numpy.ndarray of shape ``(N, 6)``
    """
    if not segs3d:
        return np.zeros((0, 6), dtype=np.float64)

    rows = np.empty((len(segs3d), 6), dtype=np.float64)
    for i, seg in enumerate(segs3d):
        sp = seg.start_point()  # type: ignore[attr-defined]
        ep = seg.end_point()  # type: ignore[attr-defined]
        rows[i] = (*sp, *ep)
    return rows


# ---------------------------------------------------------------------------
# LIMAP BaseDetector subclass
# ---------------------------------------------------------------------------


class LsfmLimapDetector:
    """LIMAP ``BaseDetector`` subclass backed by our native LSD detector.

    Integrates with LIMAP's full multi-view pipeline
    (``limap.runners.line_triangulation``) by implementing the
    ``detect(camview)`` and ``get_module_name()`` interface that
    ``BaseDetector.detect_all_images()`` calls.

    The class lazily inherits from ``limap.line2d.base_detector.BaseDetector``
    at instantiation time so the module works even when LIMAP is not
    installed (import guarded).

    Usage::

        from lsfm.limap_compat import LsfmLimapDetector
        detector = LsfmLimapDetector()
        # Use like any LIMAP detector:
        all_segs = detector.detect_all_images(folder, imagecols)

    :param detector_cls: Name of the ``le_lsd`` detector class.
    :type detector_cls: str
    :param max_num_2d_segs: Maximum number of segments to keep (longest).
    :type max_num_2d_segs: int
    :param do_merge_lines: Whether to merge close similar lines.
    :type do_merge_lines: bool
    :param visualize: Whether to output detection visualizations.
    :type visualize: bool
    :param kwargs: Extra keyword arguments for the detector constructor.
    """

    def __new__(
        cls,
        detector_cls: str = "LsdCC",
        *,
        max_num_2d_segs: int = 3000,
        do_merge_lines: bool = False,
        visualize: bool = False,
        **kwargs: object,
    ) -> LsfmLimapDetector:
        """Create instance inheriting from LIMAP BaseDetector at runtime."""
        from limap.line2d.base_detector import (
            BaseDetector,
            BaseDetectorOptions,
        )

        # Dynamically create a subclass that inherits from both
        if not issubclass(cls, BaseDetector):
            cls = type(
                cls.__name__,
                (cls, BaseDetector),
                {},
            )

        options = BaseDetectorOptions(
            set_gray=True,
            max_num_2d_segs=max_num_2d_segs,
            do_merge_lines=do_merge_lines,
            visualize=visualize,
        )
        instance = object.__new__(cls)
        BaseDetector.__init__(instance, options)
        return instance

    def __init__(
        self,
        detector_cls: str = "LsdCC",
        *,
        max_num_2d_segs: int = 3000,
        do_merge_lines: bool = False,
        visualize: bool = False,
        **kwargs: object,
    ) -> None:
        import le_lsd as _le_lsd  # noqa: N812

        det_class = getattr(_le_lsd, detector_cls)
        self._det = det_class(**kwargs)
        self._detector_cls_name = detector_cls

    def get_module_name(self) -> str:
        """Return the module name for LIMAP's folder-based I/O.

        :return: Module identifier string.
        :rtype: str
        """
        return "lsfm_lsd"

    def detect(self, camview: object) -> NDArray[np.float64]:
        """Detect line segments from a LIMAP ``CameraView``.

        Reads the grayscale image from the camera view, runs our LSD
        detector, and returns segments in LIMAP's ``(N, 5)`` format.

        :param camview: A ``limap.base.CameraView`` instance.
        :type camview: limap.base.CameraView
        :return: Segments as ``[x1, y1, x2, y2, score]`` rows.
        :rtype: numpy.ndarray of shape ``(N, 5)``
        """
        import cv2

        img = camview.read_image(True)  # type: ignore[attr-defined]
        if img is None:
            return np.zeros((0, 5), dtype=np.float64)
        # Ensure grayscale uint8
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.dtype != np.uint8:
            img = np.clip(img, 0, 255).astype(np.uint8)

        self._det.detect(img)
        segs = self._det.line_segments()
        return segments_to_limap(segs)
