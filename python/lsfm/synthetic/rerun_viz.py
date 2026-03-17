"""Rerun visualization helpers and blueprint factory.

Provides granular logging functions for each pipeline stage and a
reconstruction blueprint with 3D + 2D views.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from lsfm.synthetic.conversions import segments3d_to_array, segments_to_array

if TYPE_CHECKING:
    from numpy.typing import NDArray

    import le_geometry

    from lsfm.synthetic.camera_rig import CameraRig
    from lsfm.synthetic.scene import Scene


def log_scene(
    scene: Scene,
    *,
    entity_path: str = "world/scene",
    radius: float = 0.02,
) -> None:
    """Log all 3D GT lines and primitive geometry to Rerun.

    Sets the world coordinate system to right-handed Y-up and logs all
    ground truth 3D line segments.

    :param scene: The 3D scene.
    :type scene: Scene
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param radius: Line radius in world units.
    :type radius: float
    """
    import rerun as rr

    # Set Y-up coordinate system (the scene uses Y as vertical axis)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    gt_lines = scene.ground_truth_lines()
    if not gt_lines:
        return

    arr = segments3d_to_array(gt_lines)
    n = arr.shape[0]

    rr.log(
        f"{entity_path}/gt_lines",
        rr.LineStrips3D(
            arr.tolist(),
            colors=[(0, 200, 0)] * n,
            radii=[radius] * n,
        ),
    )


def log_cameras(
    rig: CameraRig,
    *,
    entity_path: str = "world/cameras",
    image_width: int | None = None,
    image_height: int | None = None,
    highlight: dict[int, tuple[tuple[int, int, int], str]] | None = None,
) -> None:
    """Log camera frustums to Rerun.

    :param rig: Camera rig to visualize.
    :type rig: CameraRig
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param image_width: Override image width (uses camera width by default).
    :type image_width: int, optional
    :param image_height: Override image height (uses camera height by default).
    :type image_height: int, optional
    :param highlight: Map from camera index to ``(color, label)`` for visual
        emphasis.  Highlighted cameras get a colored 3D marker and text label
        at their position.
    :type highlight: dict[int, tuple[tuple[int, int, int], str]], optional
    """
    import rerun as rr

    highlight = highlight or {}
    marker_positions: list[list[float]] = []
    marker_colors: list[tuple[int, int, int]] = []
    marker_labels: list[str] = []

    for i, cam in enumerate(rig.cameras):
        cam_path = f"{entity_path}/cam_{i:02d}"
        fx, fy = cam.focal
        cx, cy = cam.offset
        w = image_width or int(cam.width)
        h = image_height or int(cam.height)

        # Log pinhole intrinsics
        rr.log(
            f"{cam_path}",
            rr.Pinhole(
                focal_length=[fx, fy],
                principal_point=[cx, cy],
                resolution=[w, h],
                image_plane_distance=0.3,
            ),
        )

        # Log extrinsics: hom_matrix() returns camera-to-world (c2w)
        hom = cam.hom_matrix()
        rot_c2w = np.array(
            [[hom[r][c] for c in range(3)] for r in range(3)], dtype=np.float64
        )
        t_c2w = np.array([hom[0][3], hom[1][3], hom[2][3]], dtype=np.float64)

        rr.log(
            f"{cam_path}",
            rr.Transform3D(
                translation=t_c2w.tolist(),
                mat3x3=rot_c2w.tolist(),
                from_parent=False,
            ),
        )

        if i in highlight:
            color, label = highlight[i]
            marker_positions.append(t_c2w.tolist())
            marker_colors.append(color)
            marker_labels.append(label)

    # Log highlight markers in a single batch
    if marker_positions:
        rr.log(
            f"{entity_path}/highlights",
            rr.Points3D(
                marker_positions,
                colors=marker_colors,
                labels=marker_labels,
                radii=[0.25] * len(marker_positions),
            ),
        )


def log_image(
    view_index: int,
    image: NDArray[np.uint8],
    *,
    entity_path: str = "views",
) -> None:
    """Log a per-view image to Rerun.

    :param view_index: Camera view index.
    :type view_index: int
    :param image: Grayscale or color image.
    :type image: numpy.ndarray
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    """
    import rerun as rr

    rr.log(f"{entity_path}/view_{view_index:02d}/image", rr.Image(image))


def log_gt_segments(
    view_index: int,
    segments: list[le_geometry.LineSegment_f64],
    *,
    entity_path: str = "views",
    color: tuple[int, int, int] = (0, 200, 0),
) -> None:
    """Log ground truth 2D segments for a view.

    :param view_index: Camera view index.
    :type view_index: int
    :param segments: GT 2D line segments.
    :type segments: list[le_geometry.LineSegment_f64]
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param color: RGB color for GT segments.
    :type color: tuple[int, int, int]
    """
    import rerun as rr

    if not segments:
        return
    arr = segments_to_array(segments)
    rr.log(
        f"{entity_path}/view_{view_index:02d}/gt_segments",
        rr.LineStrips2D(
            arr.tolist(),
            colors=[color] * len(segments),
        ),
    )


def log_detected_segments(
    view_index: int,
    segments: list[le_geometry.LineSegment_f64],
    *,
    entity_path: str = "views",
    color: tuple[int, int, int] = (50, 100, 255),
) -> None:
    """Log detected 2D segments for a view.

    :param view_index: Camera view index.
    :type view_index: int
    :param segments: Detected 2D line segments.
    :type segments: list[le_geometry.LineSegment_f64]
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param color: RGB color for detected segments.
    :type color: tuple[int, int, int]
    """
    import rerun as rr

    if not segments:
        return
    arr = segments_to_array(segments)
    rr.log(
        f"{entity_path}/view_{view_index:02d}/detected_segments",
        rr.LineStrips2D(
            arr.tolist(),
            colors=[color] * len(segments),
        ),
    )


def log_view(
    view_index: int,
    image: NDArray[np.uint8],
    gt_segs: list[le_geometry.LineSegment_f64],
    det_segs: list[le_geometry.LineSegment_f64],
    *,
    entity_path: str = "views",
) -> None:
    """Convenience wrapper to log image + GT + detected segments for a view.

    :param view_index: Camera view index.
    :type view_index: int
    :param image: View image.
    :type image: numpy.ndarray
    :param gt_segs: Ground truth 2D segments.
    :type gt_segs: list[le_geometry.LineSegment_f64]
    :param det_segs: Detected 2D segments.
    :type det_segs: list[le_geometry.LineSegment_f64]
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    """
    log_image(view_index, image, entity_path=entity_path)
    log_gt_segments(view_index, gt_segs, entity_path=entity_path)
    log_detected_segments(view_index, det_segs, entity_path=entity_path)


def log_matches(
    matches: list[tuple[int, int]],
    segs_i: list[le_geometry.LineSegment_f64],
    segs_j: list[le_geometry.LineSegment_f64],
    view_i: int,
    view_j: int,
    *,
    entity_path: str = "matches",
    color: tuple[int, int, int] = (255, 200, 0),
) -> None:
    """Log match connections between two views.

    :param matches: Match pairs ``(idx_view_i, idx_view_j)``.
    :type matches: list[tuple[int, int]]
    :param segs_i: Segments in view i.
    :type segs_i: list[le_geometry.LineSegment_f64]
    :param segs_j: Segments in view j.
    :type segs_j: list[le_geometry.LineSegment_f64]
    :param view_i: First view index.
    :type view_i: int
    :param view_j: Second view index.
    :type view_j: int
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param color: RGB color for match lines.
    :type color: tuple[int, int, int]
    """
    import rerun as rr

    if not matches:
        return

    # Log midpoint connections as text annotations
    lines = []
    for idx_i, idx_j in matches:
        mid_i = segs_i[idx_i].center_point()
        mid_j = segs_j[idx_j].center_point()
        lines.append([[mid_i[0], mid_i[1]], [mid_j[0], mid_j[1]]])

    rr.log(
        f"{entity_path}/v{view_i}_v{view_j}",
        rr.LineStrips2D(
            lines,
            colors=[color] * len(lines),
        ),
    )


def log_reconstruction(
    segs_3d: list[le_geometry.LineSegment3_f64],
    *,
    errors: list[float] | None = None,
    entity_path: str = "world/reconstruction",
    color: tuple[int, int, int] = (255, 100, 50),
    radius: float = 0.04,
) -> None:
    """Log reconstructed 3D lines, optionally color-coded by error.

    :param segs_3d: Reconstructed 3D line segments.
    :type segs_3d: list[le_geometry.LineSegment3_f64]
    :param errors: Per-segment error values for color coding.
    :type errors: list[float], optional
    :param entity_path: Rerun entity path prefix.
    :type entity_path: str
    :param color: Default RGB color when no error coding is used.
    :type color: tuple[int, int, int]
    :param radius: Line radius in world units.
    :type radius: float
    """
    import rerun as rr

    if not segs_3d:
        return

    arr = segments3d_to_array(segs_3d)

    if errors is not None:
        err_arr = np.array(errors, dtype=np.float64)
        max_err = max(float(err_arr.max()), 1e-6)
        # Color map: green (low error) -> red (high error)
        colors = []
        for e in err_arr:
            t = min(float(e) / max_err, 1.0)
            r = int(255 * t)
            g = int(255 * (1 - t))
            colors.append((r, g, 0))
    else:
        colors = [color] * len(segs_3d)

    rr.log(
        entity_path,
        rr.LineStrips3D(
            arr.tolist(),
            colors=colors,
            radii=[radius] * len(segs_3d),
        ),
    )


def reconstruction_blueprint(n_views: int) -> object:
    """Create a Rerun blueprint with a single 3D scene panel.

    :param n_views: Number of camera views (unused, kept for API compat).
    :type n_views: int
    :return: A Rerun Blueprint object.
    :rtype: rerun.blueprint.Blueprint
    """
    import rerun.blueprint as rrb

    scene_3d = rrb.Spatial3DView(name="3D Scene", origin="world")
    return rrb.Blueprint(scene_3d, collapse_panels=True)
