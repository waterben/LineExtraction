"""Ground truth projection pipeline.

Factored into composable pure functions:

- :func:`filter_behind_camera` — exclude/clip segments behind the camera
- :func:`filter_occluded` — depth-buffer occlusion test
- :func:`project_segments` — project 3D to 2D via Pluecker projection
- :func:`clip_to_frustum` — clip 2D segments to image bounds

:class:`SceneGroundTruth` is a thin coordinator that runs the pipeline
for all views and tracks parent 3D line indices and cross-view
correspondences.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import le_geometry

    from lsfm.synthetic.camera_rig import CameraRig
    from lsfm.synthetic.scene import Scene


# ---------------------------------------------------------------------------
# Pure pipeline functions
# ---------------------------------------------------------------------------

_NEAR_PLANE_EPS = 1e-3


def filter_behind_camera(
    segments_3d: list[le_geometry.LineSegment3_f64],
    camera: le_geometry.Camera_f64,
    near: float = _NEAR_PLANE_EPS,
) -> list[tuple[int, le_geometry.LineSegment3_f64]]:
    """Exclude segments fully behind the camera and clip straddlers.

    :param segments_3d: 3D line segments in world space.
    :type segments_3d: list[le_geometry.LineSegment3_f64]
    :param camera: Camera whose coordinate frame defines "behind".
    :type camera: le_geometry.Camera_f64
    :param near: Near-plane Z distance (in camera frame).
    :type near: float
    :return: ``(original_index, segment)`` pairs for segments (or
        clipped portions) that are in front of the camera.
    :rtype: list[tuple[int, le_geometry.LineSegment3_f64]]
    """
    rot, trans = _camera_extrinsics(camera)
    result: list[tuple[int, le_geometry.LineSegment3_f64]] = []

    for idx, seg in enumerate(segments_3d):
        sp_w = np.array(seg.start_point(), dtype=np.float64)
        ep_w = np.array(seg.end_point(), dtype=np.float64)

        # Transform to camera frame
        sp_c = rot @ sp_w + trans
        ep_c = rot @ ep_w + trans

        z0, z1 = sp_c[2], ep_c[2]

        if z0 < near and z1 < near:
            # Fully behind camera
            continue

        if z0 >= near and z1 >= near:
            # Fully in front
            result.append((idx, seg))
            continue

        # Clip to near plane
        t = (near - z0) / (z1 - z0)
        clipped_world = sp_w + t * (ep_w - sp_w)

        import le_geometry as _lg

        if z0 < near:
            new_seg = _lg.LineSegment3_f64.from_endpoints(
                float(clipped_world[0]),
                float(clipped_world[1]),
                float(clipped_world[2]),
                float(ep_w[0]),
                float(ep_w[1]),
                float(ep_w[2]),
            )
        else:
            new_seg = _lg.LineSegment3_f64.from_endpoints(
                float(sp_w[0]),
                float(sp_w[1]),
                float(sp_w[2]),
                float(clipped_world[0]),
                float(clipped_world[1]),
                float(clipped_world[2]),
            )
        result.append((idx, new_seg))

    return result


def project_segments(
    segments_3d: list[le_geometry.LineSegment3_f64],
    projector: le_geometry.CameraPluecker_f64,
) -> list[le_geometry.LineSegment_f64]:
    """Project 3D segments to 2D using Pluecker line projection.

    :param segments_3d: 3D line segments to project.
    :type segments_3d: list[le_geometry.LineSegment3_f64]
    :param projector: Camera projector for Pluecker projection.
    :type projector: le_geometry.CameraPluecker_f64
    :return: Projected 2D line segments.
    :rtype: list[le_geometry.LineSegment_f64]
    """
    return projector.project_line_segments(segments_3d)


def clip_to_frustum(
    segments_2d: list[le_geometry.LineSegment_f64],
    width: float,
    height: float,
    margin: float = 0.0,
) -> list[tuple[int, le_geometry.LineSegment_f64]]:
    """Clip 2D segments to the image rectangle using Liang-Barsky.

    :param segments_2d: 2D line segments to clip.
    :type segments_2d: list[le_geometry.LineSegment_f64]
    :param width: Image width in pixels.
    :type width: float
    :param height: Image height in pixels.
    :type height: float
    :param margin: Extra margin around the image border.
    :type margin: float
    :return: ``(original_index, clipped_segment)`` pairs for visible
        segments.
    :rtype: list[tuple[int, le_geometry.LineSegment_f64]]
    """
    x_min = -margin
    y_min = -margin
    x_max = width + margin
    y_max = height + margin

    result: list[tuple[int, le_geometry.LineSegment_f64]] = []
    for idx, seg in enumerate(segments_2d):
        clipped = _liang_barsky_clip(seg, x_min, y_min, x_max, y_max)
        if clipped is not None:
            result.append((idx, clipped))
    return result


def _liang_barsky_clip(
    seg: le_geometry.LineSegment_f64,
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
) -> le_geometry.LineSegment_f64 | None:
    """Clip a 2D segment to a rectangle using Liang-Barsky algorithm.

    :param seg: The segment to clip.
    :type seg: le_geometry.LineSegment_f64
    :param x_min: Left boundary.
    :type x_min: float
    :param y_min: Top boundary.
    :type y_min: float
    :param x_max: Right boundary.
    :type x_max: float
    :param y_max: Bottom boundary.
    :type y_max: float
    :return: Clipped segment, or ``None`` if fully outside.
    :rtype: le_geometry.LineSegment_f64 or None
    """
    import le_geometry

    sp = seg.start_point()
    ep = seg.end_point()
    x1, y1 = float(sp[0]), float(sp[1])
    x2, y2 = float(ep[0]), float(ep[1])

    dx = x2 - x1
    dy = y2 - y1

    p = [-dx, dx, -dy, dy]
    q = [x1 - x_min, x_max - x1, y1 - y_min, y_max - y1]

    t0 = 0.0
    t1 = 1.0

    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return None
        else:
            t = qi / pi
            if pi < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)

    if t0 > t1:
        return None

    cx1 = x1 + t0 * dx
    cy1 = y1 + t0 * dy
    cx2 = x1 + t1 * dx
    cy2 = y1 + t1 * dy

    return le_geometry.LineSegment_f64.from_endpoints(cx1, cy1, cx2, cy2)


# ---------------------------------------------------------------------------
# Depth-buffer occlusion
# ---------------------------------------------------------------------------


def _scene_triangles(scene: Scene) -> np.ndarray:
    """Collect all scene face triangles in world coordinates.

    :param scene: The 3D scene.
    :type scene: Scene
    :return: Triangle vertices ``(N, 3, 3)`` in world space.
    :rtype: numpy.ndarray
    """
    from lsfm.synthetic.conversions import _transform_vertices

    all_tris: list[np.ndarray] = []
    for prim, pose in scene.entries:
        world_verts = _transform_vertices(prim.vertices(), pose)
        for face in prim.face_indices():
            for k in range(1, len(face) - 1):
                tri = np.array(
                    [
                        world_verts[face[0]],
                        world_verts[face[k]],
                        world_verts[face[k + 1]],
                    ],
                    dtype=np.float64,
                )
                all_tris.append(tri)
    if not all_tris:
        return np.zeros((0, 3, 3), dtype=np.float64)
    return np.stack(all_tris)


def _rasterize_triangle(
    u: np.ndarray,
    v: np.ndarray,
    z: np.ndarray,
    depth_buf: np.ndarray,
) -> None:
    """Rasterize a single triangle into the depth buffer (vectorized)."""
    h, w = depth_buf.shape
    min_u = max(0, int(math.floor(u.min())))
    max_u = min(w - 1, int(math.ceil(u.max())))
    min_v = max(0, int(math.floor(v.min())))
    max_v = min(h - 1, int(math.ceil(v.max())))

    if min_u > max_u or min_v > max_v:
        return

    area = (u[1] - u[0]) * (v[2] - v[0]) - (u[2] - u[0]) * (v[1] - v[0])
    if abs(area) < 1e-10:
        return
    inv_area = 1.0 / area

    px = np.arange(min_u, max_u + 1, dtype=np.float64) + 0.5
    py = np.arange(min_v, max_v + 1, dtype=np.float64) + 0.5
    px_grid, py_grid = np.meshgrid(px, py)

    w0 = (
        (u[1] - px_grid) * (v[2] - py_grid) - (u[2] - px_grid) * (v[1] - py_grid)
    ) * inv_area
    w1 = (
        (u[2] - px_grid) * (v[0] - py_grid) - (u[0] - px_grid) * (v[2] - py_grid)
    ) * inv_area
    w2 = 1.0 - w0 - w1

    inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    depth = w0 * z[0] + w1 * z[1] + w2 * z[2]
    valid = inside & (depth > 0)

    roi = depth_buf[min_v : max_v + 1, min_u : max_u + 1]
    closer = valid & (depth < roi)
    roi[closer] = depth[closer]


def render_depth_map(
    scene: Scene,
    camera: le_geometry.Camera_f64,
    width: int,
    height: int,
) -> np.ndarray:
    """Render a depth map for the scene from the given camera.

    Uses a software Z-buffer rasterizer.  Each depth value is the
    camera-frame Z coordinate at that pixel.

    :param scene: The 3D scene (primitives + poses).
    :type scene: Scene
    :param camera: Camera to render from.
    :type camera: le_geometry.Camera_f64
    :param width: Image width in pixels.
    :type width: int
    :param height: Image height in pixels.
    :type height: int
    :return: Depth map ``(height, width)`` with ``inf`` for empty pixels.
    :rtype: numpy.ndarray
    """
    depth_buf = np.full((height, width), np.inf, dtype=np.float64)

    triangles = _scene_triangles(scene)
    if len(triangles) == 0:
        return depth_buf

    rot, trans = _camera_extrinsics(camera)
    fx, fy = camera.focal
    cx, cy = camera.offset

    for tri in triangles:
        cam_pts = (rot @ tri.T + trans[:, None]).T  # (3, 3)
        zs = cam_pts[:, 2]

        if np.any(zs < 1e-3):
            continue

        pu = fx * cam_pts[:, 0] / zs + cx
        pv = fy * cam_pts[:, 1] / zs + cy

        # Back-face culling: with consistent CCW outward winding,
        # front-facing triangles have negative screen-space area
        # (because image Y points downward).
        area = (pu[1] - pu[0]) * (pv[2] - pv[0]) - (pu[2] - pu[0]) * (pv[1] - pv[0])
        if area >= 0:
            continue

        _rasterize_triangle(pu, pv, zs, depth_buf)

    return depth_buf


def filter_occluded(
    visible_3d: list[tuple[int, le_geometry.LineSegment3_f64]],
    camera: le_geometry.Camera_f64,
    depth_map: np.ndarray,
    n_samples: int = 20,
    depth_tol: float = 0.05,
    visibility_ratio: float = 0.3,
) -> list[tuple[int, le_geometry.LineSegment3_f64]]:
    """Keep segments whose depth matches the depth buffer.

    For each segment, samples points along its length, projects them,
    and checks whether the computed depth is within *depth_tol* of the
    depth buffer value.  Segments with fewer than *visibility_ratio*
    visible samples are considered occluded and discarded.

    :param visible_3d: ``(original_index, segment)`` pairs.
    :type visible_3d: list[tuple[int, le_geometry.LineSegment3_f64]]
    :param camera: Camera instance.
    :type camera: le_geometry.Camera_f64
    :param depth_map: Depth map from :func:`render_depth_map`.
    :type depth_map: numpy.ndarray
    :param n_samples: Number of sample points per segment.
    :type n_samples: int
    :param depth_tol: Depth tolerance in world units.
    :type depth_tol: float
    :param visibility_ratio: Minimum visible fraction to keep segment.
    :type visibility_ratio: float
    :return: Filtered ``(original_index, segment)`` pairs.
    :rtype: list[tuple[int, le_geometry.LineSegment3_f64]]
    """
    rot, trans = _camera_extrinsics(camera)
    fx, fy = camera.focal
    cx, cy = camera.offset
    h, w = depth_map.shape

    result: list[tuple[int, le_geometry.LineSegment3_f64]] = []
    for idx, seg in visible_3d:
        sp = np.array(seg.start_point(), dtype=np.float64)
        ep = np.array(seg.end_point(), dtype=np.float64)

        ts = np.linspace(0, 1, n_samples)
        pts = sp[None, :] + ts[:, None] * (ep - sp)[None, :]  # (N, 3)

        cam_pts = (rot @ pts.T + trans[:, None]).T  # (N, 3)
        zs = cam_pts[:, 2]
        valid_z = zs > 1e-3

        pu = np.where(valid_z, fx * cam_pts[:, 0] / zs + cx, -1)
        pv = np.where(valid_z, fy * cam_pts[:, 1] / zs + cy, -1)
        ui = pu.astype(int)
        vi = pv.astype(int)

        in_bounds = valid_z & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        n_visible = 0
        for i in range(n_samples):
            if in_bounds[i] and zs[i] <= depth_map[vi[i], ui[i]] + depth_tol:
                n_visible += 1

        if n_samples > 0 and n_visible / n_samples >= visibility_ratio:
            result.append((idx, seg))

    return result


# ---------------------------------------------------------------------------
# Camera extrinsics helper
# ---------------------------------------------------------------------------


def _camera_extrinsics(
    camera: le_geometry.Camera_f64,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract the **world-to-camera** rotation and translation.

    ``Pose.hom_matrix()`` returns the camera-to-world transform
    ``[R_c2w | eye]``.  Callers need the inverse (world-to-camera):
    ``R_w2c = R_c2w^T``, ``t_w2c = -R_w2c @ eye``.

    :param camera: Camera instance.
    :type camera: le_geometry.Camera_f64
    :return: ``(R_w2c, t_w2c)`` where ``R_w2c`` is 3x3 and ``t_w2c``
        is ``(3,)``.
    :rtype: tuple[numpy.ndarray, numpy.ndarray]
    """
    rm = camera.rot_matrix()
    r_c2w = np.array([[rm[r][c] for c in range(3)] for r in range(3)], dtype=np.float64)
    eye = np.array(camera.origin, dtype=np.float64)

    r_w2c = r_c2w.T
    t_w2c = -r_w2c @ eye
    return r_w2c, t_w2c


# ---------------------------------------------------------------------------
# SceneGroundTruth coordinator
# ---------------------------------------------------------------------------


@dataclass
class _ViewGT:
    """Ground truth data for a single view."""

    segments_2d: list = field(default_factory=list)
    parent_indices: list[int] = field(default_factory=list)


class SceneGroundTruth:
    """Orchestrate the GT projection pipeline across all views.

    Runs ``filter_behind_camera → [filter_occluded] → project_segments →
    clip_to_frustum`` for each camera, tracking which 3D line produced
    each 2D segment.

    :param scene: The 3D scene.
    :type scene: Scene
    :param rig: Camera rig with all views.
    :type rig: CameraRig
    :param occlusion: When ``True``, render depth maps and remove lines
        hidden behind other primitives.
    :type occlusion: bool
    """

    def __init__(
        self,
        scene: Scene,
        rig: CameraRig,
        *,
        occlusion: bool = False,
    ) -> None:
        self._scene = scene
        self._rig = rig
        self._gt_lines_3d = scene.ground_truth_lines()
        self._views: list[_ViewGT] = []
        self._width = float(rig.cameras[0].width)
        self._height = float(rig.cameras[0].height)
        self._occlusion = occlusion
        self._run_pipeline()

    @property
    def scene(self) -> Scene:
        """The source 3D scene."""
        return self._scene

    @property
    def rig(self) -> CameraRig:
        """The camera rig used for projection."""
        return self._rig

    @property
    def gt_lines_3d(self) -> list:
        """The deduplicated 3D ground truth line segments."""
        return self._gt_lines_3d

    @property
    def n_views(self) -> int:
        """Number of camera views."""
        return len(self._views)

    @property
    def width(self) -> float:
        """Image width in pixels."""
        return self._width

    @property
    def height(self) -> float:
        """Image height in pixels."""
        return self._height

    def segments_2d(self, view_index: int) -> list:
        """Return projected 2D GT segments for a given view.

        :param view_index: Camera view index.
        :type view_index: int
        :return: List of ``LineSegment_f64`` in image coordinates.
        :rtype: list[le_geometry.LineSegment_f64]
        """
        return self._views[view_index].segments_2d

    def parent_indices(self, view_index: int) -> list[int]:
        """Return parent 3D line indices for each 2D segment in a view.

        :param view_index: Camera view index.
        :type view_index: int
        :return: List of indices into :attr:`gt_lines_3d`.
        :rtype: list[int]
        """
        return self._views[view_index].parent_indices

    def correspondences(self, view_i: int, view_j: int) -> list[tuple[int, int]]:
        """Return cross-view correspondences sharing the same 3D parent.

        :param view_i: First view index.
        :type view_i: int
        :param view_j: Second view index.
        :type view_j: int
        :return: List of ``(seg_idx_view_i, seg_idx_view_j)`` pairs.
        :rtype: list[tuple[int, int]]
        """
        parents_i = self._views[view_i].parent_indices
        parents_j = self._views[view_j].parent_indices

        # Build reverse map: parent_3d_idx -> list of 2D segment indices
        parent_to_j: dict[int, list[int]] = {}
        for seg_idx, parent_idx in enumerate(parents_j):
            parent_to_j.setdefault(parent_idx, []).append(seg_idx)

        pairs: list[tuple[int, int]] = []
        for seg_idx_i, parent_idx in enumerate(parents_i):
            if parent_idx in parent_to_j:
                for seg_idx_j in parent_to_j[parent_idx]:
                    pairs.append((seg_idx_i, seg_idx_j))
        return pairs

    def gt_tracks(
        self,
        view_indices: list[int] | None = None,
    ) -> list[list[tuple[int, int]]]:
        """Build multiview tracks directly from GT 2D segments.

        Each track groups the GT 2D segments that originate from the
        same 3D parent line across all requested views.  Unlike
        :func:`~lsfm.synthetic.oracle.oracle_tracks`, this does *not*
        go through detection and assignment — it uses the projected GT
        segments directly.

        :param view_indices: Which views to include (default: all).
        :type view_indices: list[int] | None
        :return: List of tracks, each a list of
            ``(view_index, gt_segment_index)`` with observations in
            at least two views.
        :rtype: list[list[tuple[int, int]]]
        """
        if view_indices is None:
            view_indices = list(range(self.n_views))

        parent_to_obs: dict[int, list[tuple[int, int]]] = {}
        for v_idx in view_indices:
            for seg_idx, parent_3d in enumerate(self.parent_indices(v_idx)):
                parent_to_obs.setdefault(parent_3d, []).append((v_idx, seg_idx))

        return [obs for obs in parent_to_obs.values() if len({v for v, _ in obs}) >= 2]

    def _run_pipeline(self) -> None:
        """Run the projection pipeline for all views."""
        cameras = self._rig.cameras
        projectors = self._rig.projectors

        # Pre-render depth maps when occlusion filtering is enabled.
        depth_maps: list[np.ndarray] | None = None
        if self._occlusion:
            w, h = int(self._width), int(self._height)
            depth_maps = [render_depth_map(self._scene, cam, w, h) for cam in cameras]

        for i, (cam, proj) in enumerate(zip(cameras, projectors)):
            # Step 1: Filter behind camera
            visible = filter_behind_camera(self._gt_lines_3d, cam)
            if not visible:
                self._views.append(_ViewGT())
                continue

            # Step 1b: Occlusion filter (optional)
            if depth_maps is not None:
                visible = filter_occluded(visible, cam, depth_maps[i])
                if not visible:
                    self._views.append(_ViewGT())
                    continue

            vis_indices, vis_segments = zip(*visible)

            # Step 2: Project to 2D
            projected = project_segments(list(vis_segments), proj)

            # Step 3: Clip to image bounds
            clipped = clip_to_frustum(projected, self._width, self._height)

            # Map clipped indices back to original 3D parent index
            segs_2d = []
            parents = []
            for clip_idx, clip_seg in clipped:
                segs_2d.append(clip_seg)
                parents.append(vis_indices[clip_idx])

            self._views.append(_ViewGT(segments_2d=segs_2d, parent_indices=parents))
