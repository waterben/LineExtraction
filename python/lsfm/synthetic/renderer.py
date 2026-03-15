"""Wireframe and textured image rendering from ground truth.

- :func:`render_wireframe` — renders a single view as wireframe (grayscale)
- :func:`render_all_views` — renders all views as wireframes
- :func:`render_textured` — renders with colored procedural textures, ground
  plane, and sky background (RGB)

Uses **Pillow** (``PIL``) for drawing so that the module does not depend
on ``cv2`` at runtime — the LE native extensions bundle their own OpenCV
and ``import cv2`` is not available in the Bazel sandbox.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from PIL import Image, ImageDraw

if TYPE_CHECKING:
    from lsfm.synthetic.ground_truth import SceneGroundTruth
    from lsfm.synthetic.scene import Scene


def render_wireframe(
    gt: SceneGroundTruth,
    view_index: int,
    *,
    background: int = 255,
    line_color: int = 0,
    thickness: int = 1,
    anti_alias: bool = True,
) -> NDArray[np.uint8]:
    """Render a single view as a wireframe grayscale image.

    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param view_index: Camera view index.
    :type view_index: int
    :param background: Background gray value (0-255).
    :type background: int
    :param line_color: Line gray value (0-255).
    :type line_color: int
    :param thickness: Line thickness in pixels.
    :type thickness: int
    :param anti_alias: Whether to use anti-aliased lines.
    :type anti_alias: bool
    :return: Grayscale uint8 image.
    :rtype: numpy.ndarray
    """
    w = int(gt.width)
    h = int(gt.height)
    img = Image.new("L", (w, h), background)
    draw = ImageDraw.Draw(img)

    segments = gt.segments_2d(view_index)

    for seg in segments:
        sp = seg.start_point()
        ep = seg.end_point()
        draw.line(
            [(sp[0], sp[1]), (ep[0], ep[1])],
            fill=line_color,
            width=thickness,
        )

    return np.array(img, dtype=np.uint8)


def render_all_views(
    gt: SceneGroundTruth,
    *,
    background: int = 255,
    line_color: int = 0,
    thickness: int = 1,
    anti_alias: bool = True,
) -> list[NDArray[np.uint8]]:
    """Render all views as wireframe grayscale images.

    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param background: Background gray value.
    :type background: int
    :param line_color: Line gray value.
    :type line_color: int
    :param thickness: Line thickness in pixels.
    :type thickness: int
    :param anti_alias: Whether to use anti-aliased lines.
    :type anti_alias: bool
    :return: List of grayscale uint8 images, one per view.
    :rtype: list[numpy.ndarray]
    """
    return [
        render_wireframe(
            gt,
            i,
            background=background,
            line_color=line_color,
            thickness=thickness,
            anti_alias=anti_alias,
        )
        for i in range(gt.n_views)
    ]


# ---------------------------------------------------------------------------
# Textured rendering — per-primitive color + fine texture
# ---------------------------------------------------------------------------

# Maps primitive names to (texture_type, base_rgb_color).
_PRIM_STYLES: dict[str, tuple[str, tuple[int, int, int]]] = {
    "body": ("brick", (185, 185, 190)),
    "roof": ("shingle", (185, 60, 45)),
    "tower": ("brick", (170, 175, 185)),
    "steeple": ("shingle", (170, 50, 35)),
}
_DEFAULT_STYLE: tuple[str, tuple[int, int, int]] = ("fine_noise", (180, 180, 180))


def _generate_brick(width: int, height: int, seed: int = 0) -> NDArray[np.uint8]:
    """Generate a fine-brick pattern with subtle mortar lines."""
    rng = np.random.default_rng(seed)
    brick_w = max(width // 20, 3)
    brick_h = max(height // 14, 2)
    mortar = 1
    base = 170
    img = np.full((height, width), base, dtype=np.uint8)

    row_idx = 0
    y = 0
    while y < height:
        offset = (brick_w // 2) if (row_idx % 2) else 0
        x = -offset
        while x < width:
            brightness = int(rng.integers(base - 12, base + 12))
            x0 = max(0, x)
            x1 = min(width, x + brick_w)
            y0 = y
            y1 = min(height, y + brick_h)
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = brightness
            x += brick_w + mortar
        # Subtle mortar line
        y_m = y + brick_h
        if y_m < height:
            img[y_m : min(height, y_m + mortar), :] = base - 15
        y += brick_h + mortar
        row_idx += 1
    return img


def _generate_shingle(width: int, height: int, seed: int = 0) -> NDArray[np.uint8]:
    """Generate a fine overlapping shingle/roof-tile pattern."""
    rng = np.random.default_rng(seed)
    tile_w = max(width // 18, 3)
    tile_h = max(height // 14, 2)
    base = 165
    img = np.full((height, width), base, dtype=np.uint8)

    row_idx = 0
    y = 0
    while y < height:
        offset = (tile_w // 2) if (row_idx % 2) else 0
        x = -offset
        while x < width:
            brightness = int(rng.integers(base - 10, base + 10))
            x0 = max(0, x)
            x1 = min(width, x + tile_w - 1)
            y0 = y
            y1 = min(height, y + tile_h - 1)
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = brightness
            x += tile_w
        y += tile_h
        row_idx += 1
    return img


def _generate_grass(width: int, height: int, seed: int = 0) -> NDArray[np.uint8]:
    """Generate a fine grass-like noise pattern with low contrast."""
    rng = np.random.default_rng(seed)
    # Two-scale noise: coarse + very fine (ceil division for small sizes)
    coarse = rng.integers(
        145, 175, size=(-(-height // 6), -(-width // 6)), dtype=np.uint8
    )
    coarse_up = np.repeat(np.repeat(coarse, 6, axis=0), 6, axis=1)[:height, :width]
    fine = rng.integers(-8, 8, size=(height, width), dtype=np.int16)
    result = np.clip(coarse_up.astype(np.int16) + fine, 130, 190).astype(np.uint8)
    return result


def _generate_fine_noise(width: int, height: int, seed: int = 0) -> NDArray[np.uint8]:
    """Generate fine spatially-correlated noise with low contrast."""
    rng = np.random.default_rng(seed)
    coarse = rng.integers(
        150, 190, size=(-(-height // 5), -(-width // 5)), dtype=np.uint8
    )
    coarse_up = np.repeat(np.repeat(coarse, 5, axis=0), 5, axis=1)[:height, :width]
    fine = rng.integers(-8, 8, size=(height, width), dtype=np.int16)
    return np.clip(coarse_up.astype(np.int16) + fine, 130, 200).astype(np.uint8)


_TEXTURE_GENERATORS = {
    "brick": _generate_brick,
    "shingle": _generate_shingle,
    "grass": _generate_grass,
    "fine_noise": _generate_fine_noise,
}


def _tint_texture(
    texture_gray: NDArray[np.uint8],
    base_color: tuple[int, int, int],
) -> NDArray[np.uint8]:
    """Apply an RGB color tint to a grayscale texture.

    :param texture_gray: Grayscale texture ``(H, W)`` uint8.
    :type texture_gray: numpy.ndarray
    :param base_color: RGB base color to tint with.
    :type base_color: tuple[int, int, int]
    :return: RGB image ``(H, W, 3)`` uint8.
    :rtype: numpy.ndarray
    """
    scale = texture_gray.astype(np.float32) / 255.0
    rgb = np.empty((*texture_gray.shape, 3), dtype=np.uint8)
    for c in range(3):
        rgb[..., c] = np.clip(scale * base_color[c], 0, 255).astype(np.uint8)
    return rgb


def _sky_gradient(width: int, height: int) -> NDArray[np.uint8]:
    """Create a vertical sky gradient (dark blue top, light blue horizon).

    :return: RGB image ``(H, W, 3)`` uint8.
    :rtype: numpy.ndarray
    """
    img = np.empty((height, width, 3), dtype=np.uint8)
    for y in range(height):
        t = y / max(height - 1, 1)  # 0 at top, 1 at bottom
        # Sky: deep blue at top → lighter near horizon
        r = int(100 + 70 * t)
        g = int(150 + 70 * t)
        b = int(220 + 20 * t)
        img[y, :, 0] = r
        img[y, :, 1] = g
        img[y, :, 2] = b
    return img


def _project_face_verts(
    face_verts_3d: NDArray[np.float64],
    rot: NDArray[np.float64],
    trans: NDArray[np.float64],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64], bool]:
    """Project 3D face vertices to 2D and return per-vertex camera depths.

    :return: ``(verts_2d, z_per_vertex, all_valid)``
    :rtype: tuple[numpy.ndarray, numpy.ndarray, bool]
    """
    n = len(face_verts_3d)
    verts_2d = np.empty((n, 2), dtype=np.float64)
    z_vals = np.empty(n, dtype=np.float64)
    for i in range(n):
        v_cam = rot @ face_verts_3d[i] + trans
        if v_cam[2] < 1e-3:
            return verts_2d, z_vals, False
        verts_2d[i, 0] = fx * v_cam[0] / v_cam[2] + cx
        verts_2d[i, 1] = fy * v_cam[1] / v_cam[2] + cy
        z_vals[i] = v_cam[2]
    return verts_2d, z_vals, True


def _rasterize_triangle_textured(
    pu: NDArray[np.float64],
    pv: NDArray[np.float64],
    z: NDArray[np.float64],
    depth_buf: NDArray[np.float64],
    color_buf: NDArray[np.uint8],
    texture_rgb: NDArray[np.uint8],
    x_off: int,
    y_off: int,
) -> None:
    """Rasterize a triangle with per-pixel z-test, writing textured color.

    Uses barycentric interpolation to compute camera-space depth at
    every pixel inside the triangle.  Only pixels closer than the
    current depth buffer value are written.

    :param pu: Screen-space X of the 3 triangle vertices.
    :param pv: Screen-space Y of the 3 triangle vertices.
    :param z: Camera-space Z (depth) of the 3 triangle vertices.
    :param depth_buf: Depth buffer ``(H, W)``, updated in-place.
    :param color_buf: Color buffer ``(H, W, 3)``, updated in-place.
    :param texture_rgb: Face texture ``(TH, TW, 3)`` for the bounding box.
    :param x_off: Screen X of the texture's top-left corner.
    :param y_off: Screen Y of the texture's top-left corner.
    """
    h, w = depth_buf.shape
    min_u = max(0, int(math.floor(pu.min())))
    max_u = min(w - 1, int(math.ceil(pu.max())))
    min_v = max(0, int(math.floor(pv.min())))
    max_v = min(h - 1, int(math.ceil(pv.max())))

    if min_u > max_u or min_v > max_v:
        return

    area = (pu[1] - pu[0]) * (pv[2] - pv[0]) - (pu[2] - pu[0]) * (pv[1] - pv[0])
    if abs(area) < 1e-10:
        return
    inv_area = 1.0 / area

    px = np.arange(min_u, max_u + 1, dtype=np.float64) + 0.5
    py = np.arange(min_v, max_v + 1, dtype=np.float64) + 0.5
    px_grid, py_grid = np.meshgrid(px, py)

    w0 = (
        (pu[1] - px_grid) * (pv[2] - py_grid) - (pu[2] - px_grid) * (pv[1] - py_grid)
    ) * inv_area
    w1 = (
        (pu[2] - px_grid) * (pv[0] - py_grid) - (pu[0] - px_grid) * (pv[2] - py_grid)
    ) * inv_area
    w2 = 1.0 - w0 - w1

    inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    depth = w0 * z[0] + w1 * z[1] + w2 * z[2]
    valid = inside & (depth > 0)

    roi_depth = depth_buf[min_v : max_v + 1, min_u : max_u + 1]
    closer = valid & (depth < roi_depth)
    roi_depth[closer] = depth[closer]

    tex_h, tex_w = texture_rgb.shape[:2]
    tx = np.clip(np.arange(min_u, max_u + 1) - x_off, 0, tex_w - 1)
    ty = np.clip(np.arange(min_v, max_v + 1) - y_off, 0, tex_h - 1)
    tx_grid, ty_grid = np.meshgrid(tx, ty)

    for c in range(3):
        roi_color = color_buf[min_v : max_v + 1, min_u : max_u + 1, c]
        roi_color[closer] = texture_rgb[ty_grid, tx_grid, c][closer]


def render_textured(
    scene: Scene,
    gt: SceneGroundTruth,
    view_index: int,
    *,
    line_color: tuple[int, int, int] = (10, 10, 10),
    thickness: int = 1,
    texture_seed: int = 42,
    show_ground: bool = True,
    ground_y: float = 0.0,
    ground_size: float = 50.0,
    light_dir: NDArray[np.float64] | None = None,
    ambient: float = 0.8,
    draw_edges: bool = True,
) -> NDArray[np.uint8]:
    """Render a view with colored procedural textures, ground, and sky.

    Each primitive is rendered with a characteristic texture and color
    (e.g. red-brown roof tiles, gray stone walls).  A green ground plane
    and blue sky gradient provide environmental context.  Uses a
    **per-pixel z-buffer** with barycentric depth interpolation to
    ensure correct occlusion even when faces partially overlap in screen
    space.  Returns an **RGB** uint8 image.

    A simple **directional light** (Lambertian shading) modulates face
    brightness to add depth cues.  The *light_dir* vector points
    **toward** the light source.  If ``None`` a default sun direction of
    ``(1, -1, 0.5)`` is used (upper-right-front).

    :param scene: The 3D scene (for face geometry).
    :type scene: Scene
    :param gt: Ground truth projection data (for camera + edges).
    :type gt: SceneGroundTruth
    :param view_index: Camera view index.
    :type view_index: int
    :param line_color: RGB color for edge lines.
    :type line_color: tuple[int, int, int]
    :param thickness: Line thickness in pixels.
    :type thickness: int
    :param texture_seed: Base seed for procedural textures.
    :type texture_seed: int
    :param show_ground: Whether to render a ground plane at *ground_y*.
    :type show_ground: bool
    :param ground_y: World-space Y coordinate for the ground plane.
    :type ground_y: float
    :param ground_size: Half-extent of the ground plane quad.
    :type ground_size: float
    :param light_dir: World-space direction **toward** the light (auto-normalized).
        Defaults to ``(1, -1, 0.5)``.
    :type light_dir: numpy.ndarray or None
    :param ambient: Minimum brightness factor ``[0, 1]``.  Faces pointing
        away from the light keep this fraction of their base color.
    :type ambient: float
    :param draw_edges: Whether to draw GT edge lines on top of the
        textured image.  Set to ``False`` to get a clean image suitable
        for line detection.
    :type draw_edges: bool
    :return: RGB uint8 image of shape ``(H, W, 3)``.
    :rtype: numpy.ndarray
    """
    from lsfm.synthetic.conversions import _transform_vertices
    from lsfm.synthetic.ground_truth import _camera_extrinsics

    w = int(gt.width)
    h = int(gt.height)

    # Sky gradient background + per-pixel depth buffer
    img_arr = _sky_gradient(w, h)
    depth_buf = np.full((h, w), np.inf, dtype=np.float64)

    cam = gt.rig.cameras[view_index]
    rot, trans = _camera_extrinsics(cam)
    fx, fy = cam.focal
    cx, cy = cam.offset
    cam_pos = np.array(cam.origin, dtype=np.float64)

    # Normalize light direction (toward the light source)
    if light_dir is None:
        light_vec = np.array([1.0, -1.0, 0.5], dtype=np.float64)
    else:
        light_vec = np.asarray(light_dir, dtype=np.float64)
    light_vec = light_vec / np.linalg.norm(light_vec)

    def _face_brightness(normal: NDArray[np.float64]) -> float:
        """Lambertian shading: ambient + diffuse component."""
        n = normal / np.linalg.norm(normal)
        cos_theta = float(np.dot(n, light_vec))
        return ambient + (1.0 - ambient) * max(0.0, cos_theta)

    def _rasterize_face(
        verts_2d: NDArray[np.float64],
        z_vals: NDArray[np.float64],
        tex_type: str,
        base_color: tuple[int, int, int],
        seed: int,
        brightness: float = 1.0,
    ) -> None:
        """Fan-triangulate a face and rasterize with z-test."""
        pts = verts_2d.astype(np.int32)
        x_min = max(0, int(pts[:, 0].min()))
        y_min = max(0, int(pts[:, 1].min()))
        x_max = min(w - 1, int(pts[:, 0].max()))
        y_max = min(h - 1, int(pts[:, 1].max()))

        if x_max <= x_min or y_max <= y_min:
            return

        tw = x_max - x_min + 1
        th = y_max - y_min + 1
        gen = _TEXTURE_GENERATORS.get(tex_type, _generate_fine_noise)
        texture_gray = gen(tw, th, seed)
        lit_color = tuple(int(min(255, c * brightness)) for c in base_color)
        texture_rgb = _tint_texture(texture_gray, lit_color)

        for k in range(1, len(verts_2d) - 1):
            pu = np.array([verts_2d[0, 0], verts_2d[k, 0], verts_2d[k + 1, 0]])
            pv = np.array([verts_2d[0, 1], verts_2d[k, 1], verts_2d[k + 1, 1]])
            zv = np.array([z_vals[0], z_vals[k], z_vals[k + 1]])
            _rasterize_triangle_textured(
                pu, pv, zv, depth_buf, img_arr, texture_rgb, x_min, y_min
            )

    # Ground plane — tiled into small quads so that quads behind the
    # camera are individually culled instead of killing the entire plane.
    if show_ground:
        tile_step = 4.0
        tile_seed = texture_seed + 9000
        nx = int(2 * ground_size / tile_step)
        for ti in range(nx):
            for tj in range(nx):
                x0 = -ground_size + ti * tile_step
                z0 = -ground_size + tj * tile_step
                x1 = x0 + tile_step
                z1 = z0 + tile_step
                tile_verts = np.array(
                    [
                        [x0, ground_y, z0],
                        [x0, ground_y, z1],
                        [x1, ground_y, z1],
                        [x1, ground_y, z0],
                    ],
                    dtype=np.float64,
                )
                # Back-face cull: ground normal is +Y
                centroid_3d = tile_verts.mean(axis=0)
                ground_normal = np.array([0.0, 1.0, 0.0])
                if np.dot(ground_normal, cam_pos - centroid_3d) <= 0:
                    tile_seed += 1
                    continue
                tv_2d, tz, t_valid = _project_face_verts(
                    tile_verts, rot, trans, fx, fy, cx, cy
                )
                if t_valid:
                    _rasterize_face(
                        tv_2d,
                        tz,
                        "grass",
                        (90, 155, 60),
                        tile_seed,
                        brightness=_face_brightness(ground_normal),
                    )
                tile_seed += 1

    # Building faces
    prim_idx = 0
    for prim, pose in scene.entries:
        world_verts = _transform_vertices(prim.vertices(), pose)
        faces = prim.face_indices()
        style = _PRIM_STYLES.get(prim.name, _DEFAULT_STYLE)
        tex_type, base_color = style

        for face_idx, face in enumerate(faces):
            face_verts_3d = world_verts[face]

            # Back-face culling: skip faces whose normal points away
            e1 = face_verts_3d[1] - face_verts_3d[0]
            e2 = face_verts_3d[2] - face_verts_3d[0]
            normal = np.cross(e1, e2)
            centroid_3d = face_verts_3d.mean(axis=0)
            if np.dot(normal, cam_pos - centroid_3d) <= 0:
                continue

            fv_2d, fz, valid = _project_face_verts(
                face_verts_3d, rot, trans, fx, fy, cx, cy
            )
            if not valid:
                continue
            _rasterize_face(
                fv_2d,
                fz,
                tex_type,
                base_color,
                texture_seed + prim_idx * 100 + face_idx,
                brightness=_face_brightness(normal),
            )
        prim_idx += 1

    # Draw edges on top (wireframe overlay)
    if draw_edges:
        img = Image.fromarray(img_arr)
        draw = ImageDraw.Draw(img)
        segments = gt.segments_2d(view_index)
        for seg in segments:
            sp = seg.start_point()
            ep = seg.end_point()
            draw.line(
                [(sp[0], sp[1]), (ep[0], ep[1])],
                fill=line_color,
                width=thickness,
            )
        return np.array(img, dtype=np.uint8)

    return img_arr


def render_all_textured_views(
    scene: Scene,
    gt: SceneGroundTruth,
    *,
    line_color: tuple[int, int, int] = (10, 10, 10),
    thickness: int = 1,
    texture_seed: int = 42,
    show_ground: bool = True,
    ground_y: float = 0.0,
    ground_size: float = 20.0,
    light_dir: NDArray[np.float64] | None = None,
    ambient: float = 0.7,
    draw_edges: bool = True,
) -> list[NDArray[np.uint8]]:
    """Render all views as textured RGB images.

    Convenience wrapper around :func:`render_textured` for every view in
    the rig.

    :param scene: The 3D scene.
    :type scene: Scene
    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param line_color: RGB color for edge lines.
    :type line_color: tuple[int, int, int]
    :param thickness: Line thickness in pixels.
    :type thickness: int
    :param texture_seed: Base seed for procedural textures.
    :type texture_seed: int
    :param show_ground: Whether to render a ground plane at *ground_y*.
    :type show_ground: bool
    :param ground_y: World-space Y coordinate for the ground plane.
    :type ground_y: float
    :param ground_size: Half-extent of the ground plane quad.
    :type ground_size: float
    :param light_dir: World-space direction toward the light (auto-normalized).
        Defaults to ``(1, -1, 0.5)``.
    :type light_dir: numpy.ndarray or None
    :param ambient: Minimum brightness factor ``[0, 1]``.
    :type ambient: float
    :param draw_edges: Whether to draw GT edge lines on top.
    :type draw_edges: bool
    :return: List of RGB uint8 images, one per view.
    :rtype: list[numpy.ndarray]
    """
    return [
        render_textured(
            scene,
            gt,
            i,
            line_color=line_color,
            thickness=thickness,
            texture_seed=texture_seed,
            show_ground=show_ground,
            ground_y=ground_y,
            ground_size=ground_size,
            light_dir=light_dir,
            ambient=ambient,
            draw_edges=draw_edges,
        )
        for i in range(gt.n_views)
    ]
