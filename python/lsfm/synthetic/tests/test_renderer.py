"""Tests for wireframe and textured rendering."""

import numpy as np

import le_geometry

from lsfm.synthetic.camera_rig import CameraRig
from lsfm.synthetic.ground_truth import SceneGroundTruth
from lsfm.synthetic.primitives import Box3D
from lsfm.synthetic.renderer import (
    render_all_textured_views,
    render_all_views,
    render_textured,
    render_wireframe,
)
from lsfm.synthetic.scene import Scene


def _make_test_gt() -> tuple[Scene, SceneGroundTruth]:
    """Create a simple GT for rendering tests."""
    scene = Scene()
    box = Box3D(width=2.0, height=2.0, depth=2.0)
    scene.add(box, le_geometry.Pose_f64())
    rig = CameraRig.orbital(
        n_cameras=4,
        target=(0.0, 0.0, 0.0),
        radius=10.0,
        height=2.0,
        focal_length=500.0,
        image_width=320,
        image_height=240,
    )
    gt = SceneGroundTruth(scene, rig)
    return scene, gt


class TestRenderWireframe:
    """Tests for wireframe rendering."""

    def test_output_shape(self) -> None:
        _, gt = _make_test_gt()
        img = render_wireframe(gt, 0)
        assert img.shape == (240, 320)
        assert img.dtype == np.uint8

    def test_background_color(self) -> None:
        _, gt = _make_test_gt()
        img = render_wireframe(gt, 0, background=128)
        # Most pixels should be background
        bg_count = np.sum(img == 128)
        assert bg_count > img.size * 0.5

    def test_has_drawn_lines(self) -> None:
        _, gt = _make_test_gt()
        img = render_wireframe(gt, 0, background=255, line_color=0)
        # Should have some dark pixels (lines)
        dark_pixels = np.sum(img < 128)
        assert dark_pixels > 0


class TestRenderAllViews:
    """Tests for render_all_views."""

    def test_returns_correct_count(self) -> None:
        _, gt = _make_test_gt()
        images = render_all_views(gt)
        assert len(images) == 4

    def test_all_images_correct_shape(self) -> None:
        _, gt = _make_test_gt()
        images = render_all_views(gt)
        for img in images:
            assert img.shape == (240, 320)


class TestRenderTextured:
    """Tests for textured rendering (returns RGB)."""

    def test_output_shape(self) -> None:
        scene, gt = _make_test_gt()
        img = render_textured(scene, gt, 0)
        assert img.shape == (240, 320, 3)
        assert img.dtype == np.uint8

    def test_has_texture_variation(self) -> None:
        scene, gt = _make_test_gt()
        img = render_textured(scene, gt, 0)
        # Textured image should have meaningful color variation
        assert img.std() > 5.0

    def test_has_edges(self) -> None:
        scene, gt = _make_test_gt()
        img = render_textured(scene, gt, 0)
        # Edge lines should produce dark pixels (default line_color=(10,10,10))
        gray = img.mean(axis=2)
        dark_pixels = np.sum(gray < 50)
        assert dark_pixels > 0


class TestRenderAllTexturedViews:
    """Tests for render_all_textured_views."""

    def test_returns_correct_count(self) -> None:
        scene, gt = _make_test_gt()
        images = render_all_textured_views(scene, gt)
        assert len(images) == 4

    def test_all_images_correct_shape(self) -> None:
        scene, gt = _make_test_gt()
        images = render_all_textured_views(scene, gt)
        for img in images:
            assert img.shape == (240, 320, 3)
            assert img.dtype == np.uint8
