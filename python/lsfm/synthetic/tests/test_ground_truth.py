"""Tests for the ground truth projection pipeline."""

import numpy as np
import pytest

import le_geometry

from lsfm.synthetic.camera_rig import CameraRig, _make_camera
from lsfm.synthetic.ground_truth import (
    SceneGroundTruth,
    _camera_extrinsics,
    clip_to_frustum,
    filter_behind_camera,
)
from lsfm.synthetic.primitives import Box3D
from lsfm.synthetic.scene import Scene


def _make_simple_scene() -> Scene:
    """Create a simple scene with one box at the origin."""
    scene = Scene()
    box = Box3D(width=2.0, height=2.0, depth=2.0)
    scene.add(box, le_geometry.Pose_f64())
    return scene


def _make_simple_rig() -> CameraRig:
    """Create a simple orbital rig around the origin."""
    return CameraRig.orbital(
        n_cameras=4,
        target=(0.0, 0.0, 0.0),
        radius=10.0,
        height=2.0,
        focal_length=500.0,
        image_width=640,
        image_height=480,
    )


class TestFilterBehindCamera:
    """Tests for filter_behind_camera."""

    def test_segment_in_front(self) -> None:
        # Camera at (0,0,-10) looking toward origin along +Z
        cam = _make_camera(
            np.array([0.0, 0.0, -10.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            500.0,
            320.0,
            240.0,
            640,
            480,
        )
        seg = le_geometry.LineSegment3_f64.from_endpoints(-1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        result = filter_behind_camera([seg], cam)
        assert len(result) == 1

    def test_segment_behind(self) -> None:
        # Camera at (0,0,-10) looking toward origin
        # Segment at Z=-20 is behind the camera
        cam = _make_camera(
            np.array([0.0, 0.0, -10.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            500.0,
            320.0,
            240.0,
            640,
            480,
        )
        seg = le_geometry.LineSegment3_f64.from_endpoints(
            -1.0, 0.0, -20.0, 1.0, 0.0, -20.0
        )
        result = filter_behind_camera([seg], cam)
        assert len(result) == 0

    def test_camera_extrinsics_eye_maps_to_zero(self) -> None:
        """Eye position transformed to camera frame should be (0,0,0)."""
        cam = _make_camera(
            np.array([3.0, 5.0, -7.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            500.0,
            320.0,
            240.0,
            640,
            480,
        )
        r_w2c, t_w2c = _camera_extrinsics(cam)
        p_cam = r_w2c @ np.array([3.0, 5.0, -7.0]) + t_w2c
        np.testing.assert_allclose(p_cam, [0, 0, 0], atol=1e-6)


class TestClipToFrustum:
    """Tests for clip_to_frustum."""

    def test_segment_inside(self) -> None:
        seg = le_geometry.LineSegment_f64.from_endpoints(10.0, 10.0, 100.0, 100.0)
        result = clip_to_frustum([seg], 640.0, 480.0)
        assert len(result) == 1

    def test_segment_outside(self) -> None:
        seg = le_geometry.LineSegment_f64.from_endpoints(-100.0, -100.0, -50.0, -50.0)
        result = clip_to_frustum([seg], 640.0, 480.0)
        assert len(result) == 0

    def test_segment_partially_visible(self) -> None:
        seg = le_geometry.LineSegment_f64.from_endpoints(-100.0, 240.0, 320.0, 240.0)
        result = clip_to_frustum([seg], 640.0, 480.0)
        assert len(result) == 1
        # Clipped segment should start at x=0
        clipped_seg = result[0][1]
        sp = clipped_seg.start_point()
        assert sp[0] == pytest.approx(0.0, abs=1.0)


class TestSceneGroundTruth:
    """Tests for SceneGroundTruth coordinator."""

    def test_creation(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        assert gt.n_views == 4

    def test_has_gt_lines(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        assert len(gt.gt_lines_3d) == 12  # Box has 12 edges

    def test_segments_2d_not_empty(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        # At least some views should have visible segments
        total_segs = sum(len(gt.segments_2d(i)) for i in range(gt.n_views))
        assert total_segs > 0

    def test_parent_indices_consistent(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        for i in range(gt.n_views):
            segs = gt.segments_2d(i)
            parents = gt.parent_indices(i)
            assert len(segs) == len(parents)

    def test_correspondences(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        corrs = gt.correspondences(0, 1)
        # Should have some correspondences between adjacent views
        assert isinstance(corrs, list)

    def test_width_height(self) -> None:
        scene = _make_simple_scene()
        rig = _make_simple_rig()
        gt = SceneGroundTruth(scene, rig)
        assert gt.width == pytest.approx(640.0)
        assert gt.height == pytest.approx(480.0)
