"""End-to-end integration tests for the synthetic pipeline.

Tests each stage independently so failures are easy to isolate:

1. Camera convention — Pose stores camera-to-world params
2. Projection sanity — known 3D point projects to expected 2D pixel
3. Filter behind camera — correctness of world-to-camera transform
4. Full GT pipeline — box scene with orbital cameras
5. Wireframe rendering — output image dimensions and content
6. Textured rendering — painter's algorithm face ordering
7. Oracle matching — cross-view GT assignment
8. Evaluation metrics — detection/matching/reconstruction scoring
"""

import numpy as np
import pytest

import le_geometry

from lsfm.synthetic.camera_rig import (
    CameraRig,
    _make_camera,
)
from lsfm.synthetic.evaluation import (
    evaluate_detection,
    evaluate_reconstruction,
    evaluation_summary,
)
from lsfm.synthetic.ground_truth import (
    SceneGroundTruth,
    _camera_extrinsics,
    filter_behind_camera,
)
from lsfm.synthetic.oracle import assign_detected_to_gt, oracle_match
from lsfm.synthetic.primitives import Box3D
from lsfm.synthetic.renderer import render_all_views, render_textured, render_wireframe
from lsfm.synthetic.scene import Scene, build_church_scene


# =====================================================================
# Helper factories
# =====================================================================


def _box_scene() -> Scene:
    """Simple box at origin."""
    scene = Scene()
    scene.add(Box3D(2.0, 2.0, 2.0), le_geometry.Pose_f64())
    return scene


def _orbital_rig(
    n: int = 8,
    target: tuple[float, float, float] = (0.0, 0.0, 0.0),
    radius: float = 10.0,
    height: float = 3.0,
) -> CameraRig:
    return CameraRig.orbital(
        n_cameras=n,
        target=target,
        radius=radius,
        height=height,
        focal_length=500.0,
        image_width=640,
        image_height=480,
    )


# =====================================================================
# 1. Camera convention tests
# =====================================================================


class TestCameraConvention:
    """Verify the Camera stores camera-to-world (Pose convention)."""

    def test_origin_is_eye_position(self) -> None:
        """Camera origin should be the world-space eye position."""
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        np.testing.assert_allclose(cam.origin, eye, atol=1e-6)

    def test_hom_matrix_is_camera_to_world(self) -> None:
        """hom_matrix() should be [R_c2w | eye; 0 0 0 1]."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)

        hm = np.array(cam.hom_matrix(), dtype=np.float64)
        # Last column should contain the eye position
        np.testing.assert_allclose(hm[:3, 3], eye, atol=1e-6)
        # Bottom row should be [0 0 0 1]
        np.testing.assert_allclose(hm[3, :], [0, 0, 0, 1], atol=1e-12)

    def test_base_h_is_world_to_camera(self) -> None:
        """base_h_matrix() should invert hom_matrix()."""
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)

        hm = np.array(cam.hom_matrix(), dtype=np.float64)
        bh = np.array(cam.base_h_matrix(), dtype=np.float64)
        np.testing.assert_allclose(bh @ hm, np.eye(4), atol=1e-10)

    def test_camera_extrinsics_returns_w2c(self) -> None:
        """_camera_extrinsics must return (R_w2c, t_w2c)."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)

        r_w2c, t_w2c = _camera_extrinsics(cam)
        # A point at the eye should map to camera origin (0,0,0)
        p_cam = r_w2c @ eye + t_w2c
        np.testing.assert_allclose(p_cam, [0, 0, 0], atol=1e-6)

    def test_camera_extrinsics_target_along_z(self) -> None:
        """Target should be along +Z in camera frame."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)

        r_w2c, t_w2c = _camera_extrinsics(cam)
        target_cam = r_w2c @ target + t_w2c
        # Target should be at Z=10 in camera frame (10 units in front)
        assert target_cam[2] == pytest.approx(10.0, abs=1e-6)
        # X and Y should be near zero (target on optical axis)
        assert abs(target_cam[0]) < 1e-6
        assert abs(target_cam[1]) < 1e-6


# =====================================================================
# 2. Projection sanity
# =====================================================================


class TestProjectionSanity:
    """Verify that 3D points project to sensible 2D coordinates."""

    def test_target_projects_to_center(self) -> None:
        """The look-at target should project near the principal point."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        proj = le_geometry.CameraPluecker_f64(cam)

        px = proj.project_point(0.0, 0.0, 0.0)
        # Should be near (320, 240) — the principal point
        assert px[0] == pytest.approx(320.0, abs=1.0)
        assert px[1] == pytest.approx(240.0, abs=1.0)

    def test_point_right_projects_right(self) -> None:
        """A point to the right in world should project to the right."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        proj = le_geometry.CameraPluecker_f64(cam)

        center = proj.project_point(0.0, 0.0, 0.0)
        right = proj.project_point(1.0, 0.0, 0.0)
        # +X world → +X image (right)
        assert right[0] > center[0]

    def test_point_above_projects_up(self) -> None:
        """A point higher in world (Y-up) should project upward (lower Y)."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        proj = le_geometry.CameraPluecker_f64(cam)

        center = proj.project_point(0.0, 0.0, 0.0)
        above = proj.project_point(0.0, 1.0, 0.0)
        # +Y world (up) → -Y image (up in Y-down image coords)
        assert above[1] < center[1]

    def test_segment_projects_within_image(self) -> None:
        """A segment near the target should project within image bounds."""
        eye = np.array([0.0, 0.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        proj = le_geometry.CameraPluecker_f64(cam)

        seg3d = le_geometry.LineSegment3_f64.from_endpoints(
            -0.5, 0.0, 0.0, 0.5, 0.0, 0.0
        )
        seg2d = proj.project_line_segment(seg3d)
        sp = seg2d.start_point()
        ep = seg2d.end_point()
        for pt in [sp, ep]:
            assert 0 <= pt[0] <= 640
            assert 0 <= pt[1] <= 480


# =====================================================================
# 3. Filter behind camera
# =====================================================================


class TestFilterBehindCameraIntegration:
    """Verify filter_behind_camera with known geometry."""

    def test_in_front_segment_kept(self) -> None:
        """A segment in front of (and visible to) the camera is kept."""
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

    def test_behind_segment_removed(self) -> None:
        """A segment behind the camera is removed."""
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
        # Segment at Z=-20 is behind a camera at Z=-10 looking toward +Z
        seg = le_geometry.LineSegment3_f64.from_endpoints(
            -1.0, 0.0, -20.0, 1.0, 0.0, -20.0
        )
        result = filter_behind_camera([seg], cam)
        assert len(result) == 0

    def test_all_box_edges_visible_from_front(self) -> None:
        """All 12 box edges at origin should be visible from Z=-10."""
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
        scene = _box_scene()
        lines = scene.ground_truth_lines()
        result = filter_behind_camera(lines, cam)
        # All 12 edges should be in front
        assert len(result) == 12


# =====================================================================
# 4. Full GT pipeline
# =====================================================================


class TestGTPipeline:
    """Integration tests for the full ground truth pipeline."""

    def test_box_all_views_have_segments(self) -> None:
        """Every orbital view of a centered box should see some edges."""
        scene = _box_scene()
        rig = _orbital_rig(n=8)
        gt = SceneGroundTruth(scene, rig)
        for i in range(gt.n_views):
            segs = gt.segments_2d(i)
            assert len(segs) > 0, f"View {i} has no visible segments"

    def test_box_reasonable_segment_count(self) -> None:
        """Each view should see between 4 and 12 edges of a box."""
        scene = _box_scene()
        rig = _orbital_rig(n=8)
        gt = SceneGroundTruth(scene, rig)
        for i in range(gt.n_views):
            n_segs = len(gt.segments_2d(i))
            assert 4 <= n_segs <= 12, f"View {i}: {n_segs} segments (expected 4-12)"

    def test_segments_within_image(self) -> None:
        """Every projected 2D segment should lie within image bounds."""
        scene = _box_scene()
        rig = _orbital_rig(n=8)
        gt = SceneGroundTruth(scene, rig)
        for i in range(gt.n_views):
            for seg in gt.segments_2d(i):
                sp = seg.start_point()
                ep = seg.end_point()
                for pt in [sp, ep]:
                    assert -1.0 <= pt[0] <= 641.0, f"View {i}: x={pt[0]} out of range"
                    assert -1.0 <= pt[1] <= 481.0, f"View {i}: y={pt[1]} out of range"

    def test_adjacent_views_share_correspondences(self) -> None:
        """Adjacent camera views should share at least a few correspondences."""
        scene = _box_scene()
        rig = _orbital_rig(n=8)
        gt = SceneGroundTruth(scene, rig)
        total_corrs = 0
        for i, j in rig.pairs():
            corrs = gt.correspondences(i, j)
            total_corrs += len(corrs)
        assert total_corrs > 0, "No correspondences found between any adjacent views"

    def test_church_scene_pipeline(self) -> None:
        """Church scene with 8 cameras should produce many segments."""
        scene = build_church_scene()
        center = scene.center()
        rig = _orbital_rig(n=8, target=tuple(center), radius=12.0, height=5.0)
        gt = SceneGroundTruth(scene, rig)

        total_segs = sum(len(gt.segments_2d(i)) for i in range(gt.n_views))
        assert total_segs > 20, f"Church scene: only {total_segs} total 2D segments"

    def test_church_majority_views_have_segments(self) -> None:
        """At least 6 of 8 views should see > 5 segments for the church."""
        scene = build_church_scene()
        center = scene.center()
        rig = _orbital_rig(n=8, target=tuple(center), radius=12.0, height=5.0)
        gt = SceneGroundTruth(scene, rig)

        good_views = sum(1 for i in range(8) if len(gt.segments_2d(i)) > 5)
        assert good_views >= 6, f"Only {good_views}/8 views have >5 segments"


# =====================================================================
# 5. Rendering
# =====================================================================


class TestRendering:
    """Integration tests for wireframe and textured rendering."""

    def test_wireframe_shape(self) -> None:
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        img = render_wireframe(gt, 0)
        assert img.shape == (480, 640)
        assert img.dtype == np.uint8

    def test_wireframe_has_lines(self) -> None:
        """Wireframe should have some dark pixels (lines) on light bg."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        img = render_wireframe(gt, 0, background=255, line_color=0)
        dark_pixels = np.count_nonzero(img < 128)
        assert dark_pixels > 0, "No lines drawn in wireframe"

    def test_all_views_renderable(self) -> None:
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        images = render_all_views(gt)
        assert len(images) == 4
        for img in images:
            assert img.shape == (480, 640)

    def test_textured_has_variation(self) -> None:
        """Textured render should have more variation than plain white."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        img = render_textured(scene, gt, 0)
        assert img.shape == (480, 640, 3)
        # Should have meaningful variation (textures + edges)
        assert img.std() > 5.0, "Textured image too uniform"


# =====================================================================
# 6. Oracle matching
# =====================================================================


class TestOracleIntegration:
    """Integration tests for oracle matching."""

    def test_gt_segments_match_themselves(self) -> None:
        """Using GT as 'detected' should yield perfect assignment."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        for i in range(gt.n_views):
            segs = gt.segments_2d(i)
            if not segs:
                continue
            assignment = assign_detected_to_gt(segs, segs)
            # Every segment should match itself
            assert len(assignment) == len(segs)

    def test_oracle_match_adjacent_views(self) -> None:
        """Oracle matching between adjacent views should find correspondences."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        # Use GT segments as detections
        detected = [gt.segments_2d(i) for i in range(gt.n_views)]
        matches = oracle_match(detected, gt, 0, 1)
        assert len(matches) > 0, "No oracle matches between views 0-1"


# =====================================================================
# 7. Evaluation metrics
# =====================================================================


class TestEvaluationIntegration:
    """Integration tests for evaluation metrics."""

    def test_detection_perfect_score(self) -> None:
        """Using GT as both detected and GT should give perfect F1."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        segs = gt.segments_2d(0)
        if not segs:
            pytest.skip("No segments in view 0")
        metrics = evaluate_detection(segs, segs, threshold=5.0)
        assert metrics.precision == pytest.approx(1.0, abs=0.01)
        assert metrics.recall == pytest.approx(1.0, abs=0.01)

    def test_reconstruction_perfect(self) -> None:
        """Comparing GT 3D lines with themselves should give low error."""
        scene = _box_scene()
        gt_lines = scene.ground_truth_lines()
        metrics = evaluate_reconstruction(gt_lines, gt_lines)
        assert metrics.distance_mean == pytest.approx(0.0, abs=1e-3)
        assert metrics.n_matched == len(gt_lines)

    def test_summary_dict_keys(self) -> None:
        """evaluation_summary should produce expected keys."""
        scene = _box_scene()
        rig = _orbital_rig(n=4)
        gt = SceneGroundTruth(scene, rig)
        segs = gt.segments_2d(0)
        if not segs:
            pytest.skip("No segments in view 0")
        det = evaluate_detection(segs, segs, threshold=5.0)
        summary = evaluation_summary(detection=det)
        assert "det_precision" in summary
        assert "det_recall" in summary
        assert "det_f1" in summary


# =====================================================================
# 8. Full pipeline smoke test
# =====================================================================


class TestFullPipelineSmoke:
    """Smoke test running the entire pipeline end-to-end."""

    def test_church_end_to_end(self) -> None:
        """Build church scene, render, project, match, evaluate."""
        # 1. Scene
        scene = build_church_scene()
        assert len(scene.ground_truth_lines()) > 10

        # 2. Camera rig
        center = scene.center()
        rig = _orbital_rig(n=8, target=tuple(center), radius=12.0, height=5.0)

        # 3. Ground truth projection
        gt = SceneGroundTruth(scene, rig)
        assert gt.n_views == 8

        # 4. Rendering
        images = render_all_views(gt)
        assert len(images) == 8
        for img in images:
            assert img.shape == (480, 640)

        # 5. Textured rendering for at least one view
        tex_img = render_textured(scene, gt, 0)
        assert tex_img.shape == (480, 640, 3)

        # At least one view should have concrete segments (not empty)
        view_with_segs = [i for i in range(8) if len(gt.segments_2d(i)) > 0]
        assert len(view_with_segs) >= 4, (
            f"Only {len(view_with_segs)} views have segments"
        )

        # 6. Oracle matching between first two non-empty views
        if len(view_with_segs) >= 2:
            vi, vj = view_with_segs[0], view_with_segs[1]
            detected = {
                vi: gt.segments_2d(vi),
                vj: gt.segments_2d(vj),
            }
            detected_list = [
                gt.segments_2d(i) if i in detected else [] for i in range(8)
            ]
            matches = oracle_match(detected_list, gt, vi, vj)
            # Should get at least some matches
            assert isinstance(matches, list)

        # 7. Evaluation
        for i in view_with_segs[:2]:
            segs = gt.segments_2d(i)
            det_met = evaluate_detection(segs, segs, threshold=5.0)
            assert det_met.f1 == pytest.approx(1.0, abs=0.01)

        # 8. 3D metrics
        gt_lines = scene.ground_truth_lines()
        recon_met = evaluate_reconstruction(gt_lines, gt_lines)
        assert recon_met.distance_mean < 1e-3
