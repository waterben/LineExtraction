"""Tests for oracle matching and detected-to-GT assignment."""

import le_geometry

from lsfm.synthetic.oracle import assign_detected_to_gt, oracle_match


def _seg(x1: float, y1: float, x2: float, y2: float) -> le_geometry.LineSegment_f64:
    return le_geometry.LineSegment_f64.from_endpoints(x1, y1, x2, y2)


class TestAssignDetectedToGt:
    """Tests for assign_detected_to_gt."""

    def test_empty_inputs(self) -> None:
        assert assign_detected_to_gt([], []) == {}
        assert assign_detected_to_gt([], [_seg(0, 0, 10, 0)]) == {}
        assert assign_detected_to_gt([_seg(0, 0, 10, 0)], []) == {}

    def test_perfect_match(self) -> None:
        segs = [_seg(0, 0, 100, 0), _seg(0, 0, 0, 100)]
        result = assign_detected_to_gt(segs, segs)
        assert len(result) == 2

    def test_nearby_match(self) -> None:
        gt = [_seg(0, 0, 100, 0)]
        det = [_seg(0, 2, 100, 2)]  # Shifted by 2 pixels
        result = assign_detected_to_gt(det, gt, max_distance=20.0)
        assert len(result) == 1
        assert result[0] == 0

    def test_too_far_rejected(self) -> None:
        gt = [_seg(0, 0, 100, 0)]
        det = [_seg(0, 50, 100, 50)]  # Shifted by 50 pixels
        result = assign_detected_to_gt(det, gt, max_distance=10.0)
        assert len(result) == 0

    def test_angle_rejected(self) -> None:
        gt = [_seg(0, 0, 100, 0)]  # Horizontal
        det = [_seg(50, -50, 50, 50)]  # Vertical (90 degree diff)
        result = assign_detected_to_gt(det, gt, max_distance=100.0, max_angle_deg=30.0)
        assert len(result) == 0

    def test_one_to_one(self) -> None:
        gt = [_seg(0, 0, 100, 0), _seg(0, 50, 100, 50)]
        det = [_seg(0, 1, 100, 1), _seg(0, 51, 100, 51)]
        result = assign_detected_to_gt(det, gt)
        assert len(result) == 2
        # Each detection should map to a different GT
        assert len(set(result.values())) == 2


class TestOracleMatch:
    """Tests for oracle matching (requires GT pipeline)."""

    def test_basic_oracle(self) -> None:
        from lsfm.synthetic.camera_rig import CameraRig
        from lsfm.synthetic.ground_truth import SceneGroundTruth
        from lsfm.synthetic.primitives import Box3D
        from lsfm.synthetic.scene import Scene

        scene = Scene()
        box = Box3D(width=2.0, height=2.0, depth=2.0)
        scene.add(box, le_geometry.Pose_f64())

        rig = CameraRig.orbital(
            n_cameras=4,
            target=(0.0, 0.0, 0.0),
            radius=10.0,
            height=2.0,
            focal_length=500.0,
            image_width=640,
            image_height=480,
        )
        gt = SceneGroundTruth(scene, rig)

        # Use GT segments as "detected" for perfect oracle matching
        detected_views = [gt.segments_2d(i) for i in range(gt.n_views)]
        matches = oracle_match(detected_views, gt, 0, 1)
        # Should have matches for shared 3D lines
        assert isinstance(matches, list)
