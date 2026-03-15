"""Tests for camera rig generator."""

import numpy as np
import pytest

import le_geometry

from lsfm.synthetic.camera_rig import (
    CameraRig,
    _look_at,
    _make_camera,
    _rotation_matrix_to_rodrigues,
)


class TestLookAt:
    """Tests for the look-at helper."""

    def test_identity_look_forward(self) -> None:
        eye = np.array([0.0, 0.0, 0.0])
        target = np.array([0.0, 0.0, 1.0])
        up = np.array([0.0, 1.0, 0.0])
        rot, trans = _look_at(eye, target, up)
        assert rot.shape == (3, 3)
        assert trans.shape == (3,)
        # Camera at origin -> translation should be near zero
        np.testing.assert_allclose(trans, [0.0, 0.0, 0.0], atol=1e-10)

    def test_rotation_is_orthogonal(self) -> None:
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        rot, _ = _look_at(eye, target, up)
        # R^T * R should be identity
        np.testing.assert_allclose(rot @ rot.T, np.eye(3), atol=1e-10)

    def test_determinant_is_one(self) -> None:
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        rot, _ = _look_at(eye, target, up)
        assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-10)


class TestRodrigues:
    """Tests for rotation matrix to Rodrigues conversion."""

    def test_identity(self) -> None:
        rvec = _rotation_matrix_to_rodrigues(np.eye(3))
        np.testing.assert_allclose(rvec, [0.0, 0.0, 0.0], atol=1e-10)

    def test_roundtrip(self) -> None:
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        rot, _ = _look_at(eye, target, up)
        rvec = _rotation_matrix_to_rodrigues(rot)
        # Reconstruct using Pose
        pose = le_geometry.Pose_f64(0.0, 0.0, 0.0, rvec[0], rvec[1], rvec[2])
        rot_reconstructed = np.array(
            [[pose.rot_matrix()[r][c] for c in range(3)] for r in range(3)]
        )
        np.testing.assert_allclose(rot_reconstructed, rot, atol=1e-6)


class TestCameraRig:
    """Tests for CameraRig."""

    def test_orbital_count(self) -> None:
        rig = CameraRig.orbital(n_cameras=6, radius=10.0)
        assert len(rig) == 6

    def test_orbital_cameras_have_intrinsics(self) -> None:
        rig = CameraRig.orbital(
            n_cameras=4,
            focal_length=500.0,
            image_width=640,
            image_height=480,
        )
        for cam in rig.cameras:
            fx, fy = cam.focal
            assert fx == pytest.approx(500.0)
            assert fy == pytest.approx(500.0)
            assert cam.width == pytest.approx(640.0)
            assert cam.height == pytest.approx(480.0)

    def test_projectors_cached(self) -> None:
        rig = CameraRig.orbital(n_cameras=3)
        proj1 = rig.projectors
        proj2 = rig.projectors
        assert proj1 is proj2

    def test_camera_pair(self) -> None:
        rig = CameraRig.orbital(n_cameras=4)
        c0, c1 = rig.camera_pair(0, 1)
        assert c0 is rig.cameras[0]
        assert c1 is rig.cameras[1]

    def test_pairs_yields_adjacent(self) -> None:
        rig = CameraRig.orbital(n_cameras=5)
        pairs = list(rig.pairs())
        assert pairs == [(0, 1), (1, 2), (2, 3), (3, 4)]

    def test_linear_count(self) -> None:
        rig = CameraRig.linear(n_cameras=4)
        assert len(rig) == 4

    def test_from_poses(self) -> None:
        poses = [
            le_geometry.Pose_f64(0.0, 0.0, -10.0),
            le_geometry.Pose_f64(2.0, 0.0, -10.0),
        ]
        rig = CameraRig.from_poses(poses, focal_length=500.0)
        assert len(rig) == 2

    def test_orbital_origin_is_eye_position(self) -> None:
        """Camera origin should be the world-space eye, not w2c translation."""
        rig = CameraRig.orbital(
            n_cameras=4,
            target=(0.0, 0.0, 0.0),
            radius=10.0,
            height=2.0,
        )
        cam0 = rig.cameras[0]
        eye = np.array(cam0.origin)
        # Camera 0 is at angle=0 → eye = (10, 2, 0)
        np.testing.assert_allclose(eye, [10.0, 2.0, 0.0], atol=1e-6)

    def test_make_camera_stores_eye(self) -> None:
        """_make_camera should store eye position, not t_w2c."""
        eye = np.array([5.0, 3.0, -10.0])
        target = np.array([0.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        cam = _make_camera(eye, target, up, 500.0, 320.0, 240.0, 640, 480)
        np.testing.assert_allclose(cam.origin, eye, atol=1e-6)
