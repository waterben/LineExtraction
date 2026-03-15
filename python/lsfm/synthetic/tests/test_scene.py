"""Tests for scene composition and vertex welding."""

import numpy as np

import le_geometry

from lsfm.synthetic.primitives import Box3D
from lsfm.synthetic.scene import Scene, build_church_scene


class TestScene:
    """Tests for Scene composition."""

    def test_empty_scene(self) -> None:
        scene = Scene()
        assert scene.ground_truth_lines() == []

    def test_single_box(self) -> None:
        scene = Scene()
        box = Box3D(width=2.0, height=2.0, depth=2.0)
        pose = le_geometry.Pose_f64()
        scene.add(box, pose)
        lines = scene.ground_truth_lines()
        assert len(lines) == 12

    def test_vertex_welding_shared_edge(self) -> None:
        """Two boxes sharing a face should have shared edges deduplicated."""
        scene = Scene()
        box1 = Box3D(width=2.0, height=2.0, depth=2.0)
        box2 = Box3D(width=2.0, height=2.0, depth=2.0)

        # Place box2 adjacent to box1 (touching along X)
        pose1 = le_geometry.Pose_f64(0.0, 0.0, 0.0)
        pose2 = le_geometry.Pose_f64(2.0, 0.0, 0.0)
        scene.add(box1, pose1)
        scene.add(box2, pose2)

        lines = scene.ground_truth_lines()
        # Two separate boxes: 12+12=24 edges
        # Shared face has 4 edges, so 24-4=20
        assert len(lines) == 20

    def test_bounding_box(self) -> None:
        scene = Scene()
        box = Box3D(width=2.0, height=4.0, depth=6.0)
        scene.add(box, le_geometry.Pose_f64())
        bb_min, bb_max = scene.bounding_box()
        np.testing.assert_allclose(bb_min, [-1.0, -2.0, -3.0])
        np.testing.assert_allclose(bb_max, [1.0, 2.0, 3.0])

    def test_center(self) -> None:
        scene = Scene()
        box = Box3D(width=2.0, height=2.0, depth=2.0)
        scene.add(box, le_geometry.Pose_f64(10.0, 0.0, 0.0))
        center = scene.center()
        np.testing.assert_allclose(center, [10.0, 0.0, 0.0])


class TestBuildChurchScene:
    """Tests for the predefined church scene."""

    def test_creates_scene(self) -> None:
        scene = build_church_scene()
        assert len(scene.entries) == 4

    def test_has_ground_truth_lines(self) -> None:
        scene = build_church_scene()
        lines = scene.ground_truth_lines()
        assert len(lines) > 0

    def test_bounding_box_reasonable(self) -> None:
        scene = build_church_scene()
        bb_min, bb_max = scene.bounding_box()
        assert bb_max[1] > bb_min[1]  # has height
        assert bb_max[0] > bb_min[0]  # has width
