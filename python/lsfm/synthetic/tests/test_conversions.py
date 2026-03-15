"""Tests for segment-numpy conversion utilities."""

import numpy as np
import pytest

import le_geometry

from lsfm.synthetic.conversions import (
    build_line_segments_3d,
    segments3d_to_array,
    segments_to_array,
)


class TestSegmentsToArray:
    """Tests for 2D segment to array conversion."""

    def test_empty(self) -> None:
        arr = segments_to_array([])
        assert arr.shape == (0, 2, 2)

    def test_single_segment(self) -> None:
        seg = le_geometry.LineSegment_f64.from_endpoints(1.0, 2.0, 3.0, 4.0)
        arr = segments_to_array([seg])
        assert arr.shape == (1, 2, 2)
        np.testing.assert_allclose(arr[0, 0], [1.0, 2.0])
        np.testing.assert_allclose(arr[0, 1], [3.0, 4.0])

    def test_multiple_segments(self) -> None:
        segs = [
            le_geometry.LineSegment_f64.from_endpoints(0.0, 0.0, 10.0, 0.0),
            le_geometry.LineSegment_f64.from_endpoints(0.0, 0.0, 0.0, 10.0),
        ]
        arr = segments_to_array(segs)
        assert arr.shape == (2, 2, 2)
        assert arr.dtype == np.float64


class TestSegments3dToArray:
    """Tests for 3D segment to array conversion."""

    def test_empty(self) -> None:
        arr = segments3d_to_array([])
        assert arr.shape == (0, 2, 3)

    def test_single_segment(self) -> None:
        seg = le_geometry.LineSegment3_f64.from_endpoints(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        arr = segments3d_to_array([seg])
        assert arr.shape == (1, 2, 3)
        np.testing.assert_allclose(arr[0, 0], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(arr[0, 1], [4.0, 5.0, 6.0])


class TestBuildLineSegments3d:
    """Tests for building 3D segments from topology."""

    def test_simple_triangle(self) -> None:
        verts = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            dtype=np.float64,
        )
        edges = [(0, 1), (1, 2), (2, 0)]
        segs = build_line_segments_3d(verts, edges)
        assert len(segs) == 3
        sp = segs[0].start_point()
        ep = segs[0].end_point()
        np.testing.assert_allclose(sp, (0.0, 0.0, 0.0))
        np.testing.assert_allclose(ep, (1.0, 0.0, 0.0))

    def test_with_pose(self) -> None:
        verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
        edges = [(0, 1)]
        pose = le_geometry.Pose_f64(10.0, 0.0, 0.0)
        segs = build_line_segments_3d(verts, edges, pose=pose)
        assert len(segs) == 1
        sp = segs[0].start_point()
        assert sp[0] == pytest.approx(10.0, abs=0.1)
