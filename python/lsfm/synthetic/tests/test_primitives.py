"""Tests for 3D geometric primitives."""

import numpy as np
import pytest

from lsfm.synthetic.primitives import Box3D, Primitive, Pyramid3D


class TestBox3D:
    """Tests for Box3D primitive."""

    def test_is_primitive(self) -> None:
        box = Box3D()
        assert isinstance(box, Primitive)

    def test_name(self) -> None:
        box = Box3D(name="test_box")
        assert box.name == "test_box"

    def test_default_name(self) -> None:
        box = Box3D()
        assert box.name == "box"

    def test_vertices_shape(self) -> None:
        box = Box3D(width=2.0, height=3.0, depth=4.0)
        verts = box.vertices()
        assert verts.shape == (8, 3)
        assert verts.dtype == np.float64

    def test_vertices_centered(self) -> None:
        box = Box3D(width=2.0, height=2.0, depth=2.0)
        verts = box.vertices()
        center = verts.mean(axis=0)
        np.testing.assert_allclose(center, [0.0, 0.0, 0.0], atol=1e-12)

    def test_vertices_extents(self) -> None:
        box = Box3D(width=4.0, height=6.0, depth=8.0)
        verts = box.vertices()
        extents = verts.max(axis=0) - verts.min(axis=0)
        np.testing.assert_allclose(extents, [4.0, 6.0, 8.0])

    def test_edge_count(self) -> None:
        box = Box3D()
        edges = box.edge_indices()
        assert len(edges) == 12

    def test_edges_unique(self) -> None:
        box = Box3D()
        edges = box.edge_indices()
        normalized = {(min(a, b), max(a, b)) for a, b in edges}
        assert len(normalized) == 12

    def test_face_count(self) -> None:
        box = Box3D()
        faces = box.face_indices()
        assert len(faces) == 6

    def test_face_vertex_indices_valid(self) -> None:
        box = Box3D()
        faces = box.face_indices()
        for face in faces:
            assert len(face) == 4
            for idx in face:
                assert 0 <= idx < 8


class TestPyramid3D:
    """Tests for Pyramid3D primitive."""

    def test_is_primitive(self) -> None:
        pyr = Pyramid3D()
        assert isinstance(pyr, Primitive)

    def test_name(self) -> None:
        pyr = Pyramid3D(name="test_pyramid")
        assert pyr.name == "test_pyramid"

    def test_vertices_shape(self) -> None:
        pyr = Pyramid3D(base_width=2.0, base_depth=2.0, height=3.0)
        verts = pyr.vertices()
        assert verts.shape == (5, 3)
        assert verts.dtype == np.float64

    def test_apex_height(self) -> None:
        pyr = Pyramid3D(height=5.0)
        verts = pyr.vertices()
        apex = verts[4]
        assert apex[1] == pytest.approx(5.0)

    def test_base_at_y_zero(self) -> None:
        pyr = Pyramid3D()
        verts = pyr.vertices()
        base = verts[:4]
        np.testing.assert_allclose(base[:, 1], 0.0)

    def test_edge_count(self) -> None:
        pyr = Pyramid3D()
        edges = pyr.edge_indices()
        assert len(edges) == 8

    def test_face_count(self) -> None:
        pyr = Pyramid3D()
        faces = pyr.face_indices()
        assert len(faces) == 5

    def test_face_vertex_indices_valid(self) -> None:
        pyr = Pyramid3D()
        faces = pyr.face_indices()
        for face in faces:
            for idx in face:
                assert 0 <= idx < 5
