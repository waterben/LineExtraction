"""Integration tests for the le_geometry Python extension module.

Tests cover:
  - Module import and core types
  - Line construction, properties, distance, projection, intersection
  - LineSegment construction, properties, range checking, transforms
  - Polygon construction and properties
  - Drawing utilities
  - Visualization helpers
  - LineOptimizer functions
  - 3D geometry: Line3, LineSegment3, Plane, Pose
  - Camera models: Camera, CameraHom, CameraPluecker, Camera2P, CameraCV
  - Float (default) and double (_f64) presets
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import le_geometry


# ============================================================================
# Fixtures
# ============================================================================


@pytest.fixture
def blank_image_u8() -> np.ndarray:
    """Create a 200x200 black uint8 single-channel image."""
    return np.zeros((200, 200), dtype=np.uint8)


@pytest.fixture
def blank_image_bgr() -> np.ndarray:
    """Create a 200x200 black uint8 BGR image."""
    return np.zeros((200, 200, 3), dtype=np.uint8)


@pytest.fixture
def gradient_image_f32() -> np.ndarray:
    """Create a 100x100 gradient magnitude image (float32).

    Strong horizontal edge at row 50.
    """
    img = np.zeros((100, 100), dtype=np.float32)
    img[49, :] = 0.5
    img[50, :] = 1.0
    img[51, :] = 0.5
    return img


# ============================================================================
# Module-level tests
# ============================================================================


class TestModuleImport:
    """Test that the module and its core types are importable."""

    def test_core_geometry_types(self) -> None:
        assert hasattr(le_geometry, "Line")
        assert hasattr(le_geometry, "LineSegment")
        assert hasattr(le_geometry, "Polygon")

    def test_f64_preset(self) -> None:
        assert hasattr(le_geometry, "Line_f64")
        assert hasattr(le_geometry, "LineSegment_f64")
        assert hasattr(le_geometry, "Polygon_f64")

    def test_draw_functions(self) -> None:
        assert hasattr(le_geometry, "draw_lines")
        assert hasattr(le_geometry, "draw_lines_random")
        assert hasattr(le_geometry, "draw_lines_inplace")

    def test_visualization_functions(self) -> None:
        assert hasattr(le_geometry, "create_nms_color")
        assert hasattr(le_geometry, "apply_border")
        assert hasattr(le_geometry, "save_normalized")
        assert hasattr(le_geometry, "save_edge")
        assert hasattr(le_geometry, "random_color")

    def test_optimizer_functions(self) -> None:
        assert hasattr(le_geometry, "optimize_line_segment")
        assert hasattr(le_geometry, "optimize_line_segment_inplace")
        assert hasattr(le_geometry, "optimize_line_segments")


# ============================================================================
# Line tests
# ============================================================================


class TestLine:
    """Test Line class bindings."""

    def test_default_construction(self) -> None:
        line = le_geometry.Line()
        assert line.empty()

    def test_construction_from_normal(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 5.0)
        assert pytest.approx(line.normal_x) == 0.0
        assert pytest.approx(line.normal_y) == 1.0
        assert pytest.approx(line.origin_dist) == 5.0
        assert line.valid()

    def test_construction_from_point(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0, 5.0)
        assert pytest.approx(line.normal_x) == 0.0
        assert pytest.approx(line.normal_y) == 1.0
        assert pytest.approx(line.origin_dist) == 5.0

    def test_distance(self) -> None:
        # Horizontal line at y=10: normal=(0,1), dist=10
        line = le_geometry.Line(0.0, 1.0, 10.0)
        assert pytest.approx(line.distance(50.0, 10.0)) == 0.0
        assert pytest.approx(line.distance(50.0, 15.0)) == 5.0
        assert pytest.approx(line.distance(50.0, 5.0)) == -5.0

    def test_project(self) -> None:
        # Horizontal line: direction is (1, 0)
        line = le_geometry.Line(0.0, 1.0, 10.0)
        proj = line.project(25.0, 10.0)
        assert pytest.approx(proj) == 25.0

    def test_normal_project(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        nproj = line.normal_project(25.0, 15.0)
        assert pytest.approx(nproj) == 15.0

    def test_x_at_y_at(self) -> None:
        # 45-degree line: normal = (-sin(45°), cos(45°)), dist
        nx = -math.sin(math.pi / 4)
        ny = math.cos(math.pi / 4)
        line = le_geometry.Line(nx, ny, 0.0)
        assert pytest.approx(line.x_at(5.0), abs=1e-5) == 5.0
        assert pytest.approx(line.y_at(5.0), abs=1e-5) == 5.0

    def test_coordinate_transforms(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        # world2line and back
        lx, ly = line.world2line(25.0, 15.0)
        wx, wy = line.line2world(lx, ly)
        assert pytest.approx(wx, abs=1e-5) == 25.0
        assert pytest.approx(wy, abs=1e-5) == 15.0

    def test_intersection(self) -> None:
        line1 = le_geometry.Line(0.0, 1.0, 10.0)  # horizontal at y=10
        line2 = le_geometry.Line(1.0, 0.0, 20.0)  # vertical at x=20
        result = line1.intersection(line2)
        assert result is not None
        x, y = result
        assert pytest.approx(x, abs=1e-5) == 20.0
        assert pytest.approx(y, abs=1e-5) == 10.0

    def test_intersection_parallel(self) -> None:
        line1 = le_geometry.Line(0.0, 1.0, 10.0)
        line2 = le_geometry.Line(0.0, 1.0, 20.0)
        result = line1.intersection(line2)
        assert result is None

    def test_is_parallel(self) -> None:
        line1 = le_geometry.Line(0.0, 1.0, 10.0)
        line2 = le_geometry.Line(0.0, 1.0, 20.0)
        assert line1.is_parallel(line2)

    def test_angle_to(self) -> None:
        line1 = le_geometry.Line(0.0, 1.0, 0.0)  # horizontal
        line2 = le_geometry.Line(1.0, 0.0, 0.0)  # vertical
        angle = line1.angle_to(line2)
        assert pytest.approx(abs(angle), abs=1e-5) == math.pi / 2

    def test_translated_ortho(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        moved = line.translated_ortho(5.0)
        assert pytest.approx(moved.origin_dist) == 15.0
        assert pytest.approx(line.origin_dist) == 10.0  # original unchanged

    def test_translated_to(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        moved = line.translated_to(0.0, 20.0)
        assert pytest.approx(moved.origin_dist) == 20.0

    def test_rotated(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 0.0)
        rotated = line.rotated(math.pi / 2)
        # After 90° rotation, normal should be approximately (-1, 0) or (1, 0)
        assert pytest.approx(abs(rotated.normal_x), abs=1e-5) == 1.0

    def test_normal_flipped(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        flipped = line.normal_flipped()
        assert pytest.approx(flipped.normal_y) == -1.0
        assert pytest.approx(flipped.origin_dist) == -10.0

    def test_in_place_transforms(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        line.translate_ortho(5.0)
        assert pytest.approx(line.origin_dist) == 15.0
        line.normal_flip()
        assert pytest.approx(line.normal_y) == -1.0

    def test_properties(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        nx, ny = line.normal()
        dx, dy = line.direction()
        ox, oy = line.origin_point()
        assert pytest.approx(nx) == 0.0
        assert pytest.approx(ny) == 1.0
        assert pytest.approx(dy, abs=1e-5) == 0.0
        assert pytest.approx(oy) == 10.0

    def test_repr(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 10.0)
        r = repr(line)
        assert "Line" in r
        assert "10" in r

    def test_draw(self, blank_image_bgr: np.ndarray) -> None:
        line = le_geometry.Line(0.0, 1.0, 100.0)
        line.draw(blank_image_bgr)
        assert blank_image_bgr.sum() > 0


# ============================================================================
# LineSegment tests
# ============================================================================


class TestLineSegment:
    """Test LineSegment class bindings."""

    def test_default_construction(self) -> None:
        seg = le_geometry.LineSegment()
        assert seg.length == pytest.approx(0.0)

    def test_construction_from_normal(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        assert pytest.approx(seg.start) == 20.0
        assert pytest.approx(seg.end) == 80.0
        assert pytest.approx(seg.length) == 60.0
        assert seg.octave == 0

    def test_from_endpoints(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 20.0, 50.0, 20.0)
        assert pytest.approx(seg.length, abs=0.1) == 40.0

    def test_inherits_line(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        # Should have Line properties
        assert pytest.approx(seg.normal_y) == 1.0
        assert pytest.approx(seg.origin_dist) == 50.0
        dist = seg.distance(30.0, 55.0)
        assert pytest.approx(dist) == 5.0

    def test_point_accessors(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        sx, sy = seg.start_point()
        ex, ey = seg.end_point()
        cx, cy = seg.center_point()
        assert pytest.approx(sy) == 50.0
        assert pytest.approx(ey) == 50.0
        assert pytest.approx(cy) == 50.0

    def test_end_points_tuple(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        x1, y1, x2, y2 = seg.end_points()
        assert pytest.approx(y1) == 50.0
        assert pytest.approx(y2) == 50.0

    def test_in_range(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        assert seg.in_range(50.0)
        assert not seg.in_range(10.0)
        assert not seg.in_range(90.0)

    def test_in_range_point(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        assert seg.in_range_point(50.0, 50.0)
        assert not seg.in_range_point(5.0, 50.0)

    def test_trim_to_box(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(-10.0, 50.0, 110.0, 50.0)
        trimmed = seg.trim_to_box(100.0, 100.0, 0.0, 0.0)
        sx, _ = trimmed.start_point()
        ex, _ = trimmed.end_point()
        # After trimming, endpoints should be within [0, 100]
        assert sx >= -0.1
        assert ex <= 100.1

    def test_check_overlap(self) -> None:
        seg1 = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 60.0)
        seg2 = le_geometry.LineSegment(0.0, 1.0, 50.0, 40.0, 80.0)
        assert seg1.check_overlap(seg2)

    def test_error(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        ref = le_geometry.Line(0.0, 1.0, 52.0)
        err = seg.error(ref)
        assert err > 0

    def test_translated(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        moved = seg.translated(10.0)
        assert pytest.approx(moved.start) == 30.0
        assert pytest.approx(moved.end) == 90.0

    def test_rotated(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        pivot = seg.center_dist
        rotated = seg.rotated(math.pi / 2, pivot)
        assert pytest.approx(rotated.length, abs=0.5) == seg.length

    def test_scaled(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        scaled = seg.scaled(2.0)
        assert pytest.approx(scaled.length, abs=0.5) == seg.length * 2

    def test_normal_flipped(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        flipped = seg.normal_flipped()
        assert pytest.approx(flipped.normal_y) == -1.0

    def test_endpoint_swapped(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0)
        swapped = seg.endpoint_swapped()
        assert pytest.approx(swapped.start) == seg.end
        assert pytest.approx(swapped.end) == seg.start

    def test_draw(self, blank_image_bgr: np.ndarray) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 190.0, 150.0)
        seg.draw(blank_image_bgr)
        assert blank_image_bgr.sum() > 0

    def test_repr(self) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 50.0)
        r = repr(seg)
        assert "LineSegment" in r

    def test_octave(self) -> None:
        seg = le_geometry.LineSegment(0.0, 1.0, 50.0, 20.0, 80.0, 3)
        assert seg.octave == 3


# ============================================================================
# Polygon tests
# ============================================================================


class TestPolygon:
    """Test Polygon class bindings."""

    def test_empty_polygon(self) -> None:
        poly = le_geometry.Polygon()
        assert poly.empty
        assert poly.size == 0

    def test_add_vertices(self) -> None:
        poly = le_geometry.Polygon()
        poly.add_vertex(0.0, 0.0)
        poly.add_vertex(10.0, 0.0)
        poly.add_vertex(10.0, 10.0)
        poly.add_vertex(0.0, 10.0)
        assert poly.size == 4

    def test_is_convex(self) -> None:
        poly = le_geometry.Polygon()
        poly.add_vertex(0.0, 0.0)
        poly.add_vertex(10.0, 0.0)
        poly.add_vertex(10.0, 10.0)
        poly.add_vertex(0.0, 10.0)
        assert poly.is_convex

    def test_vertices_list(self) -> None:
        poly = le_geometry.Polygon()
        poly.add_vertex(1.0, 2.0)
        poly.add_vertex(3.0, 4.0)
        verts = poly.vertices
        assert len(verts) == 2
        assert pytest.approx(verts[0][0]) == 1.0
        assert pytest.approx(verts[0][1]) == 2.0

    def test_draw(self, blank_image_bgr: np.ndarray) -> None:
        poly = le_geometry.Polygon()
        poly.add_world_vertex(10.0, 10.0)
        poly.add_world_vertex(50.0, 10.0)
        poly.add_world_vertex(50.0, 50.0)
        poly.add_world_vertex(10.0, 50.0)
        poly.draw(blank_image_bgr)
        assert blank_image_bgr.sum() > 0

    def test_fill(self, blank_image_bgr: np.ndarray) -> None:
        poly = le_geometry.Polygon()
        poly.add_world_vertex(10.0, 10.0)
        poly.add_world_vertex(50.0, 10.0)
        poly.add_world_vertex(50.0, 50.0)
        poly.add_world_vertex(10.0, 50.0)
        poly.fill(blank_image_bgr)
        assert blank_image_bgr.sum() > 0

    def test_repr(self) -> None:
        poly = le_geometry.Polygon()
        poly.add_vertex(0.0, 0.0)
        r = repr(poly)
        assert "Polygon" in r


# ============================================================================
# Drawing utility tests
# ============================================================================


class TestDrawing:
    """Test module-level drawing functions."""

    @pytest.fixture
    def sample_segments(self) -> list:
        """Create a list of sample line segments."""
        return [
            le_geometry.LineSegment.from_endpoints(10.0, 20.0, 90.0, 20.0),
            le_geometry.LineSegment.from_endpoints(10.0, 50.0, 90.0, 80.0),
            le_geometry.LineSegment.from_endpoints(50.0, 10.0, 50.0, 90.0),
        ]

    def test_draw_lines(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        result = le_geometry.draw_lines(blank_image_bgr, sample_segments)
        assert result.shape == blank_image_bgr.shape
        assert result.sum() > 0

    def test_draw_lines_with_ids(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        result = le_geometry.draw_lines(blank_image_bgr, sample_segments, show_ids=True)
        assert result.sum() > 0

    def test_draw_lines_labeled(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        labels = ["A", "B", "C"]
        result = le_geometry.draw_lines_labeled(
            blank_image_bgr, sample_segments, labels
        )
        assert result.sum() > 0

    def test_draw_lines_random(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        result = le_geometry.draw_lines_random(blank_image_bgr, sample_segments)
        assert result.sum() > 0

    def test_draw_lines_inplace(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        le_geometry.draw_lines_inplace(blank_image_bgr, sample_segments)
        assert blank_image_bgr.sum() > 0

    def test_draw_lines_random_inplace(
        self, blank_image_bgr: np.ndarray, sample_segments: list
    ) -> None:
        le_geometry.draw_lines_random_inplace(blank_image_bgr, sample_segments)
        assert blank_image_bgr.sum() > 0

    def test_trim_to_box(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 50.0)
        seg = le_geometry.trim_to_box(line, 100.0, 100.0)
        assert seg.length > 0


# ============================================================================
# Visualization helper tests
# ============================================================================


class TestVisualization:
    """Test visualization helper functions."""

    def test_apply_border(self) -> None:
        img = np.ones((50, 50, 3), dtype=np.uint8) * 128
        bordered = le_geometry.apply_border(img, 10)
        assert bordered.shape[0] == 70
        assert bordered.shape[1] == 70

    def test_random_color(self) -> None:
        color = le_geometry.random_color()
        assert len(color) == 4


# ============================================================================
# LineOptimizer tests
# ============================================================================


class TestLineOptimizer:
    """Test LineOptimizer function bindings."""

    def test_optimize_line_segment(self, gradient_image_f32: np.ndarray) -> None:
        # Segment roughly along the horizontal edge at row 50
        seg = le_geometry.LineSegment.from_endpoints(10.0, 48.0, 90.0, 48.0)
        err, d, r = le_geometry.optimize_line_segment(gradient_image_f32, seg)
        # The optimizer should find the edge: d should shift toward row 50
        assert isinstance(err, float)
        assert isinstance(d, float)
        assert isinstance(r, float)

    def test_optimize_line_segment_inplace(
        self, gradient_image_f32: np.ndarray
    ) -> None:
        seg = le_geometry.LineSegment.from_endpoints(10.0, 48.0, 90.0, 48.0)
        original_dist = seg.origin_dist
        err = le_geometry.optimize_line_segment_inplace(gradient_image_f32, seg)
        assert isinstance(err, float)
        # Segment should have moved toward the edge
        assert seg.origin_dist != pytest.approx(original_dist, abs=1e-10)

    def test_optimize_line_segments_batch(self, gradient_image_f32: np.ndarray) -> None:
        segs = [
            le_geometry.LineSegment.from_endpoints(10.0, 48.0, 90.0, 48.0),
            le_geometry.LineSegment.from_endpoints(10.0, 52.0, 90.0, 52.0),
        ]
        optimized, errors = le_geometry.optimize_line_segments(gradient_image_f32, segs)
        assert len(optimized) == 2
        assert len(errors) == 2
        assert all(isinstance(e, float) for e in errors)

    def test_optimize_empty_batch(self, gradient_image_f32: np.ndarray) -> None:
        optimized, errors = le_geometry.optimize_line_segments(gradient_image_f32, [])
        assert len(optimized) == 0
        assert len(errors) == 0


# ============================================================================
# Double (f64) preset tests
# ============================================================================


class TestF64Preset:
    """Test that double-precision presets exist and work."""

    def test_line_f64(self) -> None:
        line = le_geometry.Line_f64(0.0, 1.0, 10.0)
        assert pytest.approx(line.origin_dist) == 10.0

    def test_line_segment_f64(self) -> None:
        seg = le_geometry.LineSegment_f64.from_endpoints(10.0, 20.0, 50.0, 20.0)
        assert pytest.approx(seg.length, abs=0.1) == 40.0

    def test_polygon_f64(self) -> None:
        poly = le_geometry.Polygon_f64()
        poly.add_vertex(0.0, 0.0)
        assert poly.size == 1

    def test_draw_functions_f64(self) -> None:
        assert hasattr(le_geometry, "draw_lines_f64")
        assert hasattr(le_geometry, "draw_lines_random_f64")

    def test_optimizer_f64(self) -> None:
        assert hasattr(le_geometry, "optimize_line_segment_f64")
        assert hasattr(le_geometry, "optimize_line_segments_f64")


# =============================================================================
# 3D Geometry: Line3
# =============================================================================


class TestLine3:
    """Test Line3 (3D infinite line)."""

    def test_default_construction(self) -> None:
        line = le_geometry.Line3()
        assert line.empty()

    def test_point_direction_construction(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        assert not line.empty()
        assert line.valid()
        assert line.direction == pytest.approx((1.0, 0.0, 0.0))
        assert line.origin == pytest.approx((0.0, 0.0, 0.0))

    def test_two_point(self) -> None:
        line = le_geometry.Line3.two_point(0, 0, 0, 3, 0, 0)
        assert line.direction == pytest.approx((1.0, 0.0, 0.0))

    def test_distance_to_point(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        # Point at (0, 5, 0) is 5 units from x-axis
        assert line.distance_to_point(0, 5, 0) == pytest.approx(5.0)

    def test_nearest_point(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        np_ = line.nearest_point(3, 5, 7)
        assert np_ == pytest.approx((3.0, 0.0, 0.0))

    def test_distance_origin(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 0, 0, 1)
        pt = line.distance_origin(10.0)
        assert pt == pytest.approx((0.0, 0.0, 10.0))

    def test_angle(self) -> None:
        l1 = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        l2 = le_geometry.Line3(0, 0, 0, 0, 1, 0)
        assert l1.angle_to(l2) == pytest.approx(math.pi / 2)

    def test_momentum(self) -> None:
        # p = (0,0,0), v = (1,0,0) => m = p x v = (0,0,0)
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        assert line.momentum == pytest.approx((0.0, 0.0, 0.0))

    def test_translate(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        line.translate(0, 5, 0)
        assert line.origin[1] == pytest.approx(5.0)

    def test_cayley_roundtrip(self) -> None:
        line = le_geometry.Line3(1, 2, 3, 0, 0, 1)
        w, s0, s1, s2 = line.cayley
        reconstructed = le_geometry.Line3.from_cayley(w, s0, s1, s2)
        assert reconstructed.direction == pytest.approx(line.direction, abs=1e-5)

    def test_repr(self) -> None:
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)
        assert "Line3" in repr(line)


# =============================================================================
# 3D Geometry: LineSegment3
# =============================================================================


class TestLineSegment3:
    """Test LineSegment3 (3D line segment)."""

    def test_default_construction(self) -> None:
        seg = le_geometry.LineSegment3()
        assert seg.empty()

    def test_from_endpoints(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        assert seg.length == pytest.approx(10.0)

    def test_start_end_points(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(1, 2, 3, 4, 5, 6)
        assert seg.start_point() == pytest.approx((1.0, 2.0, 3.0))
        assert seg.end_point() == pytest.approx((4.0, 5.0, 6.0))

    def test_center_point(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        assert seg.center_point() == pytest.approx((5.0, 0.0, 0.0))

    def test_flip(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        original_start = seg.start_point()
        seg.flip()
        assert seg.end_point() == pytest.approx(original_start)

    def test_endpoint_swap(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        sp = seg.start_point()
        ep = seg.end_point()
        seg.endpoint_swap()
        assert seg.start_point() == pytest.approx(ep)
        assert seg.end_point() == pytest.approx(sp)

    def test_inherits_line3(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        assert seg.direction == pytest.approx((1.0, 0.0, 0.0))
        assert seg.distance_to_point(0, 5, 0) == pytest.approx(5.0)

    def test_repr(self) -> None:
        seg = le_geometry.LineSegment3.from_endpoints(0, 0, 0, 10, 0, 0)
        assert "LineSegment3" in repr(seg)


# =============================================================================
# 3D Geometry: Plane
# =============================================================================


class TestPlane:
    """Test Plane (3D plane in Hesse normal form)."""

    def test_default_construction(self) -> None:
        plane = le_geometry.Plane()
        assert plane.empty()

    def test_point_normal_construction(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        assert not plane.empty()
        assert plane.valid()
        assert plane.normal == pytest.approx((0.0, 0.0, 1.0))

    def test_from_three_points(self) -> None:
        plane = le_geometry.Plane.from_three_points(
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            1,
            0,
        )
        # Normal should be (0, 0, ±1)
        assert abs(plane.normal[2]) == pytest.approx(1.0)

    def test_distance(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        assert plane.distance(0, 0, 5) == pytest.approx(5.0)

    def test_nearest_point(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        np_ = plane.nearest_point(3, 4, 5)
        assert np_ == pytest.approx((3.0, 4.0, 0.0))

    def test_intersection_with_line(self) -> None:
        plane = le_geometry.Plane(0, 0, 5, 0, 0, 1)  # z = 5
        line = le_geometry.Line3(0, 0, 0, 0, 0, 1)  # along z-axis
        pt = plane.intersection_with_line(line)
        assert pt is not None
        assert pt == pytest.approx((0.0, 0.0, 5.0))

    def test_intersection_with_parallel_line(self) -> None:
        plane = le_geometry.Plane(0, 0, 5, 0, 0, 1)  # z = 5
        line = le_geometry.Line3(0, 0, 0, 1, 0, 0)  # along x-axis
        pt = plane.intersection_with_line(line)
        assert pt is None

    def test_intersection_with_plane(self) -> None:
        p1 = le_geometry.Plane(0, 0, 0, 0, 0, 1)  # xy-plane
        p2 = le_geometry.Plane(0, 0, 0, 0, 1, 0)  # xz-plane
        result = p1.intersection_with_plane(p2)
        assert result is not None  # Should return a Line3 along x-axis

    def test_translate(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        plane.translate(0, 0, 5)
        assert plane.dist_to_origin == pytest.approx(5.0)

    def test_flip(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        original_normal = plane.normal
        plane.flip()
        assert plane.normal[2] == pytest.approx(-original_normal[2])

    def test_repr(self) -> None:
        plane = le_geometry.Plane(0, 0, 0, 0, 0, 1)
        assert "Plane" in repr(plane)


# =============================================================================
# 3D Geometry: Pose
# =============================================================================


class TestPose:
    """Test Pose (6-DOF rigid body pose)."""

    def test_default_construction(self) -> None:
        pose = le_geometry.Pose()
        assert pose.origin == pytest.approx((0.0, 0.0, 0.0))
        assert pose.orientation == pytest.approx((0.0, 0.0, 0.0))

    def test_parameterized_construction(self) -> None:
        pose = le_geometry.Pose(tx=1, ty=2, tz=3, rx=0.1, ry=0.2, rz=0.3)
        assert pose.origin == pytest.approx((1.0, 2.0, 3.0))
        assert pose.orientation == pytest.approx((0.1, 0.2, 0.3))

    def test_origin_property_setter(self) -> None:
        pose = le_geometry.Pose()
        pose.origin = (5.0, 6.0, 7.0)
        assert pose.origin == pytest.approx((5.0, 6.0, 7.0))

    def test_orientation_property_setter(self) -> None:
        pose = le_geometry.Pose()
        pose.orientation = (0.1, 0.2, 0.3)
        assert pose.orientation == pytest.approx((0.1, 0.2, 0.3))

    def test_rot_matrix(self) -> None:
        pose = le_geometry.Pose()  # Identity rotation
        rm = pose.rot_matrix()
        assert len(rm) == 3
        # Identity rotation matrix
        assert rm[0] == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)
        assert rm[1] == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
        assert rm[2] == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)

    def test_hom_matrix(self) -> None:
        pose = le_geometry.Pose(tx=1, ty=2, tz=3)
        hm = pose.hom_matrix()
        assert len(hm) == 4
        # Last column contains translation
        assert hm[0][3] == pytest.approx(1.0, abs=1e-6)
        assert hm[1][3] == pytest.approx(2.0, abs=1e-6)
        assert hm[2][3] == pytest.approx(3.0, abs=1e-6)

    def test_translate(self) -> None:
        pose = le_geometry.Pose()
        pose.translate(1, 2, 3)
        assert pose.origin == pytest.approx((1.0, 2.0, 3.0))

    def test_repr(self) -> None:
        pose = le_geometry.Pose(tx=1, ty=2, tz=3)
        assert "Pose" in repr(pose)


# =============================================================================
# Camera Models
# =============================================================================


class TestCamera:
    """Test Camera (pinhole camera model)."""

    def test_default_construction(self) -> None:
        cam = le_geometry.Camera()
        assert cam.empty

    def test_parameterized_construction(self) -> None:
        cam = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        assert not cam.empty
        assert cam.focal == pytest.approx((500.0, 500.0))
        assert cam.offset == pytest.approx((320.0, 240.0))
        assert cam.width == pytest.approx(640.0)
        assert cam.height == pytest.approx(480.0)

    def test_image_size(self) -> None:
        cam = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        assert cam.image_size == pytest.approx((640.0, 480.0))

    def test_focal_length(self) -> None:
        cam = le_geometry.Camera(focal_x=400, focal_y=400)
        assert cam.focal_length == pytest.approx(400.0)

    def test_inherits_pose(self) -> None:
        cam = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
            tx=1,
            ty=2,
            tz=3,
        )
        assert cam.origin == pytest.approx((1.0, 2.0, 3.0))

    def test_repr(self) -> None:
        cam = le_geometry.Camera(focal_x=500, focal_y=500)
        assert "Camera" in repr(cam)


class TestCameraHom:
    """Test CameraHom (projection via cached matrix)."""

    def _make_cam(self) -> object:
        return le_geometry.CameraHom(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )

    def test_construction(self) -> None:
        cam = self._make_cam()
        assert not cam.empty

    def test_from_base_camera(self) -> None:
        base = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        cam = le_geometry.CameraHom(base)
        assert not cam.empty

    def test_project_point(self) -> None:
        cam = self._make_cam()
        # Point on optical axis at z=1 should project near principal point
        u, v = cam.project_point(0, 0, 1)
        assert u == pytest.approx(320.0, abs=1.0)
        assert v == pytest.approx(240.0, abs=1.0)

    def test_project_points(self) -> None:
        cam = self._make_cam()
        pts = [(0, 0, 1), (0, 0, 2)]
        results = cam.project_points(pts)
        assert len(results) == 2


class TestCameraPluecker:
    """Test CameraPluecker (Plücker line projection)."""

    def _make_cam(self) -> object:
        return le_geometry.CameraPluecker(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )

    def test_construction(self) -> None:
        cam = self._make_cam()
        assert not cam.empty

    def test_project_line(self) -> None:
        cam = self._make_cam()
        # Vertical line at (1, 0, z) in front of camera
        line3 = le_geometry.Line3(1, 0, 5, 0, 0, 1)
        line2d = cam.project_line(line3)
        # Should get a valid 2D line
        assert hasattr(line2d, "normal_x")

    def test_project_line_segment(self) -> None:
        cam = self._make_cam()
        seg3 = le_geometry.LineSegment3.from_endpoints(1, 0, 5, 1, 0, 10)
        seg2d = cam.project_line_segment(seg3)
        assert hasattr(seg2d, "length")

    def test_project_lines_batch(self) -> None:
        cam = self._make_cam()
        lines = [
            le_geometry.Line3(1, 0, 5, 0, 0, 1),
            le_geometry.Line3(-1, 0, 5, 0, 0, 1),
        ]
        results = cam.project_lines(lines)
        assert len(results) == 2


class TestCamera2P:
    """Test Camera2P (two-point projection)."""

    def _make_cam(self) -> object:
        return le_geometry.Camera2P(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )

    def test_construction(self) -> None:
        cam = self._make_cam()
        assert not cam.empty

    def test_project_line_segment(self) -> None:
        cam = self._make_cam()
        seg3 = le_geometry.LineSegment3.from_endpoints(1, 0, 5, 1, 0, 10)
        seg2d = cam.project_line_segment(seg3)
        assert hasattr(seg2d, "length")


class TestCameraCV:
    """Test CameraCV (OpenCV projection)."""

    def _make_cam(self) -> object:
        return le_geometry.CameraCV(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )

    def test_construction(self) -> None:
        cam = self._make_cam()
        assert not cam.empty

    def test_project_point(self) -> None:
        cam = self._make_cam()
        u, v = cam.project_point(0, 0, 1)
        assert u == pytest.approx(320.0, abs=1.0)
        assert v == pytest.approx(240.0, abs=1.0)


# =============================================================================
# 3D Geometry: f64 presets
# =============================================================================


class TestGeometry3dF64:
    """Test double-precision presets for 3D geometry types."""

    def test_line3_f64(self) -> None:
        line = le_geometry.Line3_f64(0, 0, 0, 1, 0, 0)
        assert line.direction == pytest.approx((1.0, 0.0, 0.0))

    def test_line_segment3_f64(self) -> None:
        seg = le_geometry.LineSegment3_f64.from_endpoints(0, 0, 0, 10, 0, 0)
        assert seg.length == pytest.approx(10.0)

    def test_plane_f64(self) -> None:
        plane = le_geometry.Plane_f64(0, 0, 0, 0, 0, 1)
        assert plane.normal == pytest.approx((0.0, 0.0, 1.0))

    def test_pose_f64(self) -> None:
        pose = le_geometry.Pose_f64(tx=1.0, ty=2.0, tz=3.0)
        assert pose.origin == pytest.approx((1.0, 2.0, 3.0))

    def test_camera_f64(self) -> None:
        cam = le_geometry.Camera_f64(focal_x=500, focal_y=500)
        assert not cam.empty

    def test_camera_hom_f64(self) -> None:
        cam = le_geometry.CameraHom_f64(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        u, v = cam.project_point(0, 0, 1)
        assert u == pytest.approx(320.0, abs=1.0)

    def test_camera_pluecker_f64(self) -> None:
        assert hasattr(le_geometry, "CameraPluecker_f64")

    def test_camera_2p_f64(self) -> None:
        assert hasattr(le_geometry, "Camera2P_f64")

    def test_camera_cv_f64(self) -> None:
        assert hasattr(le_geometry, "CameraCV_f64")


# =============================================================================
# Stereo Triangulation
# =============================================================================


class TestStereo:
    """Test Stereo (ray-intersection triangulation)."""

    def _make_stereo_pair(self) -> tuple[object, object]:
        """Create a canonical stereo pair with 10-unit baseline."""
        cam_left = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        cam_right = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
            tx=10.0,
        )
        return cam_left, cam_right

    def test_construction(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        assert repr(s) == "Stereo(ray-intersection)"

    def test_triangulate_point(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        # A point at (5, 0, 100): projects to (320+25, 240) left; (320-25, 240) right
        # left_x = fx * (X - tx_l) / Z + cx = 500 * 5 / 100 + 320 = 345
        # right_x = fx * (X - tx_r) / Z + cx = 500 * (5-10) / 100 + 320 = 295
        x, y, z = s.triangulate_point(345, 240, 295, 240)
        assert z == pytest.approx(100.0, rel=0.1)
        assert x == pytest.approx(5.0, rel=0.1)

    def test_triangulate_points_batch(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        results = s.triangulate_points(
            [(345.0, 240.0), (320.0, 240.0)],
            [(295.0, 240.0), (270.0, 240.0)],
        )
        assert len(results) == 2

    def test_triangulate_line(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        # Create horizontal lines at y=100, with different x-intercepts
        line_l = le_geometry.Line.from_normal(0, 1, -100)
        line_r = le_geometry.Line.from_normal(0, 1, -100)
        line3 = s.triangulate_line(line_l, line_r)
        # Even if degenerate for horizontal (normalX ~ 0), check API works
        assert hasattr(line3, "direction")

    def test_triangulate_segment(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        # Create matching line segments
        seg_l = le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)
        seg_r = le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)
        seg3 = s.triangulate_segment(seg_l, seg_r)
        assert hasattr(seg3, "length")

    def test_triangulate_segments_batch(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        s = le_geometry.Stereo(cam_l, cam_r)
        segs_l = [
            le_geometry.LineSegment.from_endpoints(300, 100, 400, 300),
            le_geometry.LineSegment.from_endpoints(100, 50, 200, 200),
        ]
        segs_r = [
            le_geometry.LineSegment.from_endpoints(250, 100, 350, 300),
            le_geometry.LineSegment.from_endpoints(50, 50, 150, 200),
        ]
        result = s.triangulate_segments(segs_l, segs_r)
        assert len(result) == 2


class TestStereoPlane:
    """Test StereoPlane (plane-intersection triangulation)."""

    def _make_stereo_pair(self) -> tuple[object, object]:
        cam_left = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        cam_right = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
            tx=10.0,
        )
        return cam_left, cam_right

    def test_construction(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        sp = le_geometry.StereoPlane(cam_l, cam_r)
        assert repr(sp) == "StereoPlane(plane-intersection)"

    def test_triangulate_point_inherited(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        sp = le_geometry.StereoPlane(cam_l, cam_r)
        # Inherits point triangulation from Stereo
        x, y, z = sp.triangulate_point(345, 240, 295, 240)
        assert z == pytest.approx(100.0, rel=0.1)

    def test_triangulate_line_plane_intersection(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        sp = le_geometry.StereoPlane(cam_l, cam_r)
        # Non-horizontal lines should triangulate via plane intersection
        seg_l = le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)
        seg_r = le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)
        line3 = sp.triangulate_line(seg_l, seg_r)
        assert hasattr(line3, "direction")

    def test_triangulate_segment_plane_intersection(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        sp = le_geometry.StereoPlane(cam_l, cam_r)
        seg_l = le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)
        seg_r = le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)
        seg3 = sp.triangulate_segment(seg_l, seg_r)
        assert hasattr(seg3, "length")
        # Segment should have non-zero length for non-degenerate input
        if not seg3.empty():
            assert seg3.length > 0

    def test_triangulate_segments_batch(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        sp = le_geometry.StereoPlane(cam_l, cam_r)
        segs_l = [le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)]
        segs_r = [le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)]
        result = sp.triangulate_segments(segs_l, segs_r)
        assert len(result) == 1


class TestStereoCV:
    """Test StereoCV (OpenCV-based triangulation)."""

    def _make_stereo_pair(self) -> tuple[object, object]:
        cam_left = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
        )
        cam_right = le_geometry.Camera(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
            tx=10.0,
        )
        return cam_left, cam_right

    def test_construction(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        scv = le_geometry.StereoCV(cam_l, cam_r)
        assert repr(scv) == "StereoCV(opencv)"

    def test_triangulate_point(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        scv = le_geometry.StereoCV(cam_l, cam_r)
        x, y, z = scv.triangulate_point(345, 240, 295, 240)
        # OpenCV triangulation should give similar results
        assert z == pytest.approx(100.0, rel=0.15)

    def test_triangulate_points_batch(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        scv = le_geometry.StereoCV(cam_l, cam_r)
        results = scv.triangulate_points(
            [(345.0, 240.0)],
            [(295.0, 240.0)],
        )
        assert len(results) == 1

    def test_triangulate_segment(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        scv = le_geometry.StereoCV(cam_l, cam_r)
        seg_l = le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)
        seg_r = le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)
        seg3 = scv.triangulate_segment(seg_l, seg_r)
        assert hasattr(seg3, "length")

    def test_triangulate_segments_batch(self) -> None:
        cam_l, cam_r = self._make_stereo_pair()
        scv = le_geometry.StereoCV(cam_l, cam_r)
        segs_l = [le_geometry.LineSegment.from_endpoints(300, 100, 400, 300)]
        segs_r = [le_geometry.LineSegment.from_endpoints(250, 100, 350, 300)]
        result = scv.triangulate_segments(segs_l, segs_r)
        assert len(result) == 1


class TestStereoF64:
    """Test double-precision stereo triangulation presets."""

    def test_stereo_f64(self) -> None:
        assert hasattr(le_geometry, "Stereo_f64")
        cam_l = le_geometry.Camera_f64(
            focal_x=500, focal_y=500, offset_x=320, offset_y=240, width=640, height=480
        )
        cam_r = le_geometry.Camera_f64(
            focal_x=500,
            focal_y=500,
            offset_x=320,
            offset_y=240,
            width=640,
            height=480,
            tx=10.0,
        )
        s = le_geometry.Stereo_f64(cam_l, cam_r)
        x, y, z = s.triangulate_point(345.0, 240.0, 295.0, 240.0)
        assert z == pytest.approx(100.0, rel=0.1)

    def test_stereo_plane_f64(self) -> None:
        assert hasattr(le_geometry, "StereoPlane_f64")

    def test_stereo_cv_f64(self) -> None:
        assert hasattr(le_geometry, "StereoCV_f64")
