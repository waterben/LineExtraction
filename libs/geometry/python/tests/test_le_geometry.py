"""Integration tests for the le_geometry Python extension module.

Tests cover:
  - Module import and core types
  - Line construction, properties, distance, projection, intersection
  - LineSegment construction, properties, range checking, transforms
  - Polygon construction and properties
  - Drawing utilities
  - Visualization helpers
  - LineOptimizer functions
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
        assert line.empty

    def test_construction_from_normal(self) -> None:
        line = le_geometry.Line(0.0, 1.0, 5.0)
        assert pytest.approx(line.normal_x) == 0.0
        assert pytest.approx(line.normal_y) == 1.0
        assert pytest.approx(line.origin_dist) == 5.0
        assert line.valid

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
