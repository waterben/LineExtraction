# le_geometry — Python Bindings for Geometry Library

Python bindings for the `libs/geometry` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides 2D geometric primitives, drawing utilities, and line
optimization routines:

- **Line** — infinite line in Hesse normal form
- **LineSegment** — finite line segment (extends Line)
- **Polygon** — 2D polygon with vertex list
- **Drawing** — render line segments onto images
- **LineOptimizer** — refine segments against gradient magnitude (dlib-based)
- **Visualization helpers** — edge coloring, border padding, normalized saving

Images are passed as NumPy arrays and automatically converted to/from `cv::Mat`.

> **Standalone:** `le_geometry` has no Python-level imports.  It can be used
> independently of the other `le_*` modules.

## Quick Start

```python
import numpy as np
import le_geometry as geo

# Create a line segment from endpoints
seg = geo.LineSegment.from_endpoints(10.0, 50.0, 190.0, 50.0)
print(seg)                     # LineSegment((10, 50) -> (190, 50), len=180)
print(seg.length)              # 180.0
print(seg.start_point())       # (10.0, 50.0)
print(seg.end_point())         # (190.0, 50.0)

# Draw on an image
img = np.zeros((100, 200, 3), dtype=np.uint8)
seg.draw(img, color=[0, 255, 0], thickness=2)
```

## Line

A `Line` represents an infinite 2D line in Hesse normal form: **n · p = d**,
where _n_ is a unit normal and _d_ is the signed distance to the origin.

### Construction

```python
# Default (degenerate) line
l = geo.Line()

# From normal components and distance
l = geo.Line(nx=0.0, ny=1.0, distance=10.0)

# From normal and a point on the line
l = geo.Line(nx=0.0, ny=1.0, point_x=50.0, point_y=10.0)

# From angle (angle of normal to x-axis) and distance
l = geo.Line.from_angle(angle=1.5708, distance=10.0)
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `normal_x` | float | X-component of unit normal |
| `normal_y` | float | Y-component of unit normal |
| `origin_dist` | float | Signed perpendicular distance to origin |
| `normal_angle` | float | Angle of normal vector (radians, [-π, π)) |
| `angle` | float | Line angle to x-axis (radians, [-π, π)) |
| `gradient_angle` | float | Angle to x-axis, same direction as gradient |
| `origin_x` | float | X-coordinate of closest point to origin |
| `origin_y` | float | Y-coordinate of closest point to origin |
| `direction_x` | float | X-component of line direction |
| `direction_y` | float | Y-component of line direction |
| `valid` | bool | True if normal is unit length |
| `empty` | bool | True if normal is zero |

### Methods

```python
# Tuple accessors
l.normal()           # -> (nx, ny)
l.direction()        # -> (dx, dy)
l.origin_point()     # -> (x, y)

# Point-to-line operations
l.distance(x, y)     # signed perpendicular distance
l.project(x, y)      # project point onto line -> (x, y)
l.normal_project(x, y)  # signed distance along normal

# Line evaluation
l.x_at(y)            # x-coordinate at given y
l.y_at(x)            # y-coordinate at given x

# Coordinate transforms
l.world2line(x, y)   # world -> line-local coords -> (along, across)
l.line2world(x, y)   # line-local -> world coords -> (wx, wy)

# Line-to-line operations
l.is_parallel(other)       # True if lines are parallel
l.intersection(other)      # -> (x, y) or None if parallel
l.angle_to(other)          # angle between lines (radians)

# Transformations (return new Line)
l.translated_ortho(dist)   # shift along normal
l.translated_to(x, y)      # translate to pass through point
l.rotated(angle)            # rotate around origin
l.rotated_around(angle, x, y)  # rotate around arbitrary point
l.scaled(s)                 # scale distance from origin
l.normal_flipped()          # flip normal direction

# In-place transformations
l.translate_ortho(dist)
l.translate_to(x, y)
l.rotate(angle)
l.scale(s)
l.normal_flip()

# Drawing
l.draw(image, color=[0,0,255], thickness=1, line_type=8,
       normal_length=0.0, tip_length=0.0)
```

## LineSegment

A `LineSegment` extends `Line` with start/end distances along the line
direction, defining a finite segment.

### Construction

```python
# Default (degenerate) segment
seg = geo.LineSegment()

# From normal, distance, and line extents
seg = geo.LineSegment(nx=0.0, ny=1.0, distance=5.0,
                      start=-10.0, end=10.0, octave=0)

# From endpoint coordinates (most common)
seg = geo.LineSegment.from_endpoints(x1=10.0, y1=20.0,
                                     x2=90.0, y2=80.0, octave=0)
```

### Properties

Inherits all `Line` properties, plus:

| Property | Type | Description |
|----------|------|-------------|
| `start` | float | Start distance along line direction |
| `end` | float | End distance along line direction |
| `center_dist` | float | Center distance along line direction |
| `length` | float | Segment length |
| `octave` | int | Detection octave (scale level) |

### Methods

```python
# Endpoint access
seg.start_point()     # -> (x, y)
seg.end_point()       # -> (x, y)
seg.center_point()    # -> (x, y)
seg.end_points()      # -> (x1, y1, x2, y2)

# Range checks
seg.in_range(dist)         # True if dist is within [start, end]
seg.in_range_point(x, y)   # True if point projects into segment
seg.in_range_tol(x, y, tol)  # ... with tolerance

# Clipping
seg.trim_to_box(max_x, max_y, min_x=0, min_y=0)  # -> new segment

# Segment-to-segment
seg.check_overlap(other)  # True if segments overlap
seg.error(reference_line) # fitting error vs reference

# Transformations (return new LineSegment)
seg.translated(dist)              # shift along normal
seg.rotated(angle, pivot)         # rotate around pivot (distance along line)
seg.scaled(s)                     # scale around midpoint
seg.scaled_around(s, pivot)       # scale around pivot
seg.normal_flipped()              # flip normal
seg.endpoint_swapped()            # swap start/end

# Drawing (same as Line.draw)
seg.draw(image, color=[0,255,0], thickness=2)
```

## Polygon

A `Polygon` stores a list of vertices in line-local coordinates with a pivot
point.

```python
poly = geo.Polygon()
poly.add_vertex(0.0, 0.0)
poly.add_vertex(1.0, 0.0)
poly.add_vertex(1.0, 1.0)

print(poly.size)         # 3
print(poly.is_convex)    # True
print(poly.vertices)     # [(0, 0), (1, 0), (1, 1)]

# World-space vertices
world_verts = poly.world_vertices()  # list[(x, y)]

# Drawing
img = np.zeros((100, 100, 3), dtype=np.uint8)
poly.draw(img, color=[255, 0, 0])
poly.fill(img, color=[0, 0, 255])
```

## Drawing Utilities

Render line segments onto images — useful for visualization and debugging:

```python
import le_geometry as geo

segments = [
    geo.LineSegment.from_endpoints(10, 20, 90, 20),
    geo.LineSegment.from_endpoints(50, 10, 50, 90),
]

img = np.zeros((100, 100, 3), dtype=np.uint8)

# Uniform color (returns new image)
result = geo.draw_lines(img, segments, color=[0, 255, 0])

# Random colors per segment
result = geo.draw_lines_random(img, segments)

# With string labels
labels = ["horizontal", "vertical"]
result = geo.draw_lines_labeled(img, segments, labels)

# In-place variants (modify img directly)
geo.draw_lines_inplace(img, segments, color=[255, 0, 0])
geo.draw_lines_random_inplace(img, segments)
```

### Trim Line to Bounding Box

Convert an infinite `Line` into a `LineSegment` clipped to an image:

```python
line = geo.Line(0.0, 1.0, 50.0)
seg = geo.trim_to_box(line, max_x=640, max_y=480)
print(seg.start_point(), seg.end_point())
```

## LineOptimizer

Refine line segment positions against a gradient magnitude image using
dlib-based optimization:

```python
import le_geometry as geo
import numpy as np

# Create gradient magnitude (e.g. from le_imgproc)
mag = np.zeros((100, 100), dtype=np.float32)
mag[50, :] = 1.0   # strong horizontal edge at row 50

# Initial estimate (slightly off)
seg = geo.LineSegment.from_endpoints(10.0, 48.0, 90.0, 48.0)

# Optimize single segment (returns error, distance, rotation)
error, d, r = geo.optimize_line_segment(mag, seg)
print(f"Error: {error:.4f}, shift: {d:.2f}, rotation: {r:.4f}")

# In-place optimization (modifies seg, returns error)
error = geo.optimize_line_segment_inplace(mag, seg)

# Batch optimization
segments = [seg, geo.LineSegment.from_endpoints(20.0, 49.0, 80.0, 49.0)]
optimized, errors = geo.optimize_line_segments(mag, segments)
for s, e in zip(optimized, errors):
    print(f"  {s}, error={e:.4f}")
```

## Visualization Helpers

```python
import le_geometry as geo

# Color-code an NMS edge map
nms_map = np.zeros((100, 100), dtype=np.int8)
color_img = geo.create_nms_color(nms_map)

# Add border padding to an image
bordered = geo.apply_border(img, border_size=10)

# Save matrix as normalized grayscale PNG
geo.save_normalized(matrix, "output.png")

# Save edge map as colored PNG
geo.save_edge(edge_map, "edges.png")

# Generate a random BGR+A color
r, g, b, a = geo.random_color()
```

## Precision Presets

Each class is available in single and double precision:

| Default (float) | Double precision | Description |
|------------------|------------------|-------------|
| `Line` | `Line_f64` | Infinite line |
| `LineSegment` | `LineSegment_f64` | Finite segment |
| `Polygon` | `Polygon_f64` | Polygon |
| `draw_lines` | `draw_lines_f64` | Drawing (f64 segments) |
| `trim_to_box` | `trim_to_box_f64` | Clipping (f64 line) |
| `optimize_line_segment` | `optimize_line_segment_f64` | Optimizer (f64) |

## Build & Test

```bash
# Build the module
bazel build //libs/geometry/python:le_geometry

# Run tests
bazel test //libs/geometry/python:test_le_geometry

# Run all geometry library tests (C++ and Python)
bazel test //libs/geometry/...
```

## Architecture

```
python/
├── src/
│   ├── geometry_binding.hpp     # Binding declarations
│   ├── geometry_binding.cpp     # All geometry/draw/optimizer bindings
│   └── module_le_geometry.cpp   # PYBIND11_MODULE entry
├── tests/
│   └── test_le_geometry.py      # Python integration tests
├── BUILD.bazel
└── README.md
```

### Module Dependency Graph

```
le_geometry  (standalone — no Python imports)
    │
    └── Provides Line, LineSegment, Polygon, drawing, optimizer
```

`le_geometry` is a root module with no Python-level imports.
`le_lsd` imports it and re-exports `Line`/`LineSegment` for convenience.

## See Also

- [le_imgproc](../../imgproc/python/README.md) — gradient filters (input for LineOptimizer)
- [le_lsd](../../lsd/python/README.md) — line segment detection (uses Line/LineSegment)
- [le_eval](../../eval/python/README.md) — performance benchmarking
- [le_edge](../../edge/python/README.md) — edge detection
