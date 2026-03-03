# Geometry Library

Geometric primitives and camera/stereo utilities used across LineExtraction. This library provides 2D/3D points, lines, planes, polygons, pose/camera models, and helpers for projections, triangulation, and visualization.

All types live in the `lsfm` namespace and are templated on the scalar type `FT` (typically `float` or `double`).

## Overview

```
lsfm::
  ├── Matx<FT,R,C>          Fixed-size matrix (Eigen-based)
  │   ├── Vec2<FT>           2D vector / point
  │   ├── Vec3<FT>           3D vector / point
  │   └── Vec4<FT>           Homogeneous vector
  │
  ├── Line<FT>               2D infinite line  (Hesse normal form: n·p = d)
  │   └── LineSegment<FT>    2D finite segment (start/end distances along direction)
  │
  ├── Line3<FT>              3D infinite line  (point + direction)
  │   └── LineSegment3<FT>   3D finite segment (start/end distances)
  │
  ├── Plane<FT>              3D plane (Hesse normal form: n·x = d)
  ├── Polygon<FT>            2D polygon with pivot point
  │
  ├── Pose<FT>               6-DOF pose (translation + Rodrigues rotation)
  │   └── Camera<FT>         Pinhole camera (intrinsics + extrinsics)
  │       ├── CameraHom<FT>  Cached 3×4 projection matrix
  │       │   ├── CameraPluecker<FT>  + Plücker line projection
  │       │   └── CameraLens<FT>      + distortion model
  │       └── CameraCV<FT>   OpenCV-backed projection
  │
  ├── Stereo<FT>             Ray-intersection triangulation
  │   └── StereoPlane<FT>    Interpretation-plane triangulation
  ├── StereoCV<FT>           OpenCV-backed triangulation
  │
  ├── LineOptimizer           Gradient-based line refinement (dlib)
  └── Drawing utilities       text(), line(), quiver(), matches, polygons
```

## 2D Primitives

### Line (infinite)

Represents a 2D line in **Hesse normal form**: $\vec{n} \cdot \vec{p} = d$

```
       normal (nx, ny)
          ↑
    ──────┼──── line direction
          │
     d ←──┤ (signed distance from origin)
          O
```

```cpp
#include <geometry/line.hpp>

// From normal angle (rad) and distance
lsfm::Line2d line(M_PI / 4, 10.0);  // 45° at distance 10

// From two points
lsfm::Line2d line = lsfm::Line2d::twoPoint({0, 0}, {1, 1});

// Properties
double angle = line.angle();              // direction angle (rad)
double dist  = line.distance(point);      // signed distance to point
auto proj    = line.project(point);       // closest point on line
bool par     = line.isParallel(other);    // parallelism test
```

### LineSegment (finite)

Extends `Line` with start/end distances along the line's direction vector.

```
      startPoint            endPoint
          ●━━━━━━━━━━━━━━━━━━●
          |←── length() ────→|
          |                   |
     startDist           endDist     (measured along direction from origin)
```

```cpp
#include <geometry/line.hpp>

// From two endpoints
lsfm::LineSegment2d seg({10, 20}, {100, 80});

// Properties
auto start  = seg.startPoint();
auto end    = seg.endPoint();
auto center = seg.centerPoint();
double len  = seg.length();

// Clip to image bounds
seg.trim2Box(width, height);

// Error measurement against ground truth
lsfm::Vec4d err = seg.error(gt_segment);
// err = [start_dist, end_dist, angular_err, orthogonal_err]
```

**Common type aliases:**

| Float (`float`) | Double (`double`) | Vector type |
|------------------|-------------------|-------------|
| `Line2f` | `Line2d` | `Line2Vector` |
| `LineSegment2f` | `LineSegment2d` | `LineSegment2Vector` |

## 3D Primitives

### Line3 & LineSegment3

3D lines in point-direction form with Plücker and Cayley representations.

```
  Plücker coordinates:            Cayley parameterization:
  L = (d, m)                      Compact 4-parameter form
  d = direction                   ideal for optimization
  m = origin × direction          (used in 3D reconstruction)
```

```cpp
#include <geometry/line3.hpp>

// From two points
auto line = lsfm::Line3d::twoPoint({0,0,0}, {1,1,1});

double dist = line.distance(point);               // point-to-line
double ldist = line.distance(other_line);          // line-to-line
auto nearest = line.nearestPointOnLine(point);     // closest point

// Plücker / Cayley conversions (for optimization)
auto cayley = line.cayley();
auto back = lsfm::Line3d::lineFromCayley(cayley);
```

### Plane

3D plane in Hesse normal form: $\vec{n} \cdot \vec{x} = d$

```cpp
#include <geometry/plane.hpp>

// From 3 points
lsfm::Planed plane(p1, p2, p3);

// Intersections
lsfm::Vec3d point;
plane.intersection(line3d, point);       // line-plane → point

lsfm::Line3d isect_line;
plane.intersection(other_plane, isect_line);  // plane-plane → line
```

### Polygon

2D polygon with vertices relative to a configurable pivot point.

```cpp
#include <geometry/polygon.hpp>

// From line segment intersections (convex hull of a region)
lsfm::Polygon2d poly(segments, pivot);
poly.draw(image, color);     // outline
poly.fill(image, color);     // filled
bool convex = poly.isConvex();
```

## Camera & Stereo

### Pose

6-DOF pose: 3D translation + Rodrigues rotation vector.

```cpp
#include <geometry/pose.hpp>

lsfm::Posed pose(tx, ty, tz, rx, ry, rz);
auto R    = pose.rotM();          // 3×3 rotation matrix
auto H    = pose.homM();          // 4×4 homogeneous matrix
auto pose2 = pose.concat(other);  // composition
```

### Camera Hierarchy

```
  Camera<FT>           Pinhole (intrinsics + extrinsics, no distortion)
    ├── CameraHom<FT>       + cached 3×4 projection matrix (batch-efficient)
    │   ├── CameraPluecker<FT>  + projects 3D lines to 2D lines
    │   └── CameraLens<FT>      + lens distortion model
    └── CameraCV<FT>        OpenCV-backed solvePnP / projectPoints
```

```cpp
#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>

// From intrinsics
lsfm::Camerad cam(focal_x, focal_y, cx, cy, width, height);

// From field of view
auto cam = lsfm::Camerad::fromFov(fov_x, fov_y, width, height);

// Projection
auto P = cam.projM();     // 3×4 projection matrix
double fov = cam.fov();   // field of view
```

### Stereo Triangulation

```
  Left Camera ──── baseline ──── Right Camera
       \                          /
        \   corresponding        /
         \  pixels / lines      /
          \      ╳             /   ← triangulated 3D point/line
           \   /              /
            \ /              /
```

```cpp
#include <geometry/stereo.hpp>
#include <geometry/stereocv.hpp>

// Ray-intersection (midpoint of closest approach)
lsfm::Stereod stereo(cam_left, cam_right);
auto point3d = stereo.triangulate(pixel_left, pixel_right);
auto line3d  = stereo.triangulate(seg_left, seg_right);

// Interpretation-plane method (for non-rectified cameras)
lsfm::StereoPlaned sp(cam_left, cam_right);
auto line3d = sp.triangulate(seg_left, seg_right);
```

## Visualization

The `draw.hpp` header provides rich OpenCV-based drawing utilities:

```cpp
#include <geometry/draw.hpp>

// Draw labeled text
lsfm::text(image, "Hello", position, color, scale);

// Draw line segments with random colors
lsfm::draw_lines_random(image, segments);

// Draw line segments with IDs
lsfm::draw_lines_with_ids(image, segments, ids, color);

// Draw quiver (gradient vector field)
lsfm::draw_quiver(image, gx, gy, scale, color);

// Draw stereo matches
lsfm::draw_matches(image_left, image_right, matches, color);
```

## Line Optimization

Refines line segment positions by maximizing gradient magnitude response using dlib optimization (BFGS).

```
  Before:                       After:
  ─ ─ ─ ─ ● ─ ─ ─ ─ ─ ●       ━━━━━━●━━━━━━━━━━●
           (slightly off)              (snapped to gradient ridge)
```

```cpp
#include <geometry/line_optimizer.hpp>

lsfm::LineOptimizer optimizer;
auto error = optimizer.optimize(gradient_magnitude, segment);
```

## Utility Functions

| Header | Functions |
|--------|-----------|
| `base.hpp` | `rodrigues()` (rotation vector ↔ matrix), `composeHom()`, `decomposeHom()` |
| `point.hpp` | `set()`, `get()`, `getX()`, `getY()`, pixel access helpers |
| `eigen2cv.hpp` | Eigen ↔ OpenCV type conversions, rotation format conversions |
| `vision.hpp` | `decomposeProjectionMatrix()` |

## Dependencies

| Library | Used For |
|---------|----------|
| Eigen | Core linear algebra (all matrix/vector types) |
| OpenCV | Image I/O, drawing, calibration, `cv::projectPoints` |
| dlib | Line optimization (BFGS solver) |
| `libs/utility` | `ValueManager` base class |

## Build

```bash
# Build geometry library
bazel build //libs/geometry:lib_geometry

# Run all geometry tests (14 test suites)
bazel test //libs/geometry:all

# Build and run specific test
bazel test //libs/geometry:test_geometry
```

## Directory Structure

```
libs/geometry/
├── BUILD.bazel
├── CMakeLists.txt
├── README.md
├── include/geometry/
│   ├── base.hpp              # Matrix/vector types, Rodrigues
│   ├── point.hpp             # Pixel access, point conversions
│   ├── line.hpp              # 2D Line, LineSegment
│   ├── line3.hpp             # 3D Line3, LineSegment3
│   ├── plane.hpp             # 3D Plane
│   ├── polygon.hpp           # 2D Polygon
│   ├── pose.hpp              # 6-DOF Pose
│   ├── camera.hpp            # Camera hierarchy
│   ├── cameracv.hpp          # OpenCV-backed Camera
│   ├── stereo.hpp            # Stereo, StereoPlane
│   ├── stereocv.hpp          # OpenCV-backed Stereo
│   ├── vision.hpp            # Projection decomposition
│   ├── draw.hpp              # Drawing utilities
│   ├── eigen2cv.hpp          # Eigen/OpenCV conversions
│   └── line_optimizer.hpp    # Gradient-based line refinement
├── python/                   # Python bindings (le_geometry)
│   ├── README.md
│   ├── src/
│   └── tests/
└── tests/                    # 14 C++ test suites
```

## Related Libraries and Resources

- [lsd](../lsd/README.md) — Line Segment Detection algorithms
- [lfd](../lfd/README.md) — Line Feature Descriptors
- [edge](../edge/README.md) — Edge detection algorithms
- [imgproc](../imgproc/README.md) — Image processing and gradient computation
- [algorithm](../algorithm/README.md) — Post-processing, accuracy evaluation, parameter optimization
- [utility](../utility/README.md) — Core utilities
- [Geometry Examples](../../examples/geometry/README.md) — Demonstration programs
- [Python Bindings](python/README.md) — `le_geometry` Python module
- [Thesis Evaluations](../../evaluation/thesis/README.md) — Precision evaluations using geometric primitives
- [Main README](../../README.md) — Project overview
