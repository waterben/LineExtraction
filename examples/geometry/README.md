# Geometry Examples

Examples demonstrating geometric primitives, camera models, stereo vision, and 3D reconstruction.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `test_geometry_gt` | [gt_test.cpp](src/gt_test.cpp) | Ground truth 3D geometry: stereo camera setup, cube projection, line correspondence |
| `test_geometry_line` | [line_test.cpp](src/line_test.cpp) | 2D line primitives: construction, coordinate transforms, intersections, distances |
| `test_geometry_poly` | [poly_test.cpp](src/poly_test.cpp) | Polygon operations: area, perimeter, point-in-polygon, rendering |
| `test_geometry_stereo_pluecker` | [stereo_pluecker_test.cpp](src/stereo_pluecker_test.cpp) | Plücker line representation for 3D line triangulation from stereo views |
| `test_geometry_obj` | [obj_test.cpp](src/obj_test.cpp) | Wavefront OBJ loading: vertex/edge extraction, Eigen interop *(requires OpenGL)* |

> **Not built by default** (commented out in CMake):
> [camera_test.cpp](src/camera_test.cpp) — camera projection and intrinsic/extrinsic parameters,
> [stereo_test.cpp](src/stereo_test.cpp) — stereo calibration, rectification, and 3D reconstruction.

## Building & Running

```bash
# Build all geometry examples
bazel build //examples/geometry:all

# Run a specific example
bazel run //examples/geometry:test_geometry_line
bazel run //examples/geometry:test_geometry_stereo_pluecker
```

## Key Concepts

- **Camera Models:** Pinhole camera with intrinsic/extrinsic parameters
- **Stereo Geometry:** Epipolar constraints, triangulation, rectification
- **Plücker Coordinates:** 6D representation for 3D lines — efficient for intersection and distance computations
- **Projective Geometry:** Homogeneous coordinates and projective transformations
- **3D Primitives:** Points, lines, planes, and polygons in 2D/3D space

## Related Libraries

- [libs/geometry](../../libs/geometry/) — Geometric primitives, cameras, stereo vision
- [libs/imgproc](../../libs/imgproc/) — Used for some gradient-based visualizations
