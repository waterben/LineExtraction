# Geometry Examples

Examples demonstrating geometric primitives, camera models, and stereo vision.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [camera_test.cpp](camera_test.cpp) | Camera projection and parameter handling | `./camera_test` |
| [gt_test.cpp](gt_test.cpp) | Ground truth geometry loading | `./gt_test` |
| [line_test.cpp](line_test.cpp) | Line primitive operations | `./line_test` |
| [obj_test.cpp](obj_test.cpp) | OBJ file loading and 3D objects | `./obj_test` |
| [poly_test.cpp](poly_test.cpp) | Polygon operations | `./poly_test` |
| [render_gl_tr_test.cpp](render_gl_tr_test.cpp) | OpenGL rendering (requires OpenGL/GLUT) | `./render_gl_tr_test` |
| [stereo_pluecker_test.cpp](stereo_pluecker_test.cpp) | Plücker coordinates for stereo | `./stereo_pluecker_test` |
| [stereo_test.cpp](stereo_test.cpp) | Stereo vision and triangulation | `./stereo_test` |

## Building

**Bazel:**
```bash
bazel build //examples/geometry:all
bazel run //examples/geometry:camera_test
```

**CMake:**
```bash
make  # from build directory
```

## Key Concepts

- **Camera Models:** Pinhole camera with intrinsic/extrinsic parameters
- **Stereo Geometry:** Epipolar constraints, triangulation, rectification
- **3D Primitives:** Points, lines, planes in 3D space
- **Plücker Coordinates:** 6D representation for 3D lines
- **Projective Geometry:** Homogeneous coordinates and transformations

## Related Libraries

- [libs/geometry](../../libs/geometry/) - Geometric primitives library
