# Geometry Library

Geometric primitives and camera/stereo utilities used across LineExtraction. This library provides 2D/3D points, lines, planes, polygons, pose/camera models, and helpers for projections, triangulation, and visualization.

## Overview

Core capabilities:

- **Primitives**: 2D/3D points, lines, line segments, planes, polygons.
- **Transforms**: Pose representation and homogeneous transforms.
- **Cameras**: Pinhole camera model and OpenCV-compatible projection helpers.
- **Stereo**: Triangulation utilities for points and lines.
- **Utilities**: Eigen/OpenCV conversions, drawing helpers, OpenGL/ROS visualization (optional).

## Key Headers

- `geometry/point.hpp`, `geometry/line.hpp`, `geometry/line3.hpp`
- `geometry/plane.hpp`, `geometry/polygon.hpp`
- `geometry/pose.hpp`, `geometry/camera.hpp`, `geometry/cameracv.hpp`
- `geometry/stereo.hpp`, `geometry/stereocv.hpp`
- `geometry/eigen2cv.hpp`, `geometry/draw.hpp`

## Usage

Include the headers you need and use the `lsfm` namespace:

- Basic primitives and transforms:
  - `Point<FT>`, `Line<FT>`, `Line3<FT>`, `Plane<FT>`
  - `Pose<FT>` for translation/rotation (Rodrigues)
- Camera and stereo:
  - `Camera<FT>`, `CameraCV<FT>` for projection
  - `Stereo<FT>` and `StereoCV<FT>` for triangulation

## Build

Use Bazel (primary build system):

- Build geometry library:
  - `bazel build //libs/geometry:lib_geometry`
- Run geometry tests:
  - `bazel test //libs/geometry:all`

## Notes

- OpenGL/ROS helpers are optional and depend on system availability.
- See headers for full API documentation and examples.

## Related Libraries and Resources

- [lsd](../lsd/README.md) - Line Segment Detection algorithms
- [lfd](../lfd/README.md) - Line Feature Descriptors
- [edge](../edge/README.md) - Edge detection algorithms
- [imgproc](../imgproc/README.md) - Image processing and gradient computation
- [utility](../utility/README.md) - Core utilities
- [Geometry Examples](../../examples/geometry/README.md) - Demonstration programs
- [Thesis Evaluations](../../evaluation/thesis/README.md) - Precision evaluations using geometric primitives
- [Main README](../../README.md) - Project overview
