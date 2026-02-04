# Miscellaneous Examples

Examples demonstrating external library integration and miscellaneous utilities.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [arpack_test.cpp](arpack_test.cpp) | ARPACK++ eigenvalue computation | `./arpack_test` |
| [houghlines.cpp](houghlines.cpp) | OpenCV Hough transform comparison | `./houghlines [image_path]` |
| [opencv_features.cpp](opencv_features.cpp) | AKAZE feature detection and matching | `./opencv_features [video_path]` |
| [test_dlib.cpp](test_dlib.cpp) | dlib optimization algorithms | `./test_dlib` |

## Building

**Bazel:**
```bash
bazel build //examples/other:all
bazel run //examples/other:houghlines -- image.jpg
```

**CMake:**
```bash
make  # from build directory
```

## External Libraries

- **ARPACK++:** Sparse eigenvalue problems (used in spectral methods)
- **dlib:** Machine learning and optimization toolkit
- **OpenCV xfeatures2d:** Extended feature detectors (SIFT, SURF, AKAZE)

## Purpose

These examples demonstrate:
- Integration with external numerical libraries
- Comparison with standard OpenCV algorithms
- Optimization and eigenvalue computation for advanced algorithms

## Related Libraries

- [libs/utility](../../libs/utility/) - Utility library
