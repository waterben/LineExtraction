# Line Feature Descriptor Examples

Examples demonstrating line segment matching and feature descriptors for stereo vision, motion tracking, and video analysis.

[← Back to Examples](../README.md)

## Examples

### Basic Descriptors & Matching

| Bazel Target | Source | Description |
|---|---|---|
| `test_lfd_dsc` | [dsc_test.cpp](src/dsc_test.cpp) | Line-Region (LR) descriptor creation and distance computation |
| `test_lfd_generic` | [generic_test.cpp](src/generic_test.cpp) | Generic descriptor and matcher interfaces for flexible matching |
| `test_lfd_stereo_match` | [stereo_match_test.cpp](src/stereo_match_test.cpp) | Stereo line matching with epipolar constraint filtering (StereoLineFilter) |

### Pairwise & Advanced Matching *(requires SuperLU)*

| Bazel Target | Source | Description |
|---|---|---|
| `test_lfd_pair` | [pair_test.cpp](src/pair_test.cpp) | Pairwise matching: nearest neighbor, ratio test, bidirectional check |
| `test_lfd_match` | [match_test.cpp](src/match_test.cpp) | Full pipeline: LSD → LBD → global rotation filtering → pairwise matching *(+line_descriptor)* |
| `test_lfd_motion_match` | [motion_match_test.cpp](src/motion_match_test.cpp) | Motion-based line matching between consecutive video frames |
| `test_lfd_video_match` | [video_match_test.cpp](src/video_match_test.cpp) | Stereo video: rectification, motion filtering, 3D line triangulation |

### OpenCV Integration *(requires SuperLU + opencv_contrib line_descriptor)*

| Bazel Target | Source | Description |
|---|---|---|
| `test_lfd_cv3_linematching` | [cv3_linematching_test.cpp](src/cv3_linematching_test.cpp) | Comparison of OpenCV BinaryDescriptor vs custom LBD implementation |
| `test_lfd_video_cv_matching` | [video_cv_matching_test.cpp](src/video_cv_matching_test.cpp) | Video line tracking using OpenCV BinaryDescriptor with motion filtering |

## Building & Running

```bash
# Build all LFD examples
bazel build //examples/lfd:all

# Run stereo matching
bazel run //examples/lfd:test_lfd_stereo_match

# Run with custom stereo pair
bazel run //examples/lfd:test_lfd_match -- left.jpg right.jpg
```

## Key Concepts

- **LBD (Line Band Descriptor):** Appearance descriptor based on gradient bands around line segments
- **LR Descriptor:** Combined line geometry and region appearance features
- **StereoLineFilter:** Epipolar constraint-based candidate reduction for stereo matching
- **MotionLineFilter:** Temporal consistency enforcement between consecutive video frames
- **Pairwise Matching:** Nearest neighbor with Lowe's ratio test and bidirectional consistency
- **3D Triangulation:** Reconstructing 3D lines from matched stereo correspondences

## Related Libraries

- [libs/lfd](../../libs/lfd/) — Line Feature Descriptor library
- [libs/lsd](../../libs/lsd/) — Line Segment Detection (used for input)
- [libs/geometry](../../libs/geometry/) — Stereo geometry, Plücker lines, cameras
