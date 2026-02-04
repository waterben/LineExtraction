# Line Feature Descriptor Examples

Examples demonstrating line matching and feature descriptors for stereo vision, motion tracking, and video analysis.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [cv3_linematching_test.cpp](cv3_linematching_test.cpp) | OpenCV line descriptor comparison | `./cv3_linematching_test [left] [right]` |
| [dsc_test.cpp](dsc_test.cpp) | LR descriptor creation | `./dsc_test [left] [right]` |
| [generic_test.cpp](generic_test.cpp) | Generic matching framework | `./generic_test [left] [right]` |
| [match_test.cpp](match_test.cpp) | Pairwise line matching with LBD | `./match_test [left] [right]` |
| [motion_match_test.cpp](motion_match_test.cpp) | Motion-based matching for video | `./motion_match_test [video_path]` |
| [pair_test.cpp](pair_test.cpp) | Pairwise matching constraints | `./pair_test [left] [right]` |
| [stereo_match_test.cpp](stereo_match_test.cpp) | Stereo line matching | `./stereo_match_test [left] [right]` |
| [video_cv_matching_test.cpp](video_cv_matching_test.cpp) | Video tracking with OpenCV | `./video_cv_matching_test [video]` |
| [video_match_test.cpp](video_match_test.cpp) | Comprehensive stereo video | `./video_match_test [intrinsic] [extrinsic] [left_video] [right_video]` |

## Building

**Bazel:**
```bash
bazel build //examples/lfd:all
bazel run //examples/lfd:match_test -- left.jpg right.jpg
```

**CMake:**
```bash
make  # from build directory
```

## Key Concepts

- **LBD:** Line Band Descriptor for line segment appearance
- **LR Descriptor:** Combined line and region features
- **Motion Filtering:** Temporal consistency in video sequences
- **Stereo Filtering:** Epipolar constraint-based candidate reduction
- **Pairwise Matching:** Nearest neighbor with ratio test and bidirectional check
- **3D Triangulation:** Reconstructing 3D lines from stereo matches

## Related Libraries

- [libs/lfd](../../libs/lfd/) - Line Feature Descriptor library
- [libs/lsd](../../libs/lsd/) - Line Segment Detection library
