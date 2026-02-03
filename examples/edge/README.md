# Edge Detection Examples

Examples demonstrating edge detection, Non-Maximum Suppression (NMS), zero-crossing detection, and edge linking strategies.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [edge_test.cpp](edge_test.cpp) | Comprehensive edge detection pipeline with various linking strategies | `./edge_test [image_path]` |
| [edge_precision.cpp](edge_precision.cpp) | Sub-pixel edge localization precision evaluation | `./edge_precision` |
| [nfa_test.cpp](nfa_test.cpp) | NFA validation for edge segments | `./nfa_test [image_path]` |
| [nms_test.cpp](nms_test.cpp) | NMS benchmarking with multiple gradient operators | `./nms_test [image_path]` |
| [nms_variant_test.cpp](nms_variant_test.cpp) | Comparison of NMS variants (4/8-directional) | `./nms_variant_test [image_path]` |
| [pattern_test.cpp](pattern_test.cpp) | Pattern-based edge segment detection | `./pattern_test [image_path]` |
| [zc_test.cpp](zc_test.cpp) | Zero-crossing edge detection | `./zc_test [image_path]` |
| [zc_variant_test.cpp](zc_variant_test.cpp) | Zero-crossing detection variants | `./zc_variant_test [image_path]` |

## Building

**Bazel:**
```bash
bazel build //examples/edge:all
bazel run //examples/edge:edge_test -- /path/to/image.jpg
```

**CMake:**
```bash
make  # from build directory
```

## Key Algorithms

- **NMS (Non-Maximum Suppression):** Thinning edge responses to single-pixel width
- **Zero-Crossing:** Detecting edges at Laplacian zero-crossings
- **Edge Linking:** Connecting edge pixels into continuous segments
- **NFA Validation:** Statistical validation of detected edges using a contrario framework

## Related Libraries

- [libs/edge](../../libs/edge/) - Edge detection library implementation
- [libs/imgproc](../../libs/imgproc/) - Image processing and gradient operators
