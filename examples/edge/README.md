# Edge Detection Examples

Examples demonstrating edge detection, Non-Maximum Suppression (NMS), zero-crossing detection, and edge linking strategies.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `edge_test` | [edge_test.cpp](src/edge_test.cpp) | Full edge detection pipeline: gradient → NMS → linking (EsdSimple, EsdLinking, EsdPattern) |
| `edge_precision` | [edge_precision.cpp](src/edge_precision.cpp) | Sub-pixel edge localization precision on synthetic hexagon ground truth |
| `nfa_test` | [nfa_test.cpp](src/nfa_test.cpp) | NFA (Number of False Alarms) statistical validation of edge segments |
| `nms_test` | [nms_test.cpp](src/nms_test.cpp) | NMS benchmarking with Sobel, Scharr, RCMG, SUSAN, quadrature, and phase congruency gradients |
| `nms_variant_test` | [nms_variant_test.cpp](src/nms_variant_test.cpp) | Comparison of 4-directional vs 8-directional NMS with color-coded direction visualization |
| `pattern_test` | [pattern_test.cpp](src/pattern_test.cpp) | Pattern-based edge segment detection (EsdPattern) — robust at junctions and corners |
| `zc_test` | [zc_test.cpp](src/zc_test.cpp) | Zero-crossing edge detection with Laplacian, LoG, and quadrature filter operators |
| `zc_variant_test` | [zc_variant_test.cpp](src/zc_variant_test.cpp) | Zero-crossing variants with different direction encodings and threshold strategies |

## Building & Running

```bash
# Build all edge examples
bazel build //examples/edge:all

# Run with default test image (windmill.jpg)
bazel run //examples/edge:edge_test

# Run with custom image
bazel run //examples/edge:nms_test -- /path/to/image.jpg
```

## Key Algorithms

- **NMS (Non-Maximum Suppression):** Thinning edge responses to single-pixel width using gradient direction
- **Zero-Crossing Detection:** Edges located at Laplacian sign changes
- **Edge Linking:** Connecting edge pixels into continuous segments (simple, continuity-optimized, pattern-based)
- **NFA Validation:** A contrario statistical test rejecting edges that could arise by chance
- **Sub-Pixel Estimation:** Linear, quadratic, and Gaussian interpolation for sub-pixel edge positions

## Related Libraries

- [libs/edge](../../libs/edge/) — Edge detection, NMS, zero-crossing, linking
- [libs/imgproc](../../libs/imgproc/) — Gradient operators, quadrature filters, phase congruency
