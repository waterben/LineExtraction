# Edge Detection Library

Comprehensive edge detection and analysis library providing multiple implementations of edge detection algorithms, including Non-Maximum Suppression (NMS), zero-crossing detection, and edge linking strategies.

## Overview

The `edge` library is a modular, header-only C++ library designed for efficient edge detection in digital images. It provides:

- **Multiple edge response methods**: Gradient-based, Laplacian, Quadrature, Phase Congruency
- **Flexible edge extraction**: Non-Maximum Suppression (4/8-directional), Zero-Crossing detection
- **Edge linking strategies**: Simple linking, continuity-optimized linking, pattern-based detection
- **Sub-pixel precision**: Multiple localization estimators for accurate edge positioning
- **Validation framework**: Number of False Alarms (NFA) testing for statistical validation
- **Curve fitting**: Line and general curve fitting to edge segments

All code resides in the `lsfm` namespace.

## Key Components

### Edge Response & Extraction

The library provides a flexible framework for combining edge response filters with extraction methods:

**Edge Sources** (`edge_source.hpp`):

- `EdgeSourceGRAD`: Sobel/gradient-based edge detection
- `EdgeSourceNMS`: NMS-based edge extraction
- `EdgeSourceLAPLACE`: Laplacian zero-crossing detection
- `EdgeSourceQUAD`: Quadrature filter responses (odd/even)
- `EdgeSourcePC`: Phase Congruency responses

**NMS Methods** (`nms.hpp`):

- `PreciseNMS`: High-precision NMS with interpolation
- Multiple variants with 4/8-directional support
- Scalar and vector thresholding support

**Zero-Crossing Detection** (`zc.hpp`):

- `eps_zero`: Epsilon-based zero-crossing detection
- `zc_base`: Base zero-crossing implementation
- Multiple direction encoding schemes

### Edge Linking & Segmentation

Converts edgels (edge pixels) into connected segments:

**Simple Linking** (`edge_simple.hpp`):

```cpp
lsfm::EsdSimple<float, 8> detector(minPixels=10);
detector.detect(direction_map, magnitude_map, seed_indices);
// Results in: detector.segments(), detector.points()
```

**Continuity-Optimized Linking** (`edge_linking.hpp`):

```cpp
lsfm::EsdLinking<float, 8> detector(minPixels=10, maxGap=3, magMul=3.0, magTh=5.0);
detector.detect(direction_map, magnitude_map, seed_indices);
```

**Pattern-Based Detection** (`edge_pattern.hpp`):

```cpp
lsfm::EsdPattern<float, 8> detector(minPixels=10, maxGap=3, magMul=3.0, magTh=5.0, patTol=2);
detector.detect(direction_map, magnitude_map, seed_indices);
```

### Utility Components

**Thresholding** (`threshold.hpp`):

- `GlobalThreshold`: Global thresholding
- `ThresholdOtsu`: Otsu's automatic threshold estimation
- `ThresholdLocalWinning`: Local adaptive thresholding

**Sub-pixel Localization** (`spe.hpp`):

- `LinearEstimate`: Linear sub-pixel refinement
- `QuadraticEstimate`: Parabolic fitting
- `CoGEstimate`: Center of gravity method
- `SobelEstimate`: Sobel-based refinement

**Curve Fitting** (`fit.hpp`):

- Line fitting to edge segments
- Conic section fitting
- Robust estimation via RANSAC

**Validation** (`nfa.hpp`):

- `NfaContrast`: Contrast-based validation
- `NfaBinom`: Binomial NFA testing
- Statistical significance testing for edge segments

### Data Structures

**Edge Segments** (`edge_segment.hpp`):

- `EdgeSegment`: Represents connected edge pixel sequence
- `EsdBase`: Base class for edge detection algorithms
- Helper functions for segment analysis

**Utilities** (`index.hpp`):

- Index/coordinate conversion functions
- Matrix element access by linear index
- Batch point conversions

## Usage Examples

### Basic Edge Detection with NMS

```cpp
#include <edge/edge_source.hpp>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <opencv2/opencv.hpp>

using namespace lsfm;

int main() {
    cv::Mat image = cv::imread("input.jpg", cv::IMREAD_GRAYSCALE);

    // Compute gradients
    cv::Mat gx, gy, mag;
    DerivativeGradient<uchar, float>::compute(image, gx, gy);
    cv::magnitude(gx, gy, mag);

    // Apply NMS
    IndexVector seeds;
    cv::Mat dmap;
    PreciseNMS<float, float, false>::process(gx, gy, mag, 10.0f, 50.0f, seeds, dmap);

    // Extract edge segments
    EsdSimple<float, 8> detector;
    detector.detect(dmap, mag, seeds);

    const EdgeSegmentVector& segments = detector.segments();
    const std::vector<cv::Point2f>& points = detector.points();

    std::cout << "Detected " << segments.size() << " edge segments\n";
    return 0;
}
```

### Edge Detection with Continuity Optimization

```cpp
#include <edge/edge_linking.hpp>
#include <edge/edge_source.hpp>

using namespace lsfm;

int main() {
    // ... (setup as above) ...

    // Use linking with continuity optimization
    EsdLinking<float, 8> detector(
        10,    // minPixels
        3,     // maxGap
        3.0f,  // magMul
        5.0f   // magTh
    );

    detector.detect(dmap, mag, seeds);

    for (const auto& seg : detector.segments()) {
        std::cout << "Segment with " << seg.size() << " points\n";
    }

    return 0;
}
```

### Zero-Crossing Edge Detection

```cpp
#include <edge/edge_source.hpp>
#include <edge/zc.hpp>
#include <imgproc/laplace.hpp>

using namespace lsfm;

int main() {
    cv::Mat image = cv::imread("input.jpg", cv::IMREAD_GRAYSCALE);

    // Compute Laplacian
    cv::Mat laplace;
    cv::Laplacian(image, laplace, CV_32F);

    // Detect zero-crossings with hysteresis
    IndexVector seeds;
    cv::Mat dmap;
    zc_base<float, float, NCC_BASIC>(laplace, -10.0f, 10.0f, seeds, dmap);

    EsdSimple<float, 8> detector;
    detector.detect(dmap, cv::abs(laplace), seeds);

    return 0;
}
```

### Edge Validation with NFA

```cpp
#include <edge/nfa.hpp>

using namespace lsfm;

int main() {
    // ... (edge detection as above) ...

    NfaContrast<uchar, float, cv::Point2f> nfa(255, 0);  // range [0, 255]

    for (const auto& seg : detector.segments()) {
        double nfa_value = nfa(mag, seg, points);
        if (nfa_value < 1.0) {
            std::cout << "Valid edge segment (NFA=" << nfa_value << ")\n";
        }
    }

    return 0;
}
```

## Headers Reference

| Header | Purpose |
|--------|---------|
| `edge_source.hpp` | Abstract interface for edge response computation |
| `nms.hpp` | Non-Maximum Suppression implementations (4/8-directional) |
| `zc.hpp` | Zero-crossing edge detection methods |
| `hysteresis.hpp` | Hysteresis thresholding algorithms |
| `threshold.hpp` | Threshold estimation and application |
| `edge_simple.hpp` | Simple edge linking without optimization |
| `edge_linking.hpp` | Continuity-optimized edge linking |
| `edge_pattern.hpp` | Pattern-based edge segment detection |
| `edge_drawing.hpp` | Drawing-based edge tracing |
| `edge_segment.hpp` | Edge segment data structures |
| `index.hpp` | Index and coordinate utilities |
| `spe.hpp` | Sub-pixel edge localization estimators |
| `split.hpp` | Edge segment splitting algorithms |
| `fit.hpp` | Curve fitting for edge segments |
| `nfa.hpp` | Number of False Alarms validation |
| `line_tracer.hpp` | Line-oriented edge tracing |
| `segment_sampler.hpp` | Edge segment sampling utilities |
| `draw.hpp` | Visualization utilities |

## Build System

The edge library uses **Bazel** as the primary build system:

```bash
# Build the library
bazel build //libs/edge:lib_edge

# Run all edge library tests
bazel test //libs/edge:all

# Build specific test
bazel test //libs/edge:test_nms
```

For CMake (legacy, not recommended):

```bash
cd build
cmake --build . --target edge
ctest -R edge
```

## Design Patterns

### Template-Based Flexibility

The library uses C++ templates extensively to allow compile-time specialization:

```cpp
// Customize direction encoding and interpolation
using MyNMS = PreciseNMS<
    float,           // GT (gradient type)
    float,           // MT (magnitude type)
    false,           // SQR (magnitude is squared?)
    float,           // DT (calculation type)
    EMap8,           // Direction encoder (EMap4, EMap8)
    LinearInterpolator  // Interpolation method
>;
```

### Strategy Pattern for Detection

Different edge detection strategies implement a common interface (`EsdBase`):

```cpp
template<typename Detector>
void processEdges(const cv::Mat& dir, const cv::Mat& mag, Detector& detector) {
    IndexVector seeds;
    // ... generate seeds ...
    detector.detect(dir, mag, seeds);
    // detector.segments() available for all implementations
}
```

## Performance Considerations

- **NMS**: Fast implementations with 4 or 8-directional checking
- **Precise NMS**: Higher accuracy with sub-pixel interpolation at higher cost
- **Memory**: Header-only design enables inline optimization
- **Parallelization**: Compatible with OpenCV's parallel processing

## Dependencies

- **OpenCV 4.0+**: Core image processing (Mat, basic operations)
- **Geometry Library**: Point types and drawing utilities
- **Image Processing Library**: Gradient computation, filtering

## Threading & Safety

- Thread-safe for read-only operations
- `EsdBase` and derived classes maintain internal state - use separate instances for parallel processing

## Additional Resources

- [Edge detection examples](../../examples/edge/README.md) - Demonstration programs
- [Line Analyzer app](../../apps/line_analyzer/README.md) - Interactive GUI tool
- [Main README](../../README.md) - Project overview
- Test cases in `tests/` directory
- Full API documentation in header files

## Related Libraries

- [imgproc](../imgproc/README.md) - Image processing and gradient computation
- [geometry](../geometry/README.md) - Geometric primitives and line representation
- [lsd](../lsd/README.md) - Line Segment Detection algorithms
- [eval](../eval/README.md) - Evaluation and benchmarking framework

## Author & License

Implementation and enhancements by Benjamin Wassermann. Licensed under BSD license (see header comments in source files).

---

**Note**: For C++ coding standards, see `.github/instructions/cpp.instructions.md` for Doxygen documentation style and coding conventions used throughout this library.
