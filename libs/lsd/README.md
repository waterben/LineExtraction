# Line Segment Detector (LSD) Library

Comprehensive collection of line segment detection algorithms with high precision and configurable parameters. Supports multiple detection methods from classical algorithms to modern statistical approaches.

## Overview

The `lsd` library provides implementations of various line segment detection algorithms:

- **LSD (LSD with NFA)** - Statistical line segment detector using Number of False Alarms principle
- **EDLines (LsdEDLZ)** - Edge Drawing Lines algorithm with anchor-based detection
- **FGioi (LsdFGioi)** - Fusion of Global and Local Information with NFA-based validation
- **Burns (LsdBurns)** - Classic gradient-based line extraction with directional partitioning

## Architecture

The library uses a template-based class hierarchy:

```
LdBase (abstract base)
  └─ LsdBase (line segment base)
       ├─ LsdEDLZ (EDLines algorithm)
       ├─ LsdFGioi (FGioi/PLSD algorithm)
       ├─ LsdBurns (Burns algorithm)
       └─ LsdExt (extended interface for internal data access)
```

### Type Parameters

All detectors are template classes supporting flexible types:

```cpp
// Template parameters
template <class FT, template <class> class LPT>
class LsdDetector : public LsdBase<FT, LPT> { };

// FT: Floating-point type (float, double, long double)
// LPT: Line point template (Vec2, Vec3, etc.)
```

Common instantiations:

```cpp
lsfm::LsdEDLZ<float>      // Single-precision float lines
lsfm::LsdEDLZ<double>     // Double-precision float lines
```

## Core Components

### 1. Base Classes

#### LdBase<FT, LPT>

Abstract base class defining the line detection interface:

```cpp
#include <lsd/ld_base.hpp>

// Pure virtual interface
virtual void detect(const cv::Mat& image) = 0;
virtual const LineVector& lines() const;
virtual const DataDescriptor& imageDataDescriptor() const;
virtual const ImageData& imageData() const;
```

**Features:**

- Abstract interface for all line detectors
- Parametrizable via ValueManager
- Auxiliary image data support (gradients, edges)
- Line data descriptor for structured output

#### LsdBase<FT, LPT>

Extends LdBase with line segment support:

```cpp
#include <lsd/lsd_base.hpp>

// Get detected lines as segments
const LineSegmentVector& lineSegments() const;

// Get line endpoints as 4-element vectors (x1, y1, x2, y2)
const EndPointVector& endPoints() const;

// Convenience detection methods
void detect(const cv::Mat& image, LineSegmentVector& segments);
void detect(const cv::Mat& image, EndPointVector& endpoints);
```

**Features:**

- Line segment representation with full geometric info
- Lazy conversion between line and endpoint formats
- Consistent interface across all detectors

#### LsdExt<FT, LPT, PT>

Extended interface for internal data access:

```cpp
#include <lsd/lsd_base.hpp>

// Get edge segment support information
const EdgeSegmentVector& lineSupportSegments() const;
const PointVector& points() const;
const IndexVector& indexes() const;
```

**Use cases:**

- Algorithm analysis and debugging
- Detailed visualization of detection process
- Support region examination

### 2. Detection Algorithms

#### EDLines (LsdEDLZ)

Edge Drawing Lines algorithm - high-precision edge-based detection.

```cpp
#include <lsd/lsd_edlz.hpp>

// Create detector with parameters
lsfm::LsdEDLZ<float> detector(
    10.0f,    // Gradient threshold (5-20)
    2.0f,     // Anchor threshold (1-5)
    2,        // Scan intervals (1-3)
    15,       // Minimum line length (pixels)
    2.0f,     // Fit error threshold (pixels)
    false     // Validate with NFA
);

// Detect lines
cv::Mat image = cv::imread("input.png", cv::IMREAD_GRAYSCALE);
detector.detect(image);

// Access results
const auto& segments = detector.lineSegments();
const auto& endpoints = detector.endPoints();

// Access auxiliary data
const auto& imageData = detector.imageData();
const auto& descriptor = detector.imageDataDescriptor();
for (size_t i = 0; i < imageData.size(); ++i) {
    std::cout << descriptor[i].name << ": " << descriptor[i].description << std::endl;
}
```

**Available Auxiliary Data:**

- `gx`: Gradient in x direction
- `gy`: Gradient in y direction
- `mag`: Gradient magnitude
- `th_mag`: Thresholded gradient magnitude
- `dir_map`: Direction map
- `edge_map`: Edge map

**Configuration:**

- Parametrize via options: `detector.gradientThreshold(15.0f);`
- Or use initializer list: `LsdEDLZ<float>({{"grad_th", 15}})`

**When to use:**

- High-precision line detection required
- Weak edge handling needed
- Fast processing important

#### FGioi/PLSD (LsdFGioi)

Probabilistic Line Segment Detector using global and local information.

```cpp
#include <lsd/lsd_fgioi.hpp>

// Create detector
lsfm::LsdFGioi<float> detector(
    2.0f,         // Quantization error bound (0-5)
    22.5f,        // Angle tolerance in degrees (10-45)
    0.0f,         // NFA log threshold (0-5)
    0.7f,         // Density threshold (0.5-0.9)
    1024          // Gradient magnitude bins (1024-4096)
);

// Detect lines
detector.detect(image);

// Access line data with precision information
const auto& segments = detector.lineSegments();

// Access line-specific data (width, precision, NFA)
// Note: Extended interface provides access to LineData
```

**Line Statistics:**

- Width: Support region width
- Precision: Confidence/precision measure
- NFA: Number of False Alarms (statistical significance)

**When to use:**

- Statistical validation important
- Noise robustness needed
- Precision/confidence measures required

#### Burns Algorithm (LsdBurns)

Classic gradient-based line extraction with directional partitioning.

```cpp
#include <lsd/lsd_burns.hpp>

// Create detector
lsfm::LsdBurns<float> detector(
    0.004f,          // Lower threshold (normalized, 0-1)
    0.012f,          // Upper threshold (normalized, 0-1)
    5,               // Minimum pixels (5-20)
    12,              // Direction partitions (4-255)
    lsfm::BURNS_NMS  // Use Non-Maxima Suppression
);

// Detect lines
detector.detect(image);

// Access results
const auto& segments = detector.lineSegments();

// Configure gradient computation
detector.gradientThreshold(0.01f);
```

**Configuration Options:**

- `nms_th_low`: Lower threshold (normalized)
- `nms_th_high`: Upper threshold (normalized)
- `edge_min_pixels`: Minimum segment length
- `edge_partitions`: Direction discretization
- `line_flags`: Enable/disable NMS

**When to use:**

- Fast processing needed
- Standard edge-based detection sufficient
- Legacy compatibility important

## Usage Examples

### Basic Line Detection

```cpp
#include <lsd/lsd_edlz.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    // Load image
    cv::Mat image = cv::imread("image.jpg", cv::IMREAD_GRAYSCALE);

    // Create and configure detector
    lsfm::LsdEDLZ<float> detector;
    detector.gradientThreshold(12.0f);
    detector.minLength(20);

    // Detect lines
    detector.detect(image);

    // Get results
    const auto& lines = detector.lineSegments();
    std::cout << "Detected " << lines.size() << " lines" << std::endl;

    // Process each line
    for (const auto& line : lines) {
        auto p1 = line.point1();
        auto p2 = line.point2();
        std::cout << "Line from (" << p1.x() << ", " << p1.y() << ") to ("
                  << p2.x() << ", " << p2.y() << ")" << std::endl;
    }

    return 0;
}
```

### Comparing Multiple Detectors

```cpp
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_fgioi.hpp>
#include <lsd/lsd_burns.hpp>

void compare_detectors(const cv::Mat& image) {
    // EDLines detector
    lsfm::LsdEDLZ<float> edlines;
    edlines.detect(image);
    std::cout << "EDLines: " << edlines.lineSegments().size() << " lines" << std::endl;

    // FGioi detector
    lsfm::LsdFGioi<float> fgioi;
    fgioi.detect(image);
    std::cout << "FGioi: " << fgioi.lineSegments().size() << " lines" << std::endl;

    // Burns detector
    lsfm::LsdBurns<float> burns;
    burns.detect(image);
    std::cout << "Burns: " << burns.lineSegments().size() << " lines" << std::endl;
}
```

### Accessing Auxiliary Data

```cpp
void analyze_gradients(const cv::Mat& image) {
    lsfm::LsdEDLZ<float> detector;
    detector.detect(image);

    // Get descriptor for all data layers
    const auto& descriptor = detector.imageDataDescriptor();
    const auto& data = detector.imageData();

    std::cout << "Available data layers:" << std::endl;
    for (size_t i = 0; i < descriptor.size(); ++i) {
        std::cout << "  " << descriptor[i].name << ": "
                  << descriptor[i].description << std::endl;
        std::cout << "    Size: " << data[i].rows << "x" << data[i].cols
                  << " Type: " << data[i].type() << std::endl;
    }

    // Access specific layer by name
    cv::Mat magnitude = detector.imageData("mag");
    if (!magnitude.empty()) {
        cv::Mat norm_mag;
        cv::normalize(magnitude, norm_mag, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Gradient Magnitude", norm_mag);
    }
}
```

### Parameter Optimization

```cpp
void optimize_for_speed(const cv::Mat& image) {
    // Fast configuration
    lsfm::LsdEDLZ<float> fast(
        15.0f,  // Higher gradient threshold
        1.0f,   // Lower anchor threshold
        1,      // Fewer scan intervals
        30,     // Longer minimum length
        3.0f,   // Higher fit error tolerance
        false   // No validation
    );

    fast.detect(image);
}

void optimize_for_precision(const cv::Mat& image) {
    // Precise configuration
    lsfm::LsdEDLZ<float> precise(
        5.0f,   // Lower gradient threshold
        5.0f,   // Higher anchor threshold
        3,      // More scan intervals
        10,     // Shorter minimum length
        1.0f,   // Tighter fit error
        true    // Enable validation
    );

    precise.detect(image);
}
```

## Performance Characteristics

| Algorithm | Speed | Precision | Robustness | Memory |
|-----------|-------|-----------|------------|--------|
| EDLines   | Fast  | High      | Good       | Low    |
| FGioi     | Medium| Very High | Excellent  | Medium |
| Burns     | Very Fast | Medium | Fair      | Low    |

## Dependencies

- OpenCV 4.0+ (core, imgproc)
- Standard C++ library (STL)
- Geometry library (geometry primitives and operations)
- Image processing utilities (imgproc library)
- Edge detection utilities (edge library)

## Integration with Other Libraries

### With Geometry Library

```cpp
#include <lsd/lsd_edlz.hpp>
#include <geometry/line.hpp>

// Line segments can be converted to Line objects
lsfm::LsdEDLZ<float> detector;
detector.detect(image);

const auto& segments = detector.lineSegments();
for (const auto& seg : segments) {
    // seg is already convertible to Line<float, Vec2<float>>
    lsfm::Line<float> line = seg;
    auto length = line.length();
}
```

### With Edge Library

```cpp
// Burns detector uses configurable gradient computation
#include <lsd/lsd_burns.hpp>
#include <imgproc/derivative_gradient.hpp>

// Template allows custom gradient computation strategies
using MyGradient = lsfm::DerivativeGradient<
    uchar, short, int, float,
    lsfm::ScharrDerivative,
    lsfm::QuadraticMagnitude>;

lsfm::LsdBurns<float, Vec2, Vec2i, MyGradient> detector;
```

## Thread Safety

- Detector instances are **not thread-safe**: Each thread must use its own detector instance
- `detect()` method modifies internal state (parameter binding, caching)
- Multiple detectors can process different images in parallel

```cpp
// Safe: One detector per thread
#pragma omp parallel for
for (int i = 0; i < images.size(); ++i) {
    lsfm::LsdEDLZ<float> detector;  // Thread-local
    detector.detect(images[i]);
}

// Unsafe: Shared detector across threads
lsfm::LsdEDLZ<float> detector;  // Shared
#pragma omp parallel for
for (int i = 0; i < images.size(); ++i) {
    detector.detect(images[i]);  // Race condition!
}
```

## Common Issues and Solutions

### No Lines Detected

```cpp
// Increase sensitivity
detector.gradientThreshold(5.0f);      // Lower threshold
detector.minLength(5);                 // Accept shorter lines
detector.validate(false);              // Disable validation
```

### Too Many False Positive Lines

```cpp
// Decrease sensitivity
detector.gradientThreshold(20.0f);     // Higher threshold
detector.minLength(50);                // Require longer lines
detector.validate(true);               // Enable NFA validation
```

### Memory Issues on Large Images

```cpp
// Use Burns algorithm (lowest memory)
lsfm::LsdBurns<float> detector;

// Or reduce auxiliary data generation
// (EDLines always provides gradient data)
```

## Code Examples Location

Complete examples are available in:

- `examples/lsd/` - Standalone detector demonstrations
- `apps/line_analyzer/` - Qt-based interactive analysis tool
- `evaluation/performance/` - Benchmarking scripts

## References

1. **EDLines Algorithm**
   - Akinlar, C., & Topal, C. (2013). "Edge Drawing: A Fast Edge Detection Algorithm"

2. **FGioi/PLSD Algorithm**
   - Grompone von Gioi, R., et al. (2010). "LSD: A Fast Line Segment Detector with a False Detection Control"

3. **Burns Algorithm**
   - Burns, J. B., et al. (1986). "Extracting Straight Lines"

## See Also

- [Edge Library](../edge/README.md) - Edge detection and NMS algorithms
- [Geometry Library](../geometry/README.md) - Line representation and transformations
- [Image Processing Library](../imgproc/README.md) - Low-level image operations
- [Eval Library](../eval/README.md) - Performance measurement framework
- [LSD Examples](../../examples/lsd/README.md) - Demonstration programs
- [Line Analyzer App](../../apps/line_analyzer/README.md) - Interactive analysis tool
- [Main README](../../README.md) - Project overview
