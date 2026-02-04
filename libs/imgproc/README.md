# Image Processing Library

Low-level image processing algorithms for edge and line detection, including gradient computation, filtering, quadrature analysis, and phase congruency.

## Overview

The `imgproc` library provides foundational image processing components:

- **Gradient & Derivative**: Various derivative operators (Sobel, Scharr, Prewitt, Gaussian)
- **Magnitude & Direction**: Gradient magnitude and direction computation
- **Interpolation**: Sub-pixel interpolation (nearest, bilinear, bicubic, Lanczos)
- **Laplacian**: Laplacian of Gaussian and simple Laplacian operators
- **Quadrature Filters**: Local energy computation with even/odd filter pairs
- **Phase Congruency**: Multi-scale edge detection invariant to contrast
- **Special Filters**: SUSAN, RCMG color gradient, steerable filters

## Components

### Derivative Operators

Compute image gradients using various convolution kernels.

```cpp
#include <imgproc/derivative.hpp>

cv::Mat image = cv::imread("image.png", cv::IMREAD_GRAYSCALE);
cv::Mat gx, gy;

// Sobel derivative (configurable kernel size)
lsfm::SobelDerivative<uchar, short> sobel(3);
sobel.process(image, gx, gy);

// Scharr derivative (optimized 3x3)
lsfm::ScharrDerivative<uchar, short> scharr;
scharr.process(image, gx, gy);

// Gaussian derivative (scale-space)
lsfm::GaussianDerivative<uchar, short> gauss(5, 3.0, 1.0);
gauss.process(image, gx, gy);

// Get max response for normalization
auto dm = sobel.max();  // DerivativeMax{max_1st, max_2nd, max_3rd}
```

### Gradient Interface

Unified interface for gradient computation with magnitude and direction.

```cpp
#include <imgproc/gradient.hpp>
#include <imgproc/derivative_gradient.hpp>

// Derivative-based gradient
lsfm::DerivativeGradient<uchar, short, int, float,
    lsfm::SobelDerivative,
    lsfm::QuadraticMagnitude,
    lsfm::Direction> gradient;

gradient.process(image);
cv::Mat mag = gradient.magnitude();     // Gradient magnitude
cv::Mat dir = gradient.direction();     // Gradient direction (-PI to PI)
cv::Mat gx = gradient.gx();             // X derivative
cv::Mat gy = gradient.gy();             // Y derivative
```

### Magnitude Operators

Different norms for computing gradient magnitude.

```cpp
#include <imgproc/magnitude.hpp>

// Euclidean magnitude: sqrt(gx² + gy²)
lsfm::Magnitude<short, float>::process(gx, gy, mag);

// Quadratic magnitude: gx² + gy² (faster, no sqrt)
lsfm::QuadraticMagnitude<short, int>::process(gx, gy, qmag);

// Absolute magnitude: |gx| + |gy| (L1 norm)
lsfm::AbsoluteMagnitude<short, int>::process(gx, gy, amag);
```

### Direction Computation

Compute gradient direction from derivatives.

```cpp
#include <imgproc/direction.hpp>

cv::Mat dir;

// Precise direction: atan2 (-PI to PI)
lsfm::Direction<short, float>::process(gx, gy, dir);

// Fast direction: cv::fastAtan2 (0 to 360 degrees)
lsfm::FastDirection<short, float>::process(gx, gy, dir);
```

### Interpolation

Sub-pixel value interpolation with various methods.

```cpp
#include <imgproc/interpolate.hpp>

// Nearest neighbor
float val = lsfm::NearestInterpolator<float, uchar>::get(img, 10.5f, 20.3f);

// Bilinear interpolation
float val = lsfm::LinearInterpolator<float, uchar>::get(img, 10.5f, 20.3f);

// Bicubic interpolation
float val = lsfm::CubicInterpolator<float, uchar>::get(img, 10.5f, 20.3f);

// Lanczos interpolation
float val = lsfm::Lanczos3Interpolator<float, uchar>::get(img, 10.5f, 20.3f);

// With border handling
float val = lsfm::LinearInterpolator<float, uchar>::get(
    img, x, y, cv::BORDER_REFLECT_101);
```

### Quadrature Filters

Compute local energy using even/odd filter pairs (Hilbert transform pairs).

```cpp
#include <imgproc/quadratureLGF.hpp>

// Log-Gabor quadrature filter
lsfm::QuadratureLGF<uchar, float, float, float, float> quad(
    0, 255,     // intensity range
    3,          // number of scales
    0.65,       // sigma on frequency
    0.55,       // mult factor
    2.0         // min wavelength
);

quad.process(image);
cv::Mat even = quad.even();         // Even (symmetric) response
cv::Mat odd = quad.odd();           // Odd (antisymmetric) magnitude
cv::Mat energy = quad.energy();     // Local energy
cv::Mat phase = quad.phase();       // Local phase
cv::Mat dir = quad.direction();     // Orientation
```

### Phase Congruency

Multi-scale edge detection based on phase alignment.

```cpp
#include <imgproc/pc_lgf.hpp>

// Log-Gabor based phase congruency
lsfm::PCLGFMono<uchar, float, float, float, float> pc(
    0, 255,     // intensity range
    4,          // scales
    0.65,       // sigmaOnF
    0.55,       // mult
    3.0         // minWaveLength
);

pc.process(image);
cv::Mat pcMap = pc.phaseCongruency();  // Phase congruency [0,1]
cv::Mat energy = pc.energy();           // Local energy
cv::Mat phase = pc.phase();             // Local phase
```

### Laplacian Operators

Second-order derivative filters.

```cpp
#include <imgproc/laplace.hpp>

// Simple 3x3 Laplacian
lsfm::LaplaceSimple<uchar, short> laplace;
laplace.process(image);
cv::Mat lap = laplace.laplace();

// Laplacian of Gaussian (LoG)
lsfm::LaplaceGaussian<uchar, float> log(5, 3.0);
log.process(image);
```

### RCMG Color Gradient

Robust Color Morphological Gradient for multi-channel images.

```cpp
#include <imgproc/rcmg.hpp>

cv::Mat colorImage = cv::imread("image.png");

// 3-channel color gradient
lsfm::RCMGradient<uchar, 3, short, int, float> rcmg(
    3,              // mask size
    0,              // rejection
    cv::NORM_L2SQR  // color norm
);

rcmg.process(colorImage);
cv::Mat mag = rcmg.magnitude();
cv::Mat dir = rcmg.direction();
```

### Image Pyramid

Multi-scale image representation.

```cpp
#include <imgproc/pyramid.hpp>

// Create pyramid with automatic scale count
lsfm::Pyramid<float> pyr(image, 0);  // 0 = auto

// Access pyramid levels
for (size_t i = 0; i < pyr.size(); ++i) {
    cv::Mat level = pyr[i];
}

// Create pyramid with fixed scale count
lsfm::Pyramid<float> pyr4(image, 4);
```

### Gaussian Kernels

Generate Gaussian kernels and derivatives.

```cpp
#include <imgproc/gaussian.hpp>

// Gaussian kernel (size=5, range=3)
cv::Mat_<float> g = lsfm::gaussian<float>(5, 3.0f);

// First derivative of Gaussian
cv::Mat_<float> g1 = lsfm::gaussianD1<float>(5, 3.0f);

// Second derivative of Gaussian (LoG)
cv::Mat_<float> g2 = lsfm::gaussianD2<float>(5, 3.0f);
```

## Headers

### Core

| Header | Description |
|--------|-------------|
| `filter.hpp` | Base filter interface (`FilterI`, `FilterData`) |
| `gradient.hpp` | Gradient interface (`GradientI`) |
| `derivative.hpp` | Derivative operators (Sobel, Scharr, Prewitt, Gaussian) |
| `derivative_gradient.hpp` | Derivative-based gradient implementation |
| `magnitude.hpp` | Magnitude operators (L1, L2, L2²) |
| `direction.hpp` | Direction computation (precise, fast) |
| `laplace.hpp` | Laplacian operators |

### Interpolation

| Header | Description |
|--------|-------------|
| `interpolate.hpp` | Sub-pixel interpolation (nearest, linear, cubic, Lanczos) |

### Quadrature & Phase Congruency

| Header | Description |
|--------|-------------|
| `quadrature.hpp` | Quadrature filter interface |
| `quadratureLGF.hpp` | Log-Gabor quadrature filter |
| `quadratureG2.hpp` | Second-order Gaussian quadrature |
| `quadratureS.hpp` | Steerable quadrature filter |
| `quadratureSF.hpp` | Steerable Freeman-Adelson filter |
| `phase_congruency.hpp` | Phase congruency interface |
| `pc_lgf.hpp` | Log-Gabor phase congruency |
| `pc_sqf.hpp` | Steerable phase congruency |
| `pc_matlab.hpp` | MATLAB-compatible phase congruency |

### Utilities

| Header | Description |
|--------|-------------|
| `gaussian.hpp` | Gaussian kernel generation |
| `polar.hpp` | Polar coordinate conversion |
| `mean.hpp` | Mean filter operators |
| `pyramid.hpp` | Multi-scale image pyramid |

### Specialized

| Header | Description |
|--------|-------------|
| `rcmg.hpp` | Robust Color Morphological Gradient |
| `susan.hpp` | SUSAN edge/corner detector |
| `steerable.hpp` | Steerable filter basis functions |
| `image_operator.hpp` | Generic image operators |
| `gradient_adapter.hpp` | Gradient adapter for other filters |

## Type Parameters

Most classes are templated for flexibility:

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `IT` | Input image type | `uchar`, `short`, `float` |
| `GT` | Gradient/derivative type | `short`, `float`, `double` |
| `MT` | Magnitude type | `int`, `float`, `double` |
| `DT` | Direction/phase type | `float`, `double` |
| `ET` | Energy type | `float`, `double` |

**Recommendations:**

- 8-bit images: `IT=uchar`, `GT=short`, `MT=int`
- 16-bit images: `IT=short`, `GT=float`, `MT=float`
- Float images: Use same type throughout

## Build

```bash
# Bazel
bazel build //libs/imgproc:lib_imgproc
bazel test //libs/imgproc:test_imgproc

# CMake
cmake --build build --target lib_imgproc
```

## Dependencies

- **OpenCV**: core, imgproc modules
- **geometry**: Point types
- **utility**: Range, ValueManager

## See Also

- [edge](../edge/README.md) - Edge detection using gradient operators
- [lsd](../lsd/README.md) - Line segment detection
- [geometry](../geometry/README.md) - Geometric primitives
- [utility](../utility/README.md) - Core utilities
- [Image Processing Examples](../../examples/imgproc/README.md) - Usage examples
- [Main README](../../README.md) - Project overview
