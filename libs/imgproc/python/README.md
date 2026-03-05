# le_imgproc - Python Bindings for Image Processing Library

Python bindings for the `libs/imgproc` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module exposes the image filter interface hierarchy to Python:

- **Core types**: `Range`, `Value`, `ValueManager`, `FilterData`
- **Abstract interfaces**: `FilterI`, `GradientI`, `LaplaceI`
- **Gradient filters**: `SobelGradient`, `ScharrGradient`, `PrewittGradient`
- **Laplacian filters**: `LaplaceSimple`, `LoG`, `LaplaceCV`
- **Extra gradients**: `SusanGradient`, `RCMGradientColor`, `RCMGradient`
- **Image operators**: `NoOp`, `GaussianBlurOperator`, `MedianBlurOperator`, `BlurOperator`, `BilateralOperator`, `FastNlMeansOperator`, `GaussianNoiseOperator`, `UniformNoiseOperator`, `ResizeOperator`, `RotateOperator`, `ScaleOperator`, `TranslateOperator`, `PipelineOperator`
- **Quadrature filters**: `QuadratureG2`, `QuadratureLGF`, `QuadratureS`, `QuadratureSF`

Images are passed as numpy arrays and automatically converted to/from `cv::Mat`.

## Quick Start

```python
import numpy as np
import le_imgproc

# Create a test image with a vertical edge
img = np.zeros((100, 100), dtype=np.uint8)
img[:, 50:] = 255

# Compute Sobel gradient
grad = le_imgproc.SobelGradient()
grad.process(img)

# Access results
magnitude = grad.magnitude()   # numpy array (int32)
direction = grad.direction()   # numpy array (float32)
gx = grad.gx()                 # X-gradient (int16)
gy = grad.gy()                 # Y-gradient (int16)

# All results as dict
results = grad.results()       # {"gx": FilterData, "gy": ..., "mag": ..., "dir": ...}
```

## Available Filters

### Gradient Operators

| Class | Description | Parameters |
|-------|-------------|------------|
| `SobelGradient` | Sobel derivative gradient (3x3, 5x5, 7x7) | `grad_kernel_size` |
| `ScharrGradient` | Scharr derivative gradient (3x3, more accurate) | — |
| `PrewittGradient` | Prewitt derivative gradient (3x3) | — |
| `SusanGradient` | SUSAN edge/corner gradient | `grad_kernel_size`, `grad_susan_t` |
| `RCMGradientColor` | Robust Color Morphological Gradient (3-channel) | `grad_rcmg_mask_size`, `grad_rcmg_rejection`, `grad_rcmg_colnorm` |
| `RCMGradient` | RCMG + derivative direction (1-channel) | Same as `RCMGradientColor` + derivative params |

### Laplacian Operators

| Class | Description | Parameters |
|-------|-------------|------------|
| `LaplaceSimple` | Simple 3×3 Laplacian convolution | — |
| `LoG` | Laplacian of Gaussian | `grad_kernel_size`, `grad_kernel_spacing`, `grad_kernel_scale` |
| `LaplaceCV` | OpenCV `cv::Laplacian` wrapper | `grad_kernel_size` |

### Quadrature Filters

| Class | Description | Parameters |
|-------|-------------|------------|
| `QuadratureG2` | Steerable G2/H2 separable filter pair | `grad_kernel_size`, `grad_kernel_spacing` |
| `QuadratureLGF` | Log-Gabor filter (frequency domain) | `grad_waveLength`, `grad_sigmaOnf` |
| `QuadratureS` | Difference of Poisson (spatial domain) | `grad_kernel_size`, `grad_kernel_spacing`, `grad_kernel_scale`, `grad_scale`, `grad_muls` |
| `QuadratureSF` | Difference of Poisson (frequency domain) | `grad_kernel_spacing`, `grad_scale`, `grad_muls` |

### Image Operators

| Class | Description | Parameters |
|-------|-------------|------------|
| `ImageOperator` | Abstract base class for operators | — |
| `NoOp` | Identity (pass-through) operator | — |
| `GaussianBlurOperator` | Gaussian blur | `op_sigma`, `op_kernel_size` |
| `MedianBlurOperator` | Median blur | `op_kernel_size` |
| `BlurOperator` | Simple box blur | `op_kernel_size` |
| `BilateralOperator` | Bilateral edge-preserving filter | `op_d`, `op_sigma_color`, `op_sigma_space` |
| `FastNlMeansOperator` | Non-local means denoising | `op_h`, `op_template_size`, `op_search_size` |
| `GaussianNoiseOperator` | Add Gaussian noise | `op_mean`, `op_stddev` |
| `UniformNoiseOperator` | Add uniform random noise | `op_lower`, `op_upper` |
| `ResizeOperator` | Resize image | `op_width`, `op_height` |
| `RotateOperator` | Rotate image around pivot | `op_angle`, `op_pivot_x`, `op_pivot_y` |
| `ScaleOperator` | Scale image around pivot | `op_scale`, `op_pivot_x`, `op_pivot_y` |
| `TranslateOperator` | Translate (shift) image | `op_tx`, `op_ty` |
| `PipelineOperator` | Chain multiple operators sequentially | — |

## Type Presets

Most filter classes are available in multiple type presets via name suffixes:

| Suffix | Input Type | Float Type | Example |
|--------|-----------|------------|---------|
| *(none)* | `uint8` | `float32` | `QuadratureG2()` |
| `_f32` | `float32` | `float32` | `QuadratureG2_f32(int_lower=0.0, int_upper=1.0)` |
| `_f64` | `float64` | `float64` | `QuadratureG2_f64(int_lower=0.0, int_upper=1.0)` |

Image operators additionally have a `_16u` preset for 16-bit unsigned input.

## Quadrature Filter Usage

```python
import le_imgproc
import numpy as np

img = np.zeros((64, 64), dtype=np.uint8)
img[:, 32:] = 255

# G2: steerable Gaussian
g2 = le_imgproc.QuadratureG2(kernel_size=9)
g2.process(img)
even = g2.even()          # Symmetric (line) response
odd = g2.odd()            # Antisymmetric (edge) magnitude
ox, oy = g2.odd_xy()      # X/Y odd components
energy = g2.energy()      # Local energy
phase = g2.phase()        # Local phase
direction = g2.direction()
laplace = g2.laplace()    # Alias for even()

# Steering (G2 only)
g2_steered, h2_steered = g2.steer(direction)
g2_xy, h2_xy = g2.steer_xy(ox, oy)

# Log-Gabor (frequency domain)
lgf = le_imgproc.QuadratureLGF(wave_length=5.0, sigma_onf=0.55)
lgf.process(img)

# Thresholds and ranges
et = lgf.even_threshold(0.5)   # Threshold at 50% of range
er = lgf.energy_range()        # Range object with .lower, .upper
```

## Parameter Configuration

Filters inherit from `ValueManager` for runtime parameter tuning:

```python
grad = le_imgproc.SobelGradient()

# List all parameters
params = grad.values()  # dict {name: Value}

# Get/set specific parameter
grad.set_int("grad_kernel_size", 5)
val = grad.get_value("grad_kernel_size")
print(val.get_int())  # 5
```

## Build & Test

```bash
# Build the Python module
bazel build //libs/imgproc/python:le_imgproc

# Run C++ binding tests
bazel test //libs/imgproc/python:test_filter_binding

# Run Python tests
bazel test //libs/imgproc/python:test_le_imgproc

# Run all tests
bazel test //libs/imgproc/python/...
```

## Architecture

```
python/
├── src/
│   ├── ndarray_converter.hpp        # cv::Mat <-> numpy type caster
│   ├── filter_binding.hpp           # Binding function declarations
│   ├── filter_binding.cpp           # Core types + gradient bindings
│   ├── laplace_binding.cpp          # Laplacian filter bindings
│   ├── gradient_extra_binding.cpp   # SUSAN, RCMG gradient bindings
│   ├── image_operator_binding.cpp   # Image operator bindings
│   ├── quadrature_binding.cpp       # Quadrature filter bindings
│   └── module_le_imgproc.cpp        # PYBIND11_MODULE entry point
├── tests/
│   ├── test_filter_binding.cpp      # C++ unit tests
│   └── test_le_imgproc.py           # Python integration tests
├── BUILD.bazel
└── README.md
```

### Binding Layer Design

- **`pybind_library`** (`imgproc_python_pybind`): Reusable C++ binding code linked by other modules
- **`le_pybind_module`** (`le_imgproc`): The actual Python extension (`.so`)
- **Trampoline classes**: Enable Python subclassing of `FilterI`, `GradientI`, `LaplaceI`
- **Zero-copy conversion**: Uses `cvnp` for bidirectional `cv::Mat` ↔ numpy conversion
- **Preset templates**: Each concrete class is instantiated for multiple type combinations

## See Also

- [le_edge](../../edge/python/README.md) — edge detection (uses imgproc gradients)
- [le_geometry](../../geometry/python/README.md) — geometric primitives and drawing
- [le_lsd](../../lsd/python/README.md) — line segment detection
- [le_eval](../../eval/python/README.md) — performance benchmarking
- [Filter demo](../../../examples/imgproc/python/filter_demo.py) — runnable example script
