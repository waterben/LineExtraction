# le_imgproc - Python Bindings for Image Processing Library

Python bindings for the `libs/imgproc` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module exposes the image filter interface hierarchy to Python:

- **Core types**: `Range`, `Value`, `ValueManager`, `FilterData`
- **Abstract interfaces**: `FilterI`, `GradientI`, `LaplaceI`
- **Concrete filters**: `SobelGradient`, `ScharrGradient`, `PrewittGradient`

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

| Class | Description | Parameters |
|-------|-------------|------------|
| `SobelGradient` | Sobel derivative gradient (3x3, 5x5, 7x7) | `grad_kernel_size` |
| `ScharrGradient` | Scharr derivative gradient (3x3, more accurate) | - |
| `PrewittGradient` | Prewitt derivative gradient (3x3) | - |

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
│   ├── ndarray_converter.hpp    # cv::Mat <-> numpy type caster
│   ├── filter_binding.hpp       # Binding function declarations
│   ├── filter_binding.cpp       # Core type + interface bindings
│   └── module_le_imgproc.cpp    # PYBIND11_MODULE entry point
├── tests/
│   ├── test_filter_binding.cpp  # C++ unit tests
│   └── test_le_imgproc.py       # Python integration tests
├── BUILD.bazel
└── README.md
```

### Binding Layer Design

- **`pybind_library`** (`imgproc_python_pybind`): Reusable C++ binding code that can be linked by other modules
- **`le_pybind_module`** (`le_imgproc`): The actual Python extension (`.so`)
- **Trampoline classes**: Enable Python subclassing of `FilterI`, `GradientI`, `LaplaceI`
- **`ndarray_converter.hpp`**: Zero-copy (when possible) bidirectional `cv::Mat` ↔ numpy conversion

## Extending

To add bindings for a new filter (e.g., `LaplaceSimple`):

1. Add a `bind_*` function in `filter_binding.hpp`/`.cpp`
2. Call it from `module_le_imgproc.cpp`
3. Add tests in both `test_filter_binding.cpp` and `test_le_imgproc.py`
