# le_edge — Python Bindings for Edge Detection Library

Python bindings for the `libs/edge` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides edge detection and edge segment extraction for grayscale
images.  It builds on top of `le_imgproc` gradient filters and adds:

- **Non-maximum suppression** (`NonMaximaSuppression`)
- **Edge source pipelines** (`EdgeSourceSobel`, `EdgeSourceScharr`, `EdgeSourcePrewitt`)
- **Edge segment detectors** (`EsdDrawing`, `EsdSimple`, `EsdLinking`, `EsdPattern`)
- **Segment primitives** (`EdgeSegment`)

Images are passed as NumPy arrays and automatically converted to/from `cv::Mat`.

> **Dependency:** `le_edge` imports `le_imgproc` at load time.  Install both
> modules to use edge detection.

## Quick Start

```python
import numpy as np
import le_edge

# Create a test image with a bright rectangle
img = np.zeros((100, 100), dtype=np.uint8)
img[30:70, 30:70] = 255

# All-in-one: edge source + segment detector
es = le_edge.EdgeSourceSobel()
es.process(img)

esd = le_edge.EsdDrawing(min_pixels=5)
esd.detect(es)

segments = esd.segments()  # list[EdgeSegment]
points = esd.points()      # list[int]  (linear pixel indices)

for seg in segments:
    seg_points = points[seg.begin : seg.end]
    for idx in seg_points:
        row, col = divmod(idx, img.shape[1])
        print(f"  ({row}, {col})")
```

## Edge Source Pipeline

An `EdgeSource` computes gradients, magnitude, direction, non-maximum
suppression, and hysteresis in a single `.process()` call:

```python
es = le_edge.EdgeSourceSobel()
es.process(img)

# Low-level results
mag   = es.magnitude()        # float32 ndarray
gx    = es.gx()               # int16 ndarray
gy    = es.gy()                # int16 ndarray
direc = es.direction()         # float32 ndarray
dmap  = es.direction_map()     # int8 ndarray

# Hysteresis thresholding
binary = es.hysteresis_binary()  # uint8 edge map (0/255)
edgels = es.hysteresis_edgels()  # list[int] edge pixel indices
seeds  = es.seeds()              # list[int] strong-edge seeds
```

### Available Edge Sources

| Class | Gradient kernel | Presets |
|-------|-----------------|---------|
| `EdgeSourceSobel` | Sobel (3×3, 5×5, 7×7) | `_16u`, `_f32`, `_f64` |
| `EdgeSourceScharr` | Scharr (3×3, higher accuracy) | `_16u`, `_f32`, `_f64` |
| `EdgeSourcePrewitt` | Prewitt (3×3) | `_16u`, `_f32`, `_f64` |

All edge sources inherit from `ValueManager` (see `le_imgproc`) for runtime
parameter tuning.

## Non-Maximum Suppression

Use `NonMaximaSuppression` directly when you already have gradient data:

```python
nms = le_edge.NonMaximaSuppression(th_low=20.0, th_high=50.0)
nms.process(gx, gy, mag, low_threshold=20.0, high_threshold=50.0)

dmap  = nms.direction_map()   # int8 direction labels
seeds = nms.seeds()           # strong-edge seed indices
```

| Class | Description | Presets |
|-------|-------------|---------|
| `NonMaximaSuppression` | Standard NMS (float mag) | `_16u` (`_f32` alias), `_f64` |

## Edge Segment Detectors

Edge segment detectors trace connected edge pixels into ordered contours:

```python
# Provide an edge source …
esd = le_edge.EsdDrawing(min_pixels=10)
esd.detect(es)

# … or supply raw data
esd.detect(direction_map, magnitude, seeds)

segments = esd.segments()  # list[EdgeSegment]
points   = esd.points()    # list[int]
```

### Available Detectors

| Class | Strategy | Key Parameters | Presets |
|-------|----------|---------------|---------|
| `EsdDrawing` | Greedy edge drawing | `min_pixels`, `mag_mul`, `mag_th` | `_f64` |
| `EsdSimple` | Simple linking | `min_pixels` | `_f64` |
| `EsdLinking` | Continuity-optimised linking | `min_pixels`, `max_gap`, `mag_mul`, `mag_th` | `_f64` |
| `EsdPattern` | Pattern-based linking | `min_pixels`, `max_gap`, `mag_mul`, `mag_th`, `pat_tol` | `_f64` |

### EdgeSegment

An `EdgeSegment` stores a range `[begin, end)` into the shared point index
list, plus metadata:

```python
seg = le_edge.EdgeSegment(begin=0, end=42, flags=0, id=1)

seg.begin    # start index in points list
seg.end      # end index (exclusive)
seg.flags    # bitmask: ES_REVERSE, ES_CLOSED
seg.id       # unique segment identifier
seg.size()   # number of pixels (end - begin)
seg.closed() # True if ES_CLOSED flag set
```

### Flags & Enums

```python
# Segment flags
le_edge.ES_NONE       # 0 — no flags
le_edge.ES_REVERSE    # 1 — segment traced in reverse
le_edge.ES_CLOSED     # 2 — segment forms a closed contour

# Direction options
le_edge.ESDirectionOptions.NONE   # no direction data
le_edge.ESDirectionOptions.GXGY   # gx/gy channels
le_edge.ESDirectionOptions.DIR    # single-channel direction

# Quadrature options
le_edge.ESQuadratureOptions.MAG    # gradient magnitude
le_edge.ESQuadratureOptions.ENERGY # energy
le_edge.ESQuadratureOptions.PC     # phase congruency
```

## Complete Pipeline Example

```python
import numpy as np
import le_edge

# Load or create a test image
img = np.zeros((200, 200), dtype=np.uint8)
img[50:150, 50:150] = 200
img[80:120, 80:120] = 100

# Detect edges
es = le_edge.EdgeSourceSobel()
es.process(img)

# Extract edge segments
esd = le_edge.EsdPattern(min_pixels=10, max_gap=3)
esd.detect(es)

segments = esd.segments()
points = esd.points()
print(f"Found {len(segments)} edge segments")

# Create edge map visualisation
edge_img = np.zeros_like(img)
for seg in segments:
    for idx in points[seg.begin : seg.end]:
        row, col = divmod(idx, img.shape[1])
        edge_img[row, col] = 255
```

## Parameter Configuration

All detectors and edge sources inherit from `ValueManager`:

```python
es = le_edge.EdgeSourceSobel()
params = es.values()             # dict {name: Value}
es.set_int("grad_kernel_size", 5)

esd = le_edge.EsdDrawing()
esd_params = esd.values()
```

## Build & Test

```bash
# Build the module
bazel build //libs/edge/python:le_edge

# Run tests
bazel test //libs/edge/python:test_le_edge

# Run all edge library tests (C++ and Python)
bazel test //libs/edge/...
```

## Architecture

```
python/
├── src/
│   ├── edge_binding.hpp      # Binding declarations
│   ├── edge_binding.cpp      # Type/interface bindings
│   └── module_le_edge.cpp    # PYBIND11_MODULE entry
├── tests/
│   └── test_le_edge.py       # Python integration tests
├── BUILD.bazel
└── README.md
```

### Module Dependency Graph

```
le_imgproc  ──►  le_edge
(ValueManager,     (EdgeSource, NMS,
 GradientI, …)      ESD detectors, …)
```

`le_edge` imports `le_imgproc` at load time so that `ValueManager` and
gradient types are available as base classes.

## See Also

- [le_imgproc](../../imgproc/python/README.md) — gradient filters and core types
- [le_lsd](../../lsd/python/README.md) — line segment detection (uses edge detection)
- [le_geometry](../../geometry/python/README.md) — geometric primitives and drawing
- [Edge demo](../../../examples/edge/python/edge_demo.py) — runnable example script
