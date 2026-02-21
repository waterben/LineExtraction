# le_lsd — Python Bindings for Line Segment Detection Library

Python bindings for the `libs/lsd` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides a comprehensive collection of line segment detection
algorithms with a unified interface:

- **9 detector implementations** — CC, CP, Burns, FBW, FGioi, EDLines, EL, EP, HoughP
- **Unified API** — all detectors share `.detect()`, `.line_segments()`, `.lines()`
- **Runtime tuning** — parameter access via `ValueManager` interface
- **Auxiliary data** — gradient maps, edge maps, segment labels
- **Dual precision** — float (default) and double (`_f64` suffix)

Images are passed as NumPy arrays and automatically converted to/from `cv::Mat`.

> **Dependencies:** `le_lsd` imports `le_geometry`, `le_imgproc`, and `le_edge`
> at load time.  All three modules must be available.

> **Backward compatibility:** `le_lsd.Line` and `le_lsd.LineSegment` are
> re-exported from `le_geometry` — either import path works.

## Quick Start

```python
import numpy as np
import le_lsd

# Create a test image with a bright rectangle
img = np.zeros((100, 100), dtype=np.uint8)
img[30:70, 30:70] = 255

# Detect line segments
detector = le_lsd.LsdCC()
detector.detect(img)

# Access results
segments = detector.line_segments()  # list[LineSegment]
for seg in segments:
    sx, sy = seg.start_point()
    ex, ey = seg.end_point()
    print(f"({sx:.0f},{sy:.0f}) -> ({ex:.0f},{ey:.0f}), len={seg.length:.1f}")
```

## Available Detectors

| Class | Algorithm | Key Parameters |
|-------|-----------|----------------|
| `LsdCC` | Connected Components | `th_low`, `th_high`, `min_pix`, `max_gap`, `err_dist`, `flags` |
| `LsdCP` | Connected Pairs | (defaults) |
| `LsdBurns` | Burns gradient regions | `th_low`, `th_high`, `min_pix`, `part_num`, `flags` |
| `LsdFBW` | Fast region growing | `flags` |
| `LsdFGioi` | Grompone di Gioi (LSD/PLSD) | `quant`, `ang_th`, `log_eps`, `density_th`, `n_bins` |
| `LsdEDLZ` | EDLines | `gradient_threshold`, `anchor_threshold`, `min_line_len` |
| `LsdEL` | Edge Linking | `th_low`, `th_high`, `min_pix`, `dist`, `min_len` |
| `LsdEP` | Edge Pattern | `th_low`, `th_high`, `min_pix`, `dist`, `min_len` |
| `LsdHoughP` | Probabilistic Hough | `th_low`, `th_high`, `vote_threshold`, `min_length`, `max_gap` |

All detectors are available in double precision with a `_f64` suffix (e.g.,
`LsdCC_f64`).

## Unified Detector API

Every detector inherits from `LsdBase` and shares the same interface:

### Detection

```python
detector = le_lsd.LsdCC()
detector.detect(img)  # uint8 grayscale image
```

### Results

```python
# Detected line segments (most common)
segments = detector.line_segments()  # list[LineSegment]

# Infinite lines (without endpoint info)
lines = detector.lines()  # list[Line]

# Endpoint tuples
endpoints = detector.end_points()  # list of endpoint data
```

### Auxiliary Image Data

Each detector produces intermediate image data (gradient maps, edge maps,
segment label images, etc.) that can be inspected:

```python
# What data is available?
for entry in detector.image_data_descriptor():
    print(f"{entry.name}: {entry.description}")

# Get the data (list of numpy arrays)
data = detector.image_data()
for entry, mat in zip(detector.image_data_descriptor(), data):
    print(f"{entry.name}: shape={mat.shape}, dtype={mat.dtype}")
```

### Parameter Configuration

All detectors inherit from `ValueManager`:

```python
detector = le_lsd.LsdCC()

# List all parameters
for name, val in detector.values().items():
    print(f"{name} = {val}")

# Modify parameters
detector.set_int("cc_min_pixels", 20)
detector.set_double("cc_err_dist", 1.5)

# Read back
val = detector.get_value("cc_min_pixels")
print(val.get_int())
```

## Detector Examples

### LsdCC — Connected Components

The most versatile general-purpose detector:

```python
det = le_lsd.LsdCC()
det.detect(img)
segments = det.line_segments()

# With custom thresholds
det = le_lsd.LsdCC(th_low=10, th_high=30, min_pix=15, max_gap=3)
det.detect(img)
```

### LsdFGioi — Statistical Validation (à la Gioi)

Based on the well-known LSD algorithm with a contrario validation:

```python
det = le_lsd.LsdFGioi(
    quant=3.0,
    ang_th=30.0,
    log_eps=1.0,
    density_th=0.5,
    n_bins=512,
)
det.detect(img)
segments = det.line_segments()
```

### LsdEDLZ — EDLines

Fast anchor-based detection:

```python
det = le_lsd.LsdEDLZ(
    gradient_threshold=20,
    anchor_threshold=10,
    min_line_len=15,
)
det.detect(img)
```

### LsdBurns — Burns Gradient Regions

Classic gradient-based approach:

```python
det = le_lsd.LsdBurns(
    th_low=10, th_high=30,
    min_pix=10, part_num=5,
    flags=le_lsd.BURNS_NMS,
)
det.detect(img)
```

### LsdHoughP — Probabilistic Hough Transform

Wraps OpenCV's probabilistic Hough:

```python
det = le_lsd.LsdHoughP(
    th_low=50, th_high=150,
    vote_threshold=30,
    min_length=20,
    max_gap=5,
)
det.detect(img)
```

## Line / LineSegment

`le_lsd.Line` and `le_lsd.LineSegment` are re-exported from `le_geometry`.
Both import paths are equivalent:

```python
# These are the same type:
seg1 = le_lsd.LineSegment.from_endpoints(10, 20, 90, 80)
seg2 = le_geometry.LineSegment.from_endpoints(10, 20, 90, 80)

# Full Line/LineSegment API documented in le_geometry
```

See [le_geometry](../../geometry/python/README.md) for the complete
`Line` and `LineSegment` API reference.

## Flags & Constants

### CC Detector Flags

```python
le_lsd.CC_FIND_NEAR_COMPLEX  # 1 — find near complex structures
le_lsd.CC_CORNER_RULE         # 2 — apply corner detection rule
le_lsd.CC_ADD_THICK_PIXELS    # 4 — add thick pixel handling
```

### Burns Detector Flags

```python
le_lsd.BURNS_NMS  # 1 — enable non-maximum suppression
```

### FBW Detector Flags

```python
le_lsd.FBW_NMS    # 1 — enable non-maximum suppression
le_lsd.FBW_PATAN  # 2 — use pattern analysis
```

### Edge-Based Detector Flags

```python
le_lsd.EL_USE_NFA          # 1 — NFA-based validation (EL)
le_lsd.EL_USE_PRECISE_SPE  # 2 — precise sub-pixel estimation (EL)
le_lsd.EP_USE_PRECISE_SPE  # 2 — precise sub-pixel estimation (EP)
```

## Complete Pipeline Example

```python
"""Detect and visualize line segments on a test image."""
import numpy as np
import le_lsd
import le_geometry as geo

# Create a synthetic test image
img = np.zeros((200, 200), dtype=np.uint8)
img[40:160, 40:160] = 200           # bright square
img[80:120, 80:120] = 50            # dark inner square

# Detect with multiple algorithms
detectors = {
    "CC": le_lsd.LsdCC(),
    "FGioi": le_lsd.LsdFGioi(),
    "Burns": le_lsd.LsdBurns(),
    "EDLines": le_lsd.LsdEDLZ(),
}

for name, det in detectors.items():
    det.detect(img)
    segs = det.line_segments()
    print(f"{name}: {len(segs)} segments")

    # Draw results
    vis = np.stack([img, img, img], axis=-1)  # grayscale -> BGR
    result = geo.draw_lines_random(vis, segs)

# Inspect image data from detector
det = le_lsd.LsdCC()
det.detect(img)
desc = det.image_data_descriptor()
data = det.image_data()
for entry, mat in zip(desc, data):
    print(f"  {entry.name}: {mat.shape} ({mat.dtype})")
```

## Double Precision

All detectors and types are available in double precision:

```python
det = le_lsd.LsdCC_f64()
det.detect(img)
segments = det.line_segments()   # list[LineSegment_f64]

for seg in segments:
    print(seg.length)   # float64 precision
```

## Build & Test

```bash
# Build the module
bazel build //libs/lsd/python:le_lsd

# Run tests
bazel test //libs/lsd/python:test_le_lsd

# Run all LSD library tests (C++ and Python)
bazel test //libs/lsd/...
```

## Architecture

```
python/
├── src/
│   ├── lsd_binding.hpp        # Binding declarations
│   ├── lsd_binding.cpp        # Detector + core type bindings
│   └── module_le_lsd.cpp      # PYBIND11_MODULE entry
├── tests/
│   └── test_le_lsd.py         # Python integration tests
├── BUILD.bazel
└── README.md
```

### Module Dependency Graph

```
le_imgproc ──► le_edge ──► le_lsd ◄── le_geometry
(ValueManager)  (NMS,ESD)   (detectors)   (Line, LineSegment)
```

`le_lsd` imports all three modules at load time:

- `le_geometry` — provides `Line`/`LineSegment` types (re-exported)
- `le_imgproc` — provides `ValueManager` base class
- `le_edge` — provides edge types used internally by some detectors

## See Also

- [le_geometry](../../geometry/python/README.md) — Line/LineSegment API, drawing, optimizer
- [le_edge](../../edge/python/README.md) — edge detection pipeline
- [le_imgproc](../../imgproc/python/README.md) — gradient filters and core types
- [le_eval](../../eval/python/README.md) — performance benchmarking
- [LSD demo](../../../examples/lsd/python/lsd_demo.py) — runnable example script
