# le_lfd — Python Bindings for Line Feature Descriptor Library

Python bindings for the `libs/lfd` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides a complete line feature matching pipeline:

- **Descriptors** — LBD (Line Band Descriptor) and LR (Left-Right) descriptors
- **Creators** — `FdcLBD` and `FdcGenericLR` for computing descriptors from gradient images
- **Matchers** — `BruteForceLBD` and `BruteForceLR` with k-NN, radius, and best-match queries
- **Filters** — `GlobalRotationFilter` and `StereoLineFilter` for outlier rejection
- **Match types** — `FeatureMatch` and `DescriptorMatch` with distance metrics
- **Dual precision** — float (default) and double (`_f64` suffix)

Images and gradient matrices are passed as NumPy arrays with zero-copy conversion.

> **Dependencies:** `le_lfd` imports `le_geometry`, `le_imgproc`, `le_edge`, and
> `le_lsd` at load time. All four modules must be available.

> **Backward compatibility:** `le_lfd.Line` and `le_lfd.LineSegment` are
> re-exported from `le_geometry` — either import path works.

## Quick Start

```python
import numpy as np
import cv2
import le_lsd
import le_lfd

# Load a grayscale image
img = cv2.imread("image.jpg", cv2.IMREAD_GRAYSCALE)

# 1. Detect line segments
det = le_lsd.LsdCC()
det.detect(img)
segments = det.line_segments()

# 2. Compute gradient images
img_f = img.astype(np.float32) / 255.0
gx = cv2.Sobel(img_f, cv2.CV_32F, 1, 0, ksize=3)
gy = cv2.Sobel(img_f, cv2.CV_32F, 0, 1, ksize=3)

# 3. Create LBD descriptors
creator = le_lfd.FdcLBD(gx, gy)
descriptors = creator.create_list(segments)
print(f"Descriptor dimension: {creator.size()}")

# 4. Match against another image's descriptors
matcher = le_lfd.BruteForceLBD()
matcher.train(desc_query, desc_target)
matches = matcher.best()

for m in sorted(matches, key=lambda m: m.distance)[:10]:
    print(f"  query={m.query_idx} -> match={m.match_idx}, dist={m.distance:.3f}")
```

## Available Types

### Match Types

| Class | Description |
|-------|-------------|
| `FeatureMatch` | Base match pair with `query_idx`, `match_idx`, `filter_state` |
| `DescriptorMatch` | Extends `FeatureMatch` with `distance` metric |

### Descriptor Types

| Class | Description |
|-------|-------------|
| `FdMat` | Generic cv::Mat-based descriptor with L2 distance |
| `FdLBD` | LBD (Line Band Descriptor) — SIFT-like bands along line segments |
| `LRDescriptor` | Left-Right symmetric descriptor with L*R distance metric |

### Descriptor Creators

| Class | Input Data | Output |
|-------|-----------|--------|
| `FdcLBD` | Gradient images (`gx`, `gy`) | `FdLBD` descriptors |
| `FdcGenericLR` | Gradient + intensity (`gx`, `gy`, `img`) | `LRDescriptor` descriptors |

### Matchers

| Class | Descriptor Type | Features |
|-------|----------------|----------|
| `BruteForceLBD` | `FdLBD` | best, k-NN, radius, filter-based training |
| `BruteForceLR` | `LRDescriptor` | best, k-NN, radius |

### Filters

| Class | Strategy | Use Case |
|-------|----------|----------|
| `GlobalRotationFilter` | Angle/length histogram consensus | General image matching |
| `StereoLineFilter` | Epipolar + Y-overlap constraints | Stereo pair matching |

All types are available in double precision with a `_f64` suffix (e.g.,
`FdcLBD_f64`, `BruteForceLBD_f64`).

## Descriptor Creation

### LBD Descriptors

LBD computes SIFT-like band descriptors along each line segment using
gradient projection with Gaussian weighting:

```python
# From explicit gradient matrices
creator = le_lfd.FdcLBD(gx, gy, num_band=9, width_band=7)

# From a dict (alternative)
creator = le_lfd.FdcLBD({"gx": gx, "gy": gy})

# Create descriptors
desc_list = creator.create_list(segments)       # list[FdLBD]
desc_mat  = creator.create_mat(segments)        # np.ndarray (N, D)
desc_one  = creator.create_single(segments[0])  # FdLBD

# With mask (only compute for selected segments)
mask = [1, 0, 1, 1, 0]  # 0 = excluded
desc_masked = creator.create_list_masked(segments, mask)

# Descriptor dimension
print(creator.size())  # e.g. 48

# Update gradient data for a new image
creator.set_data({"gx": new_gx, "gy": new_gy})
```

### LR Descriptors

LR (Left-Right) descriptors sample gradient and intensity features on both
sides of a line segment:

```python
img_f = img.astype(np.float32) / 255.0
data = {"gx": gx, "gy": gy, "img": img_f}

creator = le_lfd.FdcGenericLR(data, pos=-1.0, step_dir=1.0, lstep=1.0)

desc_list = creator.create_list(segments)   # list[LRDescriptor]
desc_mat  = creator.create_mat(segments)    # np.ndarray (N, D)
print(creator.size())  # e.g. 40
```

### Accessing Descriptor Data

```python
# FdLBD — data is a NumPy array (cv::Mat)
lbd = desc_list[0]
print(lbd.data.shape)  # (1, D) float32

# LRDescriptor — data is a 1D NumPy view (zero-copy)
lr = lr_desc_list[0]
print(lr.data.shape)  # (40,) float32

# Compute distance between two descriptors
dist = lbd.distance(other_lbd)
lr_dist = lr.distance(other_lr)
```

## Matching

### Brute-Force Matcher

All matchers follow the same pattern: `train()` builds the distance graph,
then `best()` / `knn()` / `radius()` extract matches:

```python
matcher = le_lfd.BruteForceLBD()

# Train: compute all pairwise distances
matcher.train(query_descriptors, target_descriptors)

# Get single best match per query
best_matches = matcher.best()

# Get k nearest neighbors
knn_matches = matcher.knn(k=3)

# Get all matches within radius
radius_matches = matcher.radius(0.5)

# One-shot best match (no state stored)
best = le_lfd.BruteForceLBD.match_best(query, target)
```

### Training with Masks

```python
# Only match selected descriptors (0 = excluded)
query_mask = [1, 1, 0, 1, 0]
match_mask = [0, 1, 1, 1, 1]
matcher.train_masked(query, target, query_mask, match_mask)
```

### Training with Filters

```python
# Apply GlobalRotationFilter during matching
grf = le_lfd.GlobalRotationFilter()
grf.train(segments_query, segments_target)
matcher.train_filtered(query_desc, target_desc, grf)
matches = matcher.best()

# Apply StereoLineFilter for stereo pairs
slf = le_lfd.StereoLineFilter(height=480, angle_th=30.0, min_y_overlap=0.6)
slf.train(segments_left, segments_right)
matcher.train_filtered_stereo(query_desc, target_desc, slf)
```

## Filtering

### GlobalRotationFilter

Estimates the dominant rotation between two line sets and rejects
inconsistent matches:

```python
grf = le_lfd.GlobalRotationFilter()
grf.train(segments_ref, segments_target)

# Check individual pair
should_reject = grf.filter(left_idx=0, right_idx=5)

# Generate all non-filtered candidates
candidates = grf.create_matches(len(seg_ref), len(seg_tgt))

# Generate candidates with masks
candidates, left_mask, right_mask = grf.create_matches_with_masks(
    len(seg_ref), len(seg_tgt)
)

# Apply filter to existing match list
filtered = grf.filter_list(matches)
```

### StereoLineFilter

Enforces geometric constraints for stereo image pairs:

```python
slf = le_lfd.StereoLineFilter(
    height=480,           # Image height
    max_dis_px=10000,     # Max endpoint distance
    angle_th=45.0,        # Max angle difference (degrees)
    min_y_overlap=0.5,    # Min Y overlap ratio (0-1)
)
slf.train(segments_left, segments_right)

# Same API as GlobalRotationFilter
should_reject = slf.filter(left_idx=0, right_idx=3)
candidates = slf.create_matches(len(seg_left), len(seg_right))
```

## Constants

```python
le_lfd.FS_NONE    # 0 — match not filtered
le_lfd.FS_MASKED  # 1 — match marked as filtered/rejected
```

## Complete Pipeline Example

```python
"""Detect, describe, match, and filter line segments between two images."""
import numpy as np
import cv2
import le_lsd
import le_lfd

# Load image pair
img1 = cv2.imread("image1.jpg", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread("image2.jpg", cv2.IMREAD_GRAYSCALE)

# Detect line segments
det1, det2 = le_lsd.LsdCC(), le_lsd.LsdCC()
det1.detect(img1)
det2.detect(img2)
seg1, seg2 = det1.line_segments(), det2.line_segments()

# Compute gradients
def gradients(img):
    f = img.astype(np.float32) / 255.0
    return cv2.Sobel(f, cv2.CV_32F, 1, 0, ksize=3), \
           cv2.Sobel(f, cv2.CV_32F, 0, 1, ksize=3)

gx1, gy1 = gradients(img1)
gx2, gy2 = gradients(img2)

# Create LBD descriptors
desc1 = le_lfd.FdcLBD(gx1, gy1).create_list(seg1)
desc2 = le_lfd.FdcLBD(gx2, gy2).create_list(seg2)

# Filter by global rotation
grf = le_lfd.GlobalRotationFilter()
grf.train(seg1, seg2)

# Match with filter
matcher = le_lfd.BruteForceLBD()
matcher.train_filtered(desc1, desc2, grf)
matches = matcher.best()

# Print top matches
valid = [m for m in matches if not np.isnan(m.distance)]
valid.sort(key=lambda m: m.distance)
for m in valid[:10]:
    s1, s2 = seg1[m.query_idx], seg2[m.match_idx]
    print(f"  seg {m.query_idx} -> {m.match_idx}  dist={m.distance:.3f}")
```

## LIMAP Forward-Compatibility

The `lsfm.limap_compat` module provides conversion helpers for feeding
line features into [LIMAP](https://github.com/cvg/limap) 3D reconstruction:

```python
from lsfm.limap_compat import segments_to_limap, descriptors_to_limap, matches_to_limap

limap_segs  = segments_to_limap(segments)               # (N, 5) ndarray
limap_desc  = descriptors_to_limap(lbd_creator, segs)   # {"ms_lines", "line_descriptors"}
limap_pairs = matches_to_limap(matches)                  # (M, 2) int32
```

## Double Precision

All types are available in double precision:

```python
creator = le_lfd.FdcLBD_f64(gx.astype(np.float64), gy.astype(np.float64))
desc = creator.create_list(segments_f64)

matcher = le_lfd.BruteForceLBD_f64()
matcher.train(desc_q, desc_t)
```

## Build & Test

```bash
# Build the module
bazel build //libs/lfd/python:le_lfd

# Run tests (54 integration tests)
bazel test //libs/lfd/python:test_le_lfd

# Run all LFD library tests (C++ and Python)
bazel test //libs/lfd/...
```

## Architecture

```
python/
├── src/
│   ├── lfd_binding.hpp        # Binding declarations
│   ├── lfd_binding.cpp        # Type, creator, matcher, filter bindings
│   └── module_le_lfd.cpp      # PYBIND11_MODULE entry
├── tests/
│   └── test_le_lfd.py         # 54 Python integration tests
├── BUILD.bazel
└── README.md
```

### Module Dependency Graph

```
le_imgproc ──► le_edge ──► le_lsd ──► le_lfd
(ValueManager)  (NMS,ESD)  (detectors)  (descriptors, matchers, filters)
                  ▲                         ▲
le_geometry ──────┘─────────────────────────┘
(Line, LineSegment)
```

`le_lfd` imports all four prerequisite modules at load time:

- `le_geometry` — provides `Line`/`LineSegment` types (re-exported)
- `le_imgproc` — provides `ValueManager` base class
- `le_edge` — provides edge types used by gradient helpers
- `le_lsd` — provides line segment detection (pipeline input)

## See Also

- [le_lsd](../../lsd/python/README.md) — Line segment detection (pipeline input)
- [le_geometry](../../geometry/python/README.md) — Line/LineSegment API, drawing
- [le_edge](../../edge/python/README.md) — Edge detection pipeline
- [le_imgproc](../../imgproc/python/README.md) — Gradient filters and core types
- [le_eval](../../eval/python/README.md) — Performance benchmarking
- [le_algorithm](../../algorithm/python/README.md) — Post-processing, optimization, presets
- [LFD C++ Library](../README.md) — Full C++ API documentation
- [Demo Notebook](../../../examples/notebooks/demo_line_features.ipynb) — Interactive demo with Rerun visualization
