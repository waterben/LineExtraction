# le_algorithm — Python Bindings for Algorithm Library

Python bindings for the `libs/algorithm` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides post-processing, accuracy evaluation, and automated
parameter optimisation for line segment detection pipelines:

- **LineMerge** — merge collinear / near-collinear line segments
- **LineConnect** — connect nearby segments via gradient evidence
- **AccuracyMeasure** — precision, recall, F1, and structural AP (sAP)
- **GroundTruthLoader** — load / save ground truth annotations (CSV)
- **ParamOptimizer** — automated hyperparameter search (grid + random)
- **AccuracyResult**, **EvalResult**, **SearchResult** — result containers
- **ParamRange** — search space definition helpers
- **Enums**: `MergeType`, `OptimMetric`

Images are passed as NumPy arrays and automatically converted to/from
`cv::Mat`.  Line segments use `le_geometry.LineSegment_f64` (double
precision) by default.

> **Dependency:** `le_algorithm` imports `le_geometry` at load time.
> Both modules must be available.

## Quick Start

```python
import numpy as np
import le_algorithm as alg
import le_geometry as geo

# Create some test segments
seg_a = geo.LineSegment_f64.from_endpoints(0, 0, 10, 0)
seg_b = geo.LineSegment_f64.from_endpoints(12, 0, 22, 0)

# Merge collinear segments
merger = alg.LineMerge(max_dist=20.0, angle_error=10.0)
merged = merger.merge_lines([seg_a, seg_b])
print(f"Merged {2} -> {len(merged)} segments")  # 2 -> 1
```

## LineMerge

Iteratively merges pairs of line segments that are near-collinear, based
on endpoint distance, angle difference, perpendicular distance, and
parallel gap thresholds.

### Construction

```python
# Default parameters
merger = alg.LineMerge()

# Custom parameters
merger = alg.LineMerge(
    max_dist=20.0,        # max endpoint distance (px)
    angle_error=5.0,      # max angle difference (degrees)
    distance_error=3.0,   # max perpendicular distance (px)
    parallel_error=10.0,  # max parallel gap (px)
    merge_type=alg.MergeType.STANDARD,  # or MergeType.AVG
)
```

### Usage

```python
segments = [
    geo.LineSegment_f64.from_endpoints(0, 0, 50, 0),
    geo.LineSegment_f64.from_endpoints(55, 0, 100, 0),
    geo.LineSegment_f64.from_endpoints(0, 50, 100, 50),
]

merged = merger.merge_lines(segments)
print(f"{len(segments)} -> {len(merged)} segments")
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_dist` | float | 20.0 | Maximum endpoint distance for candidates |
| `angle_error` | float | 5.0 | Maximum angle difference (degrees) |
| `distance_error` | float | 3.0 | Maximum perpendicular distance |
| `parallel_error` | float | 10.0 | Maximum parallel gap |
| `merge_type` | `MergeType` | `STANDARD` | `STANDARD` (furthest endpoints) or `AVG` |

### MergeType Enum

| Value | Description |
|-------|-------------|
| `MergeType.STANDARD` | Keep the two furthest endpoints |
| `MergeType.AVG` | Average endpoint positions |

## LineConnect

Connects nearby segment endpoints when the gradient magnitude along the
connecting path is strong enough.  Evaluates all four endpoint pairings
and selects the one with the highest gradient response.

### Construction

```python
connector = alg.LineConnect()
connector = alg.LineConnect(
    max_radius=15.0,   # max endpoint distance to consider (px)
    accuracy=2.0,      # sampling step along path (px)
    threshold=10.0,    # min average gradient magnitude
)
```

### Usage

```python
import cv2

img = cv2.imread("image.png", cv2.IMREAD_GRAYSCALE)
magnitude = np.float32(img)  # or compute via Sobel

connected = connector.connect_lines(segments, magnitude)
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_radius` | float | 15.0 | Maximum endpoint distance to consider |
| `accuracy` | float | 2.0 | Sampling step along connecting path |
| `threshold` | float | 10.0 | Minimum average gradient magnitude |

## AccuracyMeasure

Computes detection quality metrics by matching detected segments to ground
truth using endpoint distance.  A match requires the minimum of the two
endpoint distance averages (forward and reversed) to be below the
threshold.

### Construction

```python
measure = alg.AccuracyMeasure(threshold=5.0)
```

### Evaluate

```python
detected = [geo.LineSegment_f64.from_endpoints(0, 0, 10, 0)]
ground_truth = [geo.LineSegment_f64.from_endpoints(0, 0, 10, 0)]

result = measure.evaluate(detected, ground_truth)
print(f"P={result.precision:.3f}  R={result.recall:.3f}  F1={result.f1:.3f}")
print(f"TP={result.true_positives}  FP={result.false_positives}  FN={result.false_negatives}")
```

### Structural AP (sAP)

Averages F1 across multiple distance thresholds in one call:

```python
sap = measure.structural_ap(detected, ground_truth, thresholds=[5, 10, 15])
print(f"sAP = {sap:.3f}")
```

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `threshold` | float | Matching threshold in pixels (read/write) |

### AccuracyResult

| Field | Type | Description |
|-------|------|-------------|
| `precision` | float | TP / (TP + FP) |
| `recall` | float | TP / (TP + FN) |
| `f1` | float | Harmonic mean of precision and recall |
| `true_positives` | int | Detected segments matching ground truth |
| `false_positives` | int | Detected segments without match |
| `false_negatives` | int | Ground truth segments without match |

## GroundTruthLoader

Load and save ground truth annotations in CSV format.

### CSV Format

```csv
image_name,x1,y1,x2,y2
img001.png,10,20,100,20
img001.png,50,10,50,90
img002.png,0,0,200,200
```

Segments are automatically grouped by image name.

### Usage

```python
# Load from file
entries = alg.GroundTruthLoader.load_csv("ground_truth.csv")
for entry in entries:
    print(f"{entry.image_name}: {len(entry.segments)} segments")

# Create programmatically
segments = [geo.LineSegment_f64.from_endpoints(10, 20, 100, 20)]
entry = alg.GroundTruthLoader.make_entry("test.png", segments)
print(entry)  # <GroundTruthEntry('test.png', 1 segments)>

# Save to file
alg.GroundTruthLoader.save_csv("output.csv", [entry])
```

### GroundTruthEntry

| Field | Type | Description |
|-------|------|-------------|
| `image_name` | str | Image file name |
| `segments` | list[LineSegment_f64] | Ground truth line segments |

## Search Strategies

Search strategies generate parameter configurations for `ParamOptimizer`.

### ParamRange

Defines a single parameter's search bounds:

```python
# Floating-point range
r = alg.ParamRange("alpha", min_val=0.1, max_val=2.0, step=0.1)

# Integer range
r = alg.ParamRange.make_int("kernel_size", min_val=3, max_val=15, step=2)

# Boolean (true/false)
r = alg.ParamRange.make_bool("refine")

# Inspect
print(r)  # ParamRange('alpha', 0.1, 2.0, step=0.1)
print(r.name, r.min_val, r.max_val, r.step)
```

### GridSearchStrategy

Exhaustive Cartesian product of all parameter values:

```python
strategy = alg.GridSearchStrategy()
space = [
    alg.ParamRange("a", 0.0, 1.0, 0.5),
    alg.ParamRange.make_bool("flag"),
]

configs = strategy.generate(space)
# configs is a list of lists of dicts:
# [[{"name": "a", "value": 0.0}, {"name": "flag", "value": False}],
#  [{"name": "a", "value": 0.0}, {"name": "flag", "value": True}],
#  [{"name": "a", "value": 0.5}, {"name": "flag", "value": False}],
#  ...]
print(f"{len(configs)} configurations")  # 3 * 2 = 6
```

### RandomSearchStrategy

Uniform random sampling with optional seed for reproducibility:

```python
strategy = alg.RandomSearchStrategy(num_samples=200, seed=42)
configs = strategy.generate(space)
print(f"{len(configs)} configurations")  # 200

# Reproducible: same seed => same configs
s1 = alg.RandomSearchStrategy(50, seed=42)
s2 = alg.RandomSearchStrategy(50, seed=42)
assert s1.generate(space) == s2.generate(space)  # True
```

## ParamOptimizer

Orchestrates automated hyperparameter search:

1. Generates candidate configurations via a `SearchStrategy`.
2. Runs a user-supplied detection function with each configuration.
3. Evaluates output against ground truth using `AccuracyMeasure`.
4. Tracks the best result and optionally reports progress.

### Construction

```python
optimizer = alg.ParamOptimizer(
    metric=alg.OptimMetric.F1,      # F1, PRECISION, or RECALL
    match_threshold=5.0,             # endpoint matching threshold (px)
    verbose=False,                   # print progress to stdout
)
```

### Detection Function

The detection callback receives:

- `src` — the source image as a NumPy array
- `params` — a **list of dicts**, each with `"name"` and `"value"` keys

It must return a **list of `LineSegment_f64`**.

```python
def detect_fn(src: np.ndarray, params: list[dict]) -> list[geo.LineSegment_f64]:
    """Run detection with the given parameters."""
    # Extract parameter values
    param_dict = {p["name"]: p["value"] for p in params}

    alpha = float(param_dict["alpha"])
    refine = bool(param_dict.get("refine", False))

    # ... run your detector ...
    return detected_segments
```

### Full Optimisation Example

```python
import numpy as np
import le_algorithm as alg
import le_geometry as geo

# 1. Prepare ground truth
gt_segments = [
    geo.LineSegment_f64.from_endpoints(10, 10, 90, 10),
    geo.LineSegment_f64.from_endpoints(10, 50, 90, 50),
]
ground_truth = [alg.GroundTruthLoader.make_entry("test.png", gt_segments)]

# 2. Prepare images
test_img = np.zeros((100, 100), dtype=np.uint8)
images = [("test.png", test_img)]

# 3. Define search space
space = [
    alg.ParamRange("sensitivity", 0.0, 1.0, 0.1),
    alg.ParamRange.make_bool("refine"),
]

# 4. Define detection function
def detect_fn(src, params):
    param_dict = {p["name"]: p["value"] for p in params}
    sensitivity = float(param_dict["sensitivity"])
    if sensitivity >= 0.3:
        return gt_segments  # simplified — return perfect detections
    return []

# 5. Run optimisation
optimizer = alg.ParamOptimizer(metric=alg.OptimMetric.F1, match_threshold=5.0)
strategy = alg.GridSearchStrategy()

result = optimizer.optimize(
    strategy, space, images, ground_truth, detect_fn,
    progress=lambda step, total, score: (
        print(f"[{step}/{total}] best={score:.3f}") or True
    ),
)

# 6. Inspect results
print(f"Best F1: {result.best_score:.3f}")
print(f"Best params: {result.best_params}")
print(f"Total configs evaluated: {result.total_configs}")

# Top 3 results
for r in result.top_n(3):
    print(f"  score={r.score:.3f}  params={r.params}")
```

### Progress Callback

An optional callback called after each configuration is evaluated.
Return `False` to cancel the search early:

```python
def progress(step: int, total: int, best_score: float) -> bool:
    print(f"[{step}/{total}] best={best_score:.3f}")
    if best_score >= 0.99:
        return False  # stop early — good enough
    return True

result = optimizer.optimize(strategy, space, images, gt, detect_fn,
                            progress=progress)
```

### OptimMetric Enum

| Value | Description |
|-------|-------------|
| `OptimMetric.F1` | F1 score (harmonic mean of precision/recall) |
| `OptimMetric.PRECISION` | Optimise for precision only |
| `OptimMetric.RECALL` | Optimise for recall only |

### SearchResult

| Field / Property | Type | Description |
|------------------|------|-------------|
| `best_params` | list[dict] | Best parameter configuration (read-only) |
| `best_score` | float | Best metric score achieved |
| `total_configs` | int | Total configurations evaluated |
| `all_results` | list[EvalResult] | All evaluation results |
| `top_n(n)` | list[EvalResult] | Top N results sorted by score |

### EvalResult

| Field / Property | Type | Description |
|------------------|------|-------------|
| `params` | list[dict] | Parameter configuration (read-only) |
| `accuracy` | AccuracyResult | Full accuracy metrics |
| `score` | float | Optimised metric value |

## Parameter Dict Convention

In Python, parameter configurations are represented as **lists of dicts**
(not C++ `NameValuePair` objects).  Each dict has two keys:

```python
{"name": "quant_error", "value": 2.5}
```

Values are native Python types (`float`, `int`, `bool`) depending on the
parameter type defined in `ParamRange`.

This convention applies to:

- `params` argument in `detect_fn` (called by `ParamOptimizer.optimize`)
- `SearchResult.best_params`
- `EvalResult.params`
- Return values of `GridSearchStrategy.generate()` / `RandomSearchStrategy.generate()`

## Type Suffixes

Templated classes are available in double (default) and float variants:

| Python Class | C++ Type | Precision |
|-------------|----------|-----------|
| `LineMerge` | `LineMerge<double>` | f64 |
| `LineMerge_f32` | `LineMerge<float>` | f32 |
| `LineConnect` | `LineConnect<double>` | f64 |
| `LineConnect_f32` | `LineConnect<float>` | f32 |
| `AccuracyMeasure` | `AccuracyMeasure<double>` | f64 |
| `AccuracyMeasure_f32` | `AccuracyMeasure<float>` | f32 |

Non-templated classes (`GroundTruthLoader`, `ParamOptimizer`, search
strategies, enums) have no suffix.

## Build & Test

```bash
# Build the Python extension module
bazel build //libs/algorithm/python:le_algorithm

# Run Python tests (20 tests, 7 test classes)
bazel test //libs/algorithm/python:test_le_algorithm

# Run with verbose output
bazel test //libs/algorithm/python:test_le_algorithm --test_output=all
```
