# Algorithm Library

Post-processing, accuracy evaluation, and parameter optimisation toolkit for
line segment detection pipelines. Provides merging, connecting, precision
refinement, ground truth handling, and automated hyperparameter search — all
usable from C++ and Python.

## Overview

The `algorithm` library sits on top of the detection modules (`lsd`, `lfd`,
`edge`) and offers reusable building blocks that are typically needed
**after** detection:

| Component | Header | Description |
|-----------|--------|-------------|
| **LineMerge** | `line_merge.hpp` | Merge collinear / near-collinear segments |
| **LineConnect** | `line_connect.hpp` | Connect nearby segments via gradient evidence |
| **AccuracyMeasure** | `accuracy_measure.hpp` | Precision, recall, F1, and sAP metrics |
| **GroundTruthLoader** | `ground_truth.hpp` | Load / save ground truth in CSV format |
| **ParamOptimizer** | `param_search.hpp` | Automated parameter search (grid + random) |
| **PrecisionOptimize** | `precision_optimize.hpp` | Sub-pixel line refinement (dlib-based) |

All code resides in the `lsfm` namespace. The library is **header-only**
(no `.cpp` files in `src/`).

## Architecture

```
ValueManager (from libs/utility)
  ├── LineMerge<FT>           — merge collinear segments
  ├── LineConnect<FT>         — gradient-guided connection
  └── PrecisionOptimize       — dlib-based sub-pixel optimisation

AccuracyMeasure<FT>           — detection quality metrics
GroundTruthLoader             — CSV I/O for ground truth segments

SearchStrategy (abstract)
  ├── GridSearchStrategy      — exhaustive Cartesian product
  └── RandomSearchStrategy    — uniform random sampling

ParamOptimizer                — orchestrates search + evaluation
```

`LineMerge`, `LineConnect`, and `PrecisionOptimize` extend `ValueManager`,
so their parameters can be read and set by name at runtime.  This is the
mechanism `ParamOptimizer` uses to inject configurations during search.

### Dependencies

| Dependency | Usage |
|------------|-------|
| `libs/geometry` | `LineSegment`, `Line`, `Vec2`, point helpers |
| `libs/imgproc` | Interpolation and mean helpers (PrecisionOptimize) |
| `libs/utility` | `Value`, `ValueManager` |
| `libs/eval` | Performance task infrastructure |
| OpenCV | `cv::Mat`, gradient computation |
| dlib | BFGS / L-BFGS / CG optimisation (PrecisionOptimize) |

## Components

### LineMerge

Iteratively merges pairs of line segments that satisfy four proximity
criteria simultaneously:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_dist` | 20 | Maximum endpoint distance (pixels) |
| `angle_error` | 5 | Maximum angle difference (degrees) |
| `distance_error` | 3 | Maximum perpendicular distance (pixels) |
| `parallel_error` | 10 | Maximum parallel gap (pixels) |
| `merge_type` | `STANDARD` | `STANDARD` (furthest endpoints) or `AVG` (average positions) |

```cpp
#include <algorithm/line_merge.hpp>

lsfm::LineMerge<double> merger(/*max_dist=*/20, /*angle_error=*/5,
                               /*dist_error=*/3, /*parallel_error=*/10);

std::vector<lsfm::LineSegment<double>> output;
merger.merge_lines(input_segments, output);
```

### LineConnect

Joins nearby segment endpoints when the gradient magnitude along the
connecting path is strong enough.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_radius` | 15 | Maximum endpoint distance to consider (pixels) |
| `accuracy` | 2 | Sampling step along the connecting path (pixels) |
| `threshold` | 10 | Minimum average gradient magnitude |

The algorithm evaluates all four endpoint pairings (start–start,
start–end, end–start, end–end) and picks the one with the highest
gradient response above the threshold.

```cpp
#include <algorithm/line_connect.hpp>

lsfm::LineConnect<double> connector(/*max_radius=*/15,
                                    /*accuracy=*/2,
                                    /*threshold=*/10);

std::vector<lsfm::LineSegment<double>> output;
connector.connect_lines(input_segments, output, gradient_magnitude);

// Alternatively, let it compute the magnitude from the source image:
cv::Mat magnitude;
connector.connect_lines(input_segments, output, grayscale_src, magnitude);
```

### AccuracyMeasure

Computes standard detection quality metrics by matching detected segments
to ground truth using endpoint distance.

A detected segment matches a ground truth segment when the minimum of
the two endpoint-to-endpoint distance averages (forward and reversed) is
below the threshold.

```cpp
#include <algorithm/accuracy_measure.hpp>

lsfm::AccuracyMeasure<double> measure(/*threshold=*/5.0);
lsfm::AccuracyResult result = measure.evaluate(detected, ground_truth);

std::cout << "Precision: " << result.precision << "\n"
          << "Recall:    " << result.recall    << "\n"
          << "F1:        " << result.f1        << "\n"
          << "TP=" << result.true_positives
          << " FP=" << result.false_positives
          << " FN=" << result.false_negatives  << "\n";
```

**Structural AP (sAP):**

```cpp
double sap = measure.structural_ap(detected, ground_truth,
                                   /*thresholds=*/{5, 10, 15});
```

Averages F1 across multiple distance thresholds in one call.

### GroundTruthLoader

Reads and writes ground truth annotations in a simple CSV format:

```
image_name,x1,y1,x2,y2
img001.png,10,20,100,20
img001.png,50,10,50,90
img002.png,0,0,200,200
```

Segments are grouped by image name automatically.

```cpp
#include <algorithm/ground_truth.hpp>

// Load
auto entries = lsfm::GroundTruthLoader::load_csv("ground_truth.csv");

// Create programmatically
auto entry = lsfm::GroundTruthLoader::make_entry("test.png", segments);

// Save
lsfm::GroundTruthLoader::save_csv("output.csv", entries);
```

### ParamOptimizer

Orchestrates a parameter search by:

1. Generating candidate configurations via a `SearchStrategy`.
2. Running a user-supplied detection function with each configuration.
3. Evaluating the output against ground truth using `AccuracyMeasure`.
4. Tracking the best result and optionally reporting progress.

```cpp
#include <algorithm/param_search.hpp>

// 1. Define the search space
lsfm::SearchSpace space = {
    lsfm::ParamRange("quant_error", 1.0, 4.0, 0.5),
    lsfm::ParamRange("angle_th", 15.0, 30.0, 5.0),
    lsfm::ParamRange::make_int("min_length", 5, 30, 5),
    lsfm::ParamRange::make_bool("refine"),
};

// 2. Define the detection callback
auto detect_fn = [&](const cv::Mat& src,
                     const lsfm::ParamConfig& params)
    -> std::vector<lsfm::LineSegment<double>> {
  LsdFGioi<double> lsd;
  lsd.value(params);   // inject parameters via ValueManager
  lsd.detect(src);
  return lsd.lineSegments();
};

// 3. Prepare images and ground truth
auto gt = lsfm::GroundTruthLoader::load_csv("ground_truth.csv");
std::vector<std::pair<std::string, cv::Mat>> images = { ... };

// 4. Run optimisation
lsfm::ParamOptimizer optimizer(lsfm::OptimMetric::F1,
                               /*match_threshold=*/5.0,
                               /*verbose=*/true);
lsfm::GridSearchStrategy strategy;

lsfm::SearchResult result = optimizer.optimize(
    strategy, space, images, gt, detect_fn,
    [](int step, int total, double best) -> bool {
      std::cout << step << "/" << total << " best=" << best << "\n";
      return true;  // return false to cancel
    });

std::cout << "Best F1: " << result.best_score << "\n";
for (const auto& p : result.best_params)
  std::cout << "  " << p.name << " = " << p.value << "\n";
```

#### Search Strategies

| Strategy | Class | Description |
|----------|-------|-------------|
| **Grid** | `GridSearchStrategy` | Exhaustive Cartesian product of all grid points |
| **Random** | `RandomSearchStrategy` | Uniform random sampling with optional seed |

```cpp
lsfm::GridSearchStrategy   grid;
lsfm::RandomSearchStrategy random(/*n_samples=*/200, /*seed=*/42);
```

#### ParamRange Types

| Factory | Type | Example |
|---------|------|---------|
| Constructor | `FLOAT` | `ParamRange("alpha", 0.1, 2.0, 0.1)` |
| `make_int` | `INT` | `ParamRange::make_int("k", 3, 15, 2)` |
| `make_bool` | `BOOL` | `ParamRange::make_bool("refine")` |

#### Result Inspection

```cpp
// Top 5 configurations
auto top5 = result.top_n(5);
for (const auto& r : top5)
  std::cout << r.score << "\n";

// Full result list (sorted by score descending)
result.sort_by_score();
```

### PrecisionOptimize

Sub-pixel refinement of line segment positions using gradient-based
numerical optimisation (BFGS / L-BFGS / Conjugate Gradient via dlib).

Searches over orthogonal translation and rotation to maximise the
mean gradient response along the segment.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `search_range_d` | 1.0 | Orthogonal search range (pixels) |
| `search_range_r` | 1.0 | Rotation search range (degrees) |
| `interpolation` | `BILINEAR` | `NEAREST` / `NEAREST_ROUND` / `BILINEAR` / `BICUBIC` |
| `search_strategy` | `BFGS` | `BFGS` / `LBFGS` / `CG` |
| `stop_strategy` | `DELTA` | `DELTA` / `GRAD_NORM` |
| `stop_delta` | 1e-7 | Stop criterion threshold |
| `max_iterations` | 0 | Maximum iterations (0 = unlimited) |
| `derivative_precision` | 1e-7 | Numerical derivative delta |
| `mean_param` | 1.0 | Mean calculation parameter |
| `use_sampled` | false | Use sampled mean calculation |
| `use_fast` | false | Use fast interpolation |

```cpp
#include <algorithm/precision_optimize.hpp>

lsfm::PrecisionOptimize optimizer;
optimizer.value("search_strategy", 0);  // BFGS
optimizer.value("interpolation", 2);    // bilinear

// Optimise in place
auto errors = optimizer.optimize_all(gradient_magnitude, segments);

// Or produce a refined copy
std::vector<lsfm::LineSegment<double>> refined;
auto errors = optimizer.optimize_copy(gradient_magnitude, segments, refined);
```

## Python Bindings (`le_algorithm`)

The library is fully exposed to Python via pybind11.  All classes in the
table above have Python counterparts in the `le_algorithm` module.

> **Note:** The default type suffix is empty for **double** precision and
> `_f32` for **float** precision (matching the `le_geometry` convention
> where `LineSegment_f64` = double).

### Installation

```python
import le_algorithm
import le_geometry
```

Or via the umbrella package:

```python
from lsfm import le_algorithm, le_geometry
```

### Quick Start

```python
import numpy as np
import le_algorithm as alg
import le_geometry as geo

# --- Merging ---
merger = alg.LineMerge(max_dist=20, angle_error=10, distance_error=3, parallel_error=15)
segments = [
    geo.LineSegment_f64.from_endpoints(0, 0, 10, 0),
    geo.LineSegment_f64.from_endpoints(12, 0, 22, 0),
]
merged = merger.merge_lines(segments)  # -> 1 segment

# --- Accuracy ---
measure = alg.AccuracyMeasure(threshold=5.0)
result = measure.evaluate(detected, ground_truth)
print(f"F1={result.f1:.3f}  P={result.precision:.3f}  R={result.recall:.3f}")

# --- Ground truth ---
entry = alg.GroundTruthLoader.make_entry("img.png", segments)
# alg.GroundTruthLoader.save_csv("gt.csv", [entry])
# entries = alg.GroundTruthLoader.load_csv("gt.csv")

# --- Parameter optimisation ---
space = [alg.ParamRange("sensitivity", 0.0, 1.0, 0.1)]
strategy = alg.GridSearchStrategy()

def detect_fn(src, params):
    """params is a list of {'name': str, 'value': float|int|bool} dicts."""
    sensitivity = float(params[0]["value"])
    # ... run detector with this sensitivity ...
    return detected_segments

optimizer = alg.ParamOptimizer(metric=alg.OptimMetric.F1, match_threshold=5.0)
result = optimizer.optimize(strategy, space, images, ground_truth, detect_fn)
print(f"Best F1: {result.best_score}")
print(f"Best params: {result.best_params}")
```

### Parameter Passing Convention

In Python, parameter configurations are represented as **lists of dicts**
rather than C++ `ValueManager::NameValuePair` objects.  Each dict has two
keys:

```python
{"name": "quant_error", "value": 2.5}
```

This applies to:

- The `params` argument passed to `detect_fn` in `ParamOptimizer.optimize()`
- The `best_params` property of `SearchResult`
- The `params` property of `EvalResult`
- The return value of `GridSearchStrategy.generate()` and
  `RandomSearchStrategy.generate()`

### API Reference

#### Classes

| Python Class | C++ Class | Suffix |
|-------------|-----------|--------|
| `LineMerge` | `LineMerge<double>` | (default) |
| `LineMerge_f32` | `LineMerge<float>` | `_f32` |
| `LineConnect` | `LineConnect<double>` | (default) |
| `LineConnect_f32` | `LineConnect<float>` | `_f32` |
| `AccuracyMeasure` | `AccuracyMeasure<double>` | (default) |
| `AccuracyMeasure_f32` | `AccuracyMeasure<float>` | `_f32` |
| `GroundTruthEntry` | `GroundTruthEntry` | — |
| `GroundTruthLoader` | `GroundTruthLoader` | — |
| `ParamRange` | `ParamRange` | — |
| `GridSearchStrategy` | `GridSearchStrategy` | — |
| `RandomSearchStrategy` | `RandomSearchStrategy` | — |
| `ParamOptimizer` | `ParamOptimizer` | — |
| `AccuracyResult` | `AccuracyResult` | — |
| `EvalResult` | `EvalResult` | — |
| `SearchResult` | `SearchResult` | — |

#### Enums

| Python Enum | Values |
|-------------|--------|
| `MergeType` | `STANDARD`, `AVG` |
| `OptimMetric` | `F1`, `PRECISION`, `RECALL` |

## Build

### Bazel (primary)

```bash
# Build library
bazel build //libs/algorithm:lib_algorithm

# Run C++ tests (30 tests, 8 suites)
bazel test //libs/algorithm:test_algorithm

# Build Python module
bazel build //libs/algorithm/python:le_algorithm

# Run Python tests (20 tests, 7 classes)
bazel test //libs/algorithm/python:test_le_algorithm

# Build everything
bazel build //libs/algorithm/...

# Test everything
bazel test //libs/algorithm/...
```

### CMake (legacy)

```bash
mkdir build && cd build
cmake ..
cmake --build . --target lib_algorithm
ctest -R test_algorithm
```

## Directory Structure

```
libs/algorithm/
├── BUILD.bazel                              # Bazel build (cc_library + cc_test)
├── CMakeLists.txt                           # CMake build (legacy)
├── README.md                                # This file
├── include/algorithm/
│   ├── accuracy_measure.hpp                 # AccuracyMeasure, AccuracyResult
│   ├── ground_truth.hpp                     # GroundTruthEntry, GroundTruthLoader
│   ├── line_connect.hpp                     # LineConnect
│   ├── line_merge.hpp                       # LineMerge, MergeType
│   ├── param_search.hpp                     # ParamOptimizer, SearchResult, EvalResult
│   ├── precision_optimize.hpp               # PrecisionOptimize
│   └── search_strategy.hpp                  # ParamRange, SearchStrategy, Grid, Random
├── python/
│   ├── BUILD.bazel                          # Bazel build (pybind module + py_test)
│   ├── src/
│   │   ├── algorithm_binding.hpp            # Binding declarations
│   │   ├── algorithm_binding.cpp            # Binding implementations
│   │   └── module_le_algorithm.cpp          # PYBIND11_MODULE entry point
│   └── tests/
│       └── test_le_algorithm.py             # Python integration tests (20 tests)
├── src/                                     # (empty — header-only library)
└── tests/
    └── test_algorithm.cpp                   # C++ unit tests (30 tests, 8 suites)
```

## Testing

### C++ Tests (Google Test)

| Suite | Tests | Description |
|-------|-------|-------------|
| `LineMergeTest` | 6 | Collinear merge, distant preserve, perpendicular, empty, single, AVG |
| `LineConnectTest` | 3 | Empty input, single segment, ValueManager |
| `AccuracyMeasureTest` | 7 | Perfect match, no detections, all FP, partial, both empty, reversed, sAP |
| `GridSearchTest` | 5 | Single param, two params, empty space, int, bool |
| `RandomSearchTest` | 3 | Correct count, reproducible, within bounds |
| `ParamOptimizerTest` | 3 | Basic optimisation, progress callback, cancellation |
| `GroundTruthTest` | 1 | make_entry |
| `SearchResultTest` | 1 | top_n |

### Python Tests (pytest)

| Class | Tests | Description |
|-------|-------|-------------|
| `TestLineMerge` | 4 | Construction, collinear merge, distant preserve, empty |
| `TestLineConnect` | 2 | Construction, empty input |
| `TestAccuracyMeasure` | 4 | Perfect match, no detections, threshold property, sAP |
| `TestGroundTruth` | 2 | make_entry, repr |
| `TestSearchStrategy` | 4 | Grid single param, grid bool, random count, reproducible |
| `TestParamOptimizer` | 2 | Construction, basic optimisation |
| `TestEnums` | 2 | MergeType, OptimMetric |
