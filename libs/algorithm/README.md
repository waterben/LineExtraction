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
| **ImageAnalyzer** | `image_analyzer.hpp` | Image property analysis (contrast, noise, edges) |
| **DetectorProfile** | `detector_profile.hpp` | Intuitive high-level parameter profiles for LSD detectors |

All code resides in the `lsfm` namespace.

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

ImageAnalyzer                 — contrast/noise/edge/range analysis
ImageProperties               — measured image characteristics
ProfileHints                  — suggested knob values + adaptive factors

DetectorProfile               — 4 percentage knobs → detector parameters
  └── maps to all 9 LSD detectors (CC, CP, Burns, FBW, FGioi,
                                    EDLZ, EL, EP, HoughP)
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

#### Merge Criteria Illustrated

Two segments A and B are merged when **all four** criteria pass:

```
  1. Endpoint distance ≤ max_dist (smallest of 4 endpoint pairs)

        max_dist
       |<------->|
  A ===●          ●=== B

  2. Angle difference ≤ angle_error (in degrees, 0–90° range)

                     ╱ B
  A ══════════      ╱
              α° ──╱       α must be ≤ angle_error

  3. Perpendicular distance ≤ distance_error
     (midpoint of B projected onto line through A)

  A ════════════════════
                  |
           d⊥     |  ← perpendicular distance
                  |
             ●────── midpoint of B
        B ═══════════

  4. Parallel gap ≤ parallel_error
     (projection of all endpoints onto A's direction)

  A ══════════                 B ══════
             |<── gap ──>|
  gap = 0 if segments overlap along the direction.
```

The process repeats until convergence (no more merges possible).

> **GUI:** The [Continuity Optimizer](../../apps/line_analyzer/extensions/continuityoptimizer/README.md) in the Line Analyzer app provides an interactive UI for this algorithm.

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

#### Connection Process Illustrated

```
  A ════●              ●════ B
        |              |
        |<── radius ──>|    (must be ≤ max_radius)
        |              |
        ○──○──○──○──○──○    gradient samples (every `accuracy` px)
              ↓
  avg(samples) ≥ threshold  → connect!

  Before:  A ●════●         ●════● B
  After:   A ●════════════════════● B   (endpoints extended)
```

> **GUI:** The [Connection Optimizer](../../apps/line_analyzer/extensions/connectionoptimizer/README.md) in the Line Analyzer app provides an interactive UI for this algorithm.

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

```
  GT:  A ●════════════● B

  Det: C ●═══════════● D     avg(dist(A,C) + dist(B,D)) ≤ threshold → TP
       E ●══════● F                                    dist > threshold → FP
                                                     GT not matched → FN
```

> **GUI:** The [Accuracy Measure](../../apps/line_analyzer/extensions/accuracy/README.md) and [Line Analyser 2D](../../apps/line_analyzer/extensions/lineanalyser2d/README.md) panels in the Line Analyzer app provide interactive UIs for accuracy evaluation.

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

```
  Search space (2 degrees of freedom):

       ←── d (orthogonal offset, ±pixels) ──→

              ╱  r (rotation, ±degrees)
       ●═════╱══════════●
             ╱
            ╱
       ●══════════════════●   ← optimized position (max gradient)
```

> **GUI:** The [Precision Optimizer](../../apps/line_analyzer/extensions/precisionoptimizer/README.md) and [PO Function Plot](../../apps/line_analyzer/extensions/pofuncplot/README.md) panels in the Line Analyzer app provide interactive UIs for this algorithm.

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

### ImageAnalyzer

Analyzes an image to extract measurable properties (contrast, noise level,
edge density, dynamic range), all normalized to [0, 1]. These properties
can be used to suggest adaptive detector profile knob values via
`suggest_profile()`.

The analyzer accepts both grayscale and color images (auto-converted
to 8-bit grayscale internally).

> **GUI:** The [Image Analyzer](../../apps/line_analyzer/extensions/imageanalyzer/README.md) panel in the Line Analyzer app provides an interactive UI for this analysis.

#### Measured Properties (`ImageProperties`)

| Property | Range | Derived From | Description |
|----------|-------|--------------|-------------|
| `contrast` | 0–1 | `meanStdDev` | Normalized intensity standard deviation. 0 = flat/uniform, 1 = maximum contrast. |
| `noise_level` | 0–1 | MAD of Laplacian | Robust noise estimate (Immerkær method). 0 = noise-free, 1 = very noisy. |
| `edge_density` | 0–1 | Sobel threshold | Fraction of strong-gradient pixels. 0 = no edges, 1 = edges everywhere. |
| `dynamic_range` | 0–1 | Histogram percentiles | Spread between 5th–95th percentile. 0 = narrow, 1 = full range. |

#### Profile Hints

`ImageProperties::suggest_profile()` returns a `ProfileHints` struct
with heuristic knob suggestions and adaptive scaling factors:

| Field | Range | Derivation Logic |
|-------|-------|------------------|
| `detail` | 10–90 | Increases with contrast, decreases with noise |
| `gap_tolerance` | 10–90 | Increases with noise, decreases with edge density |
| `min_length` | 10–90 | Increases with noise and edge density |
| `precision` | 10–90 | Increases with dynamic range, decreases with noise |
| `contrast_factor` | 0.5–2.0 | >1 for low-contrast images (raises thresholds) |
| `noise_factor` | 0.5–2.0 | >1 for noisy images (raises thresholds) |

#### Usage

```cpp
#include <algorithm/image_analyzer.hpp>

cv::Mat img = cv::imread("photo.png", cv::IMREAD_GRAYSCALE);
lsfm::ImageProperties props = lsfm::ImageAnalyzer::analyze(img);

std::cout << "Contrast:      " << props.contrast      << "\n"
          << "Noise level:   " << props.noise_level    << "\n"
          << "Edge density:  " << props.edge_density   << "\n"
          << "Dynamic range: " << props.dynamic_range  << "\n";

// Get suggested profile knobs based on image analysis
lsfm::ProfileHints hints = props.suggest_profile();
std::cout << "Suggested detail:    " << hints.detail         << "%\n"
          << "Suggested gap_tol:   " << hints.gap_tolerance  << "%\n"
          << "Suggested min_len:   " << hints.min_length     << "%\n"
          << "Suggested precision: " << hints.precision      << "%\n"
          << "Contrast factor:     " << hints.contrast_factor << "\n"
          << "Noise factor:        " << hints.noise_factor    << "\n";
```

### DetectorProfile

Translates 4 intuitive percentage knobs into concrete detector parameters
for all 9 supported LSD detectors.  This abstraction lets users control
detection behaviour without knowing the internal parameter names of each
algorithm.

> **GUI:** The [Detector Profile](../../apps/line_analyzer/extensions/detectorprofile/README.md) panel in the Line Analyzer app provides an interactive UI with sliders for this algorithm.

#### Knob Semantics

| Knob | Range | Low (0%) | High (100%) |
|------|-------|----------|-------------|
| `detail` | 0–100 | Coarse / salient edges only | Fine details included |
| `gap_tolerance` | 0–100 | No gaps allowed — strict chaining | Very tolerant of gaps |
| `min_length` | 0–100 | Keep all segments (even tiny) | Long segments only |
| `precision` | 0–100 | Rough / fast | Sub-pixel accurate |

#### Adaptive Scaling Factors

When constructed `from_image()` or `from_hints()`, the profile carries
two multiplicative factors that modulate the parameter translation:

| Factor | Range | Effect |
|--------|-------|--------|
| `contrast_factor` | 0.5–2.0 | Scales threshold-like parameters. >1 for low-contrast images. |
| `noise_factor` | 0.5–2.0 | Scales threshold-like parameters. >1 for noisy images. |

These factors are multiplied into threshold / gradient parameters during
`to_params()` so that the same knob settings produce adapted behaviour
on different image types.

#### How the Knob Mapping Works

Each of the 9 detectors has its own mapping function (`map_lsd_cc()`, etc.)
that translates the 4 human-readable knobs to a `ParamConfig` vector:

1. The knob percentage is normalized to `[0, 1]`.
2. Linear interpolation (`lerp`) maps each knob to the detector-specific
   parameter range (e.g., `detail 0%→100%` maps `quant_error 3.0→0.5`
   for LsdFGioi).
3. Threshold-related parameters are multiplied by the combined adaptive
   factor `contrast_factor * noise_factor`.
4. The resulting `ParamConfig` can be injected into a detector via
   `ValueManager::value()`.

#### Supported Detectors

| DetectorId | Name | Key Mapped Parameters |
|-----------|------|----------------------|
| `LSD_CC` | LsdCC | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `quant_error` |
| `LSD_CP` | LsdCP | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `quant_error` |
| `LSD_BURNS` | LsdBurns | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `angle_th` |
| `LSD_FBW` | LsdFBW | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `angle_th` |
| `LSD_FGIOI` | LsdFGioi | `quant_error`, `angle_th`, `min_length`, `refine` |
| `LSD_EDLZ` | LsdEDLZ | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `min_length` |
| `LSD_EL` | LsdEL | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `quant_error` |
| `LSD_EP` | LsdEP | `nms_th_low`, `nms_th_high`, `min_pix`, `max_gap`, `quant_error`, `pat_tol` |
| `LSD_HOUGHP` | LsdHoughP | `nms_th_low`, `nms_th_high`, `rho`, `theta`, `threshold`, `min_line_length`, `max_line_gap` |

#### Usage

```cpp
#include <algorithm/detector_profile.hpp>

// Option 1: Manual knob values
lsfm::DetectorProfile profile(/*detail=*/70, /*gap_tolerance=*/30,
                               /*min_length=*/50, /*precision=*/80);

// Option 2: Image-adaptive (analyze + suggest + translate)
auto profile = lsfm::DetectorProfile::from_image(image);

// Generate concrete parameters for a specific detector
auto params = profile.to_params(lsfm::DetectorId::LSD_CC);

// Apply directly to a detector instance
LsdCC<double> detector;
profile.apply(detector, lsfm::DetectorId::LSD_CC);
detector.detect(image);

// Inspect generated parameters
for (const auto& p : params)
    std::cout << p.name << " = " << p.value << "\n";

// List supported detectors
auto names = lsfm::DetectorProfile::supported_detectors();
// -> {"LsdCC", "LsdCP", "LsdBurns", "LsdFBW", "LsdFGioi",
//     "LsdEDLZ", "LsdEL", "LsdEP", "LsdHoughP"}
```

#### End-to-End Workflow

The typical image-adaptive workflow is:

```cpp
// 1. Analyze the image
auto props = lsfm::ImageAnalyzer::analyze(image);

// 2. Get suggested knobs + adaptive factors
auto hints = props.suggest_profile();

// 3. Create profile (carries factors automatically)
auto profile = lsfm::DetectorProfile::from_hints(hints);

// 4. (Optional) Override individual knobs
profile.set_detail(80);  // user wants more detail

// 5. Apply to any number of detectors
LsdFGioi<double> fgioi;
profile.apply(fgioi, lsfm::DetectorId::LSD_FGIOI);
fgioi.detect(image);

LsdEDLZ<double> edlz;
profile.apply(edlz, lsfm::DetectorId::LSD_EDLZ);
edlz.detect(image);
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

# --- Image analysis + Detector profiles ---
# Analyze image properties
img = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
props = alg.ImageAnalyzer.analyze(img)
print(f"Contrast={props.contrast:.2f}, Noise={props.noise_level:.2f}")

# Suggest adaptive profile knobs
hints = props.suggest_profile()
print(f"Suggested detail: {hints.detail:.0f}%")

# Create detector profile (manual or image-adaptive)
profile = alg.DetectorProfile(detail=70, gap_tolerance=30,
                              min_length=50, precision=80)
# or: profile = alg.DetectorProfile.from_image(img)

# Generate detector-specific parameters
params = profile.to_params(alg.DetectorId.LSD_CC)       # by enum
params = profile.to_params_by_name("LsdFGioi")           # by name
for p in params:
    print(f"  {p['name']} = {p['value']}")

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
| `ImageProperties` | `ImageProperties` | — |
| `ProfileHints` | `ProfileHints` | — |
| `ImageAnalyzer` | `ImageAnalyzer` | — |
| `DetectorProfile` | `DetectorProfile` | — |

#### Enums

| Python Enum | Values |
|-------------|--------|
| `MergeType` | `STANDARD`, `AVG` |
| `OptimMetric` | `F1`, `PRECISION`, `RECALL` |
| `DetectorId` | `LSD_CC`, `LSD_CP`, `LSD_BURNS`, `LSD_FBW`, `LSD_FGIOI`, `LSD_EDLZ`, `LSD_EL`, `LSD_EP`, `LSD_HOUGHP` |

## Build

### Bazel (primary)

```bash
# Build library
bazel build //libs/algorithm:lib_algorithm

# Run C++ tests
bazel test //libs/algorithm:test_algorithm
bazel test //libs/algorithm:test_detector_profile

# Build Python module
bazel build //libs/algorithm/python:le_algorithm

# Run Python tests
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
│   ├── detector_profile.hpp                 # DetectorProfile, DetectorId
│   ├── ground_truth.hpp                     # GroundTruthEntry, GroundTruthLoader
│   ├── image_analyzer.hpp                   # ImageAnalyzer, ImageProperties, ProfileHints
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
│       └── test_le_algorithm.py             # Python integration tests
├── src/
│   ├── detector_profile.cpp                 # DetectorProfile per-detector mapping
│   └── image_analyzer.cpp                   # ImageAnalyzer analysis implementation
└── tests/
    ├── test_algorithm.cpp                   # C++ unit tests (LineMerge, Connect, etc.)
    └── test_detector_profile.cpp             # C++ tests for ImageAnalyzer + DetectorProfile
```

## Testing

### C++ Tests (Google Test)

**test_algorithm** (30 tests):

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

**test_detector_profile** (31 tests):

| Suite | Tests | Description |
|-------|-------|-------------|
| `ImageAnalyzerTest` | 6 | Uniform, high contrast, noisy, empty throws, color, range validation |
| `ProfileHintsTest` | 3 | Suggest from uniform, high noise factors, clamp |
| `DetectorProfileTest` | 21 | Construction, from_hints, from_image, setters, name resolution, params for all 9 detectors, monotonicity (detail/gap/length/precision), adaptive factors, extreme values |
| `IntegrationTest` | 1 | Analyze → profile → params for all detectors |

### Python Tests (pytest)

| Class | Tests | Description |
|-------|-------|-------------|
| `TestLineMerge` | 4 | Construction, collinear merge, distant preserve, empty |
| `TestLineConnect` | 2 | Construction, empty input |
| `TestAccuracyMeasure` | 4 | Perfect match, no detections, threshold property, sAP |
| `TestGroundTruth` | 2 | make_entry, repr |
| `TestSearchStrategy` | 4 | Grid single param, grid bool, random count, reproducible |
| `TestParamOptimizer` | 2 | Construction, basic optimisation |
| `TestEnums` | 3 | MergeType, OptimMetric, DetectorId |
| `TestImageAnalyzer` | 6 | Uniform, noisy, high contrast, repr, suggest_profile, hints clamp |
| `TestDetectorProfile` | 12 | Construction, from_hints, from_image, supported_detectors, to_params, all detectors, name conversion, setters, repr, detail affects thresholds |
