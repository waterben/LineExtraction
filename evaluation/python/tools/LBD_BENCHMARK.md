# LBD Descriptor Benchmark & Parameter Sweep

Standalone command-line tools for evaluating LBD (Line Band Descriptor) matching
accuracy and comparing our float-based LBD implementation against OpenCV's
binary LBD (`BinaryDescriptor`) on the HPatches benchmark.

[← Back to Evaluation](../README.md) · [Benchmark Report](../reports/lbd_benchmark_report.md)

---

## Table of Contents

1. [Overview](#overview)
2. [Background](#background)
3. [Architecture](#architecture)
4. [Prerequisites](#prerequisites)
5. [lbd_benchmark.py — Accuracy & Runtime Benchmark](#lbd_benchmarkpy--accuracy--runtime-benchmark)
6. [lbd_param_sweep.py — Parameter Grid Search](#lbd_param_sweeppy--parameter-grid-search)
7. [Evaluation Methodology](#evaluation-methodology)
8. [Output Format](#output-format)
9. [Key Results](#key-results)
10. [Extending the Benchmark](#extending-the-benchmark)
11. [Related Resources](#related-resources)

---

## Overview

Two scripts live in this directory:

| Script | Purpose | Bazel Target |
|--------|---------|--------------|
| [`lbd_benchmark.py`](lbd_benchmark.py) | Evaluate matching accuracy and runtime of both LBD variants across HPatches | `//evaluation/python:lbd_benchmark` |
| [`lbd_param_sweep.py`](lbd_param_sweep.py) | Compare different `numBand`/`widthBand` configurations to find the best descriptor parameterization | `//evaluation/python:lbd_param_sweep` |

Both scripts:

- Are fully self-contained CLI tools with `argparse` interfaces.
- Use lazy pybind11 imports (run-safe outside Bazel for linting/type checking).
- Follow the project's Python conventions (Sphinx docstrings, type hints, American English).
- Produce human-readable tables on stdout.
- Require the **HPatches** dataset (see [Prerequisites](#prerequisites)).

---

## Background

### The LBD Descriptor

The **Line Band Descriptor** (LBD) characterizes a line segment by sampling
gradient information in a set of rectangular bands perpendicular to the line.
Two parameters control the sampling:

| Parameter   | Description | Default |
|-------------|-------------|---------|
| `numBand`   | Number of bands along the line segment (longitudinal resolution) | 9 |
| `widthBand` | Pixel width of each band (perpendicular extent) | 7 |

Our implementation (`FdcLBD`) produces a **float descriptor** matched with
**L2 distance**, while OpenCV's `BinaryDescriptor` (`FdcOpenCVLBD`) produces
a **binary descriptor** matched with **Hamming distance**.

### Why Two Implementations?

| Property | Our LBD (`FdcLBD`) | OpenCV LBD (`FdcOpenCVLBD`) |
|----------|--------------------|-----------------------------|
| Descriptor type | Float (continuous) | Binary (256-bit) |
| Distance metric | L2 (Euclidean) | Hamming |
| Gradient source | LSD native Sobel (int16 → float32) | Computed internally from grayscale image |
| Normalization | Component-wise L2 per band pair | Binarized via thresholding |
| Configurable bands | Yes (`numBand`, `widthBand`) | Fixed internally |
| Typical accuracy | Higher (see [results](#key-results)) | Lower, but faster descriptor computation |

### The HPatches Benchmark

[HPatches](https://github.com/hpatches/hpatches-dataset) (Balntas et al.,
CVPR 2017) is a standard benchmark for local feature evaluation. It contains
116 sequences, each with 6 images and 5 ground-truth homographies (`H_1_2`
through `H_1_6`). Higher indices represent more extreme transformations.

We evaluate on a **curated subset of 10 architectural viewpoint sequences**
(`HPATCHES_LINE_SEQUENCES` from `lsfm.data`) chosen for their abundance
of line structure:

| Sequence | Scene Description |
|----------|-------------------|
| `v_london` | London skyline |
| `v_coffeehouse` | Coffeehouse facade |
| `v_underground` | Underground station |
| `v_yard` | Courtyard with walls |
| `v_grace` | Grace church |
| `v_wall` | Stone wall texture |
| `v_churchill` | Churchill statue |
| `v_gardens` | Formal gardens |
| `v_artisans` | Artisan building |
| `v_wapping` | Wapping waterfront |

---

## Architecture

```
evaluation/python/
├── BUILD.bazel                    # Bazel py_binary targets
├── README.md                      # Evaluation tools overview
├── reports/
│   └── lbd_benchmark_report.md    # Latest benchmark results
└── tools/
    ├── LBD_BENCHMARK.md           # This file
    ├── lbd_benchmark.py           # Core benchmark script
    └── lbd_param_sweep.py         # Parameter sweep (imports from lbd_benchmark)
```

### Module Dependencies

```
lbd_param_sweep.py
    │
    ├── lbd_benchmark.evaluate_pair()   # Shared core evaluation function
    └── lbd_benchmark._import_modules() # Shared lazy imports
```

The `lbd_param_sweep` Bazel target includes both `.py` files in `srcs` so the
import `from evaluation.python.tools.lbd_benchmark import ...` resolves correctly
via Bazel's Python package structure.

### Key Functions

**`evaluate_pair(images, seq_name, target_idx, **kwargs) → dict | None`**

The central evaluation function, reused by both scripts. For a single
HPatches image pair:

1. Loads reference image (`1.ppm`) and target image, plus ground-truth homography.
2. Downscales both images to `max_dim` and adjusts `H` accordingly.
3. Runs `LsdCC` line segment detection on both images.
4. Computes **Our LBD** descriptors from LSD's native gradient maps (`float32`).
5. Computes **OpenCV LBD** descriptors from grayscale images.
6. Matches both descriptor sets: forward + backward brute force.
7. Applies **cross-check** and **Lowe's ratio test** filters (Our LBD only).
8. Applies **cross-check** filter (OpenCV LBD).
9. Evaluates top-K matches against the ground-truth homography.
10. Returns accuracy metrics and per-stage timing.

**`_classify_matches(match_list, segs_r, segs_t, H, thresh) → (errors, correct)`**

Classifies each match by projecting the reference segment center through `H`
and measuring Euclidean distance to the target segment center.

**`_match_error(seg_r, seg_t, H) → float`**

Computes the reprojection error for a single segment match.

---

## Prerequisites

### 1. HPatches Dataset

```bash
./tools/scripts/setup_hpatches.sh
```

This downloads all 116 sequences (~1.6 GB) to `resources/datasets/HPatches/`.
The benchmark only uses the 10 curated viewpoint sequences listed above.

### 2. Python Bindings

The scripts depend on `le_lsd` and `le_lfd` pybind11 modules, which are
built automatically when running through Bazel:

```bash
bazel build //libs/lsd/python:le_lsd_lib //libs/lfd/python:le_lfd_lib
```

### 3. Python Packages

Managed by Bazel via `@pip//numpy` and `@pip//pillow`. No manual installation
needed when using `bazel run`.

---

## lbd_benchmark.py — Accuracy & Runtime Benchmark

### What It Does

Evaluates both LBD implementations across all 10 curated sequences and all
5 difficulty levels (idx 2–6), producing:

- Per-sequence accuracy table (Our LBD vs OpenCV LBD)
- Overall mean/median accuracy
- Per-difficulty-level breakdown
- Average per-pair runtime comparison (descriptor computation + matching)

### Usage

```bash
# Full benchmark: 10 sequences × 5 difficulty levels = 50 pairs
bazel run //evaluation/python:lbd_benchmark

# Restrict difficulty range (faster)
bazel run //evaluation/python:lbd_benchmark -- --idx-range 2 4

# Use best known parameters
bazel run //evaluation/python:lbd_benchmark -- --num-band 11 --width-band 7

# Evaluate more matches per pair
bazel run //evaluation/python:lbd_benchmark -- --top-k 100

# Full custom configuration
bazel run //evaluation/python:lbd_benchmark -- \
    --idx-range 2 6 \
    --max-dim 600 \
    --min-len 20 \
    --ratio-thresh 0.80 \
    --gt-threshold 10.0 \
    --top-k 50 \
    --num-band 11 \
    --width-band 7
```

### CLI Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--idx-range MIN MAX` | int int | `2 6` | Homography difficulty range (inclusive). Higher = harder viewpoint change. |
| `--max-dim` | int | `800` | Downscale longest image side to this many pixels. |
| `--min-len` | int | `15` | Discard line segments shorter than this (pixels). |
| `--ratio-thresh` | float | `0.85` | Lowe's ratio test threshold. Lower = stricter filtering. |
| `--gt-threshold` | float | `15.0` | Reprojection error threshold (pixels) for "correct" classification. |
| `--top-k` | int | `30` | Number of top-ranked matches to evaluate per pair. |
| `--num-band` | int | `9` | LBD `numBand` parameter (number of longitudinal bands). |
| `--width-band` | int | `7` | LBD `widthBand` parameter (perpendicular pixel width). |

### Example Output

```
12:37:14 [INFO] LBD Benchmark — numBand=9, widthBand=7, idx 2-6
.....  v_london
.....  v_coffeehouse
....x  v_underground
...

50 pairs evaluated in 550.7s.

Sequence           LBD mean  OCV mean  pairs
--------------------------------------------
v_london             100.0%     92.0%      5
v_coffeehouse         93.3%     78.0%      5
...

============================================
Overall (n=50 pairs):
  Our LBD:    mean=79.2%  median=88.3%
  OpenCV LBD: mean=67.9%  median=70.0%

Per difficulty level:
  idx=2: LBD=92.0%  OCV=91.3%  (n=10)
  idx=3: LBD=79.7%  OCV=65.3%  (n=10)
  ...

Average runtime per pair (ms):
                               Our LBD  OpenCV LBD
  --------------------------------------------------
  Descriptor computation       663.2ms     183.3ms
  Matching (fwd+bwd+knn)      5160.4ms    4740.3ms
  Total                       5823.6ms    4923.6ms
```

The progress dots show real-time status: `.` = ≥50% accuracy, `x` = <50%, `-` = skipped.

---

## lbd_param_sweep.py — Parameter Grid Search

### What It Does

Tests multiple `numBand × widthBand` configurations on a subset of difficulty
levels to find the best descriptor parameterization. Uses the same `evaluate_pair`
core as the benchmark.

### Usage

```bash
# Default: 6 configs × 10 sequences × 3 levels = 180 evaluations
bazel run //evaluation/python:lbd_param_sweep

# Custom parameter grid
bazel run //evaluation/python:lbd_param_sweep -- \
    --params "9,7" "11,7" "13,9" "7,5"

# Full difficulty range (slower but more thorough)
bazel run //evaluation/python:lbd_param_sweep -- --idx-range 2 6

# Custom max dimension and top-k
bazel run //evaluation/python:lbd_param_sweep -- \
    --max-dim 600 --top-k 50
```

### CLI Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--params NB,WB [...]` | string | `9,7 7,5 7,7 9,5 5,5 11,7` | Parameter combinations as comma-separated `numBand,widthBand` pairs. |
| `--idx-range MIN MAX` | int int | `2 4` | Difficulty range. Default is 2–4 to keep runtime reasonable. |
| `--max-dim` | int | `800` | Downscale longest image side to this many pixels. |
| `--top-k` | int | `30` | Number of top-ranked matches to evaluate per pair. |

### Default Parameter Grid

| numBand | widthBand | Note |
|--------:|----------:|------|
| 9 | 7 | Library default |
| 7 | 5 | Used in `match_test.cpp` |
| 7 | 7 | — |
| 9 | 5 | — |
| 5 | 5 | Minimal configuration |
| 11 | 7 | Extended longitudinal sampling |

### Example Output

```
12:46:40 [INFO] LBD Parameter Sweep — 6 configs, idx 2-4, 10 sequences
numBand widBand    mean  median      n      time
--------------------------------------------------
      9       7   85.2%   91.7%     30   335.6s
      7       5   84.2%   90.0%     30   323.7s
      7       7   85.0%   91.7%     30   328.2s
      9       5   85.6%   95.0%     30   330.4s
      5       5   79.3%   86.7%     30   313.9s
     11       7   87.2%   93.3%     30   337.5s

Best config: numBand=11, widthBand=7 (mean=87.2%)
Total time: 1969.4s
```

---

## Evaluation Methodology

### Matching Pipeline

Both descriptors use the same matching pipeline to ensure a fair comparison:

```
Line Segments (LsdCC)
        │
        ▼
┌───────────────┐    ┌───────────────────┐
│  Our LBD      │    │  OpenCV LBD       │
│  FdcLBD       │    │  FdcOpenCVLBD     │
│  (float, L2)  │    │  (binary, Hamming)│
└───────┬───────┘    └────────┬──────────┘
        │                     │
        ▼                     ▼
   BruteForceLBD         BruteForceOpenCVLBD
   Forward + Backward    Forward + Backward
        │                     │
        ▼                     ▼
   Cross-check filter    Cross-check filter
        │                     │
        ▼                     │
   Ratio test filter          │
   (nn0/nn1 < 0.85)          │
        │                     │
        ▼                     ▼
   Sort by distance      Sort by distance
        │                     │
        ▼                     ▼
   Top-K evaluation      Top-K evaluation
```

### Filtering Stages

1. **Minimum length filter**: Segments shorter than `min_len` are excluded.
2. **Cross-check**: A match (i→j) is kept only if j's best match is also i.
3. **Ratio test** (Our LBD only): The ratio of the best to second-best
   distance must be below `ratio_thresh`. This is Lowe's ratio test,
   a standard filter for discriminative matches.

> **Note:** The ratio test is applied only to Our LBD because it uses
> continuous L2 distances where the ratio is meaningful. OpenCV's binary
> Hamming distances have lower discriminative power for ratio testing.

### Accuracy Metric

For each match pair (reference segment i, target segment j):

1. Project the center of segment i through the ground-truth homography H.
2. Compute the Euclidean distance to the center of segment j.
3. If the distance is below `gt_threshold` (default: 15 pixels), the match
   is classified as **correct**.

**Top-K accuracy** = (number of correct matches in top K) / K × 100%.

### Runtime Measurement

Each evaluation pair records four timing measurements using `time.perf_counter()`:

| Measurement | What Is Timed |
|-------------|---------------|
| `t_lbd_desc` | `FdcLBD.create_list()` for reference + target |
| `t_lbd_match` | `BruteForceLBD.train()` + `.best()` + `.knn(2)` for forward + backward |
| `t_ocv_desc` | `FdcOpenCVLBD.create_list()` for reference + target |
| `t_ocv_match` | `BruteForceOpenCVLBD.train()` + `.best()` for forward + backward |

---

## Output Format

### Per-Pair Return Dict

The `evaluate_pair()` function returns a dict (or `None` if the pair is
unavailable or has too few segments):

```python
{
    "seq": "v_london",     # Sequence name
    "idx": 3,              # Difficulty level
    "lbd_acc": 100.0,      # Our LBD top-K accuracy (%)
    "lbd_ok": 30,          # Number of correct matches (Our LBD)
    "lbd_n": 30,           # Number of evaluated matches (Our LBD)
    "lbd_matches": 85,     # Total matches after filtering (Our LBD)
    "ocv_acc": 93.3,       # OpenCV LBD top-K accuracy (%)
    "ocv_ok": 28,          # Number of correct matches (OpenCV)
    "ocv_n": 30,           # Number of evaluated matches (OpenCV)
    "ocv_matches": 120,    # Total matches after filtering (OpenCV)
    "t_lbd_desc": 0.65,    # Descriptor computation time in seconds (Our LBD)
    "t_lbd_match": 5.1,    # Matching time in seconds (Our LBD)
    "t_ocv_desc": 0.18,    # Descriptor computation time in seconds (OpenCV)
    "t_ocv_match": 4.7,    # Matching time in seconds (OpenCV)
}
```

### Progress Indicator

During execution, each sequence prints a row of characters:

| Symbol | Meaning |
|--------|---------|
| `.` | Pair evaluated, Our LBD accuracy ≥ 50% |
| `x` | Pair evaluated, Our LBD accuracy < 50% |
| `-` | Pair skipped (data not found or <5 segments) |

---

## Key Results

See [lbd_benchmark_report.md](../reports/lbd_benchmark_report.md) for the full
report. Summary of the latest run (2026-03-04):

### Accuracy

| Metric | Our LBD | OpenCV LBD | Advantage |
|--------|--------:|-----------:|----------:|
| Mean   |  79.2%  |     67.9%  | **+11.3 pp** |
| Median |  88.3%  |     70.0%  | **+18.3 pp** |

- Our LBD wins on **9 / 10** sequences.
- Advantage grows with difficulty: +0.7 pp at idx=2 → +16.5 pp at idx=5.
- The only loss is `v_wapping` (extreme perspective), where both struggle.

### Runtime

| Stage | Our LBD | OpenCV LBD | Ratio |
|-------|--------:|-----------:|------:|
| Descriptor | 663 ms | 183 ms | 3.6× |
| Matching | 5160 ms | 4740 ms | 1.1× |
| **Total** | **5824 ms** | **4924 ms** | **1.2×** |

Matching dominates overall runtime. The 3.6× descriptor speed difference
has limited impact on total time.

### Best Parameters

| numBand | widthBand | Mean Accuracy |
|--------:|----------:|--------------:|
| **11** | **7** | **87.2%** |
| 9 | 5 | 85.6% |
| 9 | 7 | 85.2% |
| 7 | 7 | 85.0% |
| 7 | 5 | 84.2% |
| 5 | 5 | 79.3% |

`numBand=11, widthBand=7` provides +2.0 pp over the default (9, 7).

---

## Extending the Benchmark

### Adding New Sequences

Edit `HPATCHES_LINE_SEQUENCES` in [`python/lsfm/data.py`](../../../python/lsfm/data.py)
to include additional HPatches sequences. All 59 viewpoint sequences are
available in `HPATCHES_VIEWPOINT_SEQUENCES`.

### Adding New Descriptors

To benchmark a third descriptor variant:

1. Add computation + matching in `evaluate_pair()` following the existing
   `ocv_*` pattern.
2. Add new keys to the return dict (e.g. `"new_acc"`, `"t_new_desc"`).
3. Update the `main()` reporting sections.

### Using as a Library

The `evaluate_pair()` function is importable for use in other scripts:

```python
from evaluation.python.tools.lbd_benchmark import evaluate_pair, _import_modules
from lsfm.data import TestImages

_import_modules()
images = TestImages()

result = evaluate_pair(
    images,
    "v_london",
    target_idx=3,
    num_band=11,
    width_band=7,
    top_k=50,
)
if result:
    print(f"Our LBD: {result['lbd_acc']:.1f}%")
    print(f"OpenCV:  {result['ocv_acc']:.1f}%")
```

---

## Related Resources

- [Benchmark Report](../reports/lbd_benchmark_report.md) — Full results with tables
- [Demo Notebook](../../../examples/notebooks/demo_line_features.ipynb) — Interactive
  single-pair visualization with Rerun
- [LFD Library](../../../libs/lfd/README.md) — C++ LBD implementation
- [LFD Python Bindings](../../../libs/lfd/python/README.md) — `le_lfd` module reference
- [HPatches Dataset](https://github.com/hpatches/hpatches-dataset) — Original benchmark
- [HPatches Setup Script](../../../tools/scripts/setup_hpatches.sh) — Download script
- [lsfm.data Module](../../../python/lsfm/data.py) — `TestImages`, `HPATCHES_LINE_SEQUENCES`
- [Evaluation README](../README.md) — All evaluation tools overview
