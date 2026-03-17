# Python Evaluation Tools

Python utilities for processing evaluation results and optimizing LSD detector parameters.

[← Back to Evaluation](../README.md)

## Tools

All tools are in [tools/](tools/).

| Script | Description |
|--------|-------------|
| [optimize_presets.py](tools/optimize_presets.py) | Optimize LSD detector parameters over image databases to generate Fast/Balanced/Accurate presets |
| [detector_statistics.py](tools/detector_statistics.py) | Compute detection statistics for LSD detectors across benchmark datasets |
| [lbd_benchmark.py](tools/lbd_benchmark.py) | Benchmark LBD descriptor matching accuracy + runtime across HPatches sequences |
| [lbd_param_sweep.py](tools/lbd_param_sweep.py) | Parameter sweep for LBD `numBand` / `widthBand` configuration |
| [chart_plot.py](tools/chart_plot.py) | Parse Excel chart XML files and render publication-quality charts using matplotlib |
| [csv_table.py](tools/csv_table.py) | Convert CSV files into formatted PDF tables using ReportLab (simple and scientific styles) |
| [replace_underscore.py](tools/replace_underscore.py) | Replace underscores with hyphens in LaTeX `\label{}` and `\ref{}` commands |
| [reconstruction_benchmark.py](tools/reconstruction_benchmark.py) | Benchmark 3D line reconstruction pipeline: detection, matching, and triangulation on MDB stereo pairs |
| [diag_stereo.py](tools/diag_stereo.py) | Diagnose stereo matching in `reconstruct_lines_stereo` on a single MDB scene |

## Preset Optimization

The `optimize_presets.py` script runs parameter optimization for all 9 LSD detectors using
ground truth line segment annotations. Three presets per detector are generated:

- **Fast** — optimized for maximum Precision (fewer, confident lines)
- **Balanced** — optimized for maximum F1 (harmonic mean of P and R)
- **Accurate** — optimized for maximum Recall (find all lines)

### Prerequisites

Download at least one dataset with ground truth annotations:

```bash
# York Urban Line Segment Database (102 images, recommended)
./tools/scripts/setup_york_urban.sh

# Wireframe Dataset (462+ images, larger)
./tools/scripts/setup_wireframe.sh
```

### Usage

```bash
# Quick test on one detector (50 random samples)
bazel run //evaluation/python:optimize_presets -- \
    --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \
    --image-dir resources/datasets/YorkUrban/images \
    --detectors LsdFGioi --samples 50

# Full optimization on all 9 detectors (300 samples each)
bazel run //evaluation/python:optimize_presets -- \
    --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \
    --image-dir resources/datasets/YorkUrban/images \
    --samples 300 --output lsd_presets.json

# Combined datasets
bazel run //evaluation/python:optimize_presets -- \
    --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \
                   resources/datasets/ground_truth/wireframe_gt.csv \
    --image-dir resources/datasets/YorkUrban/images \
                resources/datasets/Wireframe/images \
    --samples 300
```

### Output

JSON file with optimized parameters per detector and profile:

```json
{
  "metadata": { "num_images": 102, "num_samples": 300, ... },
  "detectors": {
    "LsdCC": {
      "fast":     { "params": { "nms_th_low": 0.008, ... }, "score": 0.85 },
      "balanced": { "params": { "nms_th_low": 0.004, ... }, "score": 0.72 },
      "accurate": { "params": { "nms_th_low": 0.002, ... }, "score": 0.90 }
    },
    ...
  }
}
```

## LBD Descriptor Benchmark

The `lbd_benchmark.py` script evaluates both the internal float LBD and OpenCV's
binary LBD across all curated architectural HPatches sequences, reporting accuracy
and runtime per difficulty level.

> **Extensive documentation:** See [`tools/LBD_BENCHMARK.md`](tools/LBD_BENCHMARK.md)
> for full architecture details, methodology, output format, and extension guide.
>
> **Latest results:** See [`reports/lbd_benchmark_report.md`](reports/lbd_benchmark_report.md).

### Prerequisites

```bash
./tools/scripts/setup_hpatches.sh
```

### Usage

```bash
# Run full benchmark (10 sequences x 5 difficulty levels = 50 pairs)
bazel run //evaluation/python:lbd_benchmark

# Custom difficulty range and descriptor parameters
bazel run //evaluation/python:lbd_benchmark -- \
    --idx-range 2 4 --num-band 11 --width-band 7 --top-k 50
```

## LBD Parameter Sweep

The `lbd_param_sweep.py` script tests multiple `numBand` / `widthBand`
combinations and reports accuracy for each configuration.

### Usage

```bash
# Default 6 configurations on idx 2-4 (30 pairs per config)
bazel run //evaluation/python:lbd_param_sweep

# Custom configurations and full difficulty range
bazel run //evaluation/python:lbd_param_sweep -- \
    --params 9,7 11,7 13,9 --idx-range 2 6
```

## Other Tools

```bash
# Chart plotting from Excel XML
python evaluation/python/tools/chart_plot.py chart.xml --title "Results" --output plot.png

# CSV to PDF table conversion
python evaluation/python/tools/csv_table.py results.csv --style scientific --output table.pdf

# LaTeX label/ref cleanup
python evaluation/python/tools/replace_underscore.py document.tex
```

## 3D Line Reconstruction Benchmark

The `reconstruction_benchmark.py` script evaluates the full stereo line
reconstruction pipeline on Middlebury stereo pairs, measuring detection
throughput, matching quality, and triangulation accuracy.

### Prerequisites

```bash
./tools/scripts/setup_mdb_dataset.sh
```

### Usage

```bash
# Quick benchmark on default MDB scenes (Half resolution)
bazel run //evaluation/python:reconstruction_benchmark

# Select specific scenes
bazel run //evaluation/python:reconstruction_benchmark -- \
    --scenes Adirondack Jadeplant Piano

# Compare triangulation methods
bazel run //evaluation/python:reconstruction_benchmark -- \
    --methods midpoint plane opencv
```

## Stereo Matching Diagnostics

The `diag_stereo.py` script is a developer diagnostic tool that runs stereo
matching on a single MDB scene and prints summary statistics for inspecting
filter and matcher behaviour during development.

### Usage

```bash
bazel run //evaluation/python:diag_stereo
```

## Dependencies

- `le_algorithm`, `le_lsd`, `le_lfd`, `le_geometry` — Native pybind11 modules (Bazel-managed)
- `lsfm` — Python package with `data`, `reconstruction`, and `synthetic` utilities (in `python/lsfm/`)
- `numpy` — Numerical arrays
- `Pillow` — Image loading (LBD scripts)
- `matplotlib` — Chart rendering
- `pandas` — Data manipulation
- `reportlab` — PDF table generation

Dependencies are managed in the project's [pyproject.toml](../../pyproject.toml) via `uv`.

## Related

- [LBD Benchmark Documentation](tools/LBD_BENCHMARK.md) — Extensive documentation for the LBD benchmark and comparison scripts
- [LBD Benchmark Report](reports/lbd_benchmark_report.md) — Latest benchmark results
- [evaluation/performance](../performance/) — C++ performance benchmarks (generates CSV results)
- [evaluation/thesis](../thesis/) — C++ precision evaluations (generates CSV and image results)
- [libs/lfd](../../libs/lfd/) — C++ LBD descriptor implementation
- [libs/algorithm](../../libs/algorithm/) — C++ parameter optimizer library
- [libs/algorithm/python](../../libs/algorithm/python/) — Python bindings for the optimizer
- [Resources](../../resources/README.md) — Datasets, ground truth annotations, presets
- [Main README](../../README.md) — Project overview
