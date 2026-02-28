# Python Evaluation Tools

Python utilities for processing evaluation results and optimizing LSD detector parameters.

[← Back to Evaluation](../README.md)

## Tools

All tools are in [tools/](tools/).

| Script | Description |
|--------|-------------|
| [optimize_presets.py](tools/optimize_presets.py) | Optimize LSD detector parameters over image databases to generate Fast/Balanced/Accurate presets |
| [chart_plot.py](tools/chart_plot.py) | Parse Excel chart XML files and render publication-quality charts using matplotlib |
| [csv_table.py](tools/csv_table.py) | Convert CSV files into formatted PDF tables using ReportLab (simple and scientific styles) |
| [replace_underscore.py](tools/replace_underscore.py) | Replace underscores with hyphens in LaTeX `\label{}` and `\ref{}` commands |

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

## Other Tools

```bash
# Chart plotting from Excel XML
python evaluation/python/tools/chart_plot.py chart.xml --title "Results" --output plot.png

# CSV to PDF table conversion
python evaluation/python/tools/csv_table.py results.csv --style scientific --output table.pdf

# LaTeX label/ref cleanup
python evaluation/python/tools/replace_underscore.py document.tex
```

## Dependencies

- `le_algorithm`, `le_lsd`, `le_geometry` — Native pybind11 modules (Bazel-managed)
- `numpy` — Numerical arrays
- `matplotlib` — Chart rendering
- `pandas` — Data manipulation
- `reportlab` — PDF table generation

Dependencies are managed in the project's [pyproject.toml](../../pyproject.toml) via `uv`.

## Related

- [evaluation/performance](../performance/) — C++ performance benchmarks (generates CSV results)
- [evaluation/thesis](../thesis/) — C++ precision evaluations (generates CSV and image results)
- [libs/algorithm](../../libs/algorithm/) — C++ parameter optimizer library
- [libs/algorithm/python](../../libs/algorithm/python/) — Python bindings for the optimizer
- [Main README](../../README.md) — Project overview
