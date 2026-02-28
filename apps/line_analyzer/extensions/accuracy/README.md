# Accuracy Measure

Quantitative evaluation of detected lines against ground truth.

[← Back to Line Analyzer](../../README.md)

## Overview

Evaluates detected line segments against ground truth data by computing standard detection metrics (Precision, Recall, F1 Score, sAP). Uses the [`AccuracyMeasure`](../../../../libs/algorithm/README.md#accuracymeasure) algorithm from `libs/algorithm`.

## How Matching Works

A detected segment is a **true positive** if any of its endpoints is within the Match Threshold distance of a ground truth segment's endpoints. The matcher tests both forward (det→GT) and reversed (GT→det) endpoint orderings and takes the minimum average distance.

```
  Ground truth (GT):    A ●══════════════● B

  Detected (good):      C ●═════════════● D
                          ↕ < threshold  ↕ < threshold
                          → TRUE POSITIVE (matched)

  Detected (poor):      E ●══════════● F
                          ↕ > threshold
                          → FALSE POSITIVE (no match)

  Unmatched GT:         G ●═══════● H
                          (no detected line nearby)
                          → FALSE NEGATIVE (missed)
```

## Ground Truth CSV Format

```csv
image_name,x1,y1,x2,y2
image001.png,10.5,20.3,100.2,20.8
image001.png,50.0,10.0,50.0,200.0
image002.png,5.0,5.0,295.0,195.0
```

Each row defines one ground truth line segment with sub-pixel endpoints. Multiple images can be stored in one CSV file.

## Settings

| Setting | Description | Default |
|---------|-------------|---------|
| **Match Threshold (px)** | Maximum endpoint distance for a detected segment to count as a true positive | 5.0 |
| **Image Name Filter** | Select which image entry from the CSV to evaluate. Leave empty for auto-selection | — |

## Metrics

| Metric | Formula | Description |
|--------|---------|-------------|
| **Precision** | TP / (TP + FP) | Fraction of detected lines matching ground truth |
| **Recall** | TP / (TP + FN) | Fraction of ground truth lines that were detected |
| **F1 Score** | 2 × P × R / (P + R) | Harmonic mean of precision and recall |
| **sAP** | F1 averaged at thresholds {5, 10, 15} | Structural Average Precision — considers endpoint accuracy at multiple scales |
| **TP / FP / FN** | Raw counts | True/false positives and false negatives |
| **GT Segments** | — | Total number of ground truth segments |

## Workflow

1. Load an image and run detection
2. Click **Browse...** to load a ground truth CSV
3. (Optional) Enter an image name filter
4. Click **Evaluate** to compute metrics
5. Adjust detector parameters and re-evaluate for comparison

## Use Case

Quick quantitative comparison of different detector configurations or parameter presets. Use this panel when you need P/R/F1 numbers for a report or to compare before/after. For detailed per-segment analysis, use the [Line Analyser 2D](../lineanalyser2d/README.md) extension instead.

## Algorithm

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#accuracymeasure) for the full `AccuracyMeasure` API, structural AP computation, and C++/Python usage examples.

## Files

| File | Purpose |
|------|---------|
| [accuracypanel.h](accuracypanel.h) | Panel class declaration (inherits `LATool`) |
| [accuracypanel.cpp](accuracypanel.cpp) | AccuracyMeasure + GroundTruthLoader integration |
| [accuracypanel.ui](accuracypanel.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — detected line data, image sources
- **libs/algorithm** — [`AccuracyMeasure`](../../../../libs/algorithm/README.md#accuracymeasure), [`GroundTruthLoader`](../../../../libs/algorithm/README.md#groundtruthloader)
