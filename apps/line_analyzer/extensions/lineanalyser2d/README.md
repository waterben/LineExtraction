# Line Analyser 2D

Advanced interactive ground truth comparison with per-segment analysis.

[← Back to Line Analyzer](../../README.md)

## Overview

Advanced ground truth comparison tool with interactive visualization. Unlike the simple [Accuracy Measure](../accuracy/README.md) panel, this extension provides detailed per-segment analysis with visual feedback, making it the primary tool for understanding *which* lines are wrong and *why*.

## Key Features

- **Side-by-side visualization:** Ground truth lines and detected lines drawn in a dedicated plot window
- **Per-segment matching:** Each detected line is matched to its closest GT segment with detailed error metrics (angle difference, length difference, endpoint distances)
- **Interactive exploration:** Click on GT or detected lines in the table to highlight them in the plot
- **Debug mode:** Lock a GT line and a detected line to examine their geometric relationship in detail
- **Analysis mode:** Compare two different detection runs (current vs. saved) with percentage-change tables
- **Configurable thresholds:** Angle tolerance, distance threshold, and error bounds for the matching algorithm
- **Export:** Save analysis results to text files for external processing

## Workflow

1. Load an image and run detection
2. Open the Line Analyser 2D panel
3. File → **Open GT** to load ground truth data (separate from Accuracy Measure)
4. The table populates with per-segment matching results
5. Click on rows to highlight the corresponding segments in the plot
6. Use **Debug Mode** to lock pairs for detailed geometric inspection
7. Use **Analysis Mode** to compare against a saved detection run

## Bundled Examples

Two synthetic datasets are included so you can try the panel without external
data.  Click the toolbar buttons to load them:

| Button | Dataset | Description |
|--------|---------|-------------|
| **Easy** | `example_lines.png` / `example_gt.txt` | Single hexagon, 6 segments, no noise — trivially detectable |
| **Hard** | `example_challenge.png` / `example_challenge_gt.txt` | 8 shapes, 31 segments, varying contrast (13–88), Gaussian noise σ = 5 |

The buttons load both the ground truth file **and** the matching image into the
main window, so detection can be run immediately afterward.

The same images are also used by the [Accuracy Measure](../accuracy/README.md)
panel (CSV format).  The TXT files used here contain plain `x1,y1,x2,y2` lines
without a header.

## How Matching Works

Each detected segment is matched to its closest ground truth segment based on combined geometric error (angle, distance, endpoint proximity). The result is a detailed per-segment error table:

```
  Ground truth (GT):
  ─────●══════════════●─────  GT₁
  ────────●═══════●─────────  GT₂ (shorter, different angle)

  Detected:
  ───●════════════════●─────  D₁  →  matched to GT₁ ✓ (TP)
  ──────●═══●───────────────  D₂  →  no match (too short) ✗ (FP)
                                      GT₂ → not detected ✗ (FN)

  Per-segment error table for D₁ ↔ GT₁:
  ┌─────────────────────────────────┐
  │  Angle difference:    0.3°      │
  │  Endpoint distance₁:  1.2 px    │
  │  Endpoint distance₂:  0.8 px    │
  │  Length difference:    2.1 px   │
  └─────────────────────────────────┘
```

The matcher configurable thresholds:

- **Angle tolerance:** Maximum allowed angle difference
- **Distance threshold:** Maximum allowed perpendicular distance
- **Error bounds:** Combined metric weighting endpoint distances, angles, and lengths

Unmatched detected lines are false positives; unmatched GT lines are false negatives.

## Use Case

Detailed investigation of detection quality for research and publication:

- Understand which specific ground truth lines are missed and why
- Analyze geometric error patterns (systematic drift, rotation bias)
- Compare two detector configurations side-by-side with delta tables
- Generate per-segment error data for statistical analysis

## Files

| File | Purpose |
|------|---------|
| [lineanalyser2d.h](lineanalyser2d.h) | Main panel class declaration (inherits `LATool`) |
| [lineanalyser2d.cpp](lineanalyser2d.cpp) | GT matching, interactive visualization, analysis mode |
| [lineanalyser2d.ui](lineanalyser2d.ui) | Qt Designer layout |
| [analyseroptions.h](analyseroptions.h) | Options dialog for threshold configuration |
| [analyseroptions.cpp](analyseroptions.cpp) | Options dialog implementation |
| [analyseroptions.ui](analyseroptions.ui) | Options dialog layout |

## Dependencies

- **ControlWindow** — detected line data, image sources
- **[Precision Optimizer](../precisionoptimizer/README.md)** — imports `PrecisionOptimizer` for optional line refinement
- **libs/algorithm** — [`GroundTruthLoader`](../../../../libs/algorithm/README.md#groundtruthloader), matching algorithms
- **QCustomPlot** — 2D plot rendering for GT/detected line overlay
