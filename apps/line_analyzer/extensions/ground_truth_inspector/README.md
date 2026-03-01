# GT Inspector

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

## Workflows

### Basic Ground Truth Comparison

1. **Load an image** in the main Analyzer (browse or pick from the Test Images dropdown).
2. **Select a detector** from the detector dropdown and click **Process** to run line detection.
3. **Open the GT Inspector** panel from the toolbar / menu.
4. **Load ground truth data:**
   - Click **Open GT** and select a `.txt` file with `x1,y1,x2,y2` lines, *or*
   - Click the **Easy** / **Hard** button to load a bundled example (image + GT are loaded together so you can skip steps 1–2).
5. **Set matching thresholds** on the Test page:
   - *Distance Threshold* — maximum perpendicular distance (px) for a match.
   - *Angle Threshold* — maximum angle difference (°) for a match.
   - *Error Threshold* — combined metric bound for endpoint distances, angles, and lengths.
6. **Click "Compute Correct Lines"** to run the matcher. The GT table fills with per-segment results; matched lines are drawn in green, unmatched GT lines in red, and false positive detections in orange.
7. **Explore the results:**
   - Click a row in the **GT table** to see which detected lines matched that GT segment.
   - Click a row in the **matched-lines table** below to highlight the specific detected segment in the plot.
   - Click directly on a line in the plot — the panel identifies whether it is a GT, matched, or detected line and selects the corresponding table row.

### Debug Mode — Pairwise Geometric Inspection

1. **Complete the Basic GT Comparison** workflow above so that matched lines are available.
2. Switch to the **Debug** page.
3. **Click "Start Debug"** to enable debug mode.
4. **Lock a GT line:** Click a GT line in the plot or table, then click **Lock GT Line**. The locked line is highlighted and its geometry is shown in the result area.
5. **Lock a detected line:** Click a detected/matched line in the plot or table, then click **Lock Other Line**.
6. **View the pairwise metrics** that appear automatically: angle difference, perpendicular distance, endpoint distances (start↔start, start↔end, end↔start, end↔end), and length difference.
7. **Repeat** with different pairs by locking a new GT or detected line at any time.
8. Click **Reset** to clear locked selections, or **Stop Debug** to leave debug mode.

### Analysis Mode — Before/After Comparison

Use this workflow to compare detection results before and after an optimization step (e.g., Precision Optimizer):

1. **Detect lines and compute correct lines** as in the Basic workflow.
2. Switch to the **Analysis** page and click **Start Analysis**.
3. **Freeze the current state:** Click **Freeze All** (calls `freezeAllLines()` on the Analyzer). The current matched lines are saved as the "old" set.
4. **Optimize:** Open the [Precision Optimizer](../precision_optimizer/README.md) and run **Optimize All**, or apply any other modification to the lines.
5. **Observe the update:** When Analyzer emits `linesUpdated`, the panel automatically re-computes correct lines for the new set and populates two side-by-side tables:
   - **Current** — the latest matched lines after optimization.
   - **Old** — the frozen matched lines before optimization.
   - Each row shows the percentage change in error metrics (▲ / ▼).
6. **Click rows** in either table to highlight the corresponding line in the plot and compare visually.
7. **Save the analysis** by clicking **Save** — exports a LaTeX-formatted `.txt` file with both tables for publications.
8. Click **Stop Analysis** to leave analysis mode.

### Exporting Results

1. After computing correct lines (Basic workflow) or completing an analysis comparison, click **Save**.
2. Choose a filename and location in the file dialog.
3. The export file contains all per-segment error metrics in LaTeX table format, ready for inclusion in papers.

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

<!-- help:start-ignore -->
## Files

| File | Purpose |
|------|---------|
| [ground_truth_inspector.h](ground_truth_inspector.h) | Main panel class declaration (inherits `LATool`) |
| [ground_truth_inspector.cpp](ground_truth_inspector.cpp) | GT matching, interactive visualization, analysis mode |
| [ground_truth_inspector.ui](ground_truth_inspector.ui) | Qt Designer layout |
| [analyser_options.h](analyser_options.h) | Options dialog for threshold configuration |
| [analyser_options.cpp](analyser_options.cpp) | Options dialog implementation |
| [analyser_options.ui](analyser_options.ui) | Options dialog layout |

## Dependencies

- **Analyzer** — detected line data, image sources
- **[Precision Optimizer](../precision_optimizer/README.md)** — imports `PrecisionOptimizer` for optional line refinement
- **libs/algorithm** — [`GroundTruthLoader`](../../../../libs/algorithm/README.md#groundtruthloader), matching algorithms
- **QCustomPlot** — 2D plot rendering for GT/detected line overlay
<!-- help:end-ignore -->
