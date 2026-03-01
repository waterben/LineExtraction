# Profile Analyzer

Edge gradient profile visualization for detected line segments.

[← Back to Line Analyzer](../../README.md)

## Overview

Visualizes the gradient profile perpendicular to a selected line segment. This is the primary tool for understanding how edge strength varies across a detected line.

## How It Works

The analyzer samples the gradient magnitude along a cross-section perpendicular to the line at each pixel position along its length, then averages them into a single profile curve.

```
  Image with a vertical edge:

        dark │ bright
        ░░░░░│▓▓▓▓▓
        ░░░░░│▓▓▓▓▓
        ░░░░░│▓▓▓▓▓    ← detected line segment (vertical)
        ░░░░░│▓▓▓▓▓
        ░░░░░│▓▓▓▓▓

  Perpendicular profile (horizontal cross-section):

  gradient
  magnitude
      │        ╱╲
      │       ╱  ╲         ← sharp peak = true edge
      │      ╱    ╲
      │     ╱      ╲
      │────╱────────╲────
      └──────────────────→  distance from line center
         -range  0  +range
```

A **strong, narrow peak** at 0 confirms a well-localized edge. A **flat or noisy** profile suggests a false positive or weak edge.

## Visual Elements

- **Blue curve:** Mean gradient response across all positions along the line
- **Shaded band:** ±1 standard deviation of the gradient response
- **Red curve:** Smoothed profile for comparison

## Controls

| Control | Description | Default |
|---------|-------------|---------|
| **Data Source** | Choose which image source to sample (magnitude, gradient, etc.) | — |
| **Interpolation** | Sub-pixel interpolation method (nearest, bilinear, bicubic) | Bilinear |
| **Profile Range** | Half-range of the perpendicular cross-section (±pixels) | 1.0 |
| **Line Distance** | Number of support pixels along the line for averaging | 1 |
| **Subdivisions** | Sampling density per pixel in the profile (higher = smoother) | 40 |
| **Fast Interpolation** | Trade accuracy for speed in sub-pixel sampling | On |

## Workflows

### Inspecting a Single Line's Edge Profile

1. **Detect lines** in the Analyzer.
2. **Select a line** by clicking it in the main plot or table. The Profile Analyzer automatically receives the selection via `lineChanged` and plots the gradient profile.
3. **Choose a data source** from the dropdown — typically gradient magnitude (`mag`, `qmag`, or `nmag`).
4. **Adjust the profile range** (±pixels) to widen or narrow the cross-section view. A wider range reveals neighboring edges; a narrow range focuses on the primary edge.
5. **Read the plot:**
   - A **sharp, narrow peak** centered at 0 confirms a well-localized edge.
   - A **broad or shifted peak** indicates the line is slightly misaligned — consider running the [Precision Optimizer](../precision_optimizer/README.md).
   - A **flat or noisy** profile suggests a false positive or very weak edge.

### Scanning Along the Line (Single-Point Profile)

1. **Enable the single-point profile** by checking `chb_single` (the red curve checkbox).
2. **Drag the position slider** (or set the spin box) to move the sample point along the line's length. The red curve shows the cross-section at that specific position, and a green indicator line appears in the Analyzer's plot at the sample location.
3. **Compare** the red (single-point) and blue (mean) curves: if they diverge significantly, the edge strength varies along the line — the line may span a region where the edge fades or another edge interferes.

### Verifying Detection Quality

1. **Detect lines and open the Profile Analyzer.**
2. **Click through several lines** in the main table, checking each profile:
   - Lines with strong, centered peaks → good detections.
   - Lines with weak or multi-peaked profiles → candidates for removal or investigation.
3. **Toggle the ±1σ band** (`chb_std_dev`) to see gradient variance. High variance means the edge strength fluctuates along the line.

### Comparing Before/After Optimization

1. **Select a line** and note its profile shape.
2. **Run the [Precision Optimizer](../precision_optimizer/README.md)** on that line.
3. **Re-select the same line** — the Profile Analyzer updates automatically. The peak should now be sharper and better centered at 0.

## Use Case

Verify that a detected line sits on a true edge (sharp peak in the profile) and diagnose false positives (flat or noisy profiles). A strong, narrow peak indicates a well-localized edge; a broad or noisy profile suggests the line may not correspond to a meaningful image feature.

## Files

| File | Purpose |
|------|---------|
| [profile_analyzer.h](profile_analyzer.h) | Panel class declaration (inherits `LATool`) |
| [profile_analyzer.cpp](profile_analyzer.cpp) | Edge profile extraction, sub-pixel sampling, plot rendering |
| [profile_analyzer.ui](profile_analyzer.ui) | Qt Designer layout |

## Dependencies

- **Analyzer** — line selection, image sources, indicator overlay
- **QCustomPlot** — 2D plot rendering
- **libs/imgproc** — gradient computation and interpolation
