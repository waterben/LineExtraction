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

## Use Case

Verify that a detected line sits on a true edge (sharp peak in the profile) and diagnose false positives (flat or noisy profiles). A strong, narrow peak indicates a well-localized edge; a broad or noisy profile suggests the line may not correspond to a meaningful image feature.

## Files

| File | Purpose |
|------|---------|
| [profileanalyzer.h](profileanalyzer.h) | Panel class declaration (inherits `LATool`) |
| [profileanalyzer.cpp](profileanalyzer.cpp) | Edge profile extraction, sub-pixel sampling, plot rendering |
| [profileanalyzer.ui](profileanalyzer.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — line selection, image sources, indicator overlay
- **QCustomPlot** — 2D plot rendering
- **libs/imgproc** — gradient computation and interpolation
