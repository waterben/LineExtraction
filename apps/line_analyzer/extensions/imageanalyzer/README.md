# Image Analyzer

Quantitative image property analysis and detector profile suggestions.

[← Back to Line Analyzer](../../README.md)

## Overview

Analyzes the loaded source image to compute quantitative properties and suggest optimal detector profile settings. This is a read-only diagnostic panel — it does not modify lines or detector parameters directly. Uses the [`ImageAnalyzer`](../../../../libs/algorithm/README.md#imageanalyzer) from `libs/algorithm`.

## How It Works

The analyzer computes four normalized properties from the source image:

```
  Input image → ImageAnalyzer::analyze()

  ┌────────────────────────────────────────────┐
  │  Contrast     = σ(intensity) / 127.5       │ ← intensity spread
  │  Noise Level  = MAD(Laplacian) / scale      │ ← high-freq content
  │  Edge Density = #edge_pixels / #total        │ ← Sobel threshold
  │  Dynamic Range = (P95 - P5) / 255           │ ← histogram spread
  └────────────────────────────────────────────┘
        ↓
  suggest_profile()  →  ProfileHints
        ↓
  Detail, Gap Tol, Min Length, Precision, Contrast Factor, Noise Factor
```

## Image Properties

| Property | Description | Range |
|----------|-------------|-------|
| **Contrast** | Michelson contrast: (max−min)/(max+min) of pixel intensities | 0–1 |
| **Noise Level** | Estimated noise standard deviation via Median Absolute Deviation of high-frequency wavelet coefficients | ≥ 0 |
| **Edge Density** | Fraction of pixels classified as edges relative to total pixels | 0–1 |
| **Dynamic Range** | Ratio of the effective intensity range to the theoretical maximum (typically 255) | 0–1 |

## Suggested Profile

Based on the computed image properties, the panel suggests values for the [Detector Profile](../detectorprofile/README.md) panel:

| Suggestion | Description | Range |
|------------|-------------|-------|
| **Detail** | Increases with contrast, decreases with noise | 10–90% |
| **Gap Tolerance** | Increases with noise, decreases with edge density | 10–90% |
| **Min Length** | Increases with noise and edge density | 10–90% |
| **Precision** | Increases with dynamic range, decreases with noise | 10–90% |
| **Contrast Factor** | > 1 for low-contrast images (raises thresholds) | 0.5–2.0 |
| **Noise Factor** | > 1 for noisy images (raises thresholds) | 0.5–2.0 |

## Operations

| Button | Action |
|--------|--------|
| **Analyze** | Run image analysis and update all property and suggestion fields |

## Use Case

Understand the characteristics of the current image before configuring detector parameters. The suggested profile values can be manually entered into the Detector Profile panel, or you can use the Detector Profile panel's **Auto from Image** button which performs the same analysis internally.

## Algorithm

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#imageanalyzer) for the full `ImageAnalyzer` API, property derivation logic, and `ProfileHints` suggestion heuristics.

## Files

| File | Purpose |
|------|---------|
| [imageanalyzerpanel.h](imageanalyzerpanel.h) | Panel class declaration (inherits `LATool`) |
| [imageanalyzerpanel.cpp](imageanalyzerpanel.cpp) | ImageAnalyzer integration, property display |
| [imageanalyzerpanel.ui](imageanalyzerpanel.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — source image access, image change signals
- **libs/algorithm** — [`ImageAnalyzer`](../../../../libs/algorithm/README.md#imageanalyzer), `ImageProperties`, `ProfileHints`
