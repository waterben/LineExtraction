# Detector Profile

High-level intuitive parameter tuning for LSD detectors.

[← Back to Line Analyzer](../../README.md)

## Overview

Provides high-level detector parameter tuning via 4 percentage sliders and 2 adaptive factors. Translates intuitive settings into concrete detector parameters via [`DetectorProfile`](../../../../libs/algorithm/README.md#detectorprofile) from `libs/algorithm`, eliminating the need to understand individual low-level parameters.

## How It Works

The 4 sliders are mapped to detector-specific parameters through linear interpolation. For example, for LSD FGioi:

```
  Slider:       0% ──────── 50% ──────── 100%
              │                          │
  Detail:     quant_error  3.0 ──────── 0.5   (inverted: more detail = stricter)
  Gap tol:    max_gap      1   ──────── 20    (more tolerance = larger gaps)
  Min length: min_length   30  ──────── 3     (inverted: higher slider = shorter OK)
  Precision:  refine       0   ──────── 1     (0=off, 1=on)
```

The adaptive factors (Contrast, Noise) multiply threshold-like parameters so the same slider position adapts to different image characteristics.

## Profile Sliders (0–100%)

| Slider | Description | Default |
|------|-------------|---------|
| **Detail** | Detection granularity. Higher → more segments including fine features | 50% |
| **Gap Tolerance** | How tolerant the detector is of gaps in edge chains | 50% |
| **Min Length** | Minimum segment length. Higher → discard shorter segments | 50% |
| **Precision** | Sub-pixel precision emphasis. Higher → tighter fitting tolerances | 50% |

## Adaptive Factors

| Factor | Description | Default |
|--------|-------------|---------|
| **Contrast** | Multiplier for contrast-dependent thresholds. > 1 raises, < 1 lowers | 1.0 |
| **Noise** | Multiplier for noise-related thresholds. Increase for noisy images | 1.0 |

## Image Properties (auto-updated)

Displays computed image characteristics when an image is loaded:

- **Contrast** — Michelson contrast of the source image
- **Noise Level** — Estimated noise standard deviation
- **Edge Density** — Fraction of edge pixels relative to total pixels
- **Dynamic Range** — Ratio of usable intensity range to theoretical maximum

## Buttons

| Button | Action |
|--------|--------|
| **Auto from Image** | Analyze the source image and set all sliders and factors automatically |
| **Apply to Detector** | Push the current profile to the active detector |
| **Reset** | Reset all sliders to 50% and factors to 1.0 |

## Workflows

### Automatic Image-Based Tuning

1. **Load an image** in the Analyzer and **click "Load"**. The Detector Profile panel automatically analyzes the image and displays its properties (contrast, noise level, edge density, dynamic range).
2. **Select a detector** from the Analyzer dropdown.
3. **Click "Auto from Image"** (`pb_auto`). The panel runs `ImageAnalyzer::analyze()` on the source image and sets all four sliders and both adaptive factors to values derived from the image characteristics.
4. **Click "Apply to Detector"** (`pb_apply`). The profile settings are translated into concrete detector parameters and pushed to the active detector. The Analyzer's parameter table updates to reflect the new values.
5. **Click "Process"** in the Analyzer to run detection with the new parameters.
6. **(Optional) Fine-tune** individual sliders based on the visual result:
   - Increase *Detail* if fine features are missing.
   - Increase *Gap Tolerance* if edge chains are fragmented.
   - Decrease *Min Length* if short but valid segments are being discarded.
   - Increase *Precision* for tighter endpoint fitting.
7. **Click "Apply to Detector"** again after each tweak, then re-process.

### Manual Slider-Based Tuning

1. **Load an image, select a detector, and process** to establish a baseline.
2. **Open the Detector Profile panel.**
3. **Drag the sliders** (Detail, Gap Tolerance, Min Length, Precision) to desired positions (0–100%).
4. **Adjust adaptive factors** (Contrast, Noise) if the image has unusual characteristics:
   - Set *Contrast Factor* > 1 for low-contrast images (raises detection thresholds).
   - Set *Noise Factor* > 1 for noisy images (raises noise-related thresholds).
5. **Click "Apply to Detector"** and **"Process"** to see the effect.
6. **Repeat** until the visual result is satisfactory.
7. **Click "Reset"** (`pb_reset`) at any time to return all sliders to 50% and factors to 1.0.

### Cross-Detector Comparison

1. **Select detector A**, click **"Auto from Image"** → **"Apply to Detector"** → **"Process"**.
2. **Note** the image properties and profile slider positions.
3. **Switch to detector B** and repeat. The same image analysis produces the same slider positions, but the underlying parameter mapping is detector-specific, so the actual parameter values differ.
4. **Compare** detection results visually or quantitatively via the [Accuracy Measure](../accuracy/README.md) panel.

## Use Case

Quickly tune detector parameters without understanding the low-level options. Recommended workflow:

1. Load an image
2. Click **Auto from Image** for a reasonable baseline
3. Fine-tune individual sliders based on visual results
4. Click **Apply to Detector** to push settings

## Supported Detectors

LSD CC, LSD CP, LSD Burns, LSD FBW, LSD FGioi, LSD EDLZ, LSD EL, LSD EP, LSD HoughP.

Variant detectors (e.g., LSD EL QFSt Odd, LSD EL SUSAN) share the base detector's profile mapping.

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#detectorprofile) for the full slider-to-parameter mapping tables for all 9 detectors.

<!-- help:start-ignore -->
## Files

| File | Purpose |
|------|---------|
| [detector_profile_panel.h](detector_profile_panel.h) | Panel class declaration (inherits `LATool`) |
| [detector_profile_panel.cpp](detector_profile_panel.cpp) | DetectorProfile + ImageAnalyzer integration, slider-to-parameter mapping |
| [detector_profile_panel.ui](detector_profile_panel.ui) | Qt Designer layout |

## Dependencies

- **Analyzer** — detector access, image source signals
- **libs/algorithm** — [`DetectorProfile`](../../../../libs/algorithm/README.md#detectorprofile), [`ImageAnalyzer`](../../../../libs/algorithm/README.md#imageanalyzer), `DetectorId`
<!-- help:end-ignore -->
