# Precision Optimizer

Sub-pixel line localization via numerical optimization.

[← Back to Line Analyzer](../../README.md)

## Overview

Optimizes sub-pixel line localization by maximizing the mean gradient response along a line segment's profile. Uses numerical optimization to refine the position and angle of detected lines for higher geometric accuracy.

## How It Works

The optimizer searches over two degrees of freedom — orthogonal offset `d` and rotation angle `r` — to find the line position that maximizes the mean gradient magnitude sampled perpendicular to the line.

```
  Original detected line:

       ●══════════════●   (coarse position from detector)

  Optimization search space:

       ←── d (profile offset, ±pixels) ──→

              ╱  r (rotation, ±degrees)
       ●═════╱══════════●
             ╱
            ╱
       ●══════════════════●   ← optimized position (max gradient)
```

The objective function is the average gradient magnitude along the line's support region. The optimizer moves the line orthogonally and rotates it slightly to find the peak response.

## Controls

| Control | Description | Default |
|---------|-------------|---------|
| **Search Strategy** | Optimization algorithm: BFGS, L-BFGS, or Conjugate Gradient | BFGS |
| **Stopping Criterion** | Convergence test: Delta (function value change) or Gradient Norm | Delta |
| **Stop Value** | Threshold for the stopping criterion | 1e-7 |
| **Max Iterations** | Maximum number of optimizer iterations (capped for responsiveness) | 1000 |
| **Profile Range** | Orthogonal search half-range (±pixels) | 1.0 |
| **Rotation Range** | Angular search half-range (±degrees). Disabled when auto-rotation is active. | 1.0 |
| **Auto Rotation** | Derive rotation range from the detector's gradient kernel size: ±atan(1 / ⌊k/2⌋)° | Off |
| **Interpolation** | Sub-pixel gradient sampling method (nearest, bilinear, bicubic) | Bilinear |
| **Fast Interpolation** | Trade accuracy for speed in sub-pixel sampling | On |
| **Data Source** | Gradient magnitude source to evaluate | — |
| **Line Distance** | Mean calculation parameter (support pixels) | 1 |

## Operations

- **Optimize Selected:** Refine the currently selected line segment
- **Optimize All:** Batch-optimize all detected line segments
- **Undo:** Revert the last optimization step

## Use Case

Improve endpoint precision of detected lines. Particularly useful when the detector provides good coarse localization but sub-pixel accuracy matters (e.g., for 3D reconstruction or precise measurement applications).

## Auto Rotation

When **Auto Rotation from kernel size** is checked, the rotation search range is
automatically derived from the active detector's `grad_kernel_size` parameter:

$$\text{range} = \pm\arctan\!\left(\frac{1}{\lfloor k/2 \rfloor}\right)°$$

| Kernel size | Half-width | Auto range |
|:-----------:|:----------:|:----------:|
| 3 | 1 | ±45.0° |
| 5 | 2 | ±26.6° |
| 7 | 3 | ±18.4° |
| 9 | 4 | ±14.0° |
| 11 | 5 | ±11.3° |

The value is refreshed when the detector changes and also right before each
optimization run, so manual parameter edits in ControlWindow's table are always
picked up.

## Algorithm

The [`PrecisionOptimize`](../../../../libs/algorithm/README.md#precisionoptimize) optimizer uses dlib's numerical optimization routines:

1. Define the objective: mean gradient magnitude along the line's perpendicular profile.
2. Choose a search strategy (BFGS for most cases, CG for large-scale problems).
3. Iteratively adjust the line's orthogonal offset and rotation to maximize the objective.
4. Stop when the convergence criterion is met (function value delta or gradient norm).

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#precisionoptimize) for the full API and C++/Python usage examples.

## Files

| File | Purpose |
|------|---------|
| [precisionoptimizer.h](precisionoptimizer.h) | Panel class declaration (inherits `LATool`) |
| [precisionoptimizer.cpp](precisionoptimizer.cpp) | Optimization loop, objective function evaluation |
| [precisionoptimizer.ui](precisionoptimizer.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — line data, image sources, line modification signals
- **libs/algorithm** — [`PrecisionOptimize`](../../../../libs/algorithm/README.md#precisionoptimize) numerical optimization framework
- **QCustomPlot** — convergence plot
