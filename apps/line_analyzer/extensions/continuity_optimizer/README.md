# Continuity Optimizer

Optimize line segment continuity by merging collinear fragments and optionally bridging gaps with gradient evidence.

[← Back to Line Analyzer](../../README.md)

## Overview

A single unified algorithm ([`LineContinuityOptimizer`](../../../../libs/algorithm/README.md#linecontinuityoptimizer)) for improving line segment continuity:

- **Geometry-only merge** — consolidates near-collinear line segments that pass all four geometric checks (angle, endpoint distance, perpendicular distance, parallel gap).
- **Gradient-assisted merge** (optional) — within the same iterative loop, pairs that pass the angle, endpoint distance, and perpendicular distance checks but have a parallel gap exceeding the threshold can still be merged when gradient magnitude along the connecting path is strong enough. This bridges larger gaps without giving up the geometric safeguards that prevent false connections.

The gradient connection is enabled via a checkbox and only makes sense when gradient information is available from a prior detection run. All merge constraints (angle, perpendicular distance, endpoint distance) apply to both paths — the gradient is merely additional evidence for bridging gaps that are geometrically plausible but too large for pure collinear merge.

## Controls

### Merge Parameters

| Control | Description | Default |
|---------|-------------|---------|
| **Max. Distance** | Maximum endpoint distance (px) between merge candidates. Also used as search radius for gradient connection. | 10.0 |
| **Angle Error** | Maximum angle difference (deg) between candidate segments. Also used as angle limit for gradient connection. | 5.0 |
| **Distance Error** | Maximum perpendicular distance (px) | 3.0 |
| **Parallel Error** | Maximum gap along the line direction (px) | 10.0 |
| **Merge Type** | "Endpoints" connects outermost endpoints; "Average" fits averaged geometry | Endpoints |

### Gradient Connection (optional)

Enable the **Gradient Connection** checkbox to activate gradient-assisted gap bridging. All geometric constraints still apply — the gradient merely provides additional evidence to bridge gaps where the parallel error alone would reject the pair.

| Control | Description | Default |
|---------|-------------|---------|
| **Accuracy** | Sampling step (px) along the connecting path | 2.0 |
| **Threshold** | Minimum average gradient magnitude along the connection | 2.0 |

### Button

| Button | Action |
|--------|--------|
| **Run** | Execute the optimizer (single iterative loop with geometry + optional gradient) |

## Parameter Illustrations

### Merge: Max. Distance

Minimum Euclidean distance between any pair of endpoints of the two segments. All four endpoint combinations are tested. When gradient connection is enabled, this value also serves as the search radius.

```
        Max. Distance
       |<----------->|
  A ===●              ●=== B
     endpoint       endpoint
```

### Merge: Angle Error

Maximum allowed angular difference between two segments' directions. Normalized to 0–90° (undirected). When gradient connection is enabled, this also restricts connection candidates.

```
                      ╱ B
  A ══════════       ╱
               α° ──╱        α must be ≤ Angle Error
              (angle between directions)
```

### Merge: Distance Error (perpendicular)

The midpoint of segment B is projected onto the line through segment A. The perpendicular distance `d⊥` must be ≤ Distance Error.

```
  A ════════════════════
                  |
           d⊥     |  (perpendicular)
                  |
             ●────── midpoint of B
        B ═══════════
```

### Merge: Parallel Error (gap)

All four endpoints are projected onto the direction of segment A. If projections do not overlap, the gap must be ≤ Parallel Error.

```
  Direction of A ──────────────────────────>

  A ══════════                         (projection: a_min ── a_max)
                    ← gap →
                              B ══════ (projection: b_min ── b_max)

  gap = b_min - a_max   (must be ≤ Parallel Error)
```

### Merge Types

**Endpoints (STANDARD):** The merged segment connects the two outermost endpoints.

```
  Before:   A ●════●         ●════● B
  After:    M ●════════════════════●
```

**Average (AVG):** A new segment is fitted to averaged start and end positions.

```
  Before:   A ●════●
                     ●════● B
  After:      M ●═══════════●    (averaged geometry)
```

### Gradient Connection: Accuracy (sampling step)

Gradient magnitude is sampled along the connecting path at every `accuracy` px.

```
  ●─── endpoint A
  |
  ○  ← sample (every accuracy px)
  |
  ○
  |
  ●─── endpoint B

  Average of ○ samples must be ≥ Threshold.
```

### Gradient Connection: Threshold

Minimum average gradient magnitude along the connection path.

```
  Strong connection (accepted):
  A ●──▓▓▓▓▓▓▓──● B     avg = 45.2 > threshold → connect

  Weak connection (rejected):
  A ●──░░░▓░░░──● B     avg = 3.1 < threshold → skip
```

## Prerequisites

- **Merge** requires only detected lines — no image data needed.
- **Gradient Connection** requires a gradient magnitude source (`mag`, `qmag`, or `nmag`) from the current detection run, or a source image from which gradients are computed via Sobel.

## Workflows

### Quick Optimization

1. **Detect lines** in the Analyzer.
2. **Open the Continuity Optimizer** panel.
3. **Leave defaults** or adjust parameters for your scene.
4. **Enable "Gradient Connection"** if gradient sources are available.
5. **Click "Run"**. The unified iterative loop merges collinear fragments and optionally bridges gradient-supported gaps in a single pass.
6. **Inspect** in the main plot — the line count decreases as fragments are merged and gaps bridged.

### Merge Only

1. **Detect lines** in the Analyzer.
2. **Open the Continuity Optimizer** panel.
3. **Ensure "Gradient Connection" is unchecked.**
4. **Adjust merge parameters:**
   - *Max. Distance* — start with 10, increase for wider gaps.
   - *Angle Error* — keep low (e.g., 5°) for strict collinearity.
   - *Distance Error* — perpendicular tolerance, typically 2–5 px.
   - *Parallel Error* — gap tolerance, typically 5–15 px.
   - *Merge Type* — "Endpoints" for most cases; "Average" for noisy data.
5. **Click "Run"**. The algorithm iteratively merges qualifying pairs until convergence.
6. **Inspect and iterate** — increase parameters and re-run for more aggressive consolidation.

### Full Evaluation Pipeline

1. **Detect lines** in the Analyzer.
2. **Optimize** with the [Precision Optimizer](../precision_optimizer/README.md) → better alignment makes merges more accurate.
3. **Run Continuity Optimizer** with gradient connection enabled → collinear fragments merge, remaining gaps bridge.
4. **Evaluate** in [Accuracy Measure](../accuracy/README.md) → compare pre- and post-processing metrics.

## Use Cases

- **Over-segmentation:** Single edges detected as multiple short fragments → merge consolidates them.
- **Noise-induced gaps:** Small breaks in edge chains → gradient connection bridges them with gradient evidence.
- **Structured scenes:** Buildings, roads, architectural elements with long straight edges benefit most from merging.
- **Occlusion boundaries:** Partial visibility creates gaps → gradient connection recovers continuity.

## Algorithm

Uses [`LineContinuityOptimizer`](../../../../libs/algorithm/README.md#linecontinuityoptimizer) — a single iterative loop with two acceptance paths:

For each segment pair:

1. **Angle** ≤ `angle_error` (degrees, 0–90°) — reject if too different
2. **Endpoint distance** ≤ `max_dist` — reject if too far
3. **Perpendicular distance** ≤ `distance_error` — reject if not collinear
4. **Parallel gap** ≤ `parallel_error` → **merge** (geometry path)
5. If gradient enabled and gap > `parallel_error`: sample gradient along connecting path → **merge** if average ≥ `threshold` (gradient path)

This ensures gradient connections are always constrained by the same angle, endpoint distance, and perpendicular distance checks that govern standard merges — preventing the aggressive false connections that occur when gradient connection runs independently.

Process repeats until convergence (no more merges possible).

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#linecontinuityoptimizer) for full API details.

## Files

| File | Purpose |
|------|---------|
| [continuity_optimizer.h](continuity_optimizer.h) | Panel class declaration (inherits `LATool`) |
| [continuity_optimizer.cpp](continuity_optimizer.cpp) | `LineContinuityOptimizer` integration |
| [continuity_optimizer.ui](continuity_optimizer.ui) | Qt Designer layout |

## Dependencies

- **Analyzer** — line data, image sources (gradient magnitude)
- **libs/algorithm** — [`LineContinuityOptimizer`](../../../../libs/algorithm/README.md#linecontinuityoptimizer) unified algorithm
