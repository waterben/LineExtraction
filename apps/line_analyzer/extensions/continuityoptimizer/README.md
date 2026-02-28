# Continuity Optimizer

Merge near-collinear line segments into longer continuous edges.

[← Back to Line Analyzer](../../README.md)

## Overview

Merges near-collinear line segments that likely belong to the same physical edge but were split during detection. Uses the [`LineMerge`](../../../../libs/algorithm/README.md#linemerge) algorithm from `libs/algorithm` to reduce over-segmentation.

## Controls

| Control | Description | Default |
|---------|-------------|---------|
| **Max. Distance** | Maximum endpoint distance (px) between merge candidates | 10.0 |
| **Angle Error** | Maximum allowed angle difference (deg) between two candidate segments. Smaller = stricter collinearity | 0.03 |
| **Distance Error** | Maximum perpendicular distance (px) — see illustration below | 30.0 |
| **Parallel Error** | Maximum gap along the line direction (px) — see illustration below | 0.01 |
| **Merge Type** | "Endpoints" connects outermost endpoints; "Average" fits a new segment to averaged geometry | Endpoints |

> **Note on defaults:** The UI defaults differ from the `LineMerge` library defaults (max_dist=20, angle_error=5°, distance_error=3, parallel_error=10). The UI values are tuned for conservative interactive use; increase them for more aggressive merging.

## Parameter Illustrations

### Max. Distance

Minimum Euclidean distance between any pair of endpoints of the two segments. All four endpoint combinations are tested, and the smallest distance must be ≤ Max. Distance.

```
        Max. Distance
       |<----------->|
  A ===●              ●=== B
     endpoint       endpoint
```

### Angle Error

Maximum allowed angular difference between the two segments' directions. The algorithm normalizes the angle to 0–90° (undirected), so a segment pointing left-to-right and one pointing right-to-left are treated as parallel.

```
                      ╱ B
  A ══════════       ╱
               α° ──╱        α must be ≤ Angle Error
              (angle between directions)
```

### Distance Error (perpendicular distance)

The midpoint of segment B is projected onto the line through segment A. The perpendicular distance `d⊥` must be ≤ Distance Error.

```
  A ════════════════════
                  |
           d⊥     |  (perpendicular)
                  |
             ●────── midpoint of B
        B ═══════════
```

### Parallel Error (parallel gap)

All four endpoints are projected onto the direction of segment A. If the projections do not overlap, there is a gap. This gap must be ≤ Parallel Error. Overlapping segments have a gap of 0.

```
  Direction of A ──────────────────────────>

  A ══════════                         (projection: a_min ── a_max)
                    ← gap →
                              B ══════ (projection: b_min ── b_max)

  gap = b_min - a_max   (must be ≤ Parallel Error)
```

If the segments overlap along the direction, the gap is 0 and the check always passes:

```
  A ════════════════════
            B ═══════════════
         (overlap → gap = 0 → always OK)
```

### Merge Types

**Endpoints (STANDARD):** The merged segment connects the two outermost endpoints.

```
  Before:   A ●════●         ●════● B
  After:    M ●════════════════════●
```

**Average (AVG):** A new segment is fitted to the averaged start and end positions.

```
  Before:   A ●════●
                     ●════● B
  After:      M ●═══════════●    (averaged geometry)
```

## Operations

- **Merge:** Run the merge algorithm on all current line segments
- **Undo:** Revert to the previous line set

## Workflows

### Basic Merge

1. **Detect lines** in the ControlWindow.
2. **Open the Continuity Optimizer** panel.
3. **Set the merge parameters:**
   - *Max. Distance* — maximum endpoint distance (px) between merge candidates. Start with the default (10) and increase for scenes with larger gaps.
   - *Angle Error* — maximum angle difference (deg). Keep low (e.g., 0.03–5°) for strict collinearity; increase for looser merging.
   - *Distance Error* — maximum perpendicular distance (px) between the midpoint of one segment and the line through the other.
   - *Parallel Error* — maximum gap along the line direction (px). 0 means segments must overlap or touch.
   - *Merge Type* — "Endpoints" connects the outermost points; "Average" fits a new segment to averaged geometry.
4. **Click "Merge"** (`pb_corun`). The algorithm iteratively merges qualifying pairs until convergence. The line count decreases as fragments are consolidated.
5. **Inspect the result** in the main plot — merged segments are typically longer and fewer.
6. **Undo** if the merging was too aggressive — click **Undo** to revert.

### Iterative Refinement

1. **Start with conservative parameters** (small angle error, small parallel error).
2. **Merge** → inspect → increase one parameter slightly → **Merge** again.
3. **Repeat** until the desired level of consolidation is reached. Each merge pass only affects the current line set, so you can chain multiple passes with progressively looser thresholds.

### Combined Post-Processing Pipeline

Merging is most effective as part of a pipeline:

1. **Detect lines** in the ControlWindow.
2. **Optimize** with the [Precision Optimizer](../precisionoptimizer/README.md) → better alignment makes merges more accurate.
3. **Merge** with the Continuity Optimizer → collinear fragments become single segments.
4. **Connect** with the [Connection Optimizer](../connectionoptimizer/README.md) → bridge remaining gaps.
5. **Evaluate** in [Accuracy Measure](../accuracy/README.md).

## Use Case

Reduce over-segmentation where a single edge is detected as multiple short segments. Particularly useful for:

- **Structured scenes:** Buildings, roads, and architectural elements with long straight edges
- **Noisy images:** Where noise causes edge chain breaks
- **Post-detection cleanup:** Consolidate fragmented detections before further analysis

## Algorithm

The [`LineMerge`](../../../../libs/algorithm/README.md#linemerge) algorithm iteratively checks all pairs of segments against four criteria simultaneously:

1. **Endpoint distance** ≤ `max_dist` (any of the 4 endpoint pairs)
2. **Angle difference** ≤ `angle_error` (in degrees, normalized to 0–90°)
3. **Perpendicular distance** ≤ `distance_error` (midpoint of B to line through A)
4. **Parallel gap** ≤ `parallel_error` (projection gap along A's direction)

Qualifying pairs are merged into a single segment. The process repeats until no more merges are possible (convergence).

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#linemerge) for the full API and C++/Python usage examples.

## Files

| File | Purpose |
|------|---------|
| [continuityoptimizer.h](continuityoptimizer.h) | Panel class declaration (inherits `LATool`) |
| [continuityoptimizer.cpp](continuityoptimizer.cpp) | LineMerge integration, parameter management |
| [continuityoptimizer.ui](continuityoptimizer.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — line data, line update signals
- **libs/algorithm** — [`LineMerge`](../../../../libs/algorithm/README.md#linemerge) algorithm implementation
