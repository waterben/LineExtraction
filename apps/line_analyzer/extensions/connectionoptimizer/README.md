# Connection Optimizer

Bridge gaps between nearby line segment endpoints using gradient evidence.

[← Back to Line Analyzer](../../README.md)

## Overview

Connects nearby line segment endpoints when the gradient magnitude along the connecting path is strong enough. Uses the [`LineConnect`](../../../../libs/algorithm/README.md#lineconnect) algorithm from `libs/algorithm` to bridge small gaps caused by noise or occlusion.

## Controls

| Control | Description | Default |
|---------|-------------|---------|
| **Max. Radius** | Maximum endpoint distance (px) to consider a connection | 10.0 |
| **Accuracy** | Sampling step (px) along the connecting path. Smaller = denser sampling | 2.0 |
| **Threshold** | Minimum average gradient magnitude along the connection path | 2.0 |

> **Note on defaults:** The `LineConnect` library defaults are max_radius=15, accuracy=2, threshold=10. The UI uses a lower threshold (2.0) to allow more connections in interactive exploration; increase it for stricter filtering.

## Parameter Illustrations

### Max. Radius

The algorithm tests all four endpoint pairings between two segments. Only pairs where the endpoint distance ≤ Max. Radius are considered.

```
  A ════●              ●════ B
        |<── radius ──>|
        (must be ≤ Max. Radius)
```

All four pairings are tested:

```
  A_start ●════● A_end    B_start ●════● B_end

  Tested: A_start↔B_start  A_start↔B_end  A_end↔B_start  A_end↔B_end
  → The pairing with the strongest gradient response (above threshold) wins.
```

### Accuracy (sampling step)

The gradient magnitude is sampled along the straight path between two candidate endpoints. The Accuracy parameter controls the step size between samples. Smaller = more samples = more accurate but slower.

```
  ●─── endpoint A
  |
  ○  ← sample point (every `accuracy` px)
  |
  ○
  |
  ○
  |
  ●─── endpoint B

  Average of all ○ samples must be ≥ Threshold.
```

### Threshold

Minimum average gradient magnitude along the connection path. The gradient image (Sobel magnitude) is sampled at each step. If the average exceeds the threshold, the connection is accepted.

```
  Gradient magnitude along path:

  ▓▓▓ = strong gradient (edge)

  ░░░ = weak gradient (no edge)

  Strong connection (accepted):

  A ●──▓▓▓▓▓▓▓──● B     avg = 45.2 > threshold → connect

  Weak connection (rejected):

  A ●──░░░▓░░░──● B     avg = 3.1 < threshold → skip
```

## Prerequisites

At least one gradient magnitude source (`mag`, `qmag`, or `nmag`) must be available from the current detection run, or a source image from which gradients are computed via Sobel.

## Operations

- **Connect:** Run the connection algorithm on all current line segments
- **Undo:** Revert to the previous line set

## Workflows

### Basic Connection

1. **Detect lines** in the ControlWindow. Ensure that gradient magnitude sources are available (most detectors produce `mag` or `qmag`).
2. **Open the Connection Optimizer** panel.
3. **Set the parameters:**
   - *Max. Radius* — how far apart endpoints can be (px). Start with the default (10) and increase if you see gaps that should be bridged.
   - *Accuracy* — sampling step along the connecting path (px). Smaller = more accurate but slower.
   - *Threshold* — minimum average gradient along the connection path. Lower values accept weaker connections.
4. **Click "Connect"** (`pb_coprun`). The algorithm tests all endpoint pairs within radius, samples the gradient along each candidate path, and extends matched segments.
5. **Inspect the result** in the main plot — newly connected segments replace the original pair. The line count decreases when connections succeed.
6. **Undo** if the result is unsatisfactory — click **Undo** to revert to the pre-connection line set.

### Combined Post-Processing Pipeline

Connection works best as part of a multi-step post-processing chain:

1. **Detect lines** in the ControlWindow.
2. **Optimize** with the [Precision Optimizer](../precisionoptimizer/README.md) → lines shift to true edge positions.
3. **Merge** with the [Continuity Optimizer](../continuityoptimizer/README.md) → near-collinear fragments become single long segments.
4. **Connect** with the Connection Optimizer → bridge remaining small gaps between nearby endpoints.
5. **Evaluate** the final result in the [Accuracy Measure](../accuracy/README.md) panel.

The recommended order is Optimize → Merge → Connect, because optimization improves alignment (making merges more accurate), and merging reduces the segment count (making connection faster and more targeted).

## Use Case

Bridge gaps between segment endpoints that are separated by a few pixels but belong to the same edge. Common scenarios:

- **Noise-induced gaps:** Small breaks in edge chains caused by image noise
- **Occlusion boundaries:** Partial visibility creates gaps in otherwise continuous edges
- **Texture interference:** Fine texture near edges can disrupt edge continuity

## Algorithm

The [`LineConnect`](../../../../libs/algorithm/README.md#lineconnect) algorithm:

1. For each pair of segments within Max. Radius, test all 4 endpoint pairings.
2. Sample the gradient magnitude along the straight path at every `accuracy` pixels.
3. Accept the pairing with the highest average gradient ≥ Threshold.
4. Extend the matched endpoints to form a longer connected segment.

See the [Algorithm Library documentation](../../../../libs/algorithm/README.md#lineconnect) for the full API and C++/Python usage examples.

## Files

| File | Purpose |
|------|---------|
| [connectionoptimizer.h](connectionoptimizer.h) | Panel class declaration (inherits `LATool`) |
| [connectionoptimizer.cpp](connectionoptimizer.cpp) | LineConnect integration, gradient sampling |
| [connectionoptimizer.ui](connectionoptimizer.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — line data, image sources (gradient magnitude)
- **libs/algorithm** — [`LineConnect`](../../../../libs/algorithm/README.md#lineconnect) algorithm implementation
