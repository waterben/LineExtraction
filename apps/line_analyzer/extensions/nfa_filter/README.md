# NFA Filter

Filter line segments by their **Number of False Alarms (NFA)** — a statistical
measure of how unlikely a line segment is to occur by chance in a random image.

[← Back to Line Analyzer](../../README.md)

## Overview

The NFA Filter is a post-processing tool that evaluates each detected line
segment using one of three NFA validation methods and removes segments that
fail a significance test. It can be applied after any detection step —
including after precision optimization or continuity merging — to clean up
results by removing statistically insignificant detections.

Two filter modes are available:

- **Threshold (ε)** — Remove segments whose NFA exceeds a given epsilon. In
  the log-scale used here, segments with `log₁₀(NFA) < ε` are removed.
- **Top-N** — Keep only the N most meaningful segments, ranked by their NFA
  value in descending order (higher `log₁₀(NFA)` = more meaningful).

## NFA Variants

### Contrast (Magnitude)

Based on the Desolneux et al. framework. For each segment, the minimum
gradient magnitude along its rasterized path is found. The NFA measures
how likely it is to find a segment of that length with at least that
minimum contrast in a random arrangement:

```
NFA = Nl × H(u)^l

log₁₀(NFA) = -log₁₀(Nl × H(u)^l)
```

Where:

- `l` = segment length in pixels
- `H(u)` = fraction of image pixels with magnitude ≥ the minimum `u`
- `Nl` = number of possible alignments (∑ len×(len−1)/2)

**Requirements:** Gradient magnitude image (from detector, or computed via
Sobel from the source image).

### Binomial (Orientation)

Based on Gioi et al. (2012). Counts how many pixels along the segment have
their gradient direction aligned with the line's gradient angle within an
angle tolerance τ. A binomial test determines whether this count exceeds
random chance:

```
NFA = NT × B(n, k, p)

log₁₀(NFA) = -log₁₀(NT × Binomial_tail(n, k, p))
```

Where:

- `n` = segment length in pixels
- `k` = number of aligned pixels
- `p` = probability of random alignment (default: 1/8 for τ = 22.5°)
- `NT = (W × H)²` computed as `logNT = 2 × (log₁₀(W) + log₁₀(H))`

**Requirements:** Gradient components (gx, gy, from detector or Sobel).

### Binomial 2 (Orientation)

Same binomial test as above but with a different normalization for the
number of tests:

```
N = (W × H)²
logNT = log₁₀(N)
```

This produces slightly different NFA values. The choice between Binomial and
Binomial 2 depends on the application — Binomial 2 uses a stricter
normalization that may filter more aggressively.

## Controls

### NFA Variant

| Control | Description | Default |
|---------|-------------|---------|
| **Variant** | NFA computation method (Contrast, Binomial, Binomial 2) | Contrast |

### Filter Mode

| Control | Description | Default |
|---------|-------------|---------|
| **Mode** | Threshold (ε) or Top-N | Threshold |
| **log₁₀(ε)** | Epsilon threshold — segments with `log₁₀(NFA) < ε` are removed | 0.0 |
| **Keep Top N** | Number of most meaningful segments to keep | 50 |

### Binomial Parameters

These parameters are only relevant when using the **Binomial** or
**Binomial 2** variant and are disabled for the Contrast variant.

| Control | Description | Default |
|---------|-------------|---------|
| **Angle Tol. τ** | Angle tolerance in degrees — gradient direction must be within τ of the line normal to count as aligned | 22.5° |
| **Probability p** | Probability of random alignment under the null hypothesis (`p = τ / 180`) | 0.125 |

### Button

| Button | Action |
|--------|--------|
| **Run** | Compute NFA for all segments and apply the filter |

### Statistics

After each run, the panel displays:

- NFA range (min, max, mean of `log₁₀(NFA)` values)
- Segment count before and after filtering

## Log₁₀(NFA) Scale

The NFA value is presented on a logarithmic scale. Higher values indicate
more meaningful (statistically significant) line segments:

| log₁₀(NFA) | Meaning |
|-------------|---------|
| −1 | 10 expected false alarms (very likely noise) |
| 0 | 1 expected false alarm (borderline) |
| 1 | 0.1 expected false alarms (significant) |
| 2 | 0.01 expected false alarms (very significant) |
| 3 | 0.001 expected false alarms (highly significant) |

## Prerequisites

- **Line segments** must be available (run a detector first).
- **Gradient data** is required:
  - Contrast variant: gradient magnitude (`mag`, `qmag`, or `nmag` source)
  - Binomial variants: gradient components (`gx`, `gy` sources)
- If gradient sources are not available from a detector, the filter falls
  back to computing Sobel gradients from the source image.

## Workflows

### Quick Cleanup (Default)

1. Run any detector (e.g., LSD EL).
2. Open **NFA Filter**.
3. Keep defaults (Contrast, Threshold, ε = 0).
4. Click **Run** → removes statistically insignificant segments.

### Strict Filtering

1. Set ε = 1 or higher to keep only highly significant segments.
2. Click **Run**.

### Best-N Selection

1. Select **Top-N** mode.
2. Set N to the desired number of segments (e.g., 20).
3. Click **Run** → keeps the 20 most meaningful segments.

### Post-Optimization Pipeline

1. Run detector → **Precision Optimizer** → **Continuity Optimizer**.
2. Open **NFA Filter** → select Binomial variant.
3. Apply threshold or Top-N filtering.
4. Open **Accuracy Panel** → evaluate against ground truth.

## Algorithm

For each line segment:

1. **Rasterize** the segment endpoints to integer pixel coordinates using
   Bresenham's algorithm (`cv::LineIterator`).
2. **Sample** gradient data (magnitude or orientation) at each pixel.
3. **Compute NFA** using the selected variant's formula.
4. **Apply filter** — either threshold or rank-based selection.

```
for each segment s in lines:
    pixels ← rasterize(s.startPoint, s.endPoint)

    if variant == Contrast:
        u_min ← min magnitude along pixels
        H(u)  ← fraction of image pixels with mag ≥ u_min
        nfa   ← -log10(Nl × H(u_min)^|pixels|)

    else if variant == Binomial or Binomial2:
        k ← count pixels where |angle(gx,gy) − line_angle| < τ
        nfa ← -log10(NT × B(|pixels|, k, p))

if mode == Threshold:
    keep segments where nfa ≥ log_eps
else if mode == Top-N:
    sort by nfa descending, keep first N
```

<!-- help:start-ignore -->
## Files

| File | Description |
|------|-------------|
| `nfa_filter.h` | Class declaration |
| `nfa_filter.cpp` | Implementation |
| `nfa_filter.ui` | Qt Designer layout |
| `README.md` | This documentation |

## Dependencies

- **Analyzer** — provides line segments, gradient sources, and source image
- **`NfaContrast`** / **`NfaBinom`** — static NFA computation from
  `libs/edge/include/edge/nfa.hpp`
- **OpenCV** — `cv::LineIterator` for segment rasterization, `cv::Sobel`
  for gradient fallback
<!-- help:end-ignore -->
