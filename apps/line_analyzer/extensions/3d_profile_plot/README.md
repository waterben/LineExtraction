# 3D Profile Plot

3D surface visualization of the precision optimization objective function.

[← Back to Line Analyzer](../../README.md)

## Overview

Renders a 3D surface plot of the precision optimization objective function around the current line position. Shows how the mean gradient response changes as a function of profile offset and rotation angle — the same two parameters the [Precision Optimizer](../precision_optimizer/README.md) searches over.

## How It Works

The plot evaluates the mean gradient response on a 2D grid around the current line position:

```
  3D Surface Plot (response surface):

  gradient
  response
  (Z-axis)
      │     ╱╲
      │    ╱  ╲
      │   ╱    ╲         ← peak = optimal line position
      │  ╱      ╲           (where optimizer converges)
      │ ╱        ╲
      │╱──────────╲────→  profile offset (X-axis, ±pixels)
     ╱             ╲
    ╱    .  .  .    ╲──→  rotation angle (Y-axis, ±degrees)

  X-axis: orthogonal displacement from current line position
  Y-axis: rotation around line center
  Z-axis: mean gradient magnitude (objective function value)
```

- **Well-defined peak** → reliable optimization, unambiguous line position
- **Flat surface** → weak edge, optimizer will not converge
- **Multiple peaks** → ambiguous, optimizer may snap to wrong edge

## Controls

| Control | Description | Default |
|---------|-------------|---------|
| **Profile Range** | Half-range for the profile offset axis (±pixels) | 1.0 |
| **Rotation Range** | Half-range for the rotation angle axis (±degrees) | 1.0 |
| **Subdivisions** | Mesh density — higher values produce a finer surface | 20 |
| **Line Distance** | Number of support pixels for the mean gradient computation | 1 |
| **Interpolation** | Sub-pixel gradient sampling method | Bilinear |
| **Fast Interpolation** | Trade accuracy for speed | On |

## View Controls

| Button | Action |
|--------|--------|
| **Fit Profile** | Side view emphasizing the profile (offset) dimension |
| **Fit Rotation** | Side view emphasizing the rotation (angle) dimension |
| **Reset View** | Restore default 3D perspective |

## Workflows

### Visualizing the Optimization Landscape

1. **Detect lines** in the Analyzer.
2. **Select a line** by clicking it in the main plot or table. If **Auto** (`chb_auto`) is checked, the 3D plot updates immediately.
3. **Choose a data source** from the dropdown (gradient magnitude image).
4. **Set the axis ranges:**
   - *Profile Range* — X-axis half-range (±pixels). Wider range shows more of the landscape.
   - *Rotation Range* — Y-axis half-range (±degrees). Wider range reveals multiple peaks.
5. **Increase Subdivisions** for a smoother surface (at the cost of computation time).
6. **Read the surface:**
   - A **single sharp peak** near the center → the optimizer will converge reliably and the current line position is close to optimal.
   - A **flat surface** → weak edge, the optimizer has no gradient signal to follow.
   - A **ridge** (elongated peak) → the line position is well-defined in one dimension but ambiguous in the other.
   - **Multiple peaks** → neighboring edges compete; the optimizer might snap to the wrong one.

### Comparing Views

1. **Click "Fit Profile"** to see the surface from the side, emphasizing the profile (offset) dimension. This is like looking at the [Profile Analyzer](../profile_analyzer/README.md) in 3D.
2. **Click "Fit Rotation"** to see the surface from the orthogonal side, emphasizing angular sensitivity.
3. **Click "Reset View"** to return to the default 3D perspective.
4. **Rotate the surface freely** by clicking and dragging inside the 3D plot.

### Diagnosing Optimization Problems

1. **Run the [Precision Optimizer](../precision_optimizer/README.md)** on a line and observe that it does not converge or converges to an unexpected position.
2. **Open the 3D Profile Plot** for that line and check the surface shape:
   - If the surface is flat → increase the profile range or check the data source.
   - If there are multiple peaks → the line is near parallel edges; consider narrowing the search range so the optimizer stays on the correct peak.
   - If the peak is at the edge of the range → increase the range so the true optimum falls within the search window.

## Use Case

Understand the optimization landscape before or after running the [Precision Optimizer](../precision_optimizer/README.md). Use the "Fit Profile" and "Fit Rotation" views to inspect each dimension independently.

## Files

| File | Purpose |
|------|---------|
| [3d_profile_plot.h](3d_profile_plot.h) | Panel class declaration (inherits `LATool`) |
| [3d_profile_plot.cpp](3d_profile_plot.cpp) | 3D surface computation and rendering |
| [3d_profile_plot.ui](3d_profile_plot.ui) | Qt Designer layout |

## Dependencies

- **Analyzer** — line selection, image sources
- **QwtPlot3D** (`third-party/qplot3d`) — 3D surface rendering with OpenGL
- **OpenGL** — hardware-accelerated 3D visualization
