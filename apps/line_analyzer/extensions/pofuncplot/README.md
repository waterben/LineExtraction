# PO Function Plot

3D surface visualization of the precision optimization objective function.

[← Back to Line Analyzer](../../README.md)

## Overview

Renders a 3D surface plot of the precision optimization objective function around the current line position. Shows how the mean gradient response changes as a function of profile offset and rotation angle — the same two parameters the [Precision Optimizer](../precisionoptimizer/README.md) searches over.

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

## Use Case

Understand the optimization landscape before or after running the [Precision Optimizer](../precisionoptimizer/README.md). Use the "Fit Profile" and "Fit Rotation" views to inspect each dimension independently.

## Files

| File | Purpose |
|------|---------|
| [pofuncplot.h](pofuncplot.h) | Panel class declaration (inherits `LATool`) |
| [pofuncplot.cpp](pofuncplot.cpp) | 3D surface computation and rendering |
| [pofuncplot.ui](pofuncplot.ui) | Qt Designer layout |

## Dependencies

- **ControlWindow** — line selection, image sources
- **QwtPlot3D** (`third-party/qplot3d`) — 3D surface rendering with OpenGL
- **OpenGL** — hardware-accelerated 3D visualization
