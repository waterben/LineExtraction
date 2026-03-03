# Quivers Extension

Displays gradient vector fields as arrow overlays on the image plot. Useful for visually inspecting the direction and magnitude of image gradients at each pixel (or block of pixels).

## How It Works

The quiver overlay samples the gradient field on a regular grid (controlled by **Pixel Area**), computes arrow direction and optionally magnitude-scaled length, then renders all arrows as a single `QCPCurve` plottable with NaN-separated segments and inline arrowheads.

```
Gradient Field (Gx, Gy)           Quiver Overlay (arrows on image)
┌─────────────────────┐           ┌─────────────────────┐
│  →  →  ↗  ↑  ↖     │           │                     │
│  →  →  ↗  ↑  ↖     │    ──→    │   →  ↗  ↑           │
│  →  →  →  ↗  ↑     │  (apply)  │   →  →  ↗           │
│  ↘  →  →  →  ↗     │           │   ↘  →  →           │
│  ↓  ↘  →  →  →     │           │                     │
└─────────────────────┘           └─────────────────────┘
    full resolution                downsampled (Pixel Area = 3)
```

## Visualization Modes

| Mode | Arrows Represent | Use Case |
|------|------------------|----------|
| **Gx/Gy** | Raw gradient components $(G_x, G_y)$ | Inspect raw Sobel/Scharr output |
| **Direction** | Edge normal direction (from NMS) | Verify edge orientation |
| **Phase** | Gradient phase angle | Check phase consistency |
| **Phase+Direction** | Phase modulated by direction | Combined phase/direction analysis |

```
Gx/Gy mode:                  Direction mode:
Arrow = (Gx, Gy) vector      Arrow = unit normal at each edge pixel

  ─→  strong horizontal         ↑  edge runs horizontally
  ↗   diagonal gradient         ↗  edge runs SW→NE
  ↑   strong vertical           →  edge runs vertically
```

## Features

- **Multiple visualization modes:** Gx/Gy gradient components, direction, phase, or combined phase+direction.
- **Thresholding:** Only display arrows where the gradient magnitude exceeds a configurable threshold.
- **Scaling:** Optionally scale arrow length proportionally to gradient magnitude.
- **Interpolation:** Downsample the gradient field by a configurable pixel area before displaying.
- **Color selection:** Choose a custom arrow color.
- **Optimized rendering:** Uses a single `QCPCurve` plottable with parametric NaN-separated segments and inline arrowheads, dramatically faster than the previous per-arrow `QCPItemLine` approach.

## Controls

| Control | Description |
|---------|-------------|
| **Quiver Mode** | Select the data source: Gx/Gy, Direction, Phase, or Phase+Direction. |
| **Use thresholding** | Filter arrows by magnitude threshold. |
| **Use scaling** | Scale arrow length by magnitude. |
| **Threshold** | Minimum magnitude value (px) for an arrow to appear. |
| **Pixel Area** | Downsampling factor — gradient values are averaged over N×N pixel blocks. Higher = fewer, coarser arrows. |
| **Interpolation Mode** | OpenCV interpolation method for downsampling (nearest, bilinear, bicubic, area, Lanczos). |
| **Select Color** | Open a color picker to change arrow color. |
| **Display Quivers** | Toggle visibility of the arrow overlay. |
| **Apply** | Recompute the quiver overlay with current settings. |
| **Delete Quivers** | Remove all arrows from the plot. |

## Typical Workflow

1. Run a detector to populate gradient data (Gx, Gy, magnitude, direction).
2. Open the Quivers panel and select the desired **Quiver Mode**.
3. Set **Pixel Area** to control density (e.g. 5–10 for overview, 1–3 for detail).
4. Enable **Use thresholding** and adjust **Threshold** to suppress noise.
5. Optionally enable **Use scaling** to see relative magnitudes.
6. Click **Apply** to render the overlay.

## See Also

- [Profile Analyzer](../profile_analyzer/README.md) — Edge profile visualization
- [Detector Profile](../detector_profile/README.md) — High-level parameter tuning
- [Line Analyzer](../../README.md) — Main application documentation
