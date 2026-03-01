# Quivers Extension

Displays gradient vector fields as arrow overlays on the image plot.

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
| **Threshold** | Minimum magnitude value for an arrow to appear. |
| **Pixel Area** | Downsampling factor — gradient values are averaged over NxN pixel blocks. |
| **Interpolation Mode** | OpenCV interpolation method for downsampling. |
| **Select Color** | Open a color picker to change arrow color. |
| **Display Quivers** | Toggle visibility of the arrow overlay. |
| **Apply** | Recompute the quiver overlay with current settings. |
| **Delete Quivers** | Remove all arrows from the plot. |
