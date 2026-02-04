# QCustomPlot3D - 3D Plotting Extension

## Overview

QCustomPlot3D extends QCustomPlot with 3D visualization capabilities. It is used in the LineExtraction project for three-dimensional data visualization.

## License

**GNU General Public License v3.0 (GPL v3)**

QCustomPlot3D follows the same licensing as QCustomPlot - GPL v3.

### License Summary

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

**See:** [`../qplot/README.md`](../qplot/README.md) for full GPL v3 license details and implications.

## Original Project

QCustomPlot3D is an extension of QCustomPlot by Emanuel Eichhammer.

- **Base Library:** QCustomPlot 1.3.0
- **3D Extension:** Community contributions
- **License:** GPL v3

## Files

- `include/qplot3d/*.h` - 3D plotting headers
- `src/*.cpp` - 3D implementation
- Wrapper classes for LineExtraction integration

## Usage in LineExtraction

QCustomPlot3D is used for:

- 3D surface plots
- Volumetric visualization
- Multi-dimensional parameter space visualization

**Integration Points:**

- `apps/line_analyzer/` - Advanced 3D visualizations (if enabled)
- `examples/qt/` - 3D plotting examples

## GPL v3 Implications

⚠️ **Same licensing restrictions as QCustomPlot apply:**

1. GPL v3 is a **strong copyleft license**
2. Applications using this must be GPL v3 compatible
3. Source code must be disclosed to users
4. Not suitable for proprietary/closed-source products without commercial license

### Commercial Use

For commercial applications:

- Purchase commercial QCustomPlot license (covers 3D extension)
- Contact: <http://www.qcustomplot.com/>

See [`../qplot/README.md`](../qplot/README.md) for detailed licensing information and alternatives.

## Qt Framework Dependency

Requires **Qt5** (LGPL v3 or commercial license).

See main Qt licensing considerations in [`../qplot/README.md`](../qplot/README.md).

## References

- **QCustomPlot:** <http://www.qcustomplot.com/>
- **GPL v3 License:** <https://www.gnu.org/licenses/gpl-3.0.html>

---

**Note:** This library is third-party code subject to GPL v3 license, which is **different and more restrictive** than the main LineExtraction project's MIT license.
