# QCustomPlot - Qt Plotting Widget

## Overview

QCustomPlot is a Qt C++ widget for plotting and data visualization. It is used in the LineExtraction project for interactive plotting in the Line Analyzer application.

## License

**GNU General Public License v3.0 (GPL v3)**

QCustomPlot is licensed under the GPL v3, which is a strong copyleft license.

### License Text

```
QCustomPlot, an easy to use, modern plotting widget for Qt
Copyright (C) 2011-2014 Emanuel Eichhammer

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see http://www.gnu.org/licenses/.
```

## Original Author

- **Author:** Emanuel Eichhammer
- **Website:** <http://www.qcustomplot.com/>
- **Version:** 1.3.0 (included in this project)
- **Date:** December 27, 2014

## Files

- `include/qplot/qcustomplot.h` - Header file
- `src/qcustomplot.cpp` - Implementation (21,574 lines)
- `include/qplot/PlotWindow.h` - LineExtraction wrapper
- `include/qplot/PlotManager.h` - LineExtraction manager
- `src/PlotWindow.cpp` - Wrapper implementation
- `src/PlotManager.cpp` - Manager implementation

## Usage in LineExtraction

QCustomPlot is used for:

- Interactive plotting in Line Analyzer GUI
- Real-time visualization of detection results
- Profile analysis plots
- Parameter optimization graphs

**Integration Points:**

- `apps/line_analyzer/` - Main application using QCustomPlot widgets
- `examples/qt/` - Qt plotting examples

## GPL v3 Implications

⚠️ **Important for Commercial Use:**

The GPL v3 is a **strong copyleft license** that:

1. **Requires Source Disclosure:**
   - Applications using this library must also be GPL v3
   - Source code must be made available to users
   - Cannot be combined with proprietary code in most cases

2. **Patent Provisions:**
   - Contributors grant patent licenses for their contributions
   - Stronger patent protection than GPL v2

3. **Anti-Tivoization:**
   - Users must be able to install modified versions
   - Hardware restrictions that prevent modification are prohibited

4. **Compatibility:**
   - GPL v3 code cannot be mixed with GPL v2 code (unless GPL v2+ "or later")
   - Compatible with LGPL v3, Apache 2.0 (under certain conditions)

### Commercial Licensing Option

For proprietary/commercial applications, QCustomPlot offers **commercial licensing**.

- Contact: <http://www.qcustomplot.com/>
- Commercial license allows closed-source distribution
- Removes GPL restrictions for commercial products

## Qt Framework Dependency

QCustomPlot requires **Qt5** (or Qt6), which is licensed under:

- **LGPL v3** - Allows dynamic linking in proprietary apps
- **GPL v2/v3** - For static linking or modification
- **Commercial Qt License** - For proprietary apps without LGPL obligations

See Qt licensing: <https://www.qt.io/licensing/>

## Alternatives for Commercial Use

If GPL v3 is incompatible with your project:

1. **Purchase commercial QCustomPlot license**
2. **Use alternative plotting libraries:**
   - QtCharts (LGPL v3 in Qt 5.7+, commercial license available)
   - QWT (LGPL v2.1 with exceptions)
   - Custom OpenGL visualization (see `libs/geometry/gl/`)

## Modifications

This version includes minor compatibility fixes for Qt 5.15+:

- `QMap::insertMulti()` deprecation handling
- `QMap::unite()` deprecation handling

Original algorithm and functionality unchanged.

## References

- **Homepage:** <http://www.qcustomplot.com/>
- **Documentation:** <http://www.qcustomplot.com/documentation/>
- **GPL v3 License:** <https://www.gnu.org/licenses/gpl-3.0.html>
- **GPL v3 FAQ:** <https://www.gnu.org/licenses/gpl-faq.html>

---

**Note:** This library is third-party code subject to GPL v3 license, which is **different and more restrictive** than the main LineExtraction project's MIT license.
