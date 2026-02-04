# Line Analyzer

Interactive Qt-based GUI application for comprehensive line detection analysis, algorithm comparison, and parameter optimization.

[← Back to Apps](../README.md)

## Overview

Line Analyzer is a sophisticated tool for:

- **Algorithm Comparison:** Side-by-side evaluation of multiple line detection algorithms
- **Interactive Tuning:** Real-time parameter adjustment with immediate visual feedback
- **Precision Analysis:** Sub-pixel localization precision measurement
- **Profile Visualization:** Edge profile analysis and gradient visualization
- **Performance Benchmarking:** Timing and efficiency comparison

## Features

### Supported Line Detectors

The application supports multiple Line Segment Detector (LSD) variants:

| Detector | Description | Edge Source |
|----------|-------------|-------------|
| LSD EL | Standard Edge Linking | Sobel gradient + NMS |
| LSD EL PCGF | Phase Congruency Gaussian | Multi-scale Gaussian filters |
| LSD EL PCSQ | Phase Congruency Square | Square filter variant |
| LSD EL PCLSq | PC Local Square | Local square filters |
| LSD EL PCLSqf | PC Local Square Fast | Optimized local filters |
| LSD EL SUSAN | SUSAN Edge Detector | Smallest Univalue Segment |
| LSD EL RMG | RGB Maximum Gradient | Single-channel color max |
| LSD EL RCMG | RGB Combined Max Gradient | Multi-channel color |

### Analysis Tools

**Profile Analyzer:**

- Visualize edge profiles perpendicular to detected lines
- Analyze sub-pixel localization accuracy
- Measure gradient strength and consistency

**Precision Optimizer:**

- Optimize detector parameters for maximum precision
- Test different threshold values
- Evaluate sub-pixel estimation quality

**PO Function Plot:**

- Visualize objective functions during optimization
- Track parameter convergence
- Display performance metrics

### Visualization Options

- **Image Sources:** Original, gradient magnitude, gradient direction, edge map
- **Overlay Modes:** Detected lines, edge points, gradient vectors
- **Color Maps:** Grayscale, jet, hot, cool for magnitude/direction display
- **Quiver Plots:** Gradient vector field visualization
- **Interactive Selection:** Click lines for detailed analysis

## Building

### Prerequisites

- Qt5 development libraries (`libqt5-dev`)
- OpenCV with GUI support
- LineExtraction core libraries

### Bazel (Recommended)

```bash
# One-time: detect Qt5 availability
./tools/scripts/detect_bazel_features.sh

# Build and run
bazel run //apps/line_analyzer:app_line_analyzer
```

### CMake (Legacy)

```bash
mkdir build && cd build
cmake -DBUILD_APPS=ON ..
cmake --build . -j$(nproc)
./bin/app_line_analyzer
```

## Usage

### Basic Workflow

1. **Load Image:**
   - File → Open Image
   - Or use default test image

2. **Select Detector:**
   - Choose from detector dropdown
   - Multiple detectors can be active simultaneously

3. **Adjust Parameters:**
   - Use sliders for real-time tuning
   - Monitor performance impact

4. **Analyze Results:**
   - View detected lines in main window
   - Open analysis tools for detailed inspection
   - Compare different detector configurations

### Keyboard Shortcuts

- `Ctrl+O` - Open image
- `Ctrl+S` - Save results
- `Ctrl+Q` - Quit application

### Configuration

Parameters are stored per detector and include:

- **Gradient:** Operator type, kernel size, smoothing
- **NMS:** Threshold, direction quantization
- **Linking:** Min pixels, max gap, magnitude multiplier
- **NFA:** Precision, log epsilon
- **Fitting:** Method, tolerance, max iterations

## Architecture

The application follows Qt's Model-View-Controller pattern:

```
ControlWindow (main controller)
  ├─ PreProcessing (image preprocessing)
  ├─ DetectorVector (detector management)
  │   └─ DetectorES<LsdVariant> (detector instances)
  ├─ ProfileAnalyzer (profile analysis tool)
  ├─ PrecisionOptimizer (parameter optimization)
  └─ POFuncPlot (function visualization)
```

### Key Components

**[controlwindow.h](controlwindow.h) / [controlwindow.cpp](controlwindow.cpp):**

- Main application window
- Detector management
- UI coordination

**[helpers.h](helpers.h) / [helpers.cpp](helpers.cpp):**

- Detector creation helpers
- Type definitions
- Utility functions

**[preprocessing.h](preprocessing.h) / [preprocessing.cpp](preprocessing.cpp):**

- Image loading and preprocessing
- Format conversion
- Size adjustment

**[profileanalyzer.h](profileanalyzer.h) / [profileanalyzer.cpp](profileanalyzer.cpp):**

- Edge profile extraction
- Sub-pixel analysis
- Gradient visualization

**[precisionoptimizer.h](precisionoptimizer.h) / [precisionoptimizer.cpp](precisionoptimizer.cpp):**

- Parameter optimization
- Precision measurement
- Automated tuning

## Code Structure

```
apps/line_analyzer/
├── main.cpp                 # Application entry point
├── controlwindow.{h,cpp,ui} # Main window
├── helpers.{h,cpp}          # Detector helpers
├── latool.h                 # Tool interface
├── preprocessing.{h,cpp,ui} # Image preprocessing
├── profileanalyzer.*        # Profile analysis
├── precisionoptimizer.*     # Precision optimization
├── pofuncplot.*             # Function plotting
├── quiver.*                 # Vector field display
├── analyseroptions.*        # Analysis options dialog
├── continuityoptimizer.*    # Continuity optimization (WIP)
├── connectionoptimizer.*    # Connection optimization (WIP)
└── lineanalyser2d.*         # 2D line analysis (WIP)
```

## Related Documentation

- [Main README](../../README.md) - Project overview
- [libs/lsd](../../libs/lsd/README.md) - LSD algorithms
- [libs/edge](../../libs/edge/README.md) - Edge detection
- [libs/imgproc](../../libs/imgproc/README.md) - Image processing
- [examples/lsd](../../examples/lsd/README.md) - LSD examples
- [docs/BAZEL.md](../../docs/BAZEL.md) - Build documentation

## Notes

- Some tools (ContinuityOptimizer, ConnectionOptimizer, LineAnalyser2D) are work-in-progress and commented out in [main.cpp](main.cpp)
- The application requires OpenCV with GUI support (highgui module)
- For headless environments, use command-line examples instead
