# Line Analyzer

Interactive Qt-based GUI application for comprehensive line detection analysis, algorithm comparison, and parameter optimization.

[← Back to Apps](../README.md)

## Overview

Line Analyzer is a sophisticated tool for:

- **Algorithm Comparison:** Side-by-side evaluation of multiple line detection algorithms
- **Interactive Tuning:** Real-time parameter adjustment with immediate visual feedback
- **Precision Analysis:** Sub-pixel localization precision measurement
- **Profile Visualization:** Edge profile analysis and gradient visualization
- **Post-Detection Optimization:** Merge and connect fragmented segments
- **Accuracy Evaluation:** Compare detected lines against ground truth data
- **Performance Benchmarking:** Timing and efficiency comparison

## Features

### Supported Line Detectors

The application supports multiple Line Segment Detector (LSD) variants:

| Detector | Description | Edge Source |
|----------|-------------|-------------|
| LSD EL | Standard Edge Linking | Sobel gradient + NMS |
| LSD ED | Edge Drawing variant | Sobel gradient + NMS + ESD Drawing |
| LSD ES | Edge Simple variant | Sobel gradient + NMS + ESD Simple |
| LSD EP | Edge Pair | Sobel gradient + NMS |
| LSD CC | Connected Components | Sobel gradient |
| LSD CP | Connected Pair | Sobel gradient |
| LSD HOUGH | Standard Hough Transform | Edge map based |
| LSD HOUGHP | Probabilistic Hough Transform | Edge map based |
| LSD BURNS | Burns algorithm | Gradient based |
| LSD FGIOI | Grompone von Gioi (a-contrario) | Internal gradient |
| LSD FBW | Finite Bandwidth | Gradient based |
| LSD EDLZ | Edge Drawing Lines (Zhang) | Internal edge drawing |
| LSD EL QFSt Odd | Quadrature Gaussian (odd) | G2 quadrature filters |
| LSD EL SQ Odd | Quadrature Steerable (odd) | Steerable quadrature |
| LSD EL SQF Odd | Quadrature Steerable Fast (odd) | Fast steerable quadrature |
| LSD EL SQLGF Odd | Quadrature Log-Gabor Fast (odd) | Log-Gabor quadrature |
| LSD EL QFSt Even | Quadrature Gaussian (even) | G2 + zero-crossing |
| LSD EL SQ Even | Quadrature Steerable (even) | Steerable + zero-crossing |
| LSD EL SQF Even | Quadrature Steerable Fast (even) | Fast steerable + zero-crossing |
| LSD EL SQLGF Even | Quadrature Log-Gabor Fast (even) | Log-Gabor + zero-crossing |
| LSD EL PCLG | Phase Congruency Log-Gabor | PC Log-Gabor filters |
| LSD EL PCSQ | Phase Congruency Square | PC Square filters |
| LSD EL PCLSq | PC Local Square | Local square PC filters |
| LSD EL PCLSqf | PC Local Square Fast | Optimized local PC filters |
| LSD EL SUSAN | SUSAN Edge Detector | SUSAN gradient |
| LSD EL RMG | RGB Maximum Gradient | Single-channel color max |
| LSD EL RCMG | RGB Combined Max Gradient | Multi-channel color gradient |

### Analysis Tools

Each tool opens as a separate dockable window and connects to the main ControlWindow via signals and slots. Hover over any control for a tooltip explaining its purpose. See the individual extension READMEs for detailed documentation:

| Extension | Description |
|-----------|-------------|
| [Profile Analyzer](extensions/profileanalyzer/README.md) | Edge gradient profile visualization |
| [Precision Optimizer](extensions/precisionoptimizer/README.md) | Sub-pixel line localization via numerical optimization |
| [PO Function Plot](extensions/pofuncplot/README.md) | 3D surface plot of optimization objective |
| [Continuity Optimizer](extensions/continuityoptimizer/README.md) | Merge near-collinear line segments |
| [Connection Optimizer](extensions/connectionoptimizer/README.md) | Bridge endpoint gaps using gradient evidence |
| [Detector Profile](extensions/detectorprofile/README.md) | High-level parameter tuning via percentage knobs |
| [Image Analyzer](extensions/imageanalyzer/README.md) | Image property analysis and profile suggestions |
| [Accuracy Measure](extensions/accuracy/README.md) | P/R/F1/sAP evaluation against ground truth |
| [Line Analyser 2D](extensions/lineanalyser2d/README.md) | Interactive per-segment GT comparison |

### Visualization Options

- **Image Sources:** Original, gradient magnitude, gradient direction, edge map
- **Overlay Modes:** Detected lines, edge points, gradient vectors
- **Color Maps:** Grayscale, jet, hot, cool for magnitude/direction display
- **Quiver Plots:** Gradient vector field visualization
- **Interactive Selection:** Click lines for detailed analysis

## Building

### Prerequisites

- **Qt5** development libraries (`libqt5-dev`)
  - Qt5 Core, Widgets, GUI, OpenGL modules
  - Version 5.15+ recommended
- **OpenCV** with GUI support
- **LineExtraction** core libraries
- **QCustomPlot** (included in `third-party/qplot/`)
- **QwtPlot3D** (included in `third-party/qplot3d/`)

**Note:** Qt5 and QCustomPlot are optional dependencies. The application will only be built if Qt5 is detected by the build system.

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
   - Multiple detector configurations can be tested

3. **Adjust Parameters:**
   - **Quick start:** Select a preset (Fast / Balanced / Accurate) from the Preset dropdown
   - Use sliders for real-time tuning
   - Or use Detector Profile for high-level control
   - Or manually edit values in the parameter table

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

### Parameter Presets

The application ships with optimized parameter presets for the core LSD detectors. Presets were generated by running random parameter search over the York Urban dataset and selecting the best configuration for each optimization target.

**Preset Profiles:**

| Preset | Optimization Target | Best For |
|--------|-------------------|----------|
| **Default** | Built-in detector defaults | Starting point, no optimization |
| **Fast** | Precision (few false positives) | Speed-critical applications, clean results |
| **Balanced** | F1 score (precision + recall) | General-purpose use, best overall quality |
| **Accurate** | Recall (few missed lines) | Completeness, research evaluation |

**Supported Detectors:**

Presets are available for the following 9 detectors:

| Detector | Fast Score | Balanced Score | Accurate Score |
|----------|-----------|----------------|----------------|
| LSD CC | 0.094 | 0.140 | 0.491 |
| LSD CP | 0.083 | 0.126 | 0.465 |
| LSD Burns | 0.050 | 0.082 | 0.502 |
| LSD FBW | 0.065 | 0.108 | 0.514 |
| LSD FGioi | 0.095 | 0.145 | 0.461 |
| LSD EDLZ | 0.115 | 0.138 | 0.381 |
| LSD EL | 0.036 | 0.064 | 0.466 |
| LSD EP | 0.037 | 0.065 | 0.464 |
| LSD HoughP | 0.034 | 0.024 | 0.039 |

Scores represent the optimization metric value (higher is better). "Accurate" consistently achieves the highest scores because recall is the dominant component.

Variant detectors (e.g., LSD EL QFSt Odd, LSD EL SUSAN) share the base detector's preset since they differ only in the edge source, not in the line fitting parameters.

Detectors without preset support (LSD ED, LSD ES, LSD HOUGH) show a disabled Preset dropdown.

**How it works:**

1. Select a detector from the dropdown
2. Choose a preset profile from the Preset dropdown
3. Parameters are applied instantly to the detector
4. Click "Process" to run detection with the new parameters
5. The parameter table updates to show the applied values
6. You can further fine-tune individual parameters after applying a preset

**Preset File:**

Presets are stored in `resources/presets/lsd_presets.json`. The file is loaded automatically at application startup. To regenerate presets with different settings, use the `optimize_presets.py` script in the evaluation directory.

## Architecture

The application follows Qt's Model-View-Controller pattern:

```
ControlWindow (main controller)
  ├─ PreProcessing (image preprocessing)
  ├─ DetectorVector (detector management)
  │   └─ DetectorES<LsdVariant> (detector instances)
  ├─ PresetStore (optimized parameter presets from JSON)
  ├─ ProfileAnalyzer (profile analysis tool)
  ├─ PrecisionOptimizer (parameter optimization)
  ├─ POFuncPlot (3D function visualization)
  ├─ ContinuityOptimizer (segment merging)
  ├─ ConnectionOptimizer (endpoint connection)
  ├─ DetectorProfilePanel (high-level tuning + image analysis)
  ├─ AccuracyPanel (P/R/F1/sAP evaluation)
  └─ LineAnalyser2D (detailed GT comparison)
```

### Key Components

**Core (`src/`):**

- [controlwindow.h](src/controlwindow.h) / [controlwindow.cpp](src/controlwindow.cpp) — Main application window, detector management, preset loading, signal hub
- [helpers.h](src/helpers.h) / [helpers.cpp](src/helpers.cpp) — Detector creation helpers, type definitions
- [latool.h](src/latool.h) — Abstract base class for all tool panels
- [help_button.hpp](src/help_button.hpp) — Reusable help button utility
- [preprocessing.h](src/preprocessing.h) / [preprocessing.cpp](src/preprocessing.cpp) — Image loading, format conversion, scaling
- [quiver.h](src/quiver.h) / [quiver.cpp](src/quiver.cpp) — Gradient vector field visualization
- [main.cpp](src/main.cpp) — Application entry point, detector registration

**Extensions (`extensions/`):** See [Analysis Tools](#analysis-tools) above for the full list with links to individual READMEs.

## Code Structure

```
apps/line_analyzer/
├── BUILD.bazel                 # Bazel build configuration
├── CMakeLists.txt              # CMake build configuration (legacy)
├── README.md                   # This file
├── src/                        # Core application sources
│   ├── main.cpp                # Entry point and detector registration
│   ├── controlwindow.{h,cpp}   # Main window controller
│   ├── helpers.{h,cpp}         # Detector creation helpers
│   ├── latool.h                # Tool panel interface
│   ├── help_button.hpp         # Help button utility
│   ├── preprocessing.{h,cpp}   # Image preprocessing
│   └── quiver.{h,cpp}         # Vector field display
├── ui/                         # Core UI layouts
│   ├── controlwindow.ui
│   ├── preprocessing.ui
│   └── quiver.ui
└── extensions/                 # Tool panel extensions
    ├── profileanalyzer/        # Edge profile visualization
    ├── precisionoptimizer/     # Sub-pixel optimization
    ├── pofuncplot/             # 3D objective function plot
    ├── continuityoptimizer/    # Segment merging
    ├── connectionoptimizer/    # Endpoint connection
    ├── detectorprofile/        # High-level parameter tuning
    ├── imageanalyzer/          # Image property analysis
    ├── accuracy/               # P/R/F1/sAP evaluation
    └── lineanalyser2d/         # Detailed GT comparison
```

## Related Documentation

- [Main README](../../README.md) - Project overview and licensing
- [libs/algorithm](../../libs/algorithm/README.md) - Algorithm library (LineMerge, LineConnect, DetectorProfile, AccuracyMeasure)
- [libs/lsd](../../libs/lsd/README.md) - LSD algorithms
- [libs/edge](../../libs/edge/README.md) - Edge detection
- [libs/imgproc](../../libs/imgproc/README.md) - Image processing
- [examples/lsd](../../examples/lsd/README.md) - LSD examples
- [docs/BAZEL.md](../../docs/BAZEL.md) - Build documentation

## License

⚠️ **Important:** This application is licensed under **GPL v3**, not MIT.

**Why GPL v3?**

The Line Analyzer GUI uses:

- **QCustomPlot** (GPL v3) - plotting library in `third-party/qplot/`
- **Qt5** (LGPL v3 / GPL v2/v3 / Commercial)

Since QCustomPlot is GPL v3, any application using it must also be GPL v3 (copyleft requirement).

### Implications

- **Source Code:** Must be provided to users
- **Modifications:** Users can modify and redistribute
- **Derivative Works:** Must be GPL v3 compatible
- **Commercial Use:** Requires QCustomPlot commercial license or GPL compliance

### For Commercial Use

If you need a proprietary/closed-source application:

1. **Purchase QCustomPlot commercial license** from <http://www.qcustomplot.com/>
2. **Purchase Qt commercial license** from <https://www.qt.io/licensing/>
3. **Replace QCustomPlot** with a commercial-friendly alternative (e.g., QtCharts with commercial Qt license)

### Alternative

The **core LineExtraction libraries** (`libs/`) remain **MIT licensed** and can be used in commercial applications without restrictions (except for components in `third-party/contrib/` - see main README).

You can build your own custom GUI using MIT-compatible visualization libraries.

**See Also:**

- [QCustomPlot License](../../third-party/qplot/README.md)
- [Main Project License](../../README.md#license)
- [Qt Licensing](https://www.qt.io/licensing/)
