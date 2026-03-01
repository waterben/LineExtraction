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

Each tool opens as a separate dockable window and connects to the main Analyzer via signals and slots. Hover over any control for a tooltip explaining its purpose. See the individual extension READMEs for detailed documentation:

| Extension | Description |
|-----------|-------------|
| [Profile Analyzer](extensions/profile_analyzer/README.md) | Edge gradient profile visualization |
| [Precision Optimizer](extensions/precision_optimizer/README.md) | Sub-pixel line localization via numerical optimization |
| [3D Profile Plot](extensions/3d_profile_plot/README.md) | 3D surface plot of optimization objective |
| [Continuity Optimizer](extensions/continuity_optimizer/README.md) | Merge collinear fragments and bridge endpoint gaps |
| [Detector Profile](extensions/detector_profile/README.md) | High-level parameter tuning via percentage sliders |
| [Image Analyzer](extensions/image_analyzer/README.md) | Image property analysis and profile suggestions |
| [Accuracy Measure](extensions/accuracy/README.md) | P/R/F1/sAP evaluation against ground truth |
| [GT Inspector](extensions/ground_truth_inspector/README.md) | Interactive per-segment GT comparison |

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

### Workflow 1 — First Detection (Getting Started)

1. **Launch the application:**

   ```bash
   bazel run //apps/line_analyzer:app_line_analyzer
   ```

   The main Analyzer opens with the default windmill test image pre-loaded.
2. **Load an image** (optional — skip if you want to use the windmill):
   - Click **Select Image** → browse the filesystem, *or*
   - Use the **Test Images** dropdown row: pick a category (General, BSDS500, Noise, York Urban, …) and then an image from that dataset.
3. **Click "Options"** to open the PreProcessing dialog (optional):
   - Adjust scale, apply Gaussian noise, or set blur parameters.
   - Close the dialog when done.
4. **Click "Load"** to read the image into memory. The plot window shows the loaded image and gradient sources become available for all panels.
5. **Select a detector** from the *Select Line Segment Detector* dropdown (e.g., "LSD EL"). The parameter table below populates with the detector's parameters.
6. **Choose a preset** (optional): Pick *Fast*, *Balanced*, or *Accurate* from the Preset dropdown — parameters are applied instantly and the table updates.
7. **Click "Process"** to run detection. Detected lines are drawn in the plot; the line table at the bottom fills with per-line geometry (angle, distance, start/end coordinates, length).
8. **Explore results:**
   - Click a line in the plot or a row in the table to select it. The selected line turns red; the spin/slider controls show its modification offsets.
   - Change the image source dropdown (original, gradient magnitude, gradient direction, edge map) to overlay lines on different visualizations.
   - Toggle quiver plots to show the gradient vector field.

### Workflow 2 — Comparing Detectors

1. **Load an image and run detection** with detector A (e.g., "LSD EL") as in Workflow 1.
2. **Note the results** (line count, visual coverage).
3. **Switch the detector** dropdown to detector B (e.g., "LSD FGioi"). The parameter table updates.
4. **Click "Process"** again. The previous lines are replaced with new detections.
5. **Compare visually.** For quantitative comparison:
   - Open the [Accuracy Measure](extensions/accuracy/README.md) panel, load a ground truth CSV, and click **Evaluate** after each detector run.
   - Or open the [Detector Profile](extensions/detector_profile/README.md) panel and use **Auto from Image** for each detector to see how their profiles differ.

### Workflow 3 — Interactive Parameter Tuning

1. **Load an image and select a detector** (Workflow 1, steps 1–5).
2. **Edit parameters directly** in the parameter table: double-click a value cell, type a new value, press Enter. Detection re-runs on the next "Process" click.
3. **Click "Process"** to see the effect of your change.
4. **Iterate:** Adjust one parameter at a time and re-process to understand each parameter's impact.
5. **Reset** if needed: Click **Reset Detector** to restore factory defaults, or **Reset All** to reset every detector.

### Workflow 4 — Preset-Based Tuning

1. **Load an image and select a detector.**
2. **Apply the "Balanced" preset** from the Preset dropdown — this applies an optimized parameter set targeting the best F1 score.
3. **Click "Process"** to detect lines.
4. **Switch to "Fast" or "Accurate"** and re-process to compare optimization targets:
   - *Fast* = high precision (few false positives, may miss some lines).
   - *Balanced* = best F1 score (good precision + recall).
   - *Accurate* = high recall (finds most lines, may include false positives).
5. **Fine-tune:** After applying a preset, manually adjust individual parameters in the table and re-process. The preset is a starting point, not a constraint.

### Workflow 5 — Line Manipulation and Freezing

1. **Detect lines** (Workflow 1).
2. **Select a line** by clicking it in the plot or table.
3. **Adjust the line** using the spin boxes / sliders:
   - *Rotation* — rotate the line around its center.
   - *Ortho Translation* — shift the line perpendicular to its direction.
   - *Start / End Translation* — move individual endpoints along the line direction.
4. **Observe** the modified line in the plot (drawn in cyan to distinguish from the original blue).
5. **Freeze the modification:** Click **Freeze** to bake the current offsets into the line's base geometry (the line turns blue again, offsets reset to 0).
6. **Freeze All:** Click **Freeze All** to batch-freeze every modified line. This emits `linesUpdated` which triggers the [GT Inspector](extensions/ground_truth_inspector/README.md) analysis workflow.
7. **Flip operations:**
   - **Flip Normals** — reverses the normal direction of all lines (useful when the detector assigned inconsistent orientations).
   - **Flip Endpoints** — swaps start↔end for all lines.

### Workflow 6 — Full Evaluation Pipeline

This is the recommended end-to-end workflow for publication-quality evaluation:

1. **Load an image** (ideally from a standard dataset like BSDS500 or York Urban).
2. **Select a detector and apply a preset** (or use Detector Profile's **Auto from Image**).
3. **Click "Process"** to detect lines.
4. **Open Accuracy Measure** → load GT CSV → **Evaluate** → note P/R/F1/sAP.
5. **Open Precision Optimizer** → **Optimize All** → lines shift to sub-pixel optimal positions.
6. **Open Continuity Optimizer** → **Run** (with optional gradient connection) → reduce over-segmentation and bridge small gaps.
7. **Re-evaluate in Accuracy Measure** → compare metrics before/after post-processing.
8. **Open GT Inspector** → load GT TXT → **Compute Correct Lines** → inspect per-segment errors.
9. **Use Analysis Mode** in GT Inspector to create a before/after comparison table and export it for your paper.

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
Analyzer (main controller)
  ├─ PreProcessing (image preprocessing)
  ├─ DetectorVector (detector management)
  │   └─ DetectorES<LsdVariant> (detector instances)
  ├─ PresetStore (optimized parameter presets from JSON)
  ├─ ProfileAnalyzer (profile analysis tool)
  ├─ PrecisionOptimizer (parameter optimization)
  ├─ POFuncPlot (3D profile visualization)
  ├─ ContinuityOptimizer (segment merging + gradient-assisted gap bridging)
  ├─ DetectorProfilePanel (high-level tuning + image analysis)
  ├─ AccuracyPanel (P/R/F1/sAP evaluation)
  └─ LineAnalyser2D (GT inspector)
```

### Key Components

**Core (`src/`):**

- [analyzer.h](src/analyzer.h) / [analyzer.cpp](src/analyzer.cpp) — Main application window, detector management, preset loading, signal hub
- [helpers.h](src/helpers.h) / [helpers.cpp](src/helpers.cpp) — Detector creation helpers, type definitions
- [latool.h](src/latool.h) — Abstract base class for all tool panels
- [help_button.hpp](src/help_button.hpp) — Reusable help button utility
- [preprocessing.h](src/preprocessing.h) / [preprocessing.cpp](src/preprocessing.cpp) — Image loading, format conversion, scaling
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
│   ├── analyzer.{h,cpp}   # Main window controller
│   ├── helpers.{h,cpp}         # Detector creation helpers
│   ├── latool.h                # Tool panel interface
│   ├── help_button.hpp         # Help button utility
│   └── preprocessing.{h,cpp}   # Image preprocessing
├── ui/                         # Core UI layouts
│   ├── analyzer.ui
│   └── preprocessing.ui
└── extensions/                 # Tool panel extensions
    ├── profile_analyzer/        # Edge profile visualization
    ├── precision_optimizer/     # Sub-pixel optimization
    ├── 3d_profile_plot/        # 3D objective function plot
    ├── continuity_optimizer/    # Continuity optimization (merge + gradient connection)
    ├── quivers/                # Gradient vector field (quiver) visualization
    ├── detector_profile/        # High-level parameter tuning
    ├── image_analyzer/          # Image property analysis
    ├── accuracy/               # P/R/F1/sAP evaluation
    └── ground_truth_inspector/         # GT inspector
```

## Related Documentation

- [Main README](../../README.md) - Project overview and licensing
- [libs/algorithm](../../libs/algorithm/README.md) - Algorithm library (LineContinuityOptimizer, LineMerge, LineConnect, DetectorProfile, AccuracyMeasure)
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
