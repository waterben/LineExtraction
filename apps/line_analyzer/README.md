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

Each tool opens as a separate dockable window and connects to the main ControlWindow via signals and slots. Hover over any control for a tooltip explaining its purpose.

---

#### Profile Analyzer

Visualizes the gradient profile perpendicular to a selected line segment. This is the primary tool for understanding how edge strength varies across a detected line.

- **Blue curve:** Mean gradient response sampled perpendicular to the line
- **Shaded band:** Standard deviation of the gradient response
- **Red curve:** Smoothed profile for comparison
- **Data Source:** Choose which image source to sample (magnitude, gradient, etc.)
- **Interpolation:** Select the sub-pixel interpolation method (nearest, bilinear, bicubic)

**Use case:** Verify that a detected line sits on a true edge (sharp peak in the profile) and diagnose false positives (flat or noisy profiles).

---

#### Precision Optimizer

Optimizes sub-pixel line localization by maximizing the mean gradient response along a line segment's profile. Uses numerical optimization to refine position and angle.

- **Search Strategy:** BFGS, L-BFGS, or Conjugate Gradient
- **Stopping Criterion:** Delta (function value change) or Gradient Norm
- **Interpolation:** Method for sampling the gradient during optimization
- **Data Source:** Gradient magnitude source to evaluate

**Use case:** Improve endpoint precision of detected lines. Run on a single selected line or batch-optimize all lines.

---

#### PO Function Plot

3D surface plot of the precision optimization objective function. Shows how the mean gradient response changes as a function of profile offset (X-axis) and rotation angle (Y-axis) around the current line position.

- **Profile Range / Rotation Range:** Half-range for each axis
- **Subdivisions:** Mesh density (higher = finer surface)
- **Line Distance:** Number of support pixels for the mean computation
- **Fit Profile:** Side view emphasizing the profile (offset) dimension
- **Fit Rotation:** Side view emphasizing the rotation (angle) dimension
- **Reset View:** Restore default 3D perspective

**Use case:** Understand the optimization landscape. A well-defined peak means reliable optimization; a flat or multi-modal surface indicates ambiguity.

---

#### Continuity Optimizer

Merges near-collinear line segments that likely belong to the same physical edge but were split during detection. Uses the `LineMerge` algorithm from `libs/algorithm`.

- **Max. Distance:** Maximum endpoint distance (px) between candidates
- **Angle Error:** Maximum allowed angle difference (rad). Smaller = more strict
- **Distance Error:** Maximum perpendicular distance (px) between endpoints and the other segment's supporting line
- **Parallel Error:** Maximum lateral gap tolerance along the line direction
- **Merge Type:** "Endpoints" connects outermost endpoints; "Average" fits a new segment to averaged geometry

**Use case:** Reduce over-segmentation where a single edge is detected as multiple short segments. Useful for structured scenes (buildings, roads).

---

#### Connection Optimizer

Connects nearby line segment endpoints when the gradient magnitude along the connecting path is strong enough. Uses the `LineConnect` algorithm from `libs/algorithm`.

- **Max. Radius:** Maximum endpoint distance (px) to consider a connection
- **Accuracy:** Sampling step (px) along the connecting path. Smaller = denser sampling
- **Threshold:** Minimum average gradient magnitude along the connection path

**Requires:** At least one gradient magnitude source (`mag`, `qmag`, or `nmag`) must be available, or a source image from which gradients are computed via Sobel.

**Use case:** Bridge gaps between segment endpoints that are separated by a few pixels but belong to the same edge (e.g., gaps caused by noise or occlusion).

---

#### Detector Profile

High-level detector parameter tuning via 4 percentage knobs and 2 adaptive factors. Translates intuitive settings into concrete detector parameters via `DetectorProfile` from `libs/algorithm`.

**Profile Knobs (0–100%):**

- **Detail:** Detection granularity. Higher → more segments including fine features
- **Gap Tolerance:** How tolerant the detector is of gaps in edge chains
- **Min Length:** Minimum segment length. Higher → discard shorter segments
- **Precision:** Sub-pixel precision emphasis. Higher → tighter fitting tolerances

**Adaptive Factors:**

- **Contrast:** Multiplier for contrast-dependent thresholds (> 1 raises, < 1 lowers)
- **Noise:** Multiplier for noise-related thresholds (increase for noisy images)

**Image Properties (auto-updated):**

- Contrast, noise level, edge density, and dynamic range of the current source image

**Buttons:**

- **Auto from Image:** Analyze the source image and set all knobs and factors automatically
- **Apply to Detector:** Push the current profile to the active detector
- **Reset:** Reset all knobs to 50% and factors to 1.0

**Use case:** Quickly tune detector parameters without understanding the low-level options. Start with "Auto from Image" for a reasonable baseline, then fine-tune.

---

#### Accuracy Measure

Evaluates detected line segments against ground truth data. Computes standard metrics for quantitative comparison.

**Ground Truth CSV Format:**

```csv
image_name,x1,y1,x2,y2
image001.png,10.5,20.3,100.2,20.8
image001.png,50.0,10.0,50.0,200.0
image002.png,5.0,5.0,295.0,195.0
```

Each row defines one ground truth line segment with sub-pixel endpoints. Multiple images can be stored in one CSV; use the "Image Name Filter" field to select the relevant entry.

**Settings:**

- **Match Threshold (px):** Maximum endpoint distance for a detected segment to count as a true positive match. Default: 5.0
- **Image Name Filter:** Select which image entry from the CSV to use. Leave empty for auto-selection (single-entry CSVs)

**Metrics:**

- **Precision:** TP / (TP + FP) — what fraction of detected lines match GT
- **Recall:** TP / (TP + FN) — what fraction of GT lines were detected
- **F1 Score:** Harmonic mean of precision and recall
- **sAP:** Structural Average Precision — considers endpoint accuracy
- **TP / FP / FN:** Raw counts for true/false positives and false negatives
- **GT Segments:** Total number of ground truth segments

**Workflow:**

1. Load an image and run detection
2. Click "Browse..." to load a ground truth CSV
3. (Optional) Enter an image name filter
4. Click "Evaluate" to compute metrics
5. Adjust detector parameters and re-evaluate for comparison

---

#### Line Analyser 2D

Advanced ground truth comparison tool with interactive visualization. Unlike the simple Accuracy Measure panel, this tool provides detailed per-segment analysis with visual feedback.

**Key capabilities:**

- **Side-by-side visualization:** GT lines and detected lines drawn in a dedicated plot window
- **Per-segment matching:** Each detected line is matched to its closest GT segment with detailed error metrics (angle difference, length difference, endpoint distances)
- **Interactive exploration:** Click on GT or detected lines in the table to highlight them in the plot
- **Debug mode:** Lock a GT line and a detected line to examine their geometric relationship in detail
- **Analysis mode:** Compare two different detection runs (current vs. saved) with percentage-change tables
- **Configurable thresholds:** Angle tolerance, distance threshold, and error bounds for the matching algorithm
- **Export:** Save analysis results to text files for external processing

**Use case:** Detailed investigation of detection quality for research and publication. When the simple P/R/F1 numbers from Accuracy Measure are not enough and you need to understand *which* lines are wrong and *why*.

**Note:** Requires its own ground truth loading (File → Open GT) separate from the Accuracy Measure panel.

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
   - Use sliders for real-time tuning
   - Or use Detector Profile for high-level control

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
  ├─ POFuncPlot (3D function visualization)
  ├─ ContinuityOptimizer (segment merging)
  ├─ ConnectionOptimizer (endpoint connection)
  ├─ DetectorProfilePanel (high-level tuning + image analysis)
  ├─ AccuracyPanel (P/R/F1/sAP evaluation)
  └─ LineAnalyser2D (detailed GT comparison)
```

### Key Components

**[controlwindow.h](controlwindow.h) / [controlwindow.cpp](controlwindow.cpp):**

- Main application window
- Detector management and line storage
- Signal hub: `sourcesChanged`, `lineChanged`, `lineSelChanged`

**[helpers.h](helpers.h) / [helpers.cpp](helpers.cpp):**

- Detector creation helpers (`createDetectorES`, `createDetectorGS`, etc.)
- Type definitions (`float_type`, `LineSegment`, `ImageSources`)

**[latool.h](latool.h):**

- Abstract base class for all tool panels
- Provides `connectTools(ControlWindow*)` interface

**[preprocessing.h](preprocessing.h) / [preprocessing.cpp](preprocessing.cpp):**

- Image loading and preprocessing
- Format conversion, size adjustment, blurring

**[profileanalyzer.h](profileanalyzer.h) / [profileanalyzer.cpp](profileanalyzer.cpp):**

- Edge profile extraction perpendicular to lines
- Sub-pixel analysis and gradient visualization

**[precisionoptimizer.h](precisionoptimizer.h) / [precisionoptimizer.cpp](precisionoptimizer.cpp):**

- Numerical optimization of line position and angle
- BFGS / L-BFGS / CG search strategies

**[pofuncplot.h](pofuncplot.h) / [pofuncplot.cpp](pofuncplot.cpp):**

- 3D surface plot (Qwt3D) of the objective function
- Parameter landscape visualization

**[continuityoptimizer.h](continuityoptimizer.h) / [continuityoptimizer.cpp](continuityoptimizer.cpp):**

- `LineMerge` integration for collinear segment merging
- Configurable distance, angle, and parallelism thresholds

**[connectionoptimizer.h](connectionoptimizer.h) / [connectionoptimizer.cpp](connectionoptimizer.cpp):**

- `LineConnect` integration for endpoint bridging
- Uses gradient magnitude to validate connections

**[detectorprofilepanel.h](detectorprofilepanel.h) / [detectorprofilepanel.cpp](detectorprofilepanel.cpp):**

- `DetectorProfile` + `ImageAnalyzer` integration
- High-level percentage knobs mapped to detector parameters
- Auto-updated image property display

**[accuracypanel.h](accuracypanel.h) / [accuracypanel.cpp](accuracypanel.cpp):**

- `AccuracyMeasure` + `GroundTruthLoader` integration
- CSV-based ground truth loading and P/R/F1/sAP evaluation

**[lineanalyser2d.h](lineanalyser2d.h) / [lineanalyser2d.cpp](lineanalyser2d.cpp):**

- Interactive per-segment GT comparison with plot visualization
- Debug mode for detailed geometric error inspection
- Analysis mode for comparing detection runs

## Code Structure

```
apps/line_analyzer/
├── main.cpp                    # Application entry point
├── controlwindow.{h,cpp,ui}    # Main window
├── helpers.{h,cpp}             # Detector helpers
├── latool.h                    # Tool interface
├── preprocessing.{h,cpp,ui}    # Image preprocessing
├── profileanalyzer.*           # Profile analysis
├── precisionoptimizer.*        # Precision optimization
├── pofuncplot.*                # 3D function plotting
├── continuityoptimizer.*       # Segment merging
├── connectionoptimizer.*       # Endpoint connection
├── detectorprofilepanel.*      # High-level tuning + image analysis
├── accuracypanel.*             # Accuracy evaluation (P/R/F1/sAP)
├── lineanalyser2d.*            # Detailed GT comparison
├── analyseroptions.*           # Analysis options dialog
├── quiver.*                    # Vector field display
└── BUILD.bazel                 # Bazel build configuration
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
