# Line Extraction Tool Library

A comprehensive C++ library for line detection and analysis in digital images, featuring multiple algorithms and evaluation tools.

## Project Structure

- **`libs/`** - Core library components:
  - [`edge/`](libs/edge/README.md) - Edge detection algorithms (NMS, gradient computation, edge drawing)
  - [`geometry/`](libs/geometry/README.md) - Geometric primitives and transformations
  - [`imgproc/`](libs/imgproc/README.md) - Image processing utilities and filters
  - [`lfd/`](libs/lfd/README.md) - Line Feature Detector implementation
  - [`lsd/`](libs/lsd/README.md) - Line Segment Detector implementation
  - [`utility/`](libs/utility/README.md) - Common utilities (option managers, data structures)
  - [`eval/`](libs/eval/README.md) - Evaluation framework with test image support

- **`resources/`** - Data resources:
  - `datasets/` - Image datasets (BSDS500 auto-download, MDB manual setup)

- **`apps/`** - [Applications](apps/README.md):
  - [`line_analyzer/`](apps/line_analyzer/README.md) - Qt-based interactive GUI for line analysis

- **`examples/`** - [Usage examples](examples/README.md) demonstrating library functionality:
  - [`edge/`](examples/edge/README.md) - Edge detection, NMS, zero-crossing, edge linking
  - [`geometry/`](examples/geometry/README.md) - Geometric primitives, cameras, stereo vision
  - [`imgproc/`](examples/imgproc/README.md) - Gradient operators, FFT, steerable filters, pyramids
  - [`lfd/`](examples/lfd/README.md) - Line feature descriptor matching (stereo, video, motion)
  - [`lsd/`](examples/lsd/README.md) - Line Segment Detection algorithm variants
  - [`thesis/`](examples/thesis/README.md) - Thesis figure generation
  - [`other/`](examples/other/README.md) - Miscellaneous demos (Hough transform)

- **`evaluation/`** - [Performance evaluation](evaluation/README.md) and thesis benchmarks:
  - [`performance/`](evaluation/performance/README.md) - Computational benchmarks (gradient, NMS, LSD, GPU)
  - [`thesis/`](evaluation/thesis/README.md) - Precision evaluations (gradient accuracy, noise robustness, SPE)
  - [`python/`](evaluation/python/README.md) - Python tools for result analysis (charts, CSV tables)

- **`third-party/`** - [Third-party libraries](third-party/README.md) (QCustomPlot, external LSD implementations)
- **`tools/`** - Build tools, scripts, and toolchains
- **`docker/`** - [Docker/DevContainer](docs/DOCKER.md) configuration files

## Quick Start

### Development Environment Setup

| Environment | Best For | Guide |
|-------------|----------|-------|
| **WSL** | Windows users, OpenGL/GUI apps | [`docs/WSL.md`](docs/WSL.md) |
| **Docker/DevContainer** | Linux/macOS, CI/CD pipelines | [`docs/DOCKER.md`](docs/DOCKER.md) |
| **Native Linux** | Linux desktop users | See below |

**Local Setup (Linux/WSL):**

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

This installs all dependencies, sets up Python environment, git hooks, and detects available build features.

### Build

The project uses **Bazel** as the primary build system, with legacy **CMake** support.

| Build System | Status | Documentation | Recommended For |
|--------------|--------|---------------|-----------------|
| **Bazel** | **Primary** | [`docs/BAZEL.md`](docs/BAZEL.md) | All development, hermetic builds, dependency management |
| **CMake** | Legacy | [`docs/CMAKE.md`](docs/CMAKE.md) | Legacy projects, specific integrations |

#### Bazel (Primary Build System)

```bash
# One-time: detect available features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build and test
bazel build //libs/...
bazel test //libs/...

# Build and run line analyzer
bazel run //apps/line_analyzer:app_line_analyzer

# Advanced: sanitizer testing
bazel test --config=asan //libs/...  # Memory errors
bazel test --config=tsan //libs/...  # Data races

# Run benchmarks
bazel run //libs/lsd:bench_lsd     # LSD algorithm benchmarks
bazel run //libs/edge:bench_edge   # Edge detection benchmarks
```

**Why Bazel?**

- Hermetic builds with reproducible results
- Automatic dependency management via Bazel Central Registry (no complex download scripts)
- Built-in caching for faster incremental builds
- Consistent behavior across different machines

#### CMake (Legacy Support)

```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
ctest
```

CMake is maintained for compatibility but Bazel is recommended for new development.

## Applications

The project includes an interactive GUI application for line detection analysis:

**Line Analyzer** ([`apps/line_analyzer/`](apps/line_analyzer/README.md)):

- Qt-based interactive GUI for comprehensive line analysis
- Real-time parameter tuning and algorithm comparison
- Profile analysis and precision optimization
- Support for multiple detection algorithms (LSD variants, Phase Congruency, SUSAN, etc.)
- Edge detection visualization and gradient analysis

```bash
# Bazel (after running ./tools/scripts/detect_bazel_features.sh)
bazel run //apps/line_analyzer:app_line_analyzer
```

See the [apps documentation](apps/README.md) for details.

## Examples

The [`examples/`](examples/README.md) directory contains standalone programs demonstrating each library module:

| Category | Library | Key Demos |
|----------|---------|----------|
| [edge](examples/edge/README.md) | `libs/edge` | NMS variants, zero-crossing, edge linking, NFA validation |
| [geometry](examples/geometry/README.md) | `libs/geometry` | 2D/3D primitives, stereo vision, Plücker coordinates |
| [imgproc](examples/imgproc/README.md) | `libs/imgproc` | Gradient operators, FFT, steerable filters, pyramids |
| [lfd](examples/lfd/README.md) | `libs/lfd` | LBD descriptors, stereo/motion matching, video tracking |
| [lsd](examples/lsd/README.md) | `libs/lsd` | All LSD variants (EDLines, FGioi, Burns, EL, EP) |
| [thesis](examples/thesis/README.md) | `libs/eval` | Publication-quality figure generation |

```bash
# Build and run an example
bazel run //examples/lsd:lsd
bazel run //examples/edge:edge_test -- /path/to/image.jpg
```

## Evaluation & Benchmarking

The [`evaluation/`](evaluation/README.md) directory contains performance benchmarks and precision evaluations:

- **[Performance Benchmarks](evaluation/performance/README.md)**: Computational efficiency of all core algorithms (gradient, NMS, LSD, GPU convolution/FFT). Uses a task registry with auto-discovery.
- **[Thesis Evaluations](evaluation/thesis/README.md)**: Precision and accuracy evaluations for gradient orientation, noise robustness, sub-pixel estimation, and color processing.
- **[Python Tools](evaluation/python/README.md)**: Chart plotting, CSV table generation, and LaTeX utilities for result processing.

```bash
# Run all performance benchmarks
bazel run //evaluation/performance:eval_performance

# Run specific precision evaluations
bazel run //evaluation/thesis:eval_gradient_orientation
bazel run //evaluation/thesis:eval_spe_precision
```

## Python Bindings

The core libraries have Python bindings via [pybind11](https://pybind11.readthedocs.io/),
providing access to all major algorithms from Python with NumPy integration:

| Module | Library | Documentation | Description |
|--------|---------|---------------|-------------|
| `le_imgproc` | `libs/imgproc` | [README](libs/imgproc/python/README.md) | Gradient filters, core types (Range, Value, FilterData) |
| `le_edge` | `libs/edge` | [README](libs/edge/python/README.md) | Edge detection, NMS, edge segment extraction |
| `le_geometry` | `libs/geometry` | [README](libs/geometry/python/README.md) | Line, LineSegment, Polygon, drawing, LineOptimizer |
| `le_eval` | `libs/eval` | [README](libs/eval/python/README.md) | Performance benchmarking framework |
| `le_lsd` | `libs/lsd` | [README](libs/lsd/python/README.md) | 9 line segment detection algorithms |

```python
import numpy as np
import le_lsd
import le_geometry as geo

# Detect line segments
img = np.zeros((200, 200), dtype=np.uint8)
img[40:160, 40:160] = 255

det = le_lsd.LsdCC()
det.detect(img)
segments = det.line_segments()

# Visualise
vis = np.stack([img, img, img], axis=-1)
result = geo.draw_lines_random(vis, segments)
```

```bash
# Build all Python bindings
bazel build //libs/imgproc/python:le_imgproc //libs/edge/python:le_edge \
            //libs/geometry/python:le_geometry //libs/eval/python:le_eval \
            //libs/lsd/python:le_lsd

# Run all Python tests
bazel test //libs/imgproc/python:test_le_imgproc \
           //libs/edge/python:test_le_edge \
           //libs/geometry/python:test_le_geometry \
           //libs/eval/python:test_le_eval \
           //libs/lsd/python:test_le_lsd

# Run Python example scripts
bazel run //examples/lsd/python:lsd_demo
bazel run //examples/edge/python:edge_demo
bazel run //examples/imgproc/python:filter_demo
```

### Module Dependency Graph

```
le_imgproc       le_geometry       le_eval
(filters)        (primitives)      (benchmarks)
    │                 │
    ▼                 │
 le_edge              │
(NMS, ESD)            │
    │                 │
    ▼                 ▼
         le_lsd
    (9 LSD detectors)
```

## Dependencies

All dependencies are automatically managed by both build systems:

| Dependency | Version | Purpose |
|------------|---------|---------|
| OpenCV | 4.12+ | Image processing, computer vision |
| opencv_contrib | 4.12+ | Extended modules (line_descriptor, ximgproc) |
| Eigen3 | 3.4+ | Linear algebra |
| arpack++ | 2.3+ | Sparse eigenvalue problems |
| SuperLU | 7.0+ | Sparse linear algebra |
| dlib | 19.24+ | Machine learning utilities |
| Google Test | 1.15+ | Unit testing |
| Google Benchmark | 1.9+ | Performance benchmarking |
| Qt5 | 5.15+ | GUI applications (optional) |

## Documentation

| Document | Description |
|----------|-------------|
| [`docs/BAZEL.md`](docs/BAZEL.md) | Bazel build system, feature flags, targets |
| [`docs/CMAKE.md`](docs/CMAKE.md) | CMake build system, options, dependencies |
| [`docs/DOCKER.md`](docs/DOCKER.md) | Docker/DevContainer development environment |
| [`docs/WSL.md`](docs/WSL.md) | Windows Subsystem for Linux setup |

### API Documentation

Generate API documentation with Doxygen:

```bash
# Bazel
bazel run //:doxygen

# CMake
cmake -DBUILD_DOC=ON .. && cmake --build . --target doc

# Direct script
./tools/scripts/generate_doxygen.sh
```

Output: `build/doc/html/index.html`

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

**[MIT license](http://opensource.org/licenses/mit-license.php)** - Main codebase

⚠️ **Important License Notice:**

The core LineExtraction library and tools are licensed under MIT. However, the project includes third-party code and GUI components under **different licenses**:

### Library Dependencies

- **OpenCV** ([BSD 3-Clause License](https://opencv.org/license/)) - Image processing library
  - Used throughout libs/ for image operations
  - License terms acknowledged in project dependencies

### Third-Party Algorithm Implementations

- **`third-party/contrib/matlab_coder/`** - MATLAB Coder generated code ([Academic License](third-party/contrib/matlab_coder/README.md))
  - Auto-generated from MATLAB implementations
  - Subject to MATLAB Coder license restrictions
  - Academic use only

- **`third-party/contrib/lsd_external/`** - External LSD implementations ([Mixed Licenses](third-party/contrib/lsd_external/README.md))
  - LSD by Grompone von Gioi: **AGPL v3.0** (strong copyleft, requires source disclosure)
  - EDLines Zero: BSD-style with citation requirement
  - KHT: Academic license with citation requirement

- **`third-party/contrib/imgproc_external/`** - External image processing implementations ([Various Licenses](third-party/contrib/imgproc_external/README.md))
  - RCMG: All rights reserved (no open-source license granted)
  - SUSAN: Crown Copyright (research only, must not be sold)

### GUI Components (Qt-based)

- **`third-party/qplot/`** - QCustomPlot plotting library ([GPL v3](third-party/qplot/README.md))
  - Interactive plotting widget
  - **GPL v3** requires derivative works to be GPL-licensed
  - Commercial licensing available from QCustomPlot author

- **`third-party/qplot3d/`** - QCustomPlot3D extension ([GPL v3](third-party/qplot3d/README.md))
  - 3D visualization extension
  - Same GPL v3 restrictions as QCustomPlot

- **`apps/line_analyzer/`** - Line Analyzer GUI application
  - Uses **Qt5** framework (LGPL v3 / GPL v2/v3 / Commercial)
  - Uses QCustomPlot (GPL v3) → **entire app is GPL v3**
  - See [Line Analyzer README](apps/line_analyzer/README.md) for details

### Qt5 Framework Licensing

The Line Analyzer application uses Qt5, which offers multiple licensing options:

- **LGPL v3**: Allows dynamic linking in proprietary apps (most common for open-source)
- **GPL v2/v3**: For applications distributed under GPL
- **Commercial Qt License**: For proprietary closed-source applications

**Note:** Since Line Analyzer uses QCustomPlot (GPL v3), the entire application is effectively **GPL v3**, regardless of Qt licensing choice.

### Summary Table

| Component | License | Commercial Use | Source Disclosure | Notes |
|-----------|---------|----------------|-------------------|-------|
| Core Libraries (`libs/`) | **MIT** | ✅ Yes | ❌ No | Main codebase |
| `contrib/matlab_coder` | Academic | ❌ No | - | MATLAB restrictions |
| `contrib/lsd_external/lsd_fgioi` | **AGPL v3** | ⚠️ Complex | ✅ Yes | Strong copyleft |
| `contrib/lsd_external/edlines` | BSD-style | ✅ Yes | ❌ No | Citation required |
| `contrib/lsd_external/kht` | Academic | ⚠️ Check terms | ❌ No | Citation required |
| `contrib/imgproc_external/rcmg` | All rights reserved | ❌ No | ❌ No | No open-source license |
| `contrib/imgproc_external/susan` | Crown Copyright | ❌ No | ❌ No | Research only |
| `qplot` / `qplot3d` | **GPL v3** | ⚠️ No* | ✅ Yes | *Unless commercial license |
| `apps/line_analyzer` | **GPL v3** | ⚠️ No* | ✅ Yes | Due to QCustomPlot |
| Qt5 Framework | LGPL v3 / GPL / Commercial | ✅ Yes* | ⚠️ LGPL: Dynamic only | *Depends on license chosen |

### For Commercial Use

If you need to use this project in proprietary/commercial software:

1. **Option 1: Use MIT components only**
   - Use core libraries (`libs/edge`, `libs/geometry`, `libs/imgproc`, etc.)
   - Avoid `third-party/contrib/` components with restrictive licenses
   - Build custom visualization (avoid QCustomPlot/Line Analyzer)

2. **Option 2: Obtain commercial licenses**
   - Purchase QCustomPlot commercial license
   - Purchase Qt commercial license
   - Contact MATLAB Coder authors for commercial terms
   - Contact LSD AGPL authors for alternative licensing

3. **Option 3: Comply with GPL**
   - Release your application under GPL v3
   - Provide source code to users
   - Accept GPL restrictions

**See individual README files in [`third-party/`](third-party/) for detailed licensing information and usage restrictions.**
