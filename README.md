# Line Extraction Tool Library

A comprehensive C++ library for line detection and analysis in digital images, featuring multiple algorithms and evaluation tools.

## Project Structure

- **`libs/`** - Core library components:
  - [`edge/`](libs/edge/README.md) - Edge detection algorithms (NMS, gradient computation, edge drawing)
  - [`geometry/`](libs/geometry/README.md) - Geometric primitives, transformations, and OpenGL rendering
  - [`imgproc/`](libs/imgproc/README.md) - Image processing utilities and filters
  - [`lfd/`](libs/lfd/README.md) - Line Feature Detector implementation
  - [`lsd/`](libs/lsd/README.md) - Line Segment Detector implementation
  - [`utility/`](libs/utility/README.md) - Common utilities (option managers, data structures)
  - [`eval/`](libs/eval/README.md) - Evaluation framework with test image support

- **`resources/`** - Data resources:
  - `datasets/` - Image datasets (BSDS500 auto-download, MDB manual setup)

- **`apps/`** - [Applications](apps/README.md):
  - [`line_analyzer/`](apps/line_analyzer/README.md) - Qt-based interactive GUI for line analysis

- **`examples/`** - [Usage examples](examples/README.md) demonstrating library functionality
- **`evaluation/`** - [Performance evaluation](evaluation/README.md) and thesis benchmarks
- **`tools/`** - Build tools and utilities

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

### Third-Party Algorithm Implementations

- **`third-party/contrib/matlab_coder/`** - MATLAB Coder generated code ([Academic License](third-party/contrib/matlab_coder/README.md))
  - Auto-generated from MATLAB implementations
  - Subject to MATLAB Coder license restrictions
  - Academic use only

- **`third-party/contrib/lsd_external/`** - External LSD implementations ([Mixed Licenses](third-party/contrib/lsd_external/README.md))
  - LSD by Grompone von Gioi: **AGPL v3.0** (strong copyleft, requires source disclosure)
  - EDLines Zero: BSD-style with citation requirement
  - KHT: Academic license with citation requirement

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
