# Line Extraction Tool Library

A comprehensive C++ library for line detection and analysis in digital images, featuring multiple algorithms and evaluation tools.

## Project Structure

- **`libs/`** - Core library components:
  - `edge/` - Edge detection algorithms (NMS, gradient computation, edge drawing)
  - `geometry/` - Geometric primitives, transformations, and OpenGL rendering
  - `imgproc/` - Image processing utilities and filters
  - `lfd/` - Line Feature Detector implementation
  - `lsd/` - Line Segment Detector implementation
  - `utility/` - Common utilities (option managers, data structures)
  - `eval/` - Evaluation framework with test image support

- **`resources/`** - Data resources:
  - `datasets/` - Image datasets (BSDS500 auto-download, MDB manual setup)

- **`apps/`** - Applications:
  - `line_analyzer/` - Qt-based interactive GUI for line analysis

- **`examples/`** - Usage examples demonstrating library functionality
- **`evaluation/`** - Performance evaluation and thesis benchmarks
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

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
