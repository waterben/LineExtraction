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
  - `eval/` - Evaluation and benchmarking framework

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

The project supports both **CMake** and **Bazel** build systems.

| Build System | Documentation | Recommended For |
|--------------|---------------|-----------------|
| **Bazel** | [`docs/BAZEL.md`](docs/BAZEL.md) | Development, hermetic builds |
| **CMake** | [`docs/CMAKE.md`](docs/CMAKE.md) | Production, full feature support |

#### Bazel (Recommended for Development)

```bash
# One-time: detect available features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build and test
bazel build //libs/...
bazel test //libs/...

# Build and run line analyzer
bazel run //apps/line_analyzer:app_line_analyzer
```

#### CMake (Full Feature Support)

```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
ctest
```

## Dependencies

All dependencies are automatically managed by both build systems:

| Dependency | Purpose |
|------------|---------|
| OpenCV 4.7+ | Image processing, computer vision |
| Eigen3 | Linear algebra |
| dlib | Machine learning utilities |
| Google Test | Unit testing |
| Qt5 | GUI applications (optional) |

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
