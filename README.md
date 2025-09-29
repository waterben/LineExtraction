# Line Extraction Tool Library

A comprehensive C++ library for line detection and analysis in digital images, featuring multiple algorithms and evaluation tools.

## Project Structure

- **`libs/`** - Core library components:
  - `edge/` - Edge detection algorithms
  - `geometry/` - Geometric primitives and operations
  - `imgproc/` - Image processing utilities
  - `lfd/` - Line Feature Detector
  - `lsd/` - Line Segment Detector
  - `phase_cong/` - Phase congruency methods
  - `utility/` - Common utilities and helpers
  - `eval/` - Evaluation and benchmarking tools

- **`apps/`** - Applications:
  - `line_analyzer/` - Interactive line analysis GUI

- **`examples/`** - Usage examples and demonstrations
- **`evaluation/`** - Performance evaluation and benchmarks
- **`tools/`** - Build tools and utilities:
  - `cmake/` - CMake modules and configuration files
  - `scripts/` - Development and setup scripts

## Quick Start

### Development Environment Setup

**Docker/DevContainer (Recommended):** Use VS Code with Dev Containers extension. See [`docker/README.md`](docker/README.md) for complete setup instructions.

**Local Setup:** Use the automated setup script:
```bash
sudo ./tools/scripts/setup_local_dev.sh
```

**Manual Setup:** See [`docker/README.md`](docker/README.md) for manual installation steps.

### Build

```bash
git clone https://github.com/waterben/LineExtraction.git
cd LineExtraction
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

### Run Tests

```bash
# Run all tests
ctest

# Run specific test
./bin/test_geometry
```

### Run Examples

```bash
# List available examples
ls bin/*_example

# Run specific example
./bin/lsd_example path/to/image.jpg
```

### Run Applications

```bash
# Launch line analyzer GUI
./bin/line_analyzer
```

## Build Configuration

Key CMake options: `BUILD_DEBUG`, `BUILD_STATIC`, `ENABLE_UNIT_TEST`, `ENABLE_QT`

```bash
# Example: Debug build
cmake -DBUILD_DEBUG=ON ..
```

## Documentation

```bash
make doc  # Output: build/doc/html/index.html
```

## Dependencies

Automatically managed: OpenCV 4.7+, Eigen3, dlib, Google Test

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
