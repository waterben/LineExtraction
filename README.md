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

#### Option 1: Docker/DevContainer (Recommended)

Use the provided DevContainer configuration with VS Code for a consistent development environment:

- Open the project in VS Code
- Install the "Dev Containers" extension
- Reopen in container when prompted
- See `docker/README.md` for details

#### Option 2: Local Setup

Use the automated setup script for local development:

```bash
./tools/scripts/setup_local_dev.sh
```

#### Option 3: Manual Prerequisites (Debian/Ubuntu)

```bash
sudo apt install cmake build-essential doxygen libblas-dev liblapack-dev \
  libsuperlu-dev libarpack2-dev freeglut3-dev qt5-default libgtk2.0-dev libeigen3-dev
```

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

Key CMake options:

- `BUILD_DEBUG` - Debug vs Release build (default: OFF)
- `BUILD_STATIC` - Static vs Shared libraries (default: ON)
- `ENABLE_UNIT_TEST` - Enable Google Test (default: ON)
- `ENABLE_QT` - Enable Qt-dependent components (default: ON)

```bash
# Example: Debug build with Qt disabled
cmake -DBUILD_DEBUG=ON -DENABLE_QT=OFF ..
```

## Documentation

Build documentation with Doxygen:

```bash
make doc
# Output in build/doc/html/index.html
```

## Dependencies

The build system automatically downloads and builds:

- OpenCV 4.7+
- Eigen3
- dlib
- Google Test (if testing enabled)

For detailed build system usage, see `tools/cmake/LineExtractionUtils.cmake`.

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
