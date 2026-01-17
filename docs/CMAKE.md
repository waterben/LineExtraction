# CMake Build System (Legacy)

**Note:** CMake is maintained for legacy support. **Bazel is the recommended build system** for new development.
See [`docs/BAZEL.md`](BAZEL.md) for the primary build system documentation.

This guide covers building LineExtraction with CMake for compatibility with existing projects.

## Quick Start

```bash
git clone https://github.com/waterben/LineExtraction.git
cd LineExtraction
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

## Prerequisites

- **CMake 3.15+**
- **C++17 compatible compiler** (GCC 7+, Clang 5+, MSVC 2017+)
- **OpenCV 4.7+** (automatically downloaded if not found)
- **Eigen3** (automatically downloaded if not found)
- **Google Test** (automatically downloaded if not found)
- **Qt5** (optional, for GUI applications)

## CMake Options

Configure the build with various options:

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_DEBUG` | `OFF` | Enable debug symbols and disable optimizations |
| `BUILD_STATIC` | `ON` | Build static libraries instead of shared |
| `ENABLE_UNIT_TEST` | `ON` | Build unit tests |
| `ENABLE_QT` | `ON` | Build Qt-based GUI applications |
| `WITH_CUDA` | Auto-detect | Enable CUDA support for OpenCV |
| `BUILD_EXAMPLES` | `ON` | Build example applications |
| `BUILD_DOC` | `OFF` | Generate Doxygen documentation |

### Usage Examples

```bash
# Debug build
cmake -DBUILD_DEBUG=ON ..

# Release build without tests
cmake -DBUILD_DEBUG=OFF -DENABLE_UNIT_TEST=OFF ..

# Build with Qt disabled
cmake -DENABLE_QT=OFF ..

# Custom OpenCV path
cmake -DOpenCV_DIR=/path/to/opencv/build ..
```

## Build Process

### Standard Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)
```

### Build Specific Targets

```bash
# Build only core libraries
cmake --build . --target lib_utility
cmake --build . --target lib_geometry
cmake --build . --target lib_lsd

# Build specific application
cmake --build . --target app_line_analyzer

# Build all tests
cmake --build . --target all_tests
```

### Build Locations

After building, files are located in:

- **Binaries:** `build/bin/`
  - Applications: `app_*`
  - Examples: `*_example`
  - Tests: `test_*`
- **Libraries:** `build/lib/`
  - Static: `lib*.a`
  - Shared: `lib*.so`
- **Documentation:** `build/doc/html/` (if `BUILD_DOC=ON`)

## Running Tests

```bash
cd build

# Run all tests
ctest

# Run all tests with verbose output
ctest --verbose

# Run specific test
./bin/test_geometry
./bin/test_edge
./bin/test_lsd

# Run tests in parallel
ctest -j$(nproc)

# Run only failed tests
ctest --rerun-failed
```

## Custom CMake Functions

The project provides custom CMake utilities in `tools/cmake/LineExtractionUtils.cmake`:

### `le_add_library()`

Creates a library target with automatic source discovery and test generation:

```cmake
le_add_library(lib_my_library
    AUTO_TESTS                          # Auto-generate test targets
    PUBLIC_DEPS lib_utility le::opencv  # Public dependencies
    PRIVATE_DEPS Eigen3::Eigen          # Private dependencies
    PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

**Features:**

- Automatic source discovery from `src/*.cpp`
- Automatic header discovery from `include/*.hpp`
- Auto-generates test targets from `tests/*.cpp` when `AUTO_TESTS` is set
- Proper dependency propagation (PUBLIC vs PRIVATE)
- Automatic include directory setup

### `le_add_executable()`

Creates an executable target with common configuration:

```cmake
le_add_executable(my_app
    SOURCES main.cpp helper.cpp
    DEPS lib_lsd lib_geometry le::opencv
)
```

## Dependency Management

### External Dependencies

CMake automatically manages external dependencies via scripts in `tools/cmake/`:

- **OpenCV:** `extern_opencv.cmake`
  - Downloads OpenCV 4.7+ if not found
  - Supports CUDA (auto-detected)
  - Custom target: `le::opencv` (use this instead of raw OpenCV targets)

- **Eigen3:** `extern_eigen.cmake`
  - Downloads Eigen 3.4+ if not found
  - Header-only library

- **dlib:** `extern_dlib.cmake`
  - Downloads dlib 19.24 if not found
  - Used for machine learning utilities

- **Google Test:** `extern_gtest.cmake`
  - Downloads GoogleTest 1.14+ if not found
  - Only when `ENABLE_UNIT_TEST=ON`

### Finding Libraries

CMake uses `find_package()` and falls back to downloading:

```cmake
# Prefer system installation
find_package(OpenCV 4.7 QUIET)
if(NOT OpenCV_FOUND)
    # Download and build
    include(FetchContent)
    # ...
endif()
```

### Custom OpenCV Location

```bash
# Specify OpenCV installation
cmake -DOpenCV_DIR=/opt/opencv/build ..

# Or set environment variable
export OpenCV_DIR=/opt/opencv/build
cmake ..
```

## Project Structure

```
LineExtraction/
├── CMakeLists.txt              # Root CMake configuration
├── libs/
│   ├── CMakeLists.txt          # All libraries
│   ├── utility/
│   │   ├── CMakeLists.txt      # Uses le_add_library()
│   │   ├── include/utility/    # Public headers
│   │   ├── src/                # Implementation files
│   │   └── tests/              # Unit tests (auto-discovered)
│   ├── geometry/
│   ├── edge/
│   └── ...
├── apps/
│   ├── CMakeLists.txt
│   └── line_analyzer/
│       └── CMakeLists.txt      # Qt application
├── examples/
│   └── CMakeLists.txt
└── tools/
    └── cmake/
        ├── LineExtractionUtils.cmake  # Custom CMake functions
        ├── extern_opencv.cmake
        ├── extern_eigen.cmake
        └── ...
```

## Library Dependencies

Internal library dependency graph:

```
utility (base)
  ├── geometry (depends on utility)
  │   ├── geometry_core
  │   ├── geometry_gl (requires OpenGL)
  │   └── geometry_tr (requires OpenGL)
  ├── imgproc (depends on utility)
  └── edge (depends on utility, imgproc, geometry)
      ├── lsd (depends on edge)
      └── lfd (depends on edge)
```

Each library declares its dependencies in `CMakeLists.txt`:

```cmake
le_add_library(lib_edge
    PUBLIC_DEPS lib_utility lib_imgproc lib_geometry_core le::opencv
    PRIVATE_DEPS Eigen3::Eigen
)
```

## Qt Applications

Qt applications require special handling:

```cmake
# In apps/line_analyzer/CMakeLists.txt
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

set(CMAKE_AUTOMOC ON)  # Enable Qt MOC
set(CMAKE_AUTOUIC ON)  # Enable Qt UIC
set(CMAKE_AUTORCC ON)  # Enable Qt RCC

le_add_executable(app_line_analyzer
    SOURCES main.cpp controlwindow.cpp ...
    DEPS lib_lsd lib_lfd qplot qplot3d Qt5::Widgets Qt5::Gui
)
```

**Note:** Qt is optional. Set `ENABLE_QT=OFF` to skip Qt applications.

## Documentation Generation

Generate Doxygen documentation:

```bash
cmake -DBUILD_DOC=ON ..
cmake --build . --target doc

# Output: build/doc/html/index.html
```

Requirements: Doxygen installed

```bash
sudo apt install doxygen graphviz
```

## Troubleshooting

### OpenCV not found

```bash
# Install OpenCV development packages
sudo apt install libopencv-dev

# Or let CMake download it (automatic)
cmake ..
```

### Qt5 not found

```bash
# Install Qt5 development packages
sudo apt install qtbase5-dev libqt5opengl5-dev

# Or disable Qt
cmake -DENABLE_QT=OFF ..
```

### Eigen3 not found

```bash
# Install Eigen3
sudo apt install libeigen3-dev

# Or let CMake download it (automatic)
cmake ..
```

### Compiler errors

```bash
# Ensure C++17 support
g++ --version  # GCC 7+
clang++ --version  # Clang 5+

# Clean build directory
rm -rf build
mkdir build && cd build
cmake ..
```

### Link errors

```bash
# Clean and rebuild
cmake --build . --clean-first
```

### Test failures

```bash
# Run tests with verbose output
ctest --verbose

# Run specific test directly
./bin/test_utility --gtest_filter=MyTest.*
```

## Advanced Configuration

### Custom Compiler

```bash
# Use Clang
cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..

# Use specific GCC version
cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 ..
```

### Build Type

```bash
# Debug
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release
cmake -DCMAKE_BUILD_TYPE=Release ..

# Release with debug info
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
```

### Install

```bash
# Install to /usr/local (requires sudo)
sudo cmake --install .

# Install to custom location
cmake --install . --prefix /opt/lineextraction

# Create package
cpack
```

## CMake vs Bazel

**Bazel is the recommended build system.** CMake is maintained for legacy compatibility.

| Feature | Bazel (Primary) | CMake (Legacy) |
|---------|-----------------|----------------|
| **Dependency Management** | ✅ Automatic via BCR, hermetic | ⚠️ Complex download scripts, error-prone |
| **Build Hermeticity** | ✅ Fully hermetic, reproducible | ❌ System-dependent |
| **Incremental Builds** | ✅ Intelligent caching | ⚠️ Limited |
| **Cross-machine Consistency** | ✅ Identical results everywhere | ❌ Varies by system |
| **Configuration** | ✅ Automatic (`.bazelrc`) | Manual (`cmake ..`) |
| **Build** | `bazel build //...` | `cmake --build .` |
| **Tests** | `bazel test //...` | `ctest` |
| **Qt5 Support** | ✅ Optional (auto-detected) | ✅ Optional (auto-detected) |
| **OpenCV Photo** | ✅ Always available (BCR 4.12.0.bcr.1+) | ✅ Always available |
| **Setup Complexity** | ✅ Simple (one command) | ⚠️ External dependency scripts |
| **Modern Ecosystem** | ✅ Active BCR development | ⚠️ Mature but stagnant |

**Why Bazel is preferred:**

- **Simpler dependencies:** All managed via `MODULE.bazel` - no complex, error-prone download scripts
- **Reproducible builds:** Same code → same binaries, regardless of machine or time
- **Better performance:** Distributed caching and intelligent incremental builds
- **Modern tooling:** Growing ecosystem with first-class support for modern workflows

**When to use CMake:**

- Existing projects with CMake infrastructure
- Integration with CMake-only toolchains
- Legacy compatibility requirements

For all new development, **use Bazel** ([docs/BAZEL.md](BAZEL.md)).

## Further Reading

- [CMake Documentation](https://cmake.org/documentation/)
- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
- [Bazel Build System](BAZEL.md)
- [WSL Setup](WSL.md)
- [Docker Setup](DOCKER.md)
