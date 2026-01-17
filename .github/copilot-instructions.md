# LineExtraction Development Guide

## Project Overview

LineExtraction is a comprehensive C++ library for line detection and analysis in digital images, featuring edge detection, line segment detection (LSD), and line feature detection (LFD) algorithms. The codebase emphasizes performance benchmarking and evaluation.

## Architecture

### Core Library Structure (`libs/`)

All code lives in the `lsfm` namespace. The library is organized into modular components:

- **`edge/`** - Edge detection algorithms (NMS, gradient computation, edge drawing)
- **`lfd/`** - Line Feature Detector implementation
- **`lsd/`** - Line Segment Detector implementation
- **`geometry/`** - Geometric primitives, transformations, and OpenGL rendering
- **`imgproc/`** - Image processing utilities and filters
- **`utility/`** - Common utilities (option managers, data structures)
- **`eval/`** - Performance benchmarking and evaluation framework

Libraries depend on each other: `edge` depends on `imgproc` and `geometry`; other dependencies are declared in each library's `CMakeLists.txt`.

### Applications & Examples

- **`apps/line_analyzer/`** - Qt-based interactive GUI for line analysis
- **`examples/`** - Standalone examples demonstrating library usage
- **`evaluation/`** - Performance tests and thesis evaluation code

### External Dependencies

Managed automatically via CMake scripts in `tools/cmake/extern_*.cmake`:
- OpenCV 4.7+ (with optional CUDA support)
- Eigen3 (linear algebra)
- dlib (machine learning utilities)
- Google Test (unit testing)
- Qt5 (GUI applications, optional)

## Build System - Dual Approach

### CMake (Primary, Stable)

**Quick Build:**
```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

**Key CMake Options:**
- `BUILD_DEBUG=ON` - Debug build with symbols
- `BUILD_STATIC=ON` - Build static libraries (default)
- `ENABLE_UNIT_TEST=ON` - Build tests (default)
- `ENABLE_QT=ON` - Build Qt apps (default)
- `WITH_CUDA=ON` - Enable CUDA support (auto-detected)

**Custom CMake Functions** (`tools/cmake/LineExtractionUtils.cmake`):
- `le_add_library()` - Creates library with automatic source discovery
  - Use `AUTO_TESTS` to auto-generate test targets from `tests/*.cpp`
  - Specify `PUBLIC_DEPS`, `PRIVATE_DEPS` for dependencies
  - Sources auto-discovered from `src/`, headers from `include/`
- `le_add_executable()` - Creates executables with common configuration

**Build Locations:**
- Binaries: `build/bin/`
- Libraries: `build/lib/`
- Tests: `build/bin/test_*`

### Bazel (Experimental, Future)

Bazel support is in development as a potential next-generation build system:
```bash
bazel build //...
```

## Development Workflows

### Environment Setup

**Local Linux/WSL (Recommended for Windows):**
```bash
sudo ./tools/scripts/setup_local_dev.sh
```
This installs dependencies, sets up Python venv, git hooks, and VS Code extensions.

**Docker/DevContainer (Recommended for CI):**
Use VS Code Dev Containers extension - see `docker/README.md`. Note: No OpenGL/GPU support in containers.

**WSL Users:** See `docs/WSL_SETUP.md` for OpenGL/GUI support setup.

### Debugging in VS Code

The project has sophisticated debug configurations in `.vscode/launch.json`:

1. **Debug CMake Target** - Builds and debugs any target via pre-launch task
2. **Debug Last Built Target** - Reuses previously built target (no rebuild)
3. **Debug Compiled Binary** - Select from existing binaries

Debug workflow uses build-and-copy pattern to fixed debug location (`.vscode/debug_target`) for consistent debugging.

### Testing

**Run all tests:**
```bash
cd build
ctest
```

**Run specific test:**
```bash
./bin/test_geometry
./bin/test_edge
```

Tests are automatically discovered from `libs/*/tests/*.cpp` when using `le_add_library()` with `AUTO_TESTS` flag.

### Code Style & Documentation

**C++ Conventions** (see `.github/instructions/cpp.instructions.md`):
- Google C++ Style Guide + C++ Core Guidelines
- Doxygen documentation (`@brief`, `@param`, `@return`)
- snake_case for functions/variables, PascalCase for classes
- C++17 standard, namespace `lsfm`
- Strict compiler warnings enabled (-Wall -Wextra -Werror)

**Python Conventions** (see `.github/instructions/python.instructions.md`):
- Sphinx-style docstrings
- Type hints required
- Managed with `uv` (see `pyproject.toml`)

## Key Integration Points

### Adding a New Library

1. Create directory under `libs/new_lib/`
2. Create structure: `include/new_lib/*.hpp`, `src/*.cpp`, `tests/*.cpp`
3. Add `CMakeLists.txt`:
   ```cmake
   le_add_library(lib_new_lib
       AUTO_TESTS
       PUBLIC_DEPS lib_utility le::opencv
       PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include
   )
   ```
4. Add `add_subdirectory(new_lib)` to `libs/CMakeLists.txt`

### OpenCV Integration

Use the `le::opencv` alias target (not raw OpenCV targets). This is configured in root CMakeLists.txt and provides consistent include paths and linking.

### Qt Applications

Qt apps use custom macros and require `ENABLE_QT=ON`. See `apps/line_analyzer/CMakeLists.txt` for reference. AUTOMOC is set per-target, not globally.

### Python Integration

Python environment is in `.venv/` (created by setup script). Dependencies in `pyproject.toml` managed with `uv`. Python is primarily used for evaluation scripts and tooling.

## Common Pitfalls

- **Don't** manually manage OpenCV - use `le::opencv` target
- **Don't** modify `build/` directory manually - regenerate with CMake
- **Don't** use bare `add_library()` - use `le_add_library()` for consistency
- **Do** run tests after changes: `ctest` or individual test binaries
- **Do** use pre-commit hooks (installed by setup script) for code quality
- **WSL users**: Configure `DISPLAY` for X server (see `docs/WSL_SETUP.md`)
