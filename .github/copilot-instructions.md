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

All dependencies are managed automatically via Bazel Central Registry (BCR) and local registry:
- OpenCV 4.12.0.bcr.1+ (includes photo module, optional CUDA support)
- opencv_contrib 4.12.0.bcr.1+ (xfeatures2d, line_descriptor, ximgproc, xphoto)
- arpack++ 2.3.1 (sparse eigenvalue problems)
- SuperLU 7.0.0 (sparse linear algebra)
- Eigen3 (linear algebra)
- dlib (machine learning utilities)
- Google Test (unit testing)
- Qt5 (GUI applications, optional, auto-detected)

## Build System

**ALWAYS use Bazel unless explicitly requested otherwise or for legacy compatibility.**

### Bazel (Primary - Use This)

Bazel is the **primary and recommended build system** for all development work.

**Quick Build:**
```bash
# One-time: detect available features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build all libraries
bazel build //libs/...

# Run all tests
bazel test //libs/...

# Build and run applications
bazel run //apps/line_analyzer:app_line_analyzer
```

**Key Features:**
- Hermetic builds with guaranteed reproducibility
- Automatic dependency management via BCR (no manual scripts)
- Built-in distributed caching for fast incremental builds
- Automatic feature detection for Qt5, OpenGL, CUDA
- All dependencies declared in `MODULE.bazel`

**Common Bazel Commands:**
- Build: `bazel build //target:name`
- Test: `bazel test //target:name`
- Run: `bazel run //target:name`
- Query: `bazel query //...`
- Clean: `bazel clean`

**See:** `docs/BAZEL.md` for complete documentation and `bazel/README.md` for feature flags.

### CMake (Legacy - Use Only When Explicitly Requested)

CMake is maintained **only for legacy compatibility**. Do not use CMake unless:
- User explicitly requests CMake
- Working with existing CMake-only infrastructure
- Debugging CMake-specific issues

**Quick Build (if needed):**
```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
```

**See:** `docs/CMAKE.md` for CMake documentation (use only when necessary).

## Documentation

For detailed documentation, see:
- **Build Systems:** `docs/CMAKE.md`, `docs/BAZEL.md`
- **Development Environments:** `docs/DOCKER.md`, `docs/WSL.md`
- **Code Guidelines:** `.github/instructions/cpp.instructions.md`, `.github/instructions/python.instructions.md`

## Development Workflows

### Environment Setup

**Local Linux/WSL (Recommended for Windows):**
```bash
sudo ./tools/scripts/setup_local_dev.sh
```
This installs dependencies, sets up Python venv, git hooks, and VS Code extensions.

**Docker/DevContainer (Recommended for CI):**
Use VS Code Dev Containers extension - see `docs/DOCKER.md`. Note: No OpenGL/GPU support in containers.

**WSL Users:** See `docs/WSL.md` for OpenGL/GUI support setup.

### Debugging in VS Code

The project has sophisticated debug configurations in `.vscode/launch.json`:

1. **Debug CMake Target** - Builds and debugs any target via pre-launch task
2. **Debug Last Built Target** - Reuses previously built target (no rebuild)
3. **Debug Compiled Binary** - Select from existing binaries

Debug workflow uses build-and-copy pattern to fixed debug location (`.vscode/debug_target`) for consistent debugging.

### Testing

**Use Bazel for testing (default):**
```bash
# Run all tests
bazel test //...

# Run specific test
bazel test //libs/geometry:test_geometry
bazel test //libs/edge:test_edge

# Run tests with verbose output
bazel test //... --test_output=all
```

**CMake testing (legacy only):**
```bash
cd build
ctest  # Only use if explicitly requested
```

Tests are automatically discovered from `libs/*/tests/*.cpp` in both build systems.

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

### Adding a New Library (Bazel)

1. Create directory under `libs/new_lib/`
2. Create structure: `include/new_lib/*.hpp`, `src/*.cpp`, `tests/*.cpp`
3. Add `BUILD.bazel`:
   ```python
   load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

   cc_library(
       name = "lib_new_lib",
       srcs = glob(["src/*.cpp"]),
       hdrs = glob(["include/**/*.hpp"]),
       includes = ["include"],
       deps = [
           "//libs/utility:lib_utility",
           "@opencv",
       ],
       visibility = ["//visibility:public"],
   )

   cc_test(
       name = "test_new_lib",
       srcs = glob(["tests/*.cpp"]),
       deps = [
           ":lib_new_lib",
           "@googletest//:gtest_main",
       ],
   )
   ```
4. Add library to `libs/BUILD.bazel` if needed

**For CMake (legacy only):** See `docs/CMAKE.md` if explicitly requested.

### OpenCV Integration

In Bazel, use `@opencv` as dependency. For photo module features:
```python
deps = [
    "@opencv",
    "@opencv//:photo",  # For advanced denoising
]
```

### Qt Applications

Qt5 is auto-detected via feature detection script. Use Qt targets in Bazel:
```python
deps = [
    "@qt//:qt_core",
    "@qt//:qt_widgets",
    # ...
]
```

### Python Integration

Python environment is in `.venv/` (created by setup script). Dependencies in `pyproject.toml` managed with `uv`. Python is primarily used for evaluation scripts and tooling.

## Common Pitfalls

- **ALWAYS use Bazel** unless explicitly asked for CMake
- **Don't** modify `bazel-*` directories (generated by Bazel)
- **Don't** use CMake by default - Bazel is the primary build system
- **Do** run `./tools/scripts/detect_bazel_features.sh` after system changes
- **Do** run tests after changes: `bazel test //...`
- **Do** use pre-commit hooks (installed by setup script) for code quality
- **WSL users**: Configure `DISPLAY` for X server (see `docs/WSL.md`)

### Build System Priority

1. **Default:** Use Bazel (`bazel build`, `bazel test`, `bazel run`)
2. **Legacy only:** Use CMake only when explicitly requested by user
3. **Debugging:** VS Code tasks support both, but prefer Bazel workflows
