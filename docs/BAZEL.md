# Bazel Build System for LineExtraction

This project supports both CMake and Bazel build systems. Bazel provides hermetic builds
with automatic dependency management via the Bazel Central Registry (BCR).

## Quick Start

```bash
# Detect and configure available features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build all libraries
bazel build //libs/...

# Run all tests
bazel test //libs/...

# Build and run the line analyzer (requires Qt5)
bazel run //apps/line_analyzer:app_line_analyzer
```

## Prerequisites

- **Bazel 8.4.2+** (managed via `.bazelversion`)
- **GCC or Clang** compiler
- **System libraries** (optional, for GUI features):
  - Qt5 (`libqt5-dev`)
  - OpenGL/GLU (`libgl1-mesa-dev libglu1-mesa-dev`)

## Feature Detection

The project includes automatic feature detection. Run once after setup:

```bash
./tools/scripts/detect_bazel_features.sh
```

This creates `.bazelrc.user` with detected features (Qt5, OpenGL, CUDA).
After running, you can build without explicit flags:

```bash
# Before: bazel build --//bazel:enable_qt5=true //apps/line_analyzer:app_line_analyzer
# After:  bazel build //apps/line_analyzer:app_line_analyzer
```

## Feature Flags

Features can be enabled/disabled via command-line flags or `.bazelrc.user`:

| Feature | Flag | Default | Description |
|---------|------|---------|-------------|
| Qt5 | `--//bazel:enable_qt5` | `false` | GUI applications (line_analyzer, qplot) |
| OpenGL | `--//bazel:enable_opengl` | `false` | 3D visualization (qplot3d, geometry_gl) |
| CUDA | `--//bazel:enable_cuda` | `false` | GPU acceleration |
| OpenCV Photo | `--//bazel:enable_photo` | `false` | Advanced denoising (not in BCR) |

### Manual Configuration

Create or edit `.bazelrc.user` (git-ignored):

```bash
# Enable Qt5 and OpenGL on this machine
build --//bazel:enable_qt5=true
build --//bazel:enable_opengl=true

# Or exclude Qt5 targets entirely
build --build_tag_filters=-qt5
test --test_tag_filters=-qt5
```

## Build Commands

### Basic Usage

```bash
# Build everything
bazel build //...

# Build specific targets
bazel build //libs/lsd:lib_lsd
bazel build //apps/line_analyzer:app_line_analyzer
bazel build //examples/edge:edge_test

# Run tests
bazel test //...
bazel test //libs/geometry:test_camera
bazel test --test_output=all //libs/...  # Verbose output

# Run binaries
bazel run //examples/lsd:lsd
bazel run //apps/line_analyzer:app_line_analyzer -- --help
```

### Compiler Selection

```bash
# GCC (default)
bazel build --config=gcc //...

# Clang
bazel build --config=clang //...
```

### Build Modes

```bash
# Release (default): -O3, optimized
bazel build //...

# Debug: -g, -O0, no optimization
bazel build --config=debug //...

# Extended debug features (gdb-friendly)
bazel build --config=debugging_feature //...
```

### Sanitizers

```bash
# Address Sanitizer (memory errors)
bazel build --config=asan //libs/utility:test_value
bazel run --config=asan //libs/utility:test_value

# Thread Sanitizer (data races)
bazel build --config=tsan //...
```

## Project Structure

```
LineExtraction/
├── MODULE.bazel              # Bzlmod dependencies (BCR)
├── .bazelrc                  # Global Bazel configuration
├── .bazelrc.user             # Local settings (git-ignored, auto-generated)
├── .bazelversion             # Bazel version (8.4.2)
├── BUILD.bazel               # Root BUILD file
├── bazel/
│   ├── BUILD.bazel           # Feature flags (bool_flag, config_setting)
│   ├── features.bzl          # Helper functions (if_qt5_enabled, etc.)
│   └── README.md             # Feature flag documentation
├── libs/*/BUILD.bazel        # Library targets
├── apps/*/BUILD.bazel        # Application targets
├── examples/*/BUILD.bazel    # Example targets
├── third-party/
│   ├── qplot/BUILD.bazel     # Qt plotting library
│   ├── qplot3d/BUILD.bazel   # Qt 3D plotting library
│   └── opencv_wrapper/       # OpenCV utilities
└── tools/
    ├── bazel/
    │   ├── qt5.bzl           # Qt5 MOC/UIC build rules
    │   ├── copts.bzl         # Shared compiler flags
    │   └── registry/         # Local Bazel registry (dlib)
    └── scripts/
        └── detect_bazel_features.sh  # Feature detection
```

## Dependencies (January 2026)

Managed via Bzlmod (`MODULE.bazel`):

| Dependency | Version | Source |
|------------|---------|--------|
| OpenCV | 4.12.0.bcr.1 | BCR |
| libjpeg_turbo | 3.1.3.bcr.2 | BCR |
| GoogleTest | 1.15.2 | BCR |
| Eigen | 3.4.0 | http_archive |
| dlib | 19.24.7 | Local Registry |
| bazel_skylib | 1.9.0 | BCR |
| rules_cc | 0.2.15 | BCR |
| rules_python | 1.1.0 | BCR |

## Component Status

### Core Libraries (✅ All working)

| Target | Status | Notes |
|--------|--------|-------|
| `//libs/utility` | ✅ | Core utilities |
| `//libs/geometry:lib_geometry_core` | ✅ | Geometry primitives |
| `//libs/geometry:lib_geometry_gl` | ✅ | Requires `--//bazel:enable_opengl=true` |
| `//libs/geometry:lib_geometry_tr` | ✅ | Requires `--//bazel:enable_opengl=true` |
| `//libs/imgproc` | ✅ | Image processing |
| `//libs/edge` | ✅ | Edge detection |
| `//libs/lsd` | ✅ | Line Segment Detector |
| `//libs/lfd` | ✅ | Line Feature Detector |
| `//libs/eval` | ✅ | Evaluation framework |

### Third-Party Libraries

| Target | Status | Notes |
|--------|--------|-------|
| `//third-party/qplot` | ✅ | Requires `--//bazel:enable_qt5=true` |
| `//third-party/qplot3d` | ✅ | Requires Qt5 + OpenGL |
| `//third-party/opencv_wrapper` | ✅ | OpenCV utilities |

### Applications

| Target | Status | Notes |
|--------|--------|-------|
| `//apps/line_analyzer:app_line_analyzer` | ✅ | Requires Qt5, OpenGL |

### Tests

**54/54 tests passing** ✅

```bash
bazel test //libs/...
```

## Adding New Features

To add a new feature flag:

1. Add `bool_flag` and `config_setting` in `bazel/BUILD.bazel`:

```python
bool_flag(
    name = "enable_my_feature",
    build_setting_default = False,
)

config_setting(
    name = "my_feature_enabled",
    flag_values = {":enable_my_feature": "true"},
)
```

1. Add helper function in `bazel/features.bzl`:

```python
def if_my_feature_enabled(if_true, if_false = []):
    return select({
        "//bazel:my_feature_enabled": if_true,
        "//conditions:default": if_false,
    })
```

1. Use in BUILD files:

```python
load("//bazel:features.bzl", "if_my_feature_enabled")

cc_library(
    name = "my_lib",
    srcs = if_my_feature_enabled(["feature_impl.cpp"]),
    deps = if_my_feature_enabled(["@feature_dep//:lib"]),
)
```

## CMake Comparison

| Feature | CMake | Bazel |
|---------|-------|-------|
| Configure | `cmake -B build` | Automatic (`.bazelrc`) |
| Build | `cmake --build build` | `bazel build //...` |
| Tests | `ctest` | `bazel test //...` |
| Run | `./build/bin/app` | `bazel run //app:app` |
| Clean | `rm -rf build` | `bazel clean` |
| Full clean | - | `bazel clean --expunge` |

Both build systems can be used in parallel.

## Troubleshooting

### Qt5 not found

```bash
# Check Qt5 installation
dpkg -l | grep qt5
pkg-config --modversion Qt5Core

# Re-run feature detection
./tools/scripts/detect_bazel_features.sh --force
```

### OpenGL not found

```bash
# Install OpenGL development packages
sudo apt install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev

# Re-run feature detection
./tools/scripts/detect_bazel_features.sh --force
```

### Build cache issues

```bash
# Clear Bazel cache
bazel clean --expunge

# Remove disk cache
rm -rf ~/.cache/bazel/line_extraction
```

### Query available targets

```bash
# List all targets
bazel query //...

# List library targets
bazel query //libs/...

# Show dependencies
bazel query 'deps(//libs/lsd:lib_lsd)'

# Show reverse dependencies
bazel query 'rdeps(//..., //libs/utility:lib_utility)'
```

## Performance Tips

```bash
# Parallel builds (auto-configured)
bazel build --jobs=auto //...

# Disk cache (enabled by default in .bazelrc)
build --disk_cache=~/.cache/bazel/line_extraction

# Keep going on errors
bazel build --keep_going //...

# Verbose build output
bazel build --subcommands //libs/utility:lib_utility
```

## Known Limitations

### OpenCV Photo Module

The BCR OpenCV build doesn't include the `photo` module (used for `FastNlMeansOperator`).
Code is guarded with `#ifdef HAVE_OPENCV_PHOTO`. Use CMake build if you need this feature.

### System Dependencies

Qt5 and OpenGL are system dependencies, not hermetically built. This means:

- Builds may differ across machines with different Qt5/OpenGL versions
- For reproducible builds, use the Docker development environment

## API Documentation

Generate Doxygen documentation:

```bash
# Generate documentation
bazel run //:doxygen

# Or run the script directly
./tools/scripts/generate_doxygen.sh

# View documentation
# Output: build/doc/html/index.html
```

**Requirements:** Doxygen must be installed on the host system:

```bash
sudo apt install doxygen graphviz
```

**Note:** Uses `bazel run` (not `bazel build`) to execute in the workspace directory
and persist the output to `build/doc/html/`.

## Further Reading

- [Bazel Documentation](https://bazel.build/docs)
- [Bzlmod Migration Guide](https://bazel.build/external/migration)
- [Bazel Central Registry](https://registry.bazel.build/)
- [CMake Build System](CMAKE.md)
- [Docker Setup](DOCKER.md)
- [WSL Setup](WSL.md)
