# Bazel Build System for LineExtraction

Bazel is the **primary build system** for LineExtraction. It provides hermetic builds,
automatic dependency management via the Bazel Central Registry (BCR), and reproducible
builds across different machines. CMake is maintained for legacy support.

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

**Note:** OpenCV photo module is always available (requires opencv >= 4.12.0.bcr.1 in [MODULE.bazel](../MODULE.bazel)).

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

Use sanitizers to detect memory errors and data races:

```bash
# Address Sanitizer (memory errors, buffer overflows, use-after-free)
bazel test --config=asan //libs/...
bazel run --config=asan //libs/utility:test_value

# Thread Sanitizer (data races, deadlocks)
bazel test --config=tsan //libs/...
```

**Note:** Sanitizer builds are slower and use more memory. Don't combine multiple sanitizers.

### Warning Configuration

The project uses strict compiler warnings matching CMake configuration (see `tools/bazel/copts.bzl`):

- **LE_COPTS**: Full warning set with `-Werror` (used for project code)
- **LE_THIRD_PARTY_COPTS**: Reduced warnings for third-party C++ code
- **LE_THIRD_PARTY_C_COPTS**: Reduced warnings for third-party C code

Warning flags include: `-Wall -Wextra -Wconversion -Weffc++ -Werror` and more.

For libraries with strict warnings, use LE_COPTS directly:

```python
load("//tools/bazel:copts.bzl", "LE_COPTS")

cc_library(
    name = "my_lib",
    copts = LE_COPTS,
    # ...
)
```

### External Header Warnings

External headers (OpenCV, Eigen) are treated as system includes via the patched
OpenCV module in the local registry (`opencv 4.12.0.bcr.1-LE`). This uses
`-isystem` like CMake does, so strict warnings only apply to project code.

See `bazel/README.md` for details on compiler options.

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

## Dependencies (February 2026)

Managed via Bzlmod (`MODULE.bazel`):

| Dependency | Version | Source |
|------------|---------|--------|
| OpenCV | 4.12.0.bcr.1 | BCR |
| opencv_contrib | 4.12.0.bcr.1 | Local Registry |
| arpack++ | 2.3.1 | Local Registry |
| SuperLU | 7.0.0 | Local Registry |
| libjpeg_turbo | 3.1.3.bcr.2 | BCR |
| GoogleTest | 1.15.2 | BCR |
| Google Benchmark | 1.9.1 | BCR |
| Eigen | 3.4.0 | http_archive |
| dlib | 19.24.7 | Local Registry |
| bazel_skylib | 1.9.0 | BCR |
| rules_cc | 0.2.15 | BCR |
| rules_python | 1.1.0 | BCR |

### opencv_contrib Modules

The following opencv_contrib modules are available for use:

| Module | Bazel Target | Description |
|--------|--------------|-------------|
| xfeatures2d | `@opencv_contrib//:xfeatures2d` | Extra 2D features (SIFT, SURF, DAISY) |
| line_descriptor | `@opencv_contrib//:line_descriptor` | Line segment detection and description |
| ximgproc | `@opencv_contrib//:ximgproc` | Extended image processing |
| xphoto | `@opencv_contrib//:xphoto` | Extra photo processing |
| features2d | `@opencv_contrib//:features2d` | Feature detection/description |
| tracking | `@opencv_contrib//:tracking` | Visual object tracking |

**Note:** opencv_contrib modules require linking against both OpenCV core and the specific module.

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

### Benchmarks

Performance benchmarks using Google Benchmark:

| Target | Algorithms | Description |
|--------|------------|-------------|
| `//libs/lsd:bench_lsd` | LsdCC, LsdCP, LsdEL, LsdEP, LsdBurns, LsdFBW, LsdFGioi, LsdEDLZ | All 8 LSD variants |
| `//libs/edge:bench_edge` | Sobel, Scharr, NMS, EsdSimple, EsdDrawing, EsdLinking, EsdPattern | Gradient, NMS and ESD benchmarks |

```bash
# Run all LSD benchmarks
bazel run //libs/lsd:bench_lsd

# Run all edge benchmarks
bazel run //libs/edge:bench_edge

# Filter specific algorithm
bazel run //libs/lsd:bench_lsd -- --benchmark_filter="BM_LsdCC"

# Export to JSON
bazel run //libs/lsd:bench_lsd -- --benchmark_format=json > lsd_bench.json
```

## Data Dependencies (Datasets)

The project supports external image datasets for evaluation and testing.

### BSDS500 (Auto-Download)

The Berkeley Segmentation Dataset is automatically downloaded when needed:

```starlark
# In BUILD.bazel
cc_test(
    name = "my_eval_test",
    data = ["@bsds500//:all"],
    deps = ["//libs/eval:lib_eval"],
)
```

**Bazel Targets:**

- `@bsds500//:all` - All images (~500 images, ~50MB)
- `@bsds500//:train` - Training set
- `@bsds500//:test` - Test set
- `@bsds500//:val` - Validation set

### MDB (Manual Setup)

The Middlebury Stereo Dataset requires manual download due to size (~1GB):

```bash
# Download and setup MDB dataset
./tools/scripts/setup_mdb_dataset.sh --resolution all

# Or only quarter resolution (~50MB)
./tools/scripts/setup_mdb_dataset.sh --resolution Q
```

**Bazel Targets:**

- `//resources/datasets:mdb_q` - Quarter resolution
- `//resources/datasets:mdb_h` - Half resolution
- `//resources/datasets:mdb_f` - Full resolution

### Using Datasets in C++ Code

```cpp
#include <eval/runfiles.hpp>

int main(int argc, char** argv) {
    auto runfiles = lsfm::Runfiles::Create(argv[0]);

    // Automatic path resolution (Bazel or CMake)
    std::string bsds_path = runfiles->Rlocation("bsds500/BSDS500/data/images/train");
    std::string mdb_path = runfiles->Rlocation("line_extraction/resources/datasets/MDB/MiddEval3-Q");
}
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

| Feature | Bazel (Primary) | CMake (Legacy) |
|---------|----------------|----------------|
| **Dependency Management** | Automatic via BCR | Manual download scripts (complex, error-prone) |
| **Build Hermeticity** | Fully hermetic | System-dependent |
| **Caching** | Built-in distributed cache | Limited |
| **Configure** | Automatic (`.bazelrc`) | `cmake -B build` |
| **Build** | `bazel build //...` | `cmake --build build` |
| **Tests** | `bazel test //...` | `ctest` |
| **Run** | `bazel run //app:app` | `./build/bin/app` |
| **Clean** | `bazel clean` | `rm -rf build` |
| **Full clean** | `bazel clean --expunge` | - |
| **Qt5 Support** | Optional (auto-detected) | Optional (auto-detected) |
| **OpenCV Photo** | ✅ Always (BCR >= 4.12.0.bcr.1) | ✅ Always |

**Why Bazel is preferred:**

- **Simpler dependencies:** All managed via `MODULE.bazel` - no complex download scripts
- **Reproducible:** Same inputs → same outputs, regardless of machine
- **Faster:** Intelligent caching and incremental builds
- **Modern:** Active development and growing ecosystem

Both build systems can be used in parallel, but Bazel is recommended for all new development.

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

# Build without using disk cache (for a single build)
bazel build --config=nocache //...
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
