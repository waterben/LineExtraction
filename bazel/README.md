# Bazel Feature Flags

This directory contains feature flags and configuration for conditional compilation, similar to CMake's `option()` command.

## Available Features

### Qt5 Support

- **Flag**: `--//bazel:enable_qt5=true/false`
- **Default**: `false`
- **Description**: Enable Qt5-dependent libraries and applications
- **Affects**:
  - `//third-party/qplot:lib_qplot`
  - `//third-party/qplot3d:lib_qplot3d`
  - `//apps/line_analyzer:app_line_analyzer`

### CUDA Support

- **Flag**: `--//bazel:enable_cuda=true/false`
- **Default**: `false`
- **Description**: Enable CUDA acceleration (future use)
- **Affects**: OpenCV CUDA modules (when available)

### OpenGL Support

- **Flag**: `--//bazel:enable_opengl=true/false`
- **Default**: `false`
- **Description**: Enable OpenGL-dependent features
- **Affects**:
  - `//third-party/qplot3d:lib_qplot3d` (requires OpenGL for 3D rendering)

### OpenCV Photo Module

- **Flag**: `--//bazel:enable_photo=true/false`
- **Default**: `true` (available since opencv 4.12.0.bcr.1)
- **Description**: Enable OpenCV photo module for advanced denoising
- **Affects**:
  - `FastNlMeansOperator` in `//libs/imgproc`

**Note:** The photo module is now available in BCR and enabled by default.

## Usage Examples

### Build everything (without optional features)

```bash
bazel build //...
bazel test //...
```

### Build with Qt5 support

```bash
bazel build --//bazel:enable_qt5=true --//bazel:enable_opengl=true //apps/line_analyzer:app_line_analyzer
```

### Build specific library tests

```bash
bazel test //libs/...
```

### Exclude Qt5 targets from tests

```bash
bazel test --build_tag_filters=-qt5 --test_tag_filters=-qt5 //...
```

## Configuration Files

You can create a `.bazelrc.user` file (git-ignored) to set your preferred defaults:

```bash
# .bazelrc.user
# Enable Qt5 by default on this machine
build --//bazel:enable_qt5=true
build --//bazel:enable_opengl=true

# Or disable Qt5 targets entirely
build --build_tag_filters=-qt5
test --test_tag_filters=-qt5
```

## Implementation Details

Features are implemented using:

- `bool_flag()` - Defines a boolean command-line flag
- `config_setting()` - Creates a condition that can be used with `select()`
- `select()` - Conditionally includes dependencies and sources
- Helper functions in `features.bzl` - Simplify conditional inclusion

## Adding New Features

To add a new feature flag:

1. Add a `bool_flag` and `config_setting` in `BUILD.bazel`:

```python
bool_flag(
    name = "enable_my_feature",
    build_setting_default = False,
)

config_setting(
    name = "my_feature_enabled",
    flag_values = {
        ":enable_my_feature": "true",
    },
)
```

1. Add a helper function in `features.bzl`:

```python
def if_my_feature_enabled(if_true, if_false = []):
    return select({
        "//bazel:my_feature_enabled": if_true,
        "//conditions:default": if_false,
    })
```

1. Use it in your BUILD files:

```python
load("//bazel:features.bzl", "if_my_feature_enabled")

cc_library(
    name = "my_lib",
    srcs = if_my_feature_enabled(["feature_impl.cpp"]),
    deps = if_my_feature_enabled(["@feature_dependency//:lib"]),
)
```

## Current Status

- ✅ Qt5: Configurable (disabled by default due to sandbox issues)
- ✅ OpenGL: Configurable (disabled by default)
- ✅ OpenCV Photo: Available and enabled by default (BCR >= 4.12.0.bcr.1)
- ⏸️ CUDA: Flag defined but not yet implemented

## Migration from CMake

| CMake Option | Bazel Flag | Notes |
|-------------|------------|-------|
| `ENABLE_QT` | `--//bazel:enable_qt5` | Qt5 support |
| `ENABLE_CUDA` | `--//bazel:enable_cuda` | CUDA support (TBD) |
| `WITH_OPENGL` | `--//bazel:enable_opengl` | OpenGL support |
| N/A | `--//bazel:enable_photo` | OpenCV photo module |
