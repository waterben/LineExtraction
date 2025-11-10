# Building with Bazel

This project now supports both CMake and Bazel build systems.

## Quick Start

```bash
# Build libraries
bazel build //libs/...

# Run tests
bazel test //libs/...

# Build with Clang
bazel build --config=clang //libs/...

# Debug build
bazel build --config=debug //libs/...
```

## Prerequisites

- Bazel 8.4.2+ (managed via `.bazelversion`)
- GCC or Clang
- CMake-built OpenCV (run CMake build first)

## Documentation

See detailed documentation in:

- `BAZEL_STATUS.md` - Current status and known issues
- `docs/BAZEL_BUILD.md` - Complete build guide
