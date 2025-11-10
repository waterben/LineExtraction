# Quick Start: Bazel Feature Flags

## TL;DR

Qt5, CUDA, and OpenGL are now **disabled by default** and can be enabled via command-line flags.

## Basic Commands

```bash
# Build everything (Qt5 disabled)
bazel build //...

# Run all tests (Qt5 disabled)
bazel test //...

# Build with Qt5 support
bazel build --//bazel:enable_qt5=true --//bazel:enable_opengl=true //apps/line_analyzer
```

## Per-Machine Configuration

Create `.bazelrc.user` in the project root (automatically git-ignored):

```bash
# Disable Qt5 on this machine
build --//bazel:enable_qt5=false
build --build_tag_filters=-qt5
test --test_tag_filters=-qt5
```

Or enable it:

```bash
# Enable Qt5 on this machine (when Qt5 sandbox issue is fixed)
build --//bazel:enable_qt5=true
build --//bazel:enable_opengl=true
```

## What Changed?

- Qt5 targets won't build unless you pass `--//bazel:enable_qt5=true`
- No more Qt5 header errors when building `//...`
- Same approach can be used for CUDA in the future

## See Also

- `bazel/README.md` - Full documentation
- `BAZEL_STATUS.md` - Current status and known issues
