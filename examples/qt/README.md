# Qt Visualization Examples

Qt-based interactive visualization examples for 3D plotting and data display.

[← Back to Examples](../README.md)

**Note:** These examples require Qt5 to be installed and detected during build.

## Contents

This directory contains Qt5-based visualization utilities:

| Directory | Description |
|-----------|-------------|
| `autoswitch/` | Automatic view switching |
| `axes/` | 3D axis rendering |
| `enrichments/` | Plot enrichments and annotations |
| `mesh2/` | 3D mesh visualization |

## Examples

| Example | Description |
|---------|-------------|
| [test_qplot3d.cpp](test_qplot3d.cpp) | 3D plotting with qplot3d library |

## Building

**Qt Detection:**
Run the feature detection script to enable Qt support:
```bash
./tools/scripts/detect_bazel_features.sh
```

**Bazel:**
```bash
bazel build //examples/qt:all  # If Qt5 detected
bazel run //examples/qt:test_qplot3d
```

**CMake:**
```bash
cmake .. -DBUILD_WITH_QT=ON
make  # from build directory
```

## Requirements

- Qt5 Core
- Qt5 Widgets
- Qt5 OpenGL (for 3D rendering)
- X11 or Wayland display server (Linux)

See [docs/WSL.md](../../docs/WSL.md) for WSL GUI setup.

## Purpose

These examples demonstrate:
- Interactive 3D visualization
- Qt integration with OpenGL
- Real-time data plotting
- Custom Qt widgets for scientific visualization

## Related

- [apps/line_analyzer](../../apps/line_analyzer/) - Full Qt line analysis application
