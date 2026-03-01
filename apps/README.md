# Applications

Interactive applications demonstrating LineExtraction library capabilities.

## Available Applications

### [Line Analyzer](line_analyzer/)

Qt-based interactive GUI for comprehensive line detection analysis and visualization.

**Features:**

- Real-time line detection with multiple algorithms
- Interactive parameter tuning
- Visual comparison of different detectors
- Profile analysis and precision optimization
- Edge detection visualization
- Support for various gradient operators and phase congruency methods

**Building:**

```bash
# Bazel (requires Qt5)
./tools/scripts/detect_bazel_features.sh  # One-time setup
bazel run //apps/line_analyzer:app_line_analyzer

# CMake (legacy)
mkdir build && cd build
cmake -DBUILD_APPS=ON ..
make
./bin/app_line_analyzer
```

**Requirements:**

- Qt5 (QtWidgets, QtCore)
- OpenCV with GUI support
- LineExtraction libraries (edge, lsd, imgproc, geometry)

**Supported Detectors:**

- **LSD EL** - Edge Linking-based Line Segment Detector
- **LSD EL PCGF** - Phase Congruency Gaussian Filter variant
- **LSD EL PCSQ** - Phase Congruency Square variant
- **LSD EL SUSAN** - SUSAN edge detector variant
- **LSD EL RMG/RCMG** - Color gradient variants

**Tools:**

- **Profile Analyzer** - Analyze edge profiles and sub-pixel precision
- **Precision Optimizer** - Optimize detector parameters for precision
- **3D Profile Plot** - Visualize optimization functions
- **Continuity Optimizer** - Merge and connect line segments

## Related Documentation

- [Main README](../README.md) - Project overview and setup
- [Examples](../examples/README.md) - Command-line examples
- [libs/lsd](../libs/lsd/README.md) - Line Segment Detection library
- [libs/edge](../libs/edge/README.md) - Edge detection library
- [libs/algorithm](../libs/algorithm/README.md) - Post-processing, accuracy, parameter optimization
- [Resources](../resources/README.md) - Datasets, ground truth, presets
- [docs/BAZEL.md](../docs/BAZEL.md) - Build system documentation
