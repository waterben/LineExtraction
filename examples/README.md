# LineExtraction Examples

Demonstration programs showcasing the capabilities of the LineExtraction library. Each subdirectory focuses on a specific library module and contains standalone example binaries.

## Directory Structure

| Directory | Library | Description |
|-----------|---------|-------------|
| [edge/](edge/) | `libs/edge` | Edge detection, NMS, zero-crossing, and edge linking |
| [geometry/](geometry/) | `libs/geometry` | Geometric primitives, camera models, stereo vision |
| [imgproc/](imgproc/) | `libs/imgproc` | Gradient operators, FFT, steerable filters, pyramids |
| [lfd/](lfd/) | `libs/lfd` | Line feature descriptor matching (stereo, video, motion) |
| [lsd/](lsd/) | `libs/lsd` | Line Segment Detection algorithm variants |
| [other/](other/) | — | Miscellaneous demos (Hough transform, **Rerun general demo**) |
| [qt/](qt/) | — | Qt-based visualization (currently empty) |
| [thesis/](thesis/) | `libs/eval` | Figure generation for thesis chapters |
| [notebooks/](notebooks/) | all | **Jupyter tutorial series and interactive demos** (Python) |

## Building

### Bazel (Recommended)

```bash
# Build all examples
bazel build //examples/...

# Build a specific category
bazel build //examples/edge:all
bazel build //examples/lsd:all

# Run a specific example
bazel run //examples/lsd:lsd
bazel run //examples/edge:edge_test
```

### CMake (Legacy)

```bash
cd build && cmake .. && cmake --build . -j$(nproc)
```

## Usage

Most examples accept an optional image path as command-line argument. Without arguments, the built-in test image (`windmill.jpg`) is used:

```bash
bazel run //examples/edge:nms_test              # default test image
bazel run //examples/edge:nms_test -- image.jpg # custom image
```

Test images are resolved via the `TestImages` utility:

```cpp
#include <utility/test_images.hpp>

int main(int argc, char** argv) {
    lsfm::TestImages::init(argv[0]);
    std::string file = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();
}
```

## Project Layout

Each example subdirectory follows the same layout:

```
<category>/
  BUILD.bazel       # Bazel build targets
  CMakeLists.txt    # Legacy CMake build
  README.md         # Documentation
  src/              # Source files (.cpp)
```

## Jupyter Notebooks (Python Tutorials)

The [`notebooks/`](notebooks/) directory contains interactive Jupyter Notebooks — the recommended way to learn the Python bindings and get started with the library.

### Tutorial Series (Recommended Learning Path)

Work through the tutorials in order for a structured introduction:

| # | Notebook | Description |
|---|----------|-------------|
| 0 | [`cv_primer.ipynb`](notebooks/cv_primer.ipynb) | **Computer Vision Primer** — CV basics with pure NumPy/Matplotlib (no library dependencies). Ideal prerequisite for newcomers. |
| 1 | [`tutorial_1_fundamentals.ipynb`](notebooks/tutorial_1_fundamentals.ipynb) | **Library Fundamentals** — Gradient filters, geometry primitives, drawing, ValueManager, test images. |
| 2 | [`tutorial_2_pipelines.ipynb`](notebooks/tutorial_2_pipelines.ipynb) | **Edge & Line Detection Pipelines** — Full pipeline: edge sources, NMS, ESD, all 9 LSD detectors, optimization, noise robustness. |
| 3 | [`tutorial_3_evaluation.ipynb`](notebooks/tutorial_3_evaluation.ipynb) | **Performance Evaluation** — Benchmarking framework: data providers, custom tasks, result analysis, full benchmarks. |

### Additional Notebooks

| Notebook | Description |
|----------|-------------|
| [`line_extraction_bindings.ipynb`](notebooks/line_extraction_bindings.ipynb) | **API Reference Guide** — Compact tour of all 5 Python modules with visualizations and performance comparisons. |
| [`pytorch_esd_demo.ipynb`](notebooks/pytorch_esd_demo.ipynb) | **PyTorch Integration** — Object segmentation (SAM/YOLO) combined with ESD line extraction. |
| [`rerun_lsd_demo.ipynb`](notebooks/rerun_lsd_demo.ipynb) | **Rerun LSD Visualization** — Interactive line segment visualization with inline [Rerun.io](https://rerun.io/) viewer widget. |

### Running Notebooks

```bash
# Prerequisites: build the Python bindings
bazel build //libs/...

# Option A: Open in VS Code (recommended)
# Just open the .ipynb file and select the .venv kernel

# Option B: JupyterLab
source .venv/bin/activate
jupyter lab examples/notebooks/
```

See [`docs/JUPYTER.md`](../docs/JUPYTER.md) for detailed setup instructions.

## See Also

- [Evaluation](../evaluation/README.md) — Performance benchmarks and precision evaluations
- [Apps](../apps/README.md) — Interactive applications (Line Analyzer)
- [Main README](../README.md) — Project overview and setup
