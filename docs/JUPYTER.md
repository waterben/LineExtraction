# Jupyter Notebook Integration

Interactive Jupyter Notebooks for learning and exploring the LineExtraction library and its Python bindings. The **tutorial series** provides a structured learning path — from computer vision basics through full detection pipelines and performance evaluation — making them the ideal starting point for new developers working with the Python API.

## Prerequisites

1. **Python Bindings built** — the native C++ modules must be compiled first:

   ```bash
   bazel build //libs/...
   ```

2. **Python environment active** — the `.venv` must be set up (includes Jupyter):

   ```bash
   # Already done if you ran setup_local_dev.sh
   source .venv/bin/activate

   # Verify Jupyter is available
   jupyter --version
   ```

   If Jupyter is missing, update your environment:

   ```bash
   uv sync
   ```

## Quick Start

### Option A: VS Code (Recommended)

1. Open the notebook in VS Code:

   ```
   examples/notebooks/intro_line_extraction_overview.ipynb
   ```

2. VS Code will prompt you to select a kernel — choose the `.venv` Python interpreter.

3. Click **Run All** or step through cells individually.

> **Tip:** Install the [Jupyter VS Code extension](https://marketplace.visualstudio.com/items?itemName=ms-toolsai.jupyter) for the best experience.

### Option B: JupyterLab (Browser)

```bash
# Activate the environment
source .venv/bin/activate

# Start JupyterLab from the project root
jupyter lab

# Or open the specific notebook directly
jupyter lab examples/notebooks/intro_line_extraction_overview.ipynb
```

JupyterLab opens automatically in your default browser at `http://localhost:8888`.

### Option C: Classic Notebook (Browser)

```bash
source .venv/bin/activate
jupyter notebook examples/notebooks/intro_line_extraction_overview.ipynb
```

## Available Notebooks

All notebooks are located in [`examples/notebooks/`](../examples/notebooks/).

### Tutorial Series (Recommended Learning Path)

The tutorial series is the recommended way to learn the Python bindings. Work through them in order:

| # | Notebook | Description | Modules |
|---|----------|-------------|---------|
| 0 | [`intro_cv_primer.ipynb`](../examples/notebooks/intro_cv_primer.ipynb) | **Computer Vision Primer** — Self-contained introduction to CV concepts (images as arrays, gradients, edge detection, lines & LSD taxonomy). Uses only NumPy + Matplotlib — no library dependencies. Ideal prerequisite for newcomers to computer vision. | — |
| 1 | [`tutorial_1_fundamentals.ipynb`](../examples/notebooks/tutorial_1_fundamentals.ipynb) | **Library Fundamentals** — Foundational modules: gradient filters, geometry primitives (Line, LineSegment, Polygon), drawing utilities, ValueManager configuration, and test image loading. | `le_imgproc`, `le_geometry` |
| 2 | [`tutorial_2_pipelines.ipynb`](../examples/notebooks/tutorial_2_pipelines.ipynb) | **Edge & Line Detection Pipelines** — Full detection pipeline from edge sources through NMS, ESD variants, all 9 LSD detectors, grand comparison, line optimization, and noise robustness analysis. | `le_edge`, `le_lsd`, `le_imgproc`, `le_geometry` |
| 3 | [`tutorial_3_evaluation.ipynb`](../examples/notebooks/tutorial_3_evaluation.ipynb) | **Performance Evaluation Framework** — Deep dive into benchmarking: StringTable, performance primitives, data providers, custom tasks, CVPerformanceTest orchestrator, result analysis, and full benchmark runs. | `le_eval`, `le_lsd`, `le_edge` |
| 4 | [`tutorial_4_algorithm.ipynb`](../examples/notebooks/tutorial_4_algorithm.ipynb) | **Algorithm Library** — Post-processing (LineMerge, LineConnect, LineContinuityOptimizer), accuracy evaluation, detector profiling, hyperparameter search (ParamOptimizer), sub-pixel refinement (PrecisionOptimize), and preset management (PresetStore). | `le_algorithm`, `le_geometry` |
| 5 | [`tutorial_5_advanced_filters.ipynb`](../examples/notebooks/tutorial_5_advanced_filters.ipynb) | **Advanced Gradient & Quadrature Filters** — SUSAN gradient, RCMG color/grayscale morphological gradient, and all four quadrature filters (G2, LGF, S, SF) with parameter sweeps and comparative analysis. | `le_imgproc` |
| 6 | [`tutorial_6_image_operators.ipynb`](../examples/notebooks/tutorial_6_image_operators.ipynb) | **Image Operators & Pipelines** — All 13 ImageOperator subclasses (blur, denoise, noise, geometric transforms), PipelineOperator composition, custom Python operators, and data augmentation workflows. | `le_imgproc` |
| 7 | [`tutorial_7_3d_geometry.ipynb`](../examples/notebooks/tutorial_7_3d_geometry.ipynb) | **3D Geometry & Camera Models** — Line3, LineSegment3, Plane, Pose, Camera hierarchy (CameraCV, CameraPluecker, Camera2P), 3D-to-2D projection, and interactive Rerun 3D visualization. | `le_geometry`, Rerun |

### Additional Notebooks

| Notebook | Description | Modules |
|----------|-------------|---------|
| [`intro_line_extraction_overview.ipynb`](../examples/notebooks/intro_line_extraction_overview.ipynb) | **API Reference Guide** — Compact tour of all Python binding modules with interactive visualizations, zero-copy NumPy integration, Python subclassing, and performance comparisons. | all |
| [`demo_pytorch_esd.ipynb`](../examples/notebooks/demo_pytorch_esd.ipynb) | **PyTorch Integration Demo** — Combines PyTorch-based object segmentation (SAM / YOLO) with the ESD line extraction framework. Interactive click-to-segment, automatic instance segmentation, and contour-to-line-segment conversion. | `le_edge`, PyTorch |
| [`demo_rerun_lsd.ipynb`](../examples/notebooks/demo_rerun_lsd.ipynb) | **Rerun LSD Visualization** — Interactive line segment visualization with inline [Rerun.io](https://rerun.io/) viewer widget. | `le_lsd`, Rerun |
| [`demo_line_features.ipynb`](../examples/notebooks/demo_line_features.ipynb) | **Line Feature Demo** — LBD/LR descriptors, brute-force matching, GlobalRotationFilter, interactive [Rerun.io](https://rerun.io/) visualization. | `le_lfd`, `le_lsd`, Rerun |

## Module Import Path

The Python bindings are native C++ shared libraries built by Bazel. The notebook automatically adds the Bazel output directories to `sys.path`:

```python
import sys, pathlib

workspace = pathlib.Path.cwd()
# ... (handled in notebook cell 1)

import le_imgproc
import le_edge
import le_geometry
import le_eval
import le_lsd
import le_algorithm
```

If imports fail, make sure you have built the bindings:

```bash
bazel build //libs/...
```

## Environment Setup

### Local Development

Jupyter is installed automatically by `setup_local_dev.sh` via `pyproject.toml` dependencies:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

### Docker / DevContainer

Jupyter is included in the Docker image via the same `pyproject.toml`. After opening the DevContainer:

```bash
jupyter lab
```

### Manual Installation

If you need to add Jupyter to an existing environment:

```bash
source .venv/bin/activate
uv sync   # Installs all deps from pyproject.toml including Jupyter
```

## WSL Notes

For WSL users, JupyterLab runs inside WSL and opens in the Windows browser automatically. If the browser does not open:

```bash
# Set browser explicitly
export BROWSER="/mnt/c/Program Files/Google/Chrome/Application/chrome.exe"
jupyter lab
```

Or copy the URL from the terminal output (with token) and paste it into your Windows browser.

## Troubleshooting

### "Module not found: le_imgproc"

The native bindings are not on `sys.path`. Ensure:

1. You built the bindings: `bazel build //libs/...`
2. The notebook's first code cell ran successfully (it adds paths automatically)
3. You are using the correct Python kernel (`.venv`)

### "No kernel found"

Install the IPython kernel into your venv:

```bash
source .venv/bin/activate
python -m ipykernel install --user --name=lineextraction --display-name "LineExtraction"
```

### Plots not displaying

Ensure `matplotlib` is installed (it is part of `pyproject.toml` dependencies):

```bash
source .venv/bin/activate
python -c "import matplotlib; print(matplotlib.__version__)"
```

For headless environments (Docker without display), matplotlib uses the `Agg` backend automatically. In the notebook, `%matplotlib inline` is not needed for JupyterLab but can be added for classic notebook compatibility.

### JupyterLab port already in use

```bash
jupyter lab --port 8889
```

## Creating New Notebooks

Place new notebooks under `examples/notebooks/`. The import pattern for the bindings is:

```python
import sys, pathlib

# Find workspace root (parent of examples/)
workspace = pathlib.Path.cwd()
while workspace.name != "LineExtraction" and workspace != workspace.parent:
    workspace = workspace.parent

# Add Bazel output paths
for lib in ["imgproc", "edge", "geometry", "eval", "lsd", "lfd", "algorithm"]:
    p = workspace / f"bazel-bin/libs/{lib}/python"
    if p.exists():
        sys.path.insert(0, str(p))

import le_imgproc, le_edge, le_geometry, le_eval, le_lsd, le_lfd, le_algorithm
```

## Further Reading

- [Python Bindings Overview](../README.md#python-bindings)
- [Rerun Visualization (Python, C++, Notebooks)](RERUN.md)
- [le_imgproc API](../libs/imgproc/python/README.md)
- [le_edge API](../libs/edge/python/README.md)
- [le_geometry API](../libs/geometry/python/README.md)
- [le_eval API](../libs/eval/python/README.md)
- [le_lsd API](../libs/lsd/python/README.md)
- [le_lfd API](../libs/lfd/python/README.md)
- [le_algorithm API](../libs/algorithm/python/README.md)
- [Docker Setup](DOCKER.md)
- [WSL Setup](WSL.md)
