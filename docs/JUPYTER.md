# Jupyter Notebook Integration

Interactive Jupyter Notebooks for exploring the LineExtraction Python bindings.

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
   examples/notebooks/line_extraction_bindings.ipynb
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
jupyter lab examples/notebooks/line_extraction_bindings.ipynb
```

JupyterLab opens automatically in your default browser at `http://localhost:8888`.

### Option C: Classic Notebook (Browser)

```bash
source .venv/bin/activate
jupyter notebook examples/notebooks/line_extraction_bindings.ipynb
```

## Available Notebooks

| Notebook | Description |
|----------|-------------|
| [`line_extraction_bindings.ipynb`](../examples/notebooks/line_extraction_bindings.ipynb) | Complete tour of all 5 Python binding modules with interactive visualizations |

### Notebook Sections

The main notebook covers:

| # | Section | Modules |
|---|---------|---------|
| 1 | Import & Setup | all |
| 2 | Gradient Filters (Sobel, Scharr, Prewitt) | `le_imgproc` |
| 3 | Data Type Mapping (Range, Value, FilterData) | `le_imgproc` |
| 4 | Geometry Primitives (Line, LineSegment, Polygon) | `le_geometry` |
| 5 | Callback Integration (Python Subclassing) | `le_eval` |
| 6 | Error Handling & Exception Propagation | all |
| 7 | Performance: C++ vs Pure Python | `le_imgproc`, `le_lsd` |
| 8 | Edge Detection Pipeline | `le_edge` |
| 9 | Real-World Use Case — All 9 LSD Detectors | all |

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
for lib in ["imgproc", "edge", "geometry", "eval", "lsd"]:
    p = workspace / f"bazel-bin/libs/{lib}/python"
    if p.exists():
        sys.path.insert(0, str(p))

import le_imgproc, le_edge, le_geometry, le_eval, le_lsd
```

## Further Reading

- [Python Bindings Overview](../README.md#python-bindings)
- [le_imgproc API](../libs/imgproc/python/README.md)
- [le_edge API](../libs/edge/python/README.md)
- [le_geometry API](../libs/geometry/python/README.md)
- [le_eval API](../libs/eval/python/README.md)
- [le_lsd API](../libs/lsd/python/README.md)
- [Docker Setup](DOCKER.md)
- [WSL Setup](WSL.md)
