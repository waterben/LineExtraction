# LIMAP Integration Guide

[CVG LIMAP](https://github.com/cvg/limap) is an **optional** dependency for multi-view
3D line mapping. It provides triangulation, track building, and bundle adjustment
for line features across multiple views.

> **Warning:** The `limap` package on PyPI (`pip install limap`) is an **unrelated**
> project (LimerBoy Mapper). Do **not** install it. Follow this guide instead.

## Requirements

| Requirement         | Version           | Provided by this project?          |
|---------------------|-------------------|------------------------------------|
| Python              | 3.10 or 3.11      | Yes — `.python-version` + `pyproject.toml` |
| CMake               | >= 3.17           | Yes — via `setup_local_dev.sh`     |
| build-essential     | any               | Yes — via `setup_local_dev.sh`     |
| Qt5                 | any               | Yes — via `setup_local_dev.sh`     |
| COLMAP              | >= 3.9            | **No** — must install separately   |
| COLMAP build deps   | (see below)       | **No** — must install separately   |
| CUDA                | (optional)        | **No** — only for DL detectors     |

## Step-by-Step Installation

### Step 1: Verify Python Version

The project is configured for Python 3.10 via `.python-version` and `pyproject.toml`
(`requires-python = ">=3.10,<3.12"`).

```bash
.venv/bin/python --version
# Expected: Python 3.10.x
```

If not on 3.10, recreate the venv:

```bash
uv venv --python 3.10
uv sync
```

### Step 2: Run the Project Setup (if not done already)

The project's setup script installs many of the system dependencies that LIMAP
also needs. Run it first to avoid duplicate work:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

This installs from `docker/base/common_packages.txt` and provides:

| Package              | Needed by LIMAP? | Status after setup |
|----------------------|------------------|--------------------|
| `build-essential`    | Yes              | **Already installed** |
| `cmake`              | Yes (>= 3.17)   | **Already installed** |
| `qtbase5-dev`        | Yes              | **Already installed** |
| `libqt5opengl5-dev`  | Yes              | **Already installed** |
| `gcc`, `g++`, `clang`| Yes              | **Already installed** |

### Step 3: Install Additional LIMAP Dependencies

After running the project setup, these packages are still **missing** and must
be installed for LIMAP. On **Ubuntu 24.04**:

```bash
# COLMAP (Structure-from-Motion backend, required by LIMAP)
sudo apt-get install -y colmap

# COLMAP/LIMAP build dependencies not included in this project
sudo apt-get install -y \
    ninja-build \
    libeigen3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libgmock-dev \
    libsqlite3-dev \
    libglew-dev \
    libcgal-dev \
    libceres-dev \
    libhdf5-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev
```

These are primarily COLMAP's own build dependencies, needed because LIMAP's
`pycolmap` package compiles against COLMAP's C++ libraries.

On **Ubuntu 22.04**, the same package names apply. Other distributions may
differ.

**Quick reference — what's already provided vs. what you need to add:**

```
Already installed by setup_local_dev.sh:
  ✓ build-essential, cmake, gcc, g++, clang
  ✓ qtbase5-dev, libqt5opengl5-dev

Must install for LIMAP (Step 3 above):
  ✗ colmap
  ✗ ninja-build
  ✗ libeigen3-dev, libflann-dev, libfreeimage-dev, libmetis-dev
  ✗ libgoogle-glog-dev, libgtest-dev, libgmock-dev
  ✗ libsqlite3-dev, libglew-dev, libcgal-dev, libceres-dev, libhdf5-dev
  ✗ libboost-{program-options,filesystem,graph,system}-dev
```

Verify after installation:

```bash
colmap -h         # Should print COLMAP help text
cmake --version   # Should be >= 3.17 (already installed)
```

### Step 4: (Optional) Install CUDA

CUDA is only needed if you want to use deep learning-based line detectors or
matchers within LIMAP (e.g., SOLD2, DeepLSD, SuperGlue). If you only use
LSD + LBD (the default in this project), CUDA is **not required**.

```bash
# Check if CUDA is already available
nvcc --version

# If not installed, on Ubuntu 24.04:
sudo apt-get install -y nvidia-cuda-toolkit
```

### Step 5: Install CVG LIMAP

**Option A — Via the project's optional extra** (recommended):

```bash
uv sync --extra limap
```

This resolves the LIMAP dependency from `pyproject.toml`, clones LIMAP from
GitHub, and builds it from source. The build can take several minutes due to
C++ compilation.

**Option B — Manual clone and install** (more control, useful for development):

```bash
source .venv/bin/activate

# Clone with submodules
git clone --recursive https://github.com/cvg/limap.git /tmp/limap
cd /tmp/limap

# Install Python dependencies
pip install -r requirements.txt

# Build and install (editable mode)
pip install -Ive .
```

**Option C — Pin to a specific commit** (for reproducibility):

```bash
source .venv/bin/activate
pip install "limap @ git+https://github.com/cvg/limap.git@4be6fca"
```

### Step 6: Install LIMAP's Undeclared Python Dependencies

CVG LIMAP's `pyproject.toml` does **not** declare its runtime Python
dependencies, so they are never installed automatically. Install them
manually after LIMAP itself:

```bash
uv pip install \
    pycolmap \
    pyceres \
    pytlsd \
    hloc@git+https://github.com/cvg/Hierarchical-Localization.git \
    open3d \
    omegaconf \
    bresenham \
    tqdm \
    h5py \
    scikit-learn \
    shapely \
    joblib \
    seaborn
```

| Package | Why LIMAP needs it |
|---|---|
| `pycolmap` | COLMAP Python bindings (SfM) |
| `pyceres` | Ceres Solver bindings (bundle adjustment) |
| `pytlsd` | Transparent LSD detector (line detection) |
| `hloc` | Hierarchical Localization (CVG, from GitHub) |
| `open3d` | 3D visualization |
| `omegaconf` | Configuration management |
| `bresenham` | Line rasterization |
| `tqdm` | Progress bars |
| `h5py` | HDF5 I/O for features/matches |
| `scikit-learn` | ML utilities (clustering) |
| `shapely` | Geometric operations |
| `joblib` | Parallel processing |
| `seaborn` | Visualization / plotting |

> **Note:** This list was verified against LIMAP v1.0.0.dev0 (commit `4be6fca`
> and later). Future LIMAP versions may add or remove dependencies.

### Step 7: Verify Installation

```bash
source .venv/bin/activate
python -c "import limap; print(limap.__version__)"
# Expected: 1.0.0.dev0 (or similar)

# Verify it's the real CVG LIMAP (has limap.base module)
python -c "from limap.base import Camera; print('CVG LIMAP OK')"
```

If you see `ModuleNotFoundError: No module named 'limap.base'`,
you may have the wrong PyPI package installed. Fix with:

```bash
pip uninstall limap
uv sync --extra limap
```

### Step 8: Test in the Project

```bash
source .venv/bin/activate
python -c "
from lsfm.limap_compat import check_limap_available
check_limap_available()
print('LIMAP integration OK')
"
```

Then open `examples/notebooks/demo_3d_reconstruction.ipynb` — it should
show "LIMAP available — multi-view reconstruction enabled".

## How LIMAP is Used in This Project

LIMAP is used for **multi-view 3D line reconstruction** via two modules:

| Module | Purpose |
|--------|---------|
| `lsfm.limap_compat` | Format conversion between our native types and LIMAP's NumPy-based API |
| `lsfm.reconstruction` | `reconstruct_lines_multiview()` — wraps LIMAP's triangulation pipeline |

When LIMAP is not installed, the code gracefully falls back to **pairwise stereo
reconstruction** using the native `le_geometry` C++ backend. No functionality is
lost for the two-view case.

### Architecture

```
Your image data
    │
    ▼
┌──────────────────┐     ┌──────────────────┐
│  le_lsd (LSD)    │────▶│  le_lfd (LBD)    │
│  Line detection  │     │  Line description │
└──────────────────┘     └──────────────────┘
    │                         │
    ▼                         ▼
┌──────────────────────────────────┐
│  lsfm.limap_compat              │
│  Adapters: LsfmLineDetector,    │
│  LsfmLineDescriptor,            │
│  LsfmLineMatcher                │
└──────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────┐
│  LIMAP (optional)                │
│  limap.runners.line_triangulation│
│  Multi-view triangulation &      │
│  track building                  │
└──────────────────────────────────┘
    │
    ▼
  3D line map
```

## Troubleshooting

### Build fails with "CMake Error" or "pybind11 not found"

Ensure all system dependencies from Step 3 are installed. LIMAP builds C++
code via scikit-build-core/CMake during `pip install`.

### `ImportError: limap.base` missing

You installed the wrong `limap` from PyPI. Uninstall and reinstall:

```bash
pip uninstall limap
uv sync --extra limap
```

### Build fails on Python 3.12+

CVG LIMAP requires Python < 3.12. Check with `.venv/bin/python --version`.
If needed, recreate the venv:

```bash
uv venv --python 3.10
uv sync --extra limap
```

### COLMAP not found during LIMAP build

LIMAP's `pycolmap` dependency builds against COLMAP. Ensure COLMAP is
installed system-wide (`sudo apt install colmap`) or set `COLMAP_PATH`:

```bash
export COLMAP_PATH=/usr/local/bin
uv sync --extra limap
```

### Out of memory during build

LIMAP's C++ compilation can be memory-intensive. Limit parallel jobs:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 uv sync --extra limap
```

## Further Reading

- [CVG LIMAP Repository](https://github.com/cvg/limap)
- [LIMAP Paper](https://arxiv.org/abs/2303.17504) — "3D Line Mapping Revisited" (CVPR 2023)
- [COLMAP Installation](https://colmap.github.io/install.html)
- [Notebook: Multi-View 3D Reconstruction](../examples/notebooks/demo_3d_reconstruction.ipynb)
- [LIMAP API — `lsfm.limap_compat`](../python/lsfm/limap_compat.py)
- [Jupyter Notebook Guide](JUPYTER.md)
