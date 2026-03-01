# LineExtraction — Next Steps

Feature overview and implementation status (as of July 2026).

---

## Feature Overview

### ✅ Completed

| # | Feature | Description |
|---|---------|-------------|
| 1 | [Bazel Data Dependencies](#1-bazel-data-dependencies) | BSDS500 auto-download, MDB setup, runfiles helper |
| 3 | [Python Bindings](#3-python-bindings) | 5 pybind11 modules, 70+ tests, NumPy zero-copy |
| 4 | [Jupyter Tutorials](#4-jupyter-tutorials) | 4-part tutorial series + PyTorch demo + API reference |
| 5 | [Google Benchmark](#5-google-benchmark) | Micro-benchmarks for LSD (8 variants) and Edge (Gradient, NMS) |
| 6 | [Test-Coverage](#6-test-coverage) | 64+ C++ unit tests, >80% coverage for critical components |
| 7 | [Doxygen Docs](#7-doxygen-documentation) | Complete API documentation, Bazel-integrated |
| 8 | [License Reorganization](#8-license-reorganization) | Clear separation: MIT (libs/) vs. GPL/AGPL (contrib/, third-party/) |
| 9 | [Library Architecture](#9-library-architecture) | GUI separation, granular OpenCV deps, include cleanup |
| 12 | [PyPI Package](#12-pypi-package) | Bazel `py_wheel` → `pip install lsfm-*.whl` |
| 13 | [Type Stubs](#13-type-stubs) | pybind11-stubgen `.pyi` stubs for IDE support (Pylance, Pyright) |
| 17 | [Rerun.io Integration](#17-rerunio-integration) | Interactive visualization of line detection results via [rerun.io](https://rerun.io) (C++ + Python SDK, Jupyter, WSL docs) |
| 2 | [Param Optimizer](#2-param-optimizer) | Automated LSD parameter optimization over image databases |
| 14 | [Pre-trained Presets](#14-pre-trained-presets) | Optimized parameter sets (Fast/Accurate/Balanced) |

### 🚀 Next Steps

| # | Feature | Effort | Description |
|---|---------|--------|-------------|
| 18 | [Line Feature Demo](#18-line-feature-demo) | 🟡 | Modern line feature descriptor based on LSD |
| 19 | [3D Reconstruction with Lines](#19-3d-reconstruction-with-lines) | 🔴 | 3D line reconstruction using LSD (e.g. [LIMAP](https://github.com/rerun-io/limap)) |

### 💡 Optional (Nice-to-have)

| # | Feature | Effort | Description |
|---|---------|--------|-------------|
| 10 | [GitHub Pages & Badges](#10-github-pages--badges) | 🟢 4–6h | Doxygen auto-deployment, README badges |
| 11 | [CI/CD Pipeline](#11-cicd-pipeline) | 🟡 2–3 days | Matrix builds, sanitizers, performance regression |
| 15 | [CUDA/GPU](#15-cudagpu-acceleration) | 🔴 1+ wk. | GPU-accelerated gradient computation, cuDNN |
| 16 | [CMake Deprecation](#16-cmake-deprecation) | 🟢 ongoing | Clean up / remove legacy CMake |

---

## Completed Features

### 1. Bazel Data Dependencies

BSDS500 as `http_archive` (auto-download ~50 MB), MDB via setup script (`setup_mdb_dataset.sh`), C++ runfiles helper for transparent path resolution in Bazel and CMake. Additionally: York Urban Line Segment DB (`setup_york_urban.sh`, 102 images with annotated ground truth) and Wireframe Dataset (`setup_wireframe.sh`, 5462 images with wireframe annotations).

### 3. Python Bindings

5 modules via pybind11 with zero-copy NumPy integration (cvnp):

| Module | Library | Contents |
|--------|---------|----------|
| `le_imgproc` | `libs/imgproc` | Gradient filters, core types |
| `le_edge` | `libs/edge` | Edge detection, NMS, ESD |
| `le_geometry` | `libs/geometry` | Line, LineSegment, Polygon, Drawing |
| `le_eval` | `libs/eval` | Performance benchmarking |
| `le_lsd` | `libs/lsd` | 9 LSD algorithms |

Docs: per module in `libs/<name>/python/README.md`.

### 4. Jupyter Tutorials

Structured tutorial series for Python onboarding (see [JUPYTER.md](JUPYTER.md)):

| Notebook | Contents |
|----------|----------|
| `cv_primer` | CV basics without library dependencies |
| `tutorial_1_fundamentals` | Gradient filters, geometry, ValueManager |
| `tutorial_2_pipelines` | Edge & line detection pipelines, all 9 LSD detectors |
| `tutorial_3_evaluation` | Performance evaluation framework |
| `pytorch_esd_demo` | PyTorch object segmentation + ESD line extraction |
| `line_extraction_bindings` | API reference for all 5 modules |

### 5. Google Benchmark

Micro-benchmarks: `bench_lsd` (8 LSD variants × 4 image sizes), `bench_edge` (Sobel, Scharr, NMS).

### 6. Test-Coverage

64+ unit tests across all modules (Edge, LSD, ImgProc, Geometry, Utility, Eval). Sanitizer support (ASan, TSan).

### 7. Doxygen Documentation

Fully configured. `bazel run //:doxygen` → `docs/generated/html/`.

### 8. License Reorganization

Clear separation: `libs/` (MIT), `contrib/matlab_coder/` (Academic), `contrib/lsd_external/` (AGPL/BSD), `third-party/qplot*` (GPL v3), `apps/line_analyzer` (GPL v3 due to Qt+QCustomPlot).

### 9. Library Architecture

GUI separation (`lib_utility` headless + `lib_utility_gui` optional), granular OpenCV modules instead of meta-target, include cleanup.

### 13. Type Stubs

`.pyi` stubs are automatically generated during `bazel build` via `pybind11-stubgen` (hermetic, no manual steps). Stubs end up in `bazel-bin/` and are automatically bundled into the wheel. For IDE support in the source tree: `./tools/scripts/generate_stubs.sh` (optional, files are .gitignored).

### 12. PyPI Package

Bazel `py_wheel` target (`//python:lsfm_wheel`) produces a pip-installable wheel. Contains all 5 native modules (.so), type stubs (.pyi), PEP 561 marker, and the `lsfm` Python package. Build: `bazel build //python:lsfm_wheel`, Install: `pip install bazel-bin/python/lsfm-*.whl`.

### 2. Param Optimizer

Automated parameter optimization for LSD algorithms over image databases. Implemented as `libs/algorithm/` with a C++ header-only library and Python bindings (`le_algorithm`). Components: `ParamOptimizer` (grid/random search), `AccuracyMeasure` (P/R/F1), `GroundTruthLoader` (CSV), `LineContinuityOptimizer` (unified merge + gradient-assisted bridging), `LineMerge` (legacy), `LineConnect` (legacy), `PrecisionOptimize` (dlib BFGS). 30 C++ tests, 20 Python tests. York Urban DB + Wireframe datasets with automatic GT conversion.

### 14. Pre-trained Presets

Preset optimization via `evaluation/python/tools/optimize_presets.py`. Three profiles per detector (Fast/Balanced/Accurate), output as `lsd_presets.json`. Uses `RandomSearchStrategy` with 300 samples across all 9 LSD detectors.

---

## Next Steps

### 17. Rerun.io Integration

Interactive visualization of line detection results using [rerun.io](https://rerun.io). Enables explorative analysis of edges, line segments, and detector comparisons in a modern, web-capable 2D/3D viewer.

**Effort:** 🟡 3–5 days

### 18. Line Feature Demo

Modern line feature descriptor based on LSD. Demonstration of a complete pipeline: LSD-based line detection → feature extraction → matching. Showcases the library's applicability for modern feature-based approaches.

**Effort:** 🟡 3–5 days

### 19. 3D Reconstruction with Lines

3D line reconstruction using LSD results. Integration with existing Structure-from-Motion frameworks, e.g. [LIMAP](https://github.com/rerun-io/limap). Combination of point and line features for more robust 3D reconstruction.

**Effort:** 🔴 1–2+ weeks

---

## Optional (Nice-to-have)

### 10. GitHub Pages & Badges

Automatically deploy Doxygen via GitHub Actions. README badges for build status, docs, license.

**Effort:** 🟢 4–6 hours

### 11. CI/CD Pipeline

Matrix builds (GCC/Clang), sanitizer runs in CI, performance regression tests, automatic releases.

**Effort:** 🟡 2–3 days

### 15. CUDA/GPU Acceleration

GPU-accelerated gradient computation (foundation exists: `gpuConv.cpp`, `gpuFFT.cpp`), cuDNN integration.

**Effort:** 🔴 1+ weeks

### 16. CMake Deprecation

Clean up legacy CMake workarounds, create migration guide, remove long-term.

**Effort:** 🟢 ongoing

---

## Roadmap

### Phase 1: Foundations ✅

Bazel Data Deps, Google Benchmark, Doxygen, Test-Coverage, License Reorganization, GUI Separation.

### Phase 2: Python Ecosystem ✅

Python Bindings (5 Modules, 70+ Tests), Jupyter Tutorial Series + PyTorch Demo, Type Stubs, PyPI Wheel.

### Phase 3: Algorithm & Optimization ✅

Param Optimizer, Pre-trained Presets (Fast/Balanced/Accurate).

### Phase 4: Applications & Demos

Rerun.io Integration, Line Feature Demo, 3D Reconstruction with Lines.

### Optional

GitHub Pages, CI/CD Pipeline, CUDA/GPU, CMake Deprecation.

---

## Effort Legend

| Symbol | Level | Timeframe |
|--------|-------|----------|
| 🟢 | Low | 1–3 days |
| 🟡 | Medium | 3–7 days |
| 🔴 | High | 1–2+ weeks |
