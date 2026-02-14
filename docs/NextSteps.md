# LineExtraction — Next Steps

Feature-Überblick und Implementierungsstand (Stand: Februar 2026).

---

## Feature-Übersicht

| # | Feature | Status | Aufwand | Beschreibung |
|---|---------|--------|---------|--------------|
| 1 | [Bazel Data Dependencies](#1-bazel-data-dependencies) | ✅ Done | — | BSDS500 Auto-Download, MDB Setup, Runfiles-Helper |
| 2 | [Param Optimizer](#2-param-optimizer) | ✅ Done | — | Automatisierte LSD-Parameter-Optimierung über Bilddatenbanken |
| 3 | [Python Bindings](#3-python-bindings) | ✅ Done | — | 5 pybind11-Module, 70+ Tests, NumPy Zero-Copy |
| 4 | [Jupyter Tutorials](#4-jupyter-tutorials) | ✅ Done | — | 4-teilige Tutorial-Serie + PyTorch Demo + API Reference |
| 5 | [Google Benchmark](#5-google-benchmark) | ✅ Done | — | Micro-Benchmarks für LSD (8 Varianten) und Edge (Gradient, NMS) |
| 6 | [Test-Coverage](#6-test-coverage) | ✅ Done | — | 64+ C++ Unit-Tests, >80% Coverage für kritische Komponenten |
| 7 | [Doxygen Docs](#7-doxygen-documentation) | ✅ Done | — | Vollständige API-Doku, Bazel-integriert |
| 8 | [Lizenz-Reorganisation](#8-lizenz-reorganisation) | ✅ Done | — | Klare Trennung: MIT (libs/) vs. GPL/AGPL (contrib/, third-party/) |
| 9 | [Library-Architektur](#9-library-architektur) | ✅ Done | — | GUI-Separation, granulare OpenCV-Deps, Include-Cleanup |
| 10 | [GitHub Pages & Badges](#10-github-pages--badges) | ⬜ Offen | 🟢 4–6h | Doxygen Auto-Deployment, README Badges |
| 11 | [CI/CD Pipeline](#11-cicd-pipeline) | ⬜ Offen | 🟡 2–3 Tage | Matrix-Builds, Sanitizer, Performance-Regression |
| 12 | [PyPI Package](#12-pypi-package) | ✅ Done | — | Bazel `py_wheel` → `pip install lsfm-*.whl` |
| 13 | [Type Stubs](#13-type-stubs) | ✅ Done | — | pybind11-stubgen `.pyi` Stubs für IDE-Support (Pylance, Pyright) |
| 14 | [Pre-trained Presets](#14-pre-trained-presets) | ✅ Done | — | Optimierte Parameter-Sets (Fast/Accurate/Balanced) |
| 15 | [CUDA/GPU](#15-cudagpu-beschleunigung) | ⬜ Offen | 🔴 1+ Wo. | GPU-beschleunigte Gradient-Berechnung, cuDNN |
| 16 | [CMake Deprecation](#16-cmake-deprecation) | ⬜ Offen | 🟢 fortl. | Legacy-CMake aufräumen / entfernen |

---

## Abgeschlossene Features

### 1. Bazel Data Dependencies

BSDS500 als `http_archive` (Auto-Download ~50MB), MDB via Setup-Script (`setup_mdb_dataset.sh`), C++ Runfiles-Helper für transparente Pfadauflösung in Bazel und CMake. Zusätzlich: York Urban Line Segment DB (`setup_york_urban.sh`, 102 Bilder mit annotated Ground Truth) und Wireframe Dataset (`setup_wireframe.sh`, 5462 Bilder mit Wireframe-Annotationen).

### 3. Python Bindings

5 Module via pybind11 mit Zero-Copy NumPy-Integration (cvnp):

| Modul | Library | Inhalt |
|-------|---------|--------|
| `le_imgproc` | `libs/imgproc` | Gradient-Filter, Core-Typen |
| `le_edge` | `libs/edge` | Edge Detection, NMS, ESD |
| `le_geometry` | `libs/geometry` | Line, LineSegment, Polygon, Drawing |
| `le_eval` | `libs/eval` | Performance Benchmarking |
| `le_lsd` | `libs/lsd` | 9 LSD-Algorithmen |

Doku: je Modul in `libs/<name>/python/README.md`.

### 4. Jupyter Tutorials

Strukturierte Tutorial-Serie für Python-Onboarding (siehe [JUPYTER.md](JUPYTER.md)):

| Notebook | Inhalt |
|----------|--------|
| `cv_primer` | CV-Grundlagen ohne Library-Dependencies |
| `tutorial_1_fundamentals` | Gradient-Filter, Geometrie, ValueManager |
| `tutorial_2_pipelines` | Edge- & Line-Detection-Pipelines, alle 9 LSD-Detektoren |
| `tutorial_3_evaluation` | Performance-Evaluation-Framework |
| `pytorch_esd_demo` | PyTorch Object Segmentation + ESD Line Extraction |
| `line_extraction_bindings` | API-Referenz aller 5 Module |

### 5. Google Benchmark

Micro-Benchmarks: `bench_lsd` (8 LSD-Varianten × 4 Bildgrößen), `bench_edge` (Sobel, Scharr, NMS).

### 6. Test-Coverage

64+ Unit-Tests über alle Module (Edge, LSD, ImgProc, Geometry, Utility, Eval). Sanitizer-Support (ASan, TSan).

### 7. Doxygen Documentation

Vollständig konfiguriert. `bazel run //:doxygen` → `docs/generated/html/`.

### 8. Lizenz-Reorganisation

Klare Trennung: `libs/` (MIT), `contrib/matlab_coder/` (Academic), `contrib/lsd_external/` (AGPL/BSD), `third-party/qplot*` (GPL v3), `apps/line_analyzer` (GPL v3 wegen Qt+QCustomPlot).

### 9. Library-Architektur

GUI-Separation (`lib_utility` headless + `lib_utility_gui` optional), granulare OpenCV-Module statt Meta-Target, Include-Cleanup.

### 13. Type Stubs

`.pyi` Stubs werden automatisch während `bazel build` via `pybind11-stubgen` generiert (hermetic, keine manuellen Schritte). Stubs landen in `bazel-bin/` und werden automatisch ins Wheel eingebunden. Für IDE-Support im Source Tree: `./tools/scripts/generate_stubs.sh` (optional, Dateien sind .gitignored).

### 12. PyPI Package

Bazel `py_wheel` Target (`//python:lsfm_wheel`) erzeugt ein pip-installierbares Wheel. Enthält alle 5 nativen Module (.so), Type Stubs (.pyi), PEP 561 Marker, und das `lsfm` Python-Package. Build: `bazel build //python:lsfm_wheel`, Install: `pip install bazel-bin/python/lsfm-*.whl`.

### 2. Param Optimizer

Automatisierte Parameter-Optimierung für LSD-Algorithmen über Bilddatenbanken. Umgesetzt als `libs/algorithm/` mit C++ Header-Only Library und Python Bindings (`le_algorithm`). Komponenten: `ParamOptimizer` (Grid/Random Search), `AccuracyMeasure` (P/R/F1), `GroundTruthLoader` (CSV), `LineMerge`, `LineConnect`, `PrecisionOptimize` (dlib BFGS). 30 C++ Tests, 20 Python Tests. York Urban DB + Wireframe Datasets mit automatischer GT-Konvertierung.

### 14. Pre-trained Presets

Preset-Optimierung via `evaluation/python/tools/optimize_presets.py`. Drei Profile pro Detektor (Fast/Balanced/Accurate), Ausgabe als `lsd_presets.json`. Verwendet `RandomSearchStrategy` mit 300 Samples über alle 9 LSD-Detektoren.

---

## Offene Features

### 10. GitHub Pages & Badges

Doxygen automatisch via GitHub Actions deployen. README Badges für Build-Status, Docs, License.

**Aufwand:** 🟢 4–6 Stunden

### 11. CI/CD Pipeline

Matrix-Builds (GCC/Clang), Sanitizer-Runs in CI, Performance-Regression-Tests, automatische Releases.

**Aufwand:** 🟡 2–3 Tage

### 15. CUDA/GPU Beschleunigung

GPU-beschleunigte Gradient-Berechnung (Basis vorhanden: `gpuConv.cpp`, `gpuFFT.cpp`), cuDNN-Integration.

**Aufwand:** 🔴 1+ Wochen

### 16. CMake Deprecation

Legacy-CMake Workarounds aufräumen, Migration Guide erstellen, langfristig entfernen.

**Aufwand:** 🟢 fortlaufend

---

## Roadmap

### Phase 1: Foundations ✅

Bazel Data Deps, Google Benchmark, Doxygen, Test-Coverage, Lizenz-Reorganisation, GUI-Separation.

### Phase 2: Python Ecosystem ✅

Python Bindings (5 Module, 70+ Tests), Jupyter Tutorial-Serie + PyTorch Demo, Type Stubs, PyPI Wheel.

### Phase 3: Deployment & Visibility

GitHub Pages, README Badges, CI/CD Pipeline.

### Phase 4: Advanced Features

Param Optimizer ✅, Pre-trained Presets ✅, CUDA/GPU Beschleunigung.

---

## Aufwands-Legende

| Symbol | Bedeutung | Zeitrahmen |
|--------|-----------|------------|
| 🟢 | Niedrig | 1-3 Tage |
| 🟡 | Mittel | 3-7 Tage |
| 🔴 | Hoch | 1-2+ Wochen |
