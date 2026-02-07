# Performance Benchmarking Suite

Benchmarks all core algorithms for computational efficiency. All benchmarks are compiled into a single binary (`eval_performance`) with a task registry that auto-discovers and runs all registered tests.

[← Back to Evaluation](../README.md)

## Benchmark Modules

All source files live in [src/](src/). The main entry point is [main.cpp](src/main.cpp), which drives the test registry.

| Source | Benchmark Area |
|---|---|
| [main.cpp](src/main.cpp) | Entry point, task registry, data provider setup, CLI options |
| [performance_test.hpp](src/performance_test.hpp) | Common header: `CVPerformanceTaskBase`, CUDA/OpenCL detection |
| [gradient.cpp](src/gradient.cpp) | Gradient operators: Sobel, Scharr, RCMG, SUSAN, quadrature, phase congruency |
| [nms.cpp](src/nms.cpp) | Non-Maximum Suppression and zero-crossing edge detection |
| [threshold.cpp](src/threshold.cpp) | Threshold estimation: Otsu, triangle, adaptive methods |
| [segment.cpp](src/segment.cpp) | Edge segment detection: connected components, edge chaining |
| [segment_eval.cpp](src/segment_eval.cpp) | NFA statistical validation of edge segments |
| [split.cpp](src/split.cpp) | Edge segment splitting and refinement |
| [spe.cpp](src/spe.cpp) | Sub-pixel edge localization: linear, quadratic, Gaussian interpolation |
| [line_fit.cpp](src/line_fit.cpp) | Line fitting on edge segments |
| [lsd.cpp](src/lsd.cpp) | Line Segment Detection variants (CC, CP, EL, EP, Burns, FGIOI) |
| [gpuConv.cpp](src/gpuConv.cpp) | GPU convolution: CPU vs OpenCL vs CUDA |
| [gpuFFT.cpp](src/gpuFFT.cpp) | GPU FFT: CPU vs OpenCL vs CUDA with memory transfer analysis |

## Building & Running

```bash
# Build
bazel build //evaluation/performance:eval_performance

# Run all benchmarks
bazel run //evaluation/performance:eval_performance

# Run with options (e.g., 50 repetitions, high priority, no show)
bazel run //evaluation/performance:eval_performance -- -r 50 -hp -ns
```

## Architecture

The benchmark framework uses a registry pattern:

- **`CVPerformanceTaskBase`** — Base class for all benchmark tasks
- **`addPerformanceTest()`** — Register a test in the global registry
- **`getDefaultProvider()`** — Standard datasets (BSDS500, MDB-Q/H/F) via Bazel runfiles
- Each `.cpp` module registers its benchmarks via static initialization

## Datasets

Benchmarks use these image datasets (auto-downloaded by Bazel):

| Dataset | Source | Description |
|---------|--------|-------------|
| BSDS500 | `@bsds500` | Berkeley Segmentation Dataset (500 images) |
| MDB-Q | `//resources/datasets:mdb_q` | Middlebury quarter resolution |
| MDB-H | `//resources/datasets:mdb_h` | Middlebury half resolution |
| MDB-F | `//resources/datasets:mdb_f` | Middlebury full resolution |

## GPU Support

GPU benchmarks (gpuConv, gpuFFT) require CUDA or OpenCL. Detect available features:

```bash
./tools/scripts/detect_bazel_features.sh
```

## Related

- [libs/eval](../../libs/eval/) — Evaluation framework, `CVPerformanceTaskBase`
- [libs/edge](../../libs/edge/) — Edge detection algorithms under test
- [libs/imgproc](../../libs/imgproc/) — Gradient operators under test
- [libs/lsd](../../libs/lsd/) — LSD variants under test
- [evaluation/thesis](../thesis/) — Precision and accuracy evaluations
