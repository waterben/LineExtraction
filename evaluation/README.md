# Evaluation Framework

Performance benchmarking and precision evaluation suite for line detection and image processing algorithms.

## Directory Structure

| Directory | Description |
|-----------|-------------|
| [performance/](performance/) | Computational benchmarks: gradient, NMS, LSD, FFT, GPU, sub-pixel estimation |
| [thesis/](thesis/) | Precision evaluations: gradient accuracy, noise robustness, color processing, SPE |
| [python/](python/) | Python tools for result analysis (chart plotting, CSV tables, LaTeX utilities) |

Each subdirectory follows the same layout:

```
<category>/
  BUILD.bazel       # Bazel build targets
  CMakeLists.txt    # Legacy CMake build
  README.md         # Documentation
  src/              # Source files (.cpp, .hpp)
```

## Building & Running

```bash
# Build everything
bazel build //evaluation/...

# Performance benchmark suite (single binary with all benchmarks)
bazel run //evaluation/performance:eval_performance

# Individual thesis evaluations
bazel run //evaluation/thesis:eval_gradient_orientation
bazel run //evaluation/thesis:eval_spe_precision
```

## Key Features

- **Performance measurement:** High-resolution timing with statistics collection
- **EvalApp framework:** Consistent command-line interface (`--input`, `--output`, `--help`)
- **Result output:** CSV data, PNG visualizations, and statistics reports
- **Data management:** Automatic dataset resolution via Bazel runfiles (BSDS500, MDB)
- **Modular design:** Each evaluation is an independent, self-contained binary

## Related Libraries

- [libs/eval](../libs/eval/) — Evaluation framework, EvalApp, data providers
- [libs/edge](../libs/edge/) — Edge detection algorithms
- [libs/imgproc](../libs/imgproc/) — Gradient operators, filters
- [libs/lsd](../libs/lsd/) — Line Segment Detection
- [libs/geometry](../libs/geometry/) — Geometric primitives

## See Also

- [Examples](../examples/README.md) — Standalone example programs
- [Apps](../apps/README.md) — Interactive applications
- [Main README](../README.md) — Project overview
