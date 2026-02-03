# Evaluation Framework

Comprehensive performance benchmarking and precision evaluation suite for line detection algorithms and image processing operations.

## Overview

The evaluation framework includes two main categories:

- **performance/** - Benchmarks for algorithm performance and computational efficiency
- **thesis/** - Precision and accuracy evaluations for thesis research

## Directory Structure

### [performance/](performance/README.md)

Performance benchmarking framework for core algorithms:

- GPU computing (FFT, Convolution)
- Gradient operators
- Line segment detection
- Edge processing (NMS, threshold)
- Sub-pixel estimation

**Building:**

```bash
bazel build //evaluation/performance:all
bazel run //evaluation/performance:main
```

### [thesis/](thesis/README.md)

Research evaluation programs for precision analysis:

- Gradient computation accuracy
- Color vs grayscale processing
- Noise robustness
- Algorithm parameter optimization
- Sub-pixel localization precision

**Building:**

```bash
bazel build //evaluation/thesis:all
bazel run //evaluation/thesis:target_name
```

## Key Features

- **Modular design:** Each evaluation program is independent and self-contained
- **Performance measurement:** Automated timing and statistics collection
- **Result output:** Results saved in standard formats (CSV, PNG, etc.)
- **Framework:** EvalApp framework for command-line options and configuration
- **Data management:** Automatic data path resolution using Bazel runfiles

## Related

- [libs/eval](../libs/eval/) - Evaluation framework library
- [libs/edge](../libs/edge/) - Edge detection algorithms
- [libs/imgproc](../libs/imgproc/) - Image processing utilities
- [libs/geometry](../libs/geometry/) - Geometric primitives
