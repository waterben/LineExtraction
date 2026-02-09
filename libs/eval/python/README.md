# le_eval — Python Bindings for Evaluation Library

Python bindings for the `libs/eval` C++ library using [pybind11](https://pybind11.readthedocs.io/).

## Overview

This module provides a performance benchmarking and evaluation framework for
image processing algorithms:

- **StringTable** — 2D table for collecting and exporting results
- **Performance measures** — timing, per-megapixel throughput, statistics
- **Data providers** — load test images from the file system
- **Task framework** — define, run, and compare benchmark tasks from Python
- **CVPerformanceTest** — orchestrator for running multi-image benchmarks

> **Standalone:** `le_eval` has no Python-level imports.  It can be used
> independently of the other `le_*` modules, though combining it with
> `le_lsd` or `le_edge` for benchmarking is the typical use case.

## Quick Start

```python
import numpy as np
import le_eval

# Measure execution time
pm = le_eval.PerformanceMeasureBase("my_source", "my_task")
pm.append_duration(1500000)   # ticks (nanoseconds)
pm.append_duration(1600000)
pm.append_duration(1400000)

result = pm.compute_result()
print(f"Mean: {result.mean:.2f}, StdDev: {result.stddev:.2f}")

# Export results
table = le_eval.StringTable(2, 3)
table[0, 0] = "Method"
table[0, 1] = "Mean (ms)"
table[0, 2] = "StdDev (ms)"
table[1, 0] = "my_task"
table[1, 1] = f"{result.mean:.2f}"
table[1, 2] = f"{result.stddev:.2f}"
table.save_csv("results.csv")
```

## StringTable

A simple 2D string table for collecting, formatting, and exporting results:

```python
table = le_eval.StringTable(3, 4)  # 3 rows, 4 columns

# Get/set cells
table[0, 0] = "Detector"
table[0, 1] = "Mean (ms)"
val = table[0, 0]   # "Detector"

# Row/column access
header = table.row(0)     # list[str]
column = table.col(1)     # list[str]

# Properties
table.rows   # 3
table.cols   # 4
table.size   # (3, 4)

# Transpose
transposed = table.transpose()

# Export to CSV
table.save_csv("results.csv")

# Convert to nested list
data = table.to_list()  # list[list[str]]
```

## Performance Measurement

### PerformanceResult

Holds computed timing statistics:

```python
result = le_eval.PerformanceResult()
result.total    # total execution time
result.mean     # mean execution time
result.stddev   # standard deviation
```

### PerformanceMeasureBase

Accumulates duration samples and computes statistics:

```python
pm = le_eval.PerformanceMeasureBase("source_name", "task_name")

# Collect timing data
pm.append_duration(1000000)
pm.append_duration(1500000)
pm.append_duration(1200000)

# Access raw durations
print(pm.durations)  # [1000000, 1500000, 1200000]

# Compute result
result = pm.compute_result()  # -> PerformanceResult
print(f"Total: {result.total}, Mean: {result.mean}, Std: {result.stddev}")

pm.clear()  # reset
```

### CVPerformanceMeasure

Extends `PerformanceMeasureBase` with image dimension metadata:

```python
cpm = le_eval.CVPerformanceMeasure("source", "task", width=1920, height=1080)
cpm.append_duration(5000000)

print(cpm.width, cpm.height)
print(cpm.mega_pixels())  # ~2.07

result = cpm.compute_result()
```

### Accumulate Measures

Combine multiple measures into a single aggregated measure:

```python
m1 = le_eval.CVPerformanceMeasure("src", "task", 640, 480)
m1.append_duration(1000)
m2 = le_eval.CVPerformanceMeasure("src", "task", 640, 480)
m2.append_duration(2000)

combined = le_eval.accumulate_measures([m1, m2], "src", "task")
```

## Data Providers

Load test images from the file system for benchmarking:

### CVData

A simple container holding image name + data:

```python
data = le_eval.CVData()
data.name    # str — image filename/identifier
data.image   # numpy array — the loaded image
```

### FileDataProvider

Load all images from a directory:

```python
provider = le_eval.FileDataProvider(
    folder="/path/to/images",
    name="test_set"
)
```

### FileCVDataProvider / FileCVPerformanceDataProvider

Lower-level data providers for custom loading logic:

```python
provider = le_eval.FileCVDataProvider()
provider = le_eval.FileCVPerformanceDataProvider()
```

## Task Framework

### ITask / Task

Abstract task interfaces:

```python
# Task is the base class with name + verbose flag
task = le_eval.Task()
task.name     # str
task.verbose  # bool
```

### CVPerformanceTask — Subclassable Benchmark

The key entry point for custom benchmarks.  Override `prepare_impl()` and
`run_impl()` in Python:

```python
import le_eval
import le_lsd
import numpy as np


class LsdBenchmark(le_eval.CVPerformanceTask):
    """Benchmark an LSD detector."""

    def __init__(self, detector_name: str, detector):
        # flags: TASK_SQR for grayscale input
        super().__init__(detector_name, le_eval.TASK_SQR)
        self.detector = detector

    def prepare_impl(self, src: np.ndarray) -> None:
        """Called once per image before timing starts."""
        self.src = src.copy()

    def run_impl(self, name: str, src: np.ndarray) -> None:
        """Called inside the timing loop."""
        self.detector.detect(src)


# Create benchmark tasks
tasks = [
    LsdBenchmark("LsdCC", le_lsd.LsdCC()),
    LsdBenchmark("LsdFGioi", le_lsd.LsdFGioi()),
    LsdBenchmark("LsdBurns", le_lsd.LsdBurns()),
]
```

### Task Flags

```python
le_eval.TASK_SQR    # 1 — convert input to grayscale
le_eval.TASK_RGB    # 2 — keep input as RGB
le_eval.TASK_NO_3   # 4 — skip 3-channel images
le_eval.TASK_NO_5   # 8 — skip 5-channel images
```

## CVPerformanceTest — Benchmark Orchestrator

Runs multiple tasks across multiple images and collects timing results:

```python
import le_eval

# Set up data providers
providers = [
    le_eval.FileDataProvider("/path/to/test_images", "dataset_1"),
]

# Create test
test = le_eval.CVPerformanceTest(
    data_providers=providers,
    name="LSD Benchmark",
    target_path="results/",
    verbose=True,
    visual_results=False,
)

# Configure output
test.show_total    = True
test.show_mean     = True
test.show_std_dev  = True
test.show_mega_pixel = True

# Add tasks
for task in tasks:
    test.add_task(task)

# Run benchmark (10 iterations per image)
test.run(loops=10)

# Get results
table = test.result_table(full_report=True)  # -> StringTable
table.save_csv("benchmark_results.csv")

# Display as nested list
for row in table.to_list():
    print("\t".join(row))

# Reset for re-run
test.clear()
```

## Complete Benchmark Example

```python
"""Benchmark LSD detectors across a folder of test images."""
import le_eval
import le_lsd


class DetectorTask(le_eval.CVPerformanceTask):
    def __init__(self, name, detector):
        super().__init__(name, le_eval.TASK_SQR)
        self.det = detector

    def prepare_impl(self, src):
        pass  # no preparation needed

    def run_impl(self, name, src):
        self.det.detect(src)


# Configure
providers = [le_eval.FileDataProvider("test_images/", "TestSet")]
test = le_eval.CVPerformanceTest(providers, "LSD Comparison")
test.show_mean = True
test.show_std_dev = True

# Add detectors to compare
for name, det in [
    ("CC", le_lsd.LsdCC()),
    ("Burns", le_lsd.LsdBurns()),
    ("FGioi", le_lsd.LsdFGioi()),
    ("EDLines", le_lsd.LsdEDLZ()),
]:
    test.add_task(DetectorTask(name, det))

# Run and export
test.run(loops=5)
table = test.result_table(full_report=True)
table.save_csv("lsd_benchmark.csv")
print("Results saved to lsd_benchmark.csv")
```

## Build & Test

```bash
# Build the module
bazel build //libs/eval/python:le_eval

# Run tests
bazel test //libs/eval/python:test_le_eval

# Run all eval library tests (C++ and Python)
bazel test //libs/eval/...
```

## Architecture

```
python/
├── src/
│   ├── eval_binding.hpp       # Binding declarations
│   ├── eval_binding.cpp       # All eval bindings
│   └── module_le_eval.cpp     # PYBIND11_MODULE entry
├── tests/
│   └── test_le_eval.py        # Python integration tests
├── BUILD.bazel
└── README.md
```

### Module Dependency Graph

```
le_eval  (standalone — no Python imports)
    │
    └── StringTable, performance measures, tasks, test runner
```

`le_eval` is a root module.  Combine with `le_lsd`, `le_edge`, or
`le_imgproc` for real benchmark workflows.

## See Also

- [le_lsd](../../lsd/python/README.md) — line segment detection (benchmark target)
- [le_edge](../../edge/python/README.md) — edge detection (benchmark target)
- [le_imgproc](../../imgproc/python/README.md) — image processing filters
- [le_geometry](../../geometry/python/README.md) — geometric primitives
