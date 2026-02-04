# Performance Benchmarking Suite

Performance evaluation framework for algorithmic efficiency and computational benchmarking.

[← Back to Evaluation](../README.md)

## Benchmarks

| Program | Description | Purpose |
|---------|-------------|---------|
| [gpuConv.cpp](gpuConv.cpp) | GPU Convolution benchmarks | CPU vs OpenCL vs CUDA convolution performance |
| [gpuFFT.cpp](gpuFFT.cpp) | GPU FFT benchmarks | CPU vs OpenCL vs CUDA FFT performance with memory transfer analysis |
| [gradient.cpp](gradient.cpp) | Gradient operator benchmarks | Performance comparison of various gradient computation methods |
| [line_fit.cpp](line_fit.cpp) | Line fitting benchmarks | Performance evaluation of line fitting algorithms |
| [lsd.cpp](lsd.cpp) | Line Segment Detector benchmarks | Performance profiling of LSD variants |
| [main.cpp](main.cpp) | Performance test framework | Main executable, registry, and test management |
| [nms.cpp](nms.cpp) | NMS and Zero-Crossing benchmarks | Non-Maximum Suppression and edge detection performance |
| [segment.cpp](segment.cpp) | Edge segment detection benchmarks | Connected component and edge chaining performance |
| [segment_eval.cpp](segment_eval.cpp) | NFA evaluation benchmarks | Statistical validation performance for edge segments |
| [split.cpp](split.cpp) | Edge splitting benchmarks | Edge segment splitting and refinement performance |
| [threshold.cpp](threshold.cpp) | Thresholding method benchmarks | Performance of various threshold computation methods |
| [spe.cpp](spe.cpp) | Sub-Pixel Estimation benchmarks | Sub-pixel edge localization performance |

## Building

**Bazel:**

```bash
bazel build //evaluation/performance:all
bazel run //evaluation/performance:main
```

**CMake:**

```bash
cmake --build build --target evaluation_performance
./build/bin/evaluation_performance
```

## Running Benchmarks

### Main Performance Test Suite

```bash
bazel run //evaluation/performance:main
```

### Individual Benchmarks

Each benchmark can be run individually:

```bash
# GPU benchmarks (requires CUDA/OpenCL)
bazel run //evaluation/performance:gpuFFT

# Gradient performance
bazel run //evaluation/performance:gradient

# Line segment detection
bazel run //evaluation/performance:lsd
```

## Benchmark Framework

The performance framework uses:

- **CVPerformanceTaskBase:** Base class for OpenCV-based performance tasks
- **Task registry:** Automatic test discovery and execution
- **Data providers:** Standard image datasets and synthetic test data
- **Timing:** High-resolution performance measurement
- **Statistics:** Automatic result collection and reporting

## GPU Computing Support

GPU benchmarks require:

- CUDA (for CUDA benchmarks)
- OpenCL (for OpenCL benchmarks)

Enable GPU support during build:

```bash
./tools/scripts/detect_bazel_features.sh
```

## Output

Results are saved as:

- CSV files with timing data and statistics
- Performance profiles with algorithm comparison
- Memory usage analysis (where applicable)

## Related

- [libs/eval](../../libs/eval/) - Evaluation framework library
- [libs/edge](../../libs/edge/) - Edge detection algorithms
- [libs/imgproc](../../libs/imgproc/) - Image processing
- [evaluation/thesis](../thesis/) - Precision evaluations
