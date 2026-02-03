# Image Processing Examples

Examples demonstrating image filtering, gradient computation, frequency analysis, and multi-scale processing.

[← Back to Examples](../README.md)

## Examples

| Example | Description | Usage |
|---------|-------------|-------|
| [fft_test.cpp](fft_test.cpp) | FFT and periodic FFT | `./fft_test [image_path]` |
| [grad_test.cpp](grad_test.cpp) | Gradient operators comparison | `./grad_test [image_path]` |
| [noise_test.cpp](noise_test.cpp) | Noise utilities and PSNR | `./noise_test [image_path]` |
| [pyramid_test.cpp](pyramid_test.cpp) | Multi-scale image pyramid | `./pyramid_test [image_path]` |
| [steer_test.cpp](steer_test.cpp) | Steerable filters | `./steer_test` |
| [step_line_test.cpp](step_line_test.cpp) | Step vs line edge models | `./step_line_test [image_path]` |

## Building

**Bazel:**
```bash
bazel build //examples/imgproc:all
bazel run //examples/imgproc:grad_test -- /path/to/image.jpg
```

**CMake:**
```bash
make  # from build directory
```

## Key Algorithms

- **Gradient Operators:** Sobel, Scharr, RCMG, SUSAN
- **Frequency Analysis:** FFT, periodic FFT with boundary handling
- **Steerable Filters:** Rotation-invariant oriented feature detection
- **Pyramid Processing:** Multi-scale image analysis
- **Noise Analysis:** Gaussian noise models and PSNR metrics

## Related Libraries

- [libs/imgproc](../../libs/imgproc/) - Image processing library
