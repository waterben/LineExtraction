# Image Processing Examples

Examples demonstrating gradient operators, frequency domain analysis, steerable filters, multi-scale processing, and noise utilities.

[← Back to Examples](../README.md)

## Examples

| Bazel Target | Source | Description |
|---|---|---|
| `test_imgproc_grad_ops` | [grad_test.cpp](src/grad_test.cpp) | Benchmarks Sobel, Scharr, RCMG, and SUSAN gradient operators with Otsu thresholding |
| `test_imgproc_steer` | [steer_test.cpp](src/steer_test.cpp) | Steerable Gaussian derivative filters (1st/2nd order) with interactive rotation |
| `test_imgproc_step_line` | [step_line_test.cpp](src/step_line_test.cpp) | Step edge vs line (ridge) edge model comparison using gradients and quadrature filters |
| `test_imgproc_fft` | [fft_test.cpp](src/fft_test.cpp) | FFT and periodic FFT (`perfft2`) with boundary artifact reduction and spectrum visualization |
| `test_imgproc_pyramid` | [pyramid_test.cpp](src/pyramid_test.cpp) | Gaussian pyramid construction with gradient and NMS at multiple scales |
| `test_imgproc_noise` | [noise_test.cpp](src/noise_test.cpp) | Gaussian noise addition, noise estimation from patches, and PSNR calculation |

## Building & Running

```bash
# Build all imgproc examples
bazel build //examples/imgproc:all

# Run with default test image
bazel run //examples/imgproc:test_imgproc_grad_ops

# Run with custom image
bazel run //examples/imgproc:test_imgproc_pyramid -- /path/to/image.jpg
```

## Key Algorithms

- **Gradient Operators:** Sobel, Scharr, RCMG (Robust Color Morphological Gradient), SUSAN
- **Frequency Analysis:** FFT, periodic FFT for reduced boundary artifacts
- **Steerable Filters:** Rotation-invariant oriented feature detection (1st and 2nd derivative)
- **Pyramid Processing:** Gaussian pyramid for multi-scale analysis
- **Noise Analysis:** Gaussian noise models, patch-based noise estimation, PSNR metrics
- **Edge Models:** Step edge (gradient-based) vs line/ridge edge (second derivative)

## Related Libraries

- [libs/imgproc](../../libs/imgproc/) — Gradient operators, quadrature filters, pyramids, FFT
- [libs/edge](../../libs/edge/) — NMS and edge linking (used in pyramid example)
