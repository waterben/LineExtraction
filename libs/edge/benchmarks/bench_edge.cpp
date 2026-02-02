/// @file bench_edge.cpp
/// @brief Google Benchmark tests for Edge Detection algorithms.
///
/// Benchmarks NMS (Non-Maximum Suppression) and gradient computation.
/// Run with: bazel run //libs/edge:bench_edge -- --benchmark_format=console

#include <benchmark/benchmark.h>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <opencv2/imgproc.hpp>

#include <random>

namespace {

/// @brief Generate a synthetic test image with gradients.
cv::Mat generateTestImage(int rows, int cols, unsigned int seed = 42) {
  cv::Mat img(rows, cols, CV_8UC1);

  std::mt19937 gen(seed);
  std::uniform_int_distribution<> dis(0, 255);

  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      img.at<uchar>(y, x) = static_cast<uchar>(dis(gen));
    }
  }

  // Add structure
  cv::line(img, cv::Point(cols / 4, rows / 4), cv::Point(3 * cols / 4, 3 * rows / 4), cv::Scalar(255), 2);
  cv::rectangle(img, cv::Point(cols / 8, rows / 8), cv::Point(7 * cols / 8, 7 * rows / 8), cv::Scalar(128), 1);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.0);

  return img;
}

/// @brief Helper to set benchmark counters consistently.
inline void setBenchmarkCounters(benchmark::State& state, const cv::Mat& img) {
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) *
                          static_cast<int64_t>(img.total() * img.elemSize()));
}

// Cache test images
struct ImageCache {
  cv::Mat img_480p;
  cv::Mat img_720p;
  cv::Mat img_1080p;

  ImageCache()
      : img_480p{generateTestImage(480, 640)},
        img_720p{generateTestImage(720, 1280)},
        img_1080p{generateTestImage(1080, 1920)} {}

  const cv::Mat& get(int64_t pixels) const {
    if (pixels <= 480 * 640) {
      return img_480p;
    }
    if (pixels <= 720 * 1280) {
      return img_720p;
    }
    return img_1080p;
  }
};

ImageCache& getImageCache() {
  static ImageCache cache;
  return cache;
}

// =============================================================================
// Gradient Computation Benchmarks
// =============================================================================

void BM_SobelGradient(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::DerivativeGradient<uchar, short, int, double, lsfm::SobelDerivative, lsfm::QuadraticMagnitude> grad;

  for (auto _ : state) {
    grad.process(img);
    benchmark::DoNotOptimize(grad.magnitude());
    benchmark::DoNotOptimize(grad.direction());
  }

  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_SobelGradient)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

void BM_ScharrGradient(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::DerivativeGradient<uchar, short, int, double, lsfm::ScharrDerivative, lsfm::QuadraticMagnitude> grad;

  for (auto _ : state) {
    grad.process(img);
    benchmark::DoNotOptimize(grad.magnitude());
    benchmark::DoNotOptimize(grad.direction());
  }

  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_ScharrGradient)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

// =============================================================================
// NMS (Non-Maximum Suppression) Benchmarks
// =============================================================================

void BM_NmsSimple(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));

  // Pre-compute gradient
  lsfm::DerivativeGradient<uchar, short, int, double, lsfm::SobelDerivative, lsfm::QuadraticMagnitude> grad;
  grad.process(img);

  lsfm::NonMaximaSuppression<short, int, double, lsfm::FastNMS8<short, int, double>> nms;

  for (auto _ : state) {
    nms.process(grad);
    benchmark::DoNotOptimize(nms.directionMap());
    benchmark::DoNotOptimize(nms.hysteresis());
  }

  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_NmsSimple)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
