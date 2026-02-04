/// @file bench_edge.cpp
/// @brief Google Benchmark tests for Edge Detection algorithms.
///
/// Benchmarks NMS (Non-Maximum Suppression), gradient computation, and
/// Edge Segment Detectors (ESD).
/// Run with: bazel run //libs/edge:bench_edge -- --benchmark_format=console

#include <benchmark/benchmark.h>
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>
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

// =============================================================================
// Edge Segment Detector (ESD) Benchmarks
// =============================================================================

// Pre-computed data cache for ESD benchmarks
struct EsdDataCache {
  using Grad = lsfm::DerivativeGradient<uchar, short, int, double, lsfm::SobelDerivative, lsfm::QuadraticMagnitude>;
  using Nms = lsfm::NonMaximaSuppression<short, int, double, lsfm::FastNMS8<short, int, double>>;

  struct Data {
    Grad grad{};
    Nms nms{};
    bool initialized = false;
  };

  Data data_480p{};
  Data data_720p{};
  Data data_1080p{};

  void ensureInitialized(int64_t pixels) {
    if (pixels <= 480 * 640) {
      if (!data_480p.initialized) {
        data_480p.grad.process(getImageCache().img_480p);
        data_480p.nms.process(data_480p.grad);
        data_480p.initialized = true;
      }
    } else if (pixels <= 720 * 1280) {
      if (!data_720p.initialized) {
        data_720p.grad.process(getImageCache().img_720p);
        data_720p.nms.process(data_720p.grad);
        data_720p.initialized = true;
      }
    } else {
      if (!data_1080p.initialized) {
        data_1080p.grad.process(getImageCache().img_1080p);
        data_1080p.nms.process(data_1080p.grad);
        data_1080p.initialized = true;
      }
    }
  }

  const Data& get(int64_t pixels) {
    ensureInitialized(pixels);
    if (pixels <= 480 * 640) {
      return data_480p;
    }
    if (pixels <= 720 * 1280) {
      return data_720p;
    }
    return data_1080p;
  }
};

EsdDataCache& getEsdDataCache() {
  static EsdDataCache cache;
  return cache;
}

void BM_EsdSimple(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  const auto& data = getEsdDataCache().get(state.range(0));

  lsfm::EsdSimple<int> esd(10);

  for (auto _ : state) {
    esd.detect(data.grad, data.nms);
    benchmark::DoNotOptimize(esd.segments());
    benchmark::DoNotOptimize(esd.points());
  }

  state.counters["segments"] = static_cast<double>(esd.segments().size());
  state.counters["points"] = static_cast<double>(esd.points().size());
  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_EsdSimple)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

void BM_EsdDrawing(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  const auto& data = getEsdDataCache().get(state.range(0));

  lsfm::EsdDrawing<int> esd(10, 3, 5);

  for (auto _ : state) {
    esd.detect(data.grad, data.nms);
    benchmark::DoNotOptimize(esd.segments());
    benchmark::DoNotOptimize(esd.points());
  }

  state.counters["segments"] = static_cast<double>(esd.segments().size());
  state.counters["points"] = static_cast<double>(esd.points().size());
  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_EsdDrawing)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

void BM_EsdLinking(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  const auto& data = getEsdDataCache().get(state.range(0));

  lsfm::EsdLinking<int> esd(10, 3, 3, 5);

  for (auto _ : state) {
    esd.detect(data.grad, data.nms);
    benchmark::DoNotOptimize(esd.segments());
    benchmark::DoNotOptimize(esd.points());
  }

  state.counters["segments"] = static_cast<double>(esd.segments().size());
  state.counters["points"] = static_cast<double>(esd.points().size());
  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_EsdLinking)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

void BM_EsdPattern(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  const auto& data = getEsdDataCache().get(state.range(0));

  lsfm::EsdPattern<int> esd(10, 3, 3, 5, 1);

  for (auto _ : state) {
    esd.detect(data.grad, data.nms);
    benchmark::DoNotOptimize(esd.segments());
    benchmark::DoNotOptimize(esd.points());
  }

  state.counters["segments"] = static_cast<double>(esd.segments().size());
  state.counters["points"] = static_cast<double>(esd.points().size());
  setBenchmarkCounters(state, img);
}

BENCHMARK(BM_EsdPattern)->Args({480 * 640})->Args({720 * 1280})->Args({1080 * 1920})->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
