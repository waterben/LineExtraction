//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file bench_lsd.cpp
/// @brief Google Benchmark tests for Line Segment Detection algorithms.
///
/// Benchmarks various LSD implementations across different image sizes.
/// Run with: bazel run //libs/lsd:bench_lsd -- --benchmark_format=console

#include <benchmark/benchmark.h>
#include <lsd/lsd_burns.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>
#include <lsd/lsd_fbw.hpp>
#include <lsd/lsd_fgioi.hpp>
#include <opencv2/imgproc.hpp>

#include <random>

namespace {

/// @brief Helper to set benchmark counters consistently.
/// @param state Benchmark state
/// @param img Input image
/// @param num_lines Number of detected lines
inline void setBenchmarkCounters(benchmark::State& state, const cv::Mat& img, size_t num_lines) {
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations()));
  state.SetBytesProcessed(static_cast<int64_t>(state.iterations()) *
                          static_cast<int64_t>(img.total() * img.elemSize()));
  state.counters["lines"] = static_cast<double>(num_lines);
}

/// @brief Generate a synthetic test image with random edges.
/// @param rows Image height
/// @param cols Image width
/// @param seed Random seed for reproducibility
/// @return Grayscale test image
cv::Mat generateTestImage(int rows, int cols, unsigned int seed = 42) {
  cv::Mat img(rows, cols, CV_8UC1);

  // Create deterministic pseudo-random pattern
  std::mt19937 gen(seed);
  std::uniform_int_distribution<> dis(0, 255);

  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      img.at<uchar>(y, x) = static_cast<uchar>(dis(gen));
    }
  }

  // Add some structure with lines
  cv::line(img, cv::Point(cols / 4, rows / 4), cv::Point(3 * cols / 4, 3 * rows / 4), cv::Scalar(255), 2);
  cv::line(img, cv::Point(cols / 4, 3 * rows / 4), cv::Point(3 * cols / 4, rows / 4), cv::Scalar(0), 2);
  cv::rectangle(img, cv::Point(cols / 8, rows / 8), cv::Point(7 * cols / 8, 7 * rows / 8), cv::Scalar(128), 1);

  // Apply Gaussian blur to create realistic gradients
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.0);

  return img;
}

// Cache test images to avoid regeneration overhead
struct ImageCache {
  cv::Mat img_480p;
  cv::Mat img_720p;
  cv::Mat img_1080p;
  cv::Mat img_4k;

  ImageCache()
      : img_480p{generateTestImage(480, 640)},
        img_720p{generateTestImage(720, 1280)},
        img_1080p{generateTestImage(1080, 1920)},
        img_4k{generateTestImage(2160, 3840)} {}

  const cv::Mat& get(int64_t pixels) const {
    if (pixels <= 480 * 640) {
      return img_480p;
    }
    if (pixels <= 720 * 1280) {
      return img_720p;
    }
    if (pixels <= 1080 * 1920) {
      return img_1080p;
    }
    return img_4k;
  }
};

ImageCache& getImageCache() {
  static ImageCache cache;
  return cache;
}

// =============================================================================
// LsdCC Benchmarks
// =============================================================================

void BM_LsdCC_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdCC<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdCC_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdCP Benchmarks
// =============================================================================

void BM_LsdCP_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdCP<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdCP_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdEL Benchmarks
// =============================================================================

void BM_LsdEL_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdEL<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdEL_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdEP Benchmarks
// =============================================================================

void BM_LsdEP_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdEP<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdEP_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdBurns Benchmarks
// =============================================================================

void BM_LsdBurns_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdBurns<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdBurns_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdFBW Benchmarks
// =============================================================================

void BM_LsdFBW_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdFBW<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdFBW_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdFGioi Benchmarks (Original LSD algorithm by Grompone, Jakubowicz, Morel, Randall)
// =============================================================================

void BM_LsdFGioi_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdFGioi<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdFGioi_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

// =============================================================================
// LsdEDLZ Benchmarks
// =============================================================================

void BM_LsdEDLZ_Detect(benchmark::State& state) {
  const auto& img = getImageCache().get(state.range(0));
  lsfm::LsdEDLZ<double> detector;

  for (auto _ : state) {
    detector.detect(img);
    benchmark::DoNotOptimize(detector.lineSegments());
  }

  setBenchmarkCounters(state, img, detector.lineSegments().size());
}

BENCHMARK(BM_LsdEDLZ_Detect)
    ->Args({480 * 640})
    ->Args({720 * 1280})
    ->Args({1080 * 1920})
    ->Args({2160 * 3840})
    ->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
