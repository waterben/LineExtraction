/// @file test_lsd_cp_crash.cpp
/// @brief Regression test: LsdCP must not crash with profile parameters on
///        various image sizes and content.
///
/// The original crash was non-deterministic and occurred only when processing
/// multiple images sequentially with modified parameters (via DetectorProfile).
/// This test exercises LsdCP with different parameter combos and synthetic
/// images that mimic edge-rich scenes to trigger the same code paths.

#include <lsd/lsd_cp.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

using namespace lsfm;

/// Helper: create a synthetic "urban" image with many edges.
static cv::Mat make_urban_image(int rows, int cols, unsigned seed) {
  cv::Mat img(rows, cols, CV_8UC1, cv::Scalar(128));
  cv::RNG rng(seed);

  // Random lines to mimic urban structure
  for (int i = 0; i < 30; ++i) {
    cv::Point p1(rng.uniform(0, cols), rng.uniform(0, rows));
    cv::Point p2(rng.uniform(0, cols), rng.uniform(0, rows));
    int thickness = rng.uniform(1, 3);
    cv::line(img, p1, p2, cv::Scalar(rng.uniform(0, 256)), thickness);
  }

  // Random rectangles
  for (int i = 0; i < 10; ++i) {
    cv::Point p1(rng.uniform(0, cols), rng.uniform(0, rows));
    cv::Point p2(rng.uniform(0, cols), rng.uniform(0, rows));
    cv::rectangle(img, p1, p2, cv::Scalar(rng.uniform(0, 256)), rng.uniform(1, 3));
  }

  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.8);
  return img;
}

/// Helper: create an image that's mostly one brightness (low contrast).
static cv::Mat make_low_contrast_image(int rows, int cols, unsigned seed) {
  cv::Mat img(rows, cols, CV_8UC1, cv::Scalar(120));
  cv::RNG rng(seed);

  for (int i = 0; i < 15; ++i) {
    cv::Point p1(rng.uniform(0, cols), rng.uniform(0, rows));
    cv::Point p2(rng.uniform(0, cols), rng.uniform(0, rows));
    cv::line(img, p1, p2, cv::Scalar(rng.uniform(110, 140)), 2);
  }

  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.0);
  return img;
}

class LsdCPCrashTest : public ::testing::Test {
 protected:
  LsdCPCrashTest() : images_() {}

  void SetUp() override {
    // Generate a bank of images with different sizes and RNG seeds
    std::vector<std::pair<int, int>> sizes = {{320, 240}, {480, 640}, {240, 320}, {200, 200},
                                              {640, 480}, {100, 100}, {500, 375}, {375, 500}};
    for (unsigned i = 0; i < sizes.size(); ++i) {
      images_.push_back(make_urban_image(sizes[i].first, sizes[i].second, i * 42));
    }
    for (unsigned i = 0; i < 4; ++i) {
      images_.push_back(make_low_contrast_image(320, 240, 1000 + i));
    }
  }

  std::vector<cv::Mat> images_;
};

/// Default parameters — should always work.
TEST_F(LsdCPCrashTest, DefaultParams_MultipleImages) {
  LsdCP<double> lsd(0.004, 0.012, 10, 0, 2.0, 2, CP_FIND_NEAR_COMPLEX);

  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Profile-style parameters: tolerance = 2, min_pix = 25, max_gap = 3
TEST_F(LsdCPCrashTest, ProfileParams_MultipleImages) {
  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    LsdCP<double> lsd(0.004, 0.012, 25, 3, 2.0, 2, CP_FIND_NEAR_COMPLEX);
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Same detector instance reused — tests clearData path.
TEST_F(LsdCPCrashTest, ProfileParams_Reuse_MultipleImages) {
  LsdCP<double> lsd(0.004, 0.012, 25, 3, 2.0, 2, CP_FIND_NEAR_COMPLEX);

  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Tiny tolerance (1).
TEST_F(LsdCPCrashTest, SmallTolerance_MultipleImages) {
  LsdCP<double> lsd(0.004, 0.012, 15, 0, 3.0, 1, CP_FIND_NEAR_COMPLEX);

  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Large tolerance (5).
TEST_F(LsdCPCrashTest, LargeTolerance_MultipleImages) {
  LsdCP<double> lsd(0.004, 0.012, 5, 0, 2.0, 5, CP_FIND_NEAR_COMPLEX);

  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Corner rule + merge mode.
TEST_F(LsdCPCrashTest, CornerRuleMerge_MultipleImages) {
  LsdCP<double> lsd(0.004, 0.012, 10, 0, 2.0, 2, CP_FIND_NEAR_COMPLEX | CP_CORNER_RULE | CP_MERGE);

  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Profile with ValueManager set_value (simulating Python set_int / set_float).
TEST_F(LsdCPCrashTest, ValueManagerParams_MultipleImages) {
  for (size_t i = 0; i < images_.size(); ++i) {
    SCOPED_TRACE("image #" + std::to_string(i));
    LsdCP<double> lsd;

    // Apply profile-style params through ValueManager API (same as Python code)
    lsd.value("edge_min_pixels", Value(25));
    lsd.value("edge_max_gap", Value(3));
    lsd.value("split_error_distance", Value(2.5));
    lsd.value("edge_pattern_tolerance", Value(2));

    EXPECT_NO_THROW(lsd.detect(images_[i]));
  }
}

/// Real-world-like dimensions with many sequential detections.
TEST_F(LsdCPCrashTest, HighVolume_50Runs) {
  LsdCP<double> lsd(0.004, 0.012, 20, 2, 2.0, 2, CP_FIND_NEAR_COMPLEX);

  for (int run = 0; run < 50; ++run) {
    cv::Mat img = make_urban_image(480, 640, static_cast<unsigned>(run * 7 + 13));
    SCOPED_TRACE("run #" + std::to_string(run));
    EXPECT_NO_THROW(lsd.detect(img));
  }
}
