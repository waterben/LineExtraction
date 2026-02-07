/// @file test_filter_binding.cpp
/// @brief C++ unit tests for types exposed via the imgproc pybind11 bindings.
///
/// Tests the C++ classes and interfaces that are wrapped by the Python bindings,
/// ensuring correctness of FilterData, Range, DerivativeGradient, and ValueManager
/// before they are exposed to Python.

#include <imgproc/derivative_gradient.hpp>
#include <imgproc/filter.hpp>
#include <imgproc/gradient.hpp>
#include <imgproc/laplace.hpp>
#include <opencv2/core/core.hpp>
#include <utility/range.hpp>

#include <gtest/gtest.h>

namespace lsfm {
namespace {

/// @brief Test that Range types work correctly.
TEST(FilterBindingTest, RangeBasics) {
  Range<double> rd(0.0, 1.0);
  EXPECT_DOUBLE_EQ(rd.lower, 0.0);
  EXPECT_DOUBLE_EQ(rd.upper, 1.0);
  EXPECT_DOUBLE_EQ(rd.size(), 1.0);

  Range<int> ri(-10, 10);
  EXPECT_EQ(ri.size(), 20);

  Range<uchar> ru(0, 255);
  EXPECT_EQ(ru.size(), 255);
}

/// @brief Test FilterData construction and access.
TEST(FilterBindingTest, FilterDataBasics) {
  cv::Mat img = cv::Mat::zeros(10, 10, CV_8U);
  FilterData fd(img, 0.0, 255.0);

  EXPECT_FALSE(fd.data.empty());
  EXPECT_EQ(fd.data.rows, 10);
  EXPECT_EQ(fd.data.cols, 10);
  EXPECT_DOUBLE_EQ(fd.range.lower, 0.0);
  EXPECT_DOUBLE_EQ(fd.range.upper, 255.0);
}

/// @brief Test SobelGradient (DerivativeGradient) construction and processing.
TEST(FilterBindingTest, SobelGradientProcess) {
  // Use float for MT (matching existing tests) to avoid cv::sqrt SIMD issues with int
  using SobelGrad = DerivativeGradient<uchar, short, float, float>;

  SobelGrad grad;
  EXPECT_EQ(grad.name(), "derivative_sobel");

  // Create test image with a vertical edge
  cv::Mat img = cv::Mat::zeros(64, 64, CV_8U);
  img(cv::Rect(32, 0, 32, 64)) = 255;

  grad.process(img);

  cv::Mat mag = grad.magnitude();
  EXPECT_FALSE(mag.empty());
  EXPECT_EQ(mag.rows, 64);
  EXPECT_EQ(mag.cols, 64);

  cv::Mat dir = grad.direction();
  EXPECT_FALSE(dir.empty());

  // Check magnitude is non-zero at the edge
  float center_mag = mag.at<float>(32, 32);
  EXPECT_GT(center_mag, 0.0f);

  // Check results() returns expected keys
  FilterResults res = grad.results();
  EXPECT_GT(res.count("gx"), 0u);
  EXPECT_GT(res.count("gy"), 0u);
  EXPECT_GT(res.count("mag"), 0u);
  EXPECT_GT(res.count("dir"), 0u);
}

/// @brief Test ScharrGradient construction.
TEST(FilterBindingTest, ScharrGradientBasics) {
  using ScharrGrad = DerivativeGradient<uchar, short, float, float, ScharrDerivative>;

  ScharrGrad grad;
  EXPECT_EQ(grad.name(), "derivative_scharr");

  auto range = grad.intensityRange();
  EXPECT_EQ(range.lower, 0);
  EXPECT_EQ(range.upper, 255);
}

/// @brief Test ValueManager parameter access.
TEST(FilterBindingTest, ValueManagerAccess) {
  using SobelGrad = DerivativeGradient<uchar, short, float, float>;

  SobelGrad grad;

  // Should have configurable parameters
  auto vals = grad.values();
  EXPECT_FALSE(vals.empty());

  // Check grad_kernel_size parameter exists
  bool found_kernel = false;
  for (const auto& nv : vals) {
    if (nv.name == "grad_kernel_size") {
      found_kernel = true;
      break;
    }
  }
  EXPECT_TRUE(found_kernel);
}

}  // namespace
}  // namespace lsfm
