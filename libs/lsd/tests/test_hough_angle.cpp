//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_hough_angle.cpp
/// @brief Test to verify Hough line angle conversion
///
/// This test verifies that OpenCV HoughLines theta values are correctly
/// converted to Line objects in the lsfm library.

#include <geometry/line.hpp>
#include <lsd/lsd_hcv.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace lsfm;
using namespace cv;

class HoughAngleTest : public ::testing::Test {
 protected:
  void SetUp() override {}
};

/**
 * @brief Test that a horizontal line (theta = PI/2) is correctly detected
 *
 * In OpenCV HoughLines:
 * - theta = 0 means a vertical line (normal points right along x-axis)
 * - theta = PI/2 means a horizontal line (normal points down along y-axis)
 */
TEST_F(HoughAngleTest, HorizontalLineConversion) {
  // Create a test image with a horizontal line
  Mat test_img = Mat::zeros(200, 200, CV_8UC1);
  line(test_img, Point(20, 100), Point(180, 100), Scalar(255), 3);
  GaussianBlur(test_img, test_img, Size(3, 3), 0.8);

  // Use LdHough directly to check the Line objects before tracing
  LdHough<float, Vec2> ld_detector(0.004f, 0.012f, 1.0, CV_PI / 180, 30);
  ld_detector.detect(test_img);

  const auto& lines = ld_detector.lines();

  // We should detect at least one line
  ASSERT_GT(lines.size(), size_t{0}) << "No lines detected by LdHough";

  // Find the horizontal line (normal angle close to PI/2)
  float min_angle_diff = std::numeric_limits<float>::max();
  const Line<float, Vec2>* best_line = nullptr;

  for (const auto& l : lines) {
    float normal_angle = l.normalAngle();
    // Horizontal line has normal pointing down, so normal_angle ~= PI/2
    float angle_diff = std::abs(normal_angle - static_cast<float>(CV_PI / 2));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      best_line = &l;
    }
  }

  ASSERT_NE(best_line, nullptr);

  // The horizontal line should have normal angle close to PI/2 (90 degrees)
  // and line angle close to 0 degrees
  // Tolerance: 15 degrees = 0.26 radians
  EXPECT_LT(min_angle_diff, 0.26f) << "Horizontal line: expected normal angle close to 90 degrees, " << "but got "
                                   << best_line->normalAngle() * 180 / CV_PI << " degrees. "
                                   << "Line angle: " << best_line->angle() * 180 / CV_PI << " degrees";

  // Also check the line angle (should be 0 for horizontal)
  float line_angle = best_line->angle();
  float line_angle_diff = std::min(std::abs(line_angle), std::abs(line_angle - static_cast<float>(CV_PI)));
  line_angle_diff = std::min(line_angle_diff, std::abs(line_angle + static_cast<float>(CV_PI)));
  EXPECT_LT(line_angle_diff, 0.26f) << "Horizontal line: expected line angle close to 0 or 180 degrees, " << "but got "
                                    << line_angle * 180 / CV_PI << " degrees";
}

/**
 * @brief Test that a vertical line (theta = 0) is correctly detected
 */
TEST_F(HoughAngleTest, VerticalLineConversion) {
  // Create a test image with a vertical line
  Mat test_img = Mat::zeros(200, 200, CV_8UC1);
  line(test_img, Point(100, 20), Point(100, 180), Scalar(255), 3);
  GaussianBlur(test_img, test_img, Size(3, 3), 0.8);

  // Use LdHough directly to check the Line objects before tracing
  LdHough<float, Vec2> ld_detector(0.004f, 0.012f, 1.0, CV_PI / 180, 30);
  ld_detector.detect(test_img);

  const auto& lines = ld_detector.lines();

  // We should detect at least one line
  ASSERT_GT(lines.size(), size_t{0}) << "No lines detected by LdHough";

  // Find the vertical line (normal angle close to 0 or PI)
  float min_angle_diff = std::numeric_limits<float>::max();
  const Line<float, Vec2>* best_line = nullptr;

  for (const auto& l : lines) {
    float normal_angle = l.normalAngle();
    // Vertical line has normal pointing right or left, so normal_angle ~= 0 or PI
    float angle_diff = std::min(std::abs(normal_angle), std::abs(normal_angle - static_cast<float>(CV_PI)));
    angle_diff = std::min(angle_diff, std::abs(normal_angle + static_cast<float>(CV_PI)));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      best_line = &l;
    }
  }

  ASSERT_NE(best_line, nullptr);

  // The vertical line should have normal angle close to 0 degrees
  // and line angle close to 90 degrees
  EXPECT_LT(min_angle_diff, 0.26f) << "Vertical line: expected normal angle close to 0 or 180 degrees, " << "but got "
                                   << best_line->normalAngle() * 180 / CV_PI << " degrees. "
                                   << "Line angle: " << best_line->angle() * 180 / CV_PI << " degrees";

  // Also check the line angle (should be ±90 for vertical)
  float line_angle = best_line->angle();
  float line_angle_diff = std::min(std::abs(line_angle - static_cast<float>(CV_PI / 2)),
                                   std::abs(line_angle + static_cast<float>(CV_PI / 2)));
  EXPECT_LT(line_angle_diff, 0.26f) << "Vertical line: expected line angle close to 90 or -90 degrees, " << "but got "
                                    << line_angle * 180 / CV_PI << " degrees";
}

/**
 * @brief Test that a diagonal line is correctly detected
 */
TEST_F(HoughAngleTest, DiagonalLineConversion) {
  // Create a test image with a 45-degree diagonal line
  Mat test_img = Mat::zeros(200, 200, CV_8UC1);
  line(test_img, Point(20, 20), Point(180, 180), Scalar(255), 3);
  GaussianBlur(test_img, test_img, Size(3, 3), 0.8);

  // Use LdHough directly to check the Line objects before tracing
  LdHough<float, Vec2> ld_detector(0.004f, 0.012f, 1.0, CV_PI / 180, 30);
  ld_detector.detect(test_img);

  const auto& lines = ld_detector.lines();

  // We should detect at least one line
  ASSERT_GT(lines.size(), size_t{0}) << "No lines detected by LdHough";

  // Find the diagonal line (normal angle close to 3*PI/4 or -PI/4)
  // A line from (0,0) to (1,1) has direction (1,1), so normal is (1,-1) normalized,
  // which gives normal_angle = -PI/4 or 3*PI/4
  float min_angle_diff = std::numeric_limits<float>::max();
  const Line<float, Vec2>* best_line = nullptr;

  for (const auto& l : lines) {
    float normal_angle = l.normalAngle();
    // Diagonal line going from top-left to bottom-right has normal at -45 or 135 degrees
    float angle_diff = std::min(std::abs(normal_angle - static_cast<float>(3 * CV_PI / 4)),
                                std::abs(normal_angle + static_cast<float>(CV_PI / 4)));
    angle_diff = std::min(angle_diff, std::abs(normal_angle - static_cast<float>(-CV_PI / 4)));
    angle_diff = std::min(angle_diff, std::abs(normal_angle + static_cast<float>(3 * CV_PI / 4)));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      best_line = &l;
    }
  }

  ASSERT_NE(best_line, nullptr);

  // Tolerance: 15 degrees = 0.26 radians
  EXPECT_LT(min_angle_diff, 0.26f) << "Diagonal line: expected normal angle close to 135 or -45 degrees, " << "but got "
                                   << best_line->normalAngle() * 180 / CV_PI << " degrees. "
                                   << "Line angle: " << best_line->angle() * 180 / CV_PI << " degrees";
}

/**
 * @brief Diagnostic test with real image to check line detection
 */
TEST_F(HoughAngleTest, RealImageDiagnostic) {
  // Try to load the real test image
  Mat test_img = cv::imread("/home/waterben/workspace/LineExtraction/6.jpg", cv::IMREAD_GRAYSCALE);
  if (test_img.empty()) {
    GTEST_SKIP() << "Test image 6.jpg not found, skipping real image test";
  }
  std::cout << "Image size: " << test_img.cols << "x" << test_img.rows << std::endl;

  // First, look at what LdHough detects (raw Hough lines)
  LdHough<float, Vec2> ld_detector(0.004f, 0.012f, 1.0, CV_PI / 180, 30);
  ld_detector.detect(test_img);
  const auto& hough_lines = ld_detector.lines();
  std::cout << "Number of Hough lines: " << hough_lines.size() << std::endl;

  // Look at edge map statistics
  cv::Mat edge_map = ld_detector.edgeSource().hysteresis_binary();
  int edge_count = cv::countNonZero(edge_map);
  std::cout << "Edge map: " << edge_map.cols << "x" << edge_map.rows << ", edge pixels: " << edge_count << std::endl;

  // Show first 10 Hough lines
  std::cout << "\nFirst 10 Hough lines:" << std::endl;
  for (size_t i = 0; i < std::min(hough_lines.size(), size_t{10}); ++i) {
    const auto& l = hough_lines[i];
    std::cout << "  Line " << i << ": normal_angle=" << std::fixed << std::setprecision(1)
              << l.normalAngle() * 180 / CV_PI << "°" << ", line_angle=" << l.angle() * 180 / CV_PI << "°"
              << ", dist=" << std::setprecision(1) << l.originDist() << std::endl;
  }

  // Skip full segment detection since it's too slow - just test individual tracing
  LineTracer<float, Vec2> tracer(10, 3);
  cv::Mat edge_copy = edge_map.clone();

  // Trace the first Hough line
  if (!hough_lines.empty()) {
    Line2Vector<float, Vec2> single_line;
    single_line.push_back(hough_lines[0]);
    tracer.traceBinary(single_line, edge_copy);
    std::cout << "\nTracing first Hough line: found " << tracer.points().size() << " points, "
              << tracer.segments().size() << " segments" << std::endl;
  }

  EXPECT_GT(hough_lines.size(), size_t{0}) << "Should detect at least some lines";
}

/**
 * @brief Diagnostic test to check if the LineTracer finds pixels along Hough lines
 *
 * This test verifies that the tracer finds enough pixels and that the fitted segments
 * have the correct orientation.
 */
/**
 * @brief Test to detect the originally reported bug: segments with -90 or 90 degrees and very short
 */
TEST_F(HoughAngleTest, TracerDiagnostic) {
  // Create a test image with a 45-degree diagonal line
  Mat test_img = Mat::zeros(200, 200, CV_8UC1);
  line(test_img, Point(20, 20), Point(180, 180), Scalar(255), 1);  // Thin line, 1 pixel

  // Use full LsdHough with DEFAULT parameters (no custom Hough params)
  // Note: Constructor takes (th_low, th_high, minPix, maxGap, flags)
  LsdHough<float, Vec2> lsd_detector;  // Use defaults
  lsd_detector.detect(test_img);

  // Check Hough lines from the internal detector
  const auto& hough_lines = lsd_detector.lineDetector().lines();
  std::cout << "Internal Hough lines: " << hough_lines.size() << std::endl;

  // Check tracer segments
  const auto& tracer_segments = lsd_detector.lineSupportSegments();
  std::cout << "Tracer segments: " << tracer_segments.size() << std::endl;
  for (size_t i = 0; i < tracer_segments.size(); ++i) {
    const auto& seg = tracer_segments[i];
    size_t seg_size = seg.end() - seg.begin();
    std::cout << "  Tracer Segment " << i << ": begin=" << seg.begin() << ", end=" << seg.end() << ", size=" << seg_size
              << ", id=" << seg.id() << std::endl;
  }

  // Check points
  const auto& points = lsd_detector.points();
  std::cout << "Points: " << points.size() << std::endl;

  // Check final segments
  const auto& segments = lsd_detector.lineSegments();
  std::cout << "\nFinal segments: " << segments.size() << std::endl;
  for (size_t i = 0; i < segments.size(); ++i) {
    const auto& seg = segments[i];
    bool is_nan = std::isnan(seg.angle()) || std::isnan(seg.length());
    std::cout << "  Segment " << i << ": angle=" << seg.angle() * 180 / CV_PI << "°" << ", length=" << seg.length();
    if (is_nan) {
      std::cout << " [NaN WARNING]";
    }
    std::cout << std::endl;
  }

  // Check: tracer_segments.size() should equal segments.size()
  EXPECT_EQ(tracer_segments.size(), segments.size()) << "Tracer segment count should equal final segment count";

  // Count NaN segments
  int nan_count = 0;
  for (const auto& seg : segments) {
    if (std::isnan(seg.angle()) || std::isnan(seg.length())) {
      ++nan_count;
    }
  }

  EXPECT_EQ(nan_count, 0) << "There should be no NaN segments";
}

/**
 * @brief Test with the exact parameters used by the line_analyzer app
 *
 * App defaults:
 * - th_low = 0.004, th_high = 0.012
 * - minPix = 10, maxGap = 3
 * - rho = 1.5, theta = CV_PI/180, voteThreshold = 150
 */
TEST_F(HoughAngleTest, AppDefaultParameters) {
  // Try to load the real test image
  Mat test_img = cv::imread("/home/waterben/workspace/LineExtraction/6.jpg", cv::IMREAD_GRAYSCALE);
  if (test_img.empty()) {
    GTEST_SKIP() << "Test image 6.jpg not found, skipping app defaults test";
  }
  std::cout << "Image size: " << test_img.cols << "x" << test_img.rows << std::endl;

  // Use LsdHough with DEFAULT parameters (same as app)
  LsdHough<float, Vec2> lsd_detector;  // Uses th_low=0.004, th_high=0.012, minPix=10, maxGap=3
  lsd_detector.detect(test_img);

  // Check Hough lines
  const auto& hough_lines = lsd_detector.lineDetector().lines();
  std::cout << "Hough lines detected: " << hough_lines.size() << std::endl;

  // Check segments
  const auto& segments = lsd_detector.lineSegments();
  std::cout << "Line segments detected: " << segments.size() << std::endl;

  // Analyze angle distribution
  std::map<int, int> angle_histogram;
  int vertical_count = 0;
  int horizontal_count = 0;
  int short_count = 0;
  int nan_count = 0;

  for (const auto& seg : segments) {
    if (std::isnan(seg.angle()) || std::isnan(seg.length())) {
      ++nan_count;
      continue;
    }

    int angle_deg = static_cast<int>(std::round(seg.angle() * 180.0f / static_cast<float>(CV_PI)));
    // Normalize to [-90, 90]
    while (angle_deg > 90) angle_deg -= 180;
    while (angle_deg < -90) angle_deg += 180;
    angle_histogram[angle_deg]++;

    // Check for vertical (90/-90) - the reported bug
    if (std::abs(angle_deg) > 80) {
      ++vertical_count;
    }
    // Check for horizontal (near 0)
    if (std::abs(angle_deg) < 10) {
      ++horizontal_count;
    }
    // Short segments
    if (seg.length() < 15.0f) {
      ++short_count;
    }
  }

  std::cout << "Angle distribution summary:" << std::endl;
  std::cout << "  Vertical (|angle| > 80°): " << vertical_count << std::endl;
  std::cout << "  Horizontal (|angle| < 10°): " << horizontal_count << std::endl;
  std::cout << "  Short segments (< 15 px): " << short_count << std::endl;
  std::cout << "  NaN segments: " << nan_count << std::endl;

  // Show first 20 segments
  std::cout << "\nFirst 20 segments:" << std::endl;
  for (size_t i = 0; i < std::min(segments.size(), size_t{20}); ++i) {
    const auto& seg = segments[i];
    std::cout << "  Seg " << i << ": angle=" << std::fixed << std::setprecision(1) << seg.angle() * 180 / CV_PI << "°"
              << ", length=" << std::setprecision(1) << seg.length() << ", start=(" << std::setprecision(0)
              << seg.startPoint()[0] << "," << seg.startPoint()[1] << ")" << ", end=(" << seg.endPoint()[0] << ","
              << seg.endPoint()[1] << ")" << std::endl;
  }

  // Expectations - these may need adjustment based on your expected behavior
  EXPECT_EQ(nan_count, 0) << "No NaN segments should be produced";
  EXPECT_GT(segments.size(), size_t{0}) << "Should detect some line segments";

  // The originally reported bug was: "alle line segments haben immer -90 oder 90°"
  // If most segments are vertical, that's the bug
  if (segments.size() > 10) {
    float vertical_ratio = static_cast<float>(vertical_count) / static_cast<float>(segments.size());
    std::cout << "Vertical segment ratio: " << std::setprecision(2) << vertical_ratio * 100 << "%" << std::endl;
    EXPECT_LT(vertical_ratio, 0.8f) << "Too many vertical segments (" << vertical_count << " of " << segments.size()
                                    << ") - this may indicate the 90-degree rotation bug";
  }
}

/**
 * @brief Direct test of Line class angle conventions with OpenCV HoughLines theta values
 *
 * This test directly checks that when we create a Line from HoughLines (rho, theta) values,
 * the resulting Line has the correct angle.
 */
TEST_F(HoughAngleTest, DirectLineAngleConversion) {
  // Test 1: theta = 0 in OpenCV means vertical line (normal points right)
  {
    float theta = 0.0f;
    float rho = 50.0f;
    Line<float, Vec2> line(theta, rho);  // (normal_ang, distance)

    // For theta = 0: nx = cos(0) = 1, ny = sin(0) = 0
    EXPECT_NEAR(line.normalX(), 1.0f, 0.001f);
    EXPECT_NEAR(line.normalY(), 0.0f, 0.001f);

    // The line angle should be -90 degrees (or 90 degrees), perpendicular to normal
    // angle() = atan2(-nx, ny) = atan2(-1, 0) = -PI/2
    float expected_angle = static_cast<float>(-CV_PI / 2);
    EXPECT_NEAR(line.angle(), expected_angle, 0.001f)
        << "For theta=0, line angle should be -90 degrees (vertical line), " << "but got " << line.angle() * 180 / CV_PI
        << " degrees";
  }

  // Test 2: theta = PI/2 in OpenCV means horizontal line (normal points down)
  {
    float theta = static_cast<float>(CV_PI / 2);
    float rho = 50.0f;
    Line<float, Vec2> line(theta, rho);

    // For theta = PI/2: nx = cos(PI/2) = 0, ny = sin(PI/2) = 1
    EXPECT_NEAR(line.normalX(), 0.0f, 0.001f);
    EXPECT_NEAR(line.normalY(), 1.0f, 0.001f);

    // The line angle should be 0 degrees (horizontal line)
    // angle() = atan2(-nx, ny) = atan2(0, 1) = 0
    float expected_angle = 0.0f;
    EXPECT_NEAR(line.angle(), expected_angle, 0.001f)
        << "For theta=PI/2, line angle should be 0 degrees (horizontal line), " << "but got "
        << line.angle() * 180 / CV_PI << " degrees";
  }

  // Test 3: theta = PI/4 in OpenCV means 45-degree line
  {
    float theta = static_cast<float>(CV_PI / 4);
    float rho = 50.0f;
    Line<float, Vec2> line(theta, rho);

    // For theta = PI/4: nx = cos(PI/4) = sqrt(2)/2, ny = sin(PI/4) = sqrt(2)/2
    EXPECT_NEAR(line.normalX(), std::sqrt(2.0f) / 2, 0.001f);
    EXPECT_NEAR(line.normalY(), std::sqrt(2.0f) / 2, 0.001f);

    // The line angle should be -45 degrees
    // angle() = atan2(-nx, ny) = atan2(-sqrt(2)/2, sqrt(2)/2) = -PI/4
    float expected_angle = static_cast<float>(-CV_PI / 4);
    EXPECT_NEAR(line.angle(), expected_angle, 0.001f) << "For theta=PI/4, line angle should be -45 degrees, "
                                                      << "but got " << line.angle() * 180 / CV_PI << " degrees";
  }
}
