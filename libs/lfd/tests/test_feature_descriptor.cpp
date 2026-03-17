//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_feature_descriptor.cpp
/// @brief Unit tests for feature descriptors.

#include <geometry/line.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/FeatureFilter.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace lsfm;
using namespace cv;

// Mock feature descriptor for testing Mat interface
template <class FT>
class MockFeatureDescriptorMat : public FdcMatI<FT, LineSegment<FT>> {
 public:
  typedef LineSegment<FT> GeometricType;

  void create(const GeometricType& input, cv::Mat& dst) override {
    dst.create(1, static_cast<int>(size()), cv::DataType<FT>::type);
    FT* ptr = dst.template ptr<FT>(0);
    create(input, ptr);
  }

  size_t size() const override {
    return 4;  // [start_x, start_y, end_x, end_y]
  }

 protected:
  void create(const GeometricType& input, FT* dst) override {
    // Simple descriptor: line endpoints
    dst[0] = static_cast<FT>(input.startPoint().x());
    dst[1] = static_cast<FT>(input.startPoint().y());
    dst[2] = static_cast<FT>(input.endPoint().x());
    dst[3] = static_cast<FT>(input.endPoint().y());
  }
};

// Mock feature descriptor for testing Object interface
template <class FT>
struct MockDescriptor {
  FT values[4];

  MockDescriptor() { std::fill(values, values + 4, FT(0)); }

  MockDescriptor(FT v0, FT v1, FT v2, FT v3) {
    values[0] = v0;
    values[1] = v1;
    values[2] = v2;
    values[3] = v3;
  }

  bool operator==(const MockDescriptor& other) const {
    for (int i = 0; i < 4; ++i) {
      if (std::abs(values[i] - other.values[i]) > FT(1e-6)) {
        return false;
      }
    }
    return true;
  }
};

template <class FT>
class MockFeatureDescriptorObj : public FdcObjI<FT, LineSegment<FT>, MockDescriptor<FT>> {
 public:
  typedef LineSegment<FT> GeometricType;
  typedef MockDescriptor<FT> DescriptorType;

  void create(const GeometricType& input, DescriptorType& dst) override {
    // Create descriptor based on line properties
    dst.values[0] = static_cast<FT>(input.startPoint().x());
    dst.values[1] = static_cast<FT>(input.startPoint().y());
    dst.values[2] = static_cast<FT>(input.endPoint().x());
    dst.values[3] = static_cast<FT>(input.endPoint().y());
  }
};

class FeatureDescriptorTest : public ::testing::Test {
 protected:
  FeatureDescriptorTest() : test_lines(), mat_descriptor(), obj_descriptor() {}

  void SetUp() override {
    // Create test lines
    test_lines.clear();
    test_lines.emplace_back(Point2f(0, 0), Point2f(1, 1));
    test_lines.emplace_back(Point2f(2, 1), Point2f(4, 3));
    test_lines.emplace_back(Point2f(1, 4), Point2f(5, 2));

    mat_descriptor = std::make_unique<MockFeatureDescriptorMat<float>>();
    obj_descriptor = std::make_unique<MockFeatureDescriptorObj<float>>();
  }

  std::vector<LineSegment<float>> test_lines;
  std::unique_ptr<MockFeatureDescriptorMat<float>> mat_descriptor;
  std::unique_ptr<MockFeatureDescriptorObj<float>> obj_descriptor;
};

TEST_F(FeatureDescriptorTest, MatDescriptorSize) { EXPECT_EQ(mat_descriptor->size(), static_cast<size_t>(4)); }

TEST_F(FeatureDescriptorTest, SingleMatDescriptor) {
  const auto& line = test_lines[0];
  Mat descriptor;

  mat_descriptor->create(line, descriptor);

  EXPECT_EQ(descriptor.rows, 1);
  EXPECT_EQ(descriptor.cols, 4);
  EXPECT_EQ(descriptor.type(), CV_32F);

  float* data = descriptor.ptr<float>(0);
  EXPECT_FLOAT_EQ(data[0], 0.0f);  // start_x
  EXPECT_FLOAT_EQ(data[1], 0.0f);  // start_y
  EXPECT_FLOAT_EQ(data[2], 1.0f);  // end_x
  EXPECT_FLOAT_EQ(data[3], 1.0f);  // end_y
}

TEST_F(FeatureDescriptorTest, MultipleMatDescriptors) {
  Mat descriptors;

  mat_descriptor->createMat(test_lines, descriptors);

  EXPECT_EQ(descriptors.rows, static_cast<int>(test_lines.size()));
  EXPECT_EQ(descriptors.cols, 4);
  EXPECT_EQ(descriptors.type(), CV_32F);

  // Check first descriptor
  float* data0 = descriptors.ptr<float>(0);
  EXPECT_FLOAT_EQ(data0[0], 0.0f);
  EXPECT_FLOAT_EQ(data0[1], 0.0f);
  EXPECT_FLOAT_EQ(data0[2], 1.0f);
  EXPECT_FLOAT_EQ(data0[3], 1.0f);

  // Check second descriptor
  float* data1 = descriptors.ptr<float>(1);
  EXPECT_FLOAT_EQ(data1[0], 2.0f);
  EXPECT_FLOAT_EQ(data1[1], 1.0f);
  EXPECT_FLOAT_EQ(data1[2], 4.0f);
  EXPECT_FLOAT_EQ(data1[3], 3.0f);
}

TEST_F(FeatureDescriptorTest, ObjectDescriptorCreation) {
  const auto& line = test_lines[0];
  MockDescriptor<float> descriptor;

  obj_descriptor->create(line, descriptor);

  EXPECT_FLOAT_EQ(descriptor.values[0], 0.0f);
  EXPECT_FLOAT_EQ(descriptor.values[1], 0.0f);
  EXPECT_FLOAT_EQ(descriptor.values[2], 1.0f);
  EXPECT_FLOAT_EQ(descriptor.values[3], 1.0f);
}

TEST_F(FeatureDescriptorTest, ObjectDescriptorList) {
  std::vector<MockDescriptor<float>> descriptors;
  const auto& const_test_lines = test_lines;  // Make const to resolve ambiguity

  obj_descriptor->createList(const_test_lines, descriptors);

  EXPECT_EQ(descriptors.size(), test_lines.size());

  // Check first descriptor
  EXPECT_FLOAT_EQ(descriptors[0].values[0], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[0].values[1], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[0].values[2], 1.0f);
  EXPECT_FLOAT_EQ(descriptors[0].values[3], 1.0f);

  // Check second descriptor
  EXPECT_FLOAT_EQ(descriptors[1].values[0], 2.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[1], 1.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[2], 4.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[3], 3.0f);
}

TEST_F(FeatureDescriptorTest, ObjectDescriptorListWithMask) {
  std::vector<MockDescriptor<float>> descriptors;
  std::vector<bool> mask = {true, false, true};  // Skip middle line

  obj_descriptor->createList(test_lines, mask, descriptors);

  EXPECT_EQ(descriptors.size(), test_lines.size());

  // First descriptor should be valid
  EXPECT_FLOAT_EQ(descriptors[0].values[0], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[0].values[1], 0.0f);

  // Second descriptor should be empty (default values)
  EXPECT_FLOAT_EQ(descriptors[1].values[0], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[1], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[2], 0.0f);
  EXPECT_FLOAT_EQ(descriptors[1].values[3], 0.0f);

  // Third descriptor should be valid
  EXPECT_FLOAT_EQ(descriptors[2].values[0], 1.0f);
  EXPECT_FLOAT_EQ(descriptors[2].values[1], 4.0f);
  EXPECT_FLOAT_EQ(descriptors[2].values[2], 5.0f);
  EXPECT_FLOAT_EQ(descriptors[2].values[3], 2.0f);
}

TEST_F(FeatureDescriptorTest, EmptyInput) {
  std::vector<LineSegment<float>> empty_lines;
  Mat descriptors;

  mat_descriptor->createMat(empty_lines, descriptors);

  EXPECT_EQ(descriptors.rows, 0);
  EXPECT_EQ(descriptors.cols, 4);
}

TEST_F(FeatureDescriptorTest, EmptyObjectList) {
  std::vector<LineSegment<float>> empty_lines;
  std::vector<MockDescriptor<float>> descriptors;
  const auto& const_empty_lines = empty_lines;  // Make const to resolve ambiguity

  obj_descriptor->createList(const_empty_lines, descriptors);

  EXPECT_EQ(descriptors.size(), size_t{0});
}

TEST_F(FeatureDescriptorTest, TypeTemplating) {
  // Test with double precision
  std::vector<LineSegment<double>> double_lines;
  double_lines.emplace_back(Point2d(0.0, 0.0), Point2d(1.5, 2.5));

  MockFeatureDescriptorMat<double> double_descriptor;
  Mat descriptors;

  double_descriptor.createMat(double_lines, descriptors);

  EXPECT_EQ(descriptors.rows, 1);
  EXPECT_EQ(descriptors.cols, 4);
  EXPECT_EQ(descriptors.type(), CV_64F);

  double* data = descriptors.ptr<double>(0);
  EXPECT_DOUBLE_EQ(data[0], 0.0);
  EXPECT_DOUBLE_EQ(data[1], 0.0);
  EXPECT_DOUBLE_EQ(data[2], 1.5);
  EXPECT_DOUBLE_EQ(data[3], 2.5);
}

TEST_F(FeatureDescriptorTest, MatResizing) {
  Mat descriptors(10, 10, CV_32F);  // Wrong size initially

  mat_descriptor->createMat(test_lines, descriptors);

  // Should have been resized correctly
  EXPECT_EQ(descriptors.rows, static_cast<int>(test_lines.size()));
  EXPECT_EQ(descriptors.cols, 4);
  EXPECT_EQ(descriptors.type(), CV_32F);
}

// ============================================================================
// LBD Normalization Tests
// ============================================================================

/// @brief Fixture for LBD descriptor tests.
/// Creates a synthetic gradient image with known structure so that LBD
/// descriptor computation exercises the full normalization pipeline.
class LbdDescriptorTest : public ::testing::Test {
 protected:
  static constexpr int kImageSize = 200;

  void SetUp() override {
    // Create a synthetic image with horizontal and vertical edges
    cv::Mat img = cv::Mat::zeros(kImageSize, kImageSize, CV_8U);
    // Draw rectangles to produce clear gradient structure
    cv::rectangle(img, cv::Point(30, 30), cv::Point(170, 170), cv::Scalar(200), -1);
    cv::rectangle(img, cv::Point(60, 60), cv::Point(140, 140), cv::Scalar(50), -1);

    // Compute Sobel gradients as short (same type as LsdCC uses)
    cv::Mat img_16s;
    img.convertTo(img_16s, CV_16S);
    cv::Sobel(img_16s, gx_, CV_16S, 1, 0, 3);
    cv::Sobel(img_16s, gy_, CV_16S, 0, 1, 3);

    // Also compute float gradients to test both code paths
    cv::Mat img_f;
    img.convertTo(img_f, CV_32F);
    cv::Sobel(img_f, gx_float_, CV_32F, 1, 0, 3);
    cv::Sobel(img_f, gy_float_, CV_32F, 0, 1, 3);

    // Lines along known edges (well inside the image)
    lines_.emplace_back(cv::Point2f(30, 50), cv::Point2f(170, 50));    // horizontal, top edge
    lines_.emplace_back(cv::Point2f(50, 30), cv::Point2f(50, 170));    // vertical, left edge
    lines_.emplace_back(cv::Point2f(60, 100), cv::Point2f(140, 100));  // horizontal, inner
    lines_.emplace_back(cv::Point2f(100, 60), cv::Point2f(100, 140));  // vertical, inner
  }

  cv::Mat gx_{};
  cv::Mat gy_{};
  cv::Mat gx_float_{};
  cv::Mat gy_float_{};
  std::vector<LineSegment<float>> lines_{};
};

TEST_F(LbdDescriptorTest, DescriptorHasCorrectSize) {
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  EXPECT_EQ(fdc.size(), static_cast<size_t>(9 * 8));  // default: 9 bands × 8 values

  FdcLBD<float, LineSegment<float>, short> fdc_custom(gx_, gy_, 7, 5);
  EXPECT_EQ(fdc_custom.size(), static_cast<size_t>(7 * 8));
}

TEST_F(LbdDescriptorTest, NoNanOrInfInDescriptor) {
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  EXPECT_EQ(descriptors.size(), lines_.size());
  for (size_t i = 0; i < descriptors.size(); ++i) {
    ASSERT_FALSE(descriptors[i].data.empty()) << "Descriptor " << i << " is empty";
    EXPECT_EQ(descriptors[i].data.cols, static_cast<int>(fdc.size()));
    const float* ptr = descriptors[i].data.ptr<float>();
    for (int j = 0; j < descriptors[i].data.cols; ++j) {
      EXPECT_FALSE(std::isnan(ptr[j])) << "NaN at descriptor " << i << " element " << j;
      EXPECT_FALSE(std::isinf(ptr[j])) << "Inf at descriptor " << i << " element " << j;
    }
  }
}

TEST_F(LbdDescriptorTest, DescriptorIsNormalized) {
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    double norm = cv::norm(descriptors[i].data, cv::NORM_L2);
    // After the full normalization pipeline (normalize + clip to 0.4 + re-normalize)
    // the L2 norm should be very close to 1.0
    EXPECT_NEAR(norm, 1.0, 1e-4) << "Descriptor " << i << " has L2 norm " << norm;
  }
}

TEST_F(LbdDescriptorTest, SelfDistanceIsZero) {
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    float dist = descriptors[i].distance(descriptors[i]);
    EXPECT_NEAR(dist, 0.0f, 1e-6f) << "Self-distance of descriptor " << i << " is " << dist;
  }
}

TEST_F(LbdDescriptorTest, DeterministicOutput) {
  // Run descriptor computation twice on the same data — results must be identical
  FdcLBD<float, LineSegment<float>, short> fdc1(gx_, gy_);
  FdcLBD<float, LineSegment<float>, short> fdc2(gx_, gy_);
  std::vector<FdLBD<float>> desc1;
  std::vector<FdLBD<float>> desc2;
  const auto& lines = lines_;
  fdc1.createList(lines, desc1);
  fdc2.createList(lines, desc2);

  ASSERT_EQ(desc1.size(), desc2.size());
  for (size_t i = 0; i < desc1.size(); ++i) {
    float dist = desc1[i].distance(desc2[i]);
    EXPECT_NEAR(dist, 0.0f, 1e-6f) << "Run-to-run deviation for descriptor " << i;
  }
}

TEST_F(LbdDescriptorTest, FloatGradientsWorkToo) {
  // The Python binding uses float32 gradients via RoundNearestInterpolator<float, float>
  FdcLBD<float, LineSegment<float>, float> fdc(gx_float_, gy_float_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    ASSERT_FALSE(descriptors[i].data.empty()) << "Descriptor " << i << " is empty";
    double norm = cv::norm(descriptors[i].data, cv::NORM_L2);
    EXPECT_NEAR(norm, 1.0, 1e-4) << "Float-gradient descriptor " << i << " has L2 norm " << norm;

    const float* ptr = descriptors[i].data.ptr<float>();
    for (int j = 0; j < descriptors[i].data.cols; ++j) {
      EXPECT_FALSE(std::isnan(ptr[j])) << "NaN at descriptor " << i << " element " << j;
      EXPECT_FALSE(std::isinf(ptr[j])) << "Inf at descriptor " << i << " element " << j;
    }
  }
}

TEST_F(LbdDescriptorTest, MatAndObjInterfacesAgree) {
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> obj_descriptors;
  cv::Mat mat_descriptors;
  const auto& lines = lines_;

  fdc.createList(lines, obj_descriptors);
  fdc.createMat(lines, mat_descriptors);

  ASSERT_EQ(static_cast<size_t>(mat_descriptors.rows), obj_descriptors.size());
  for (size_t i = 0; i < obj_descriptors.size(); ++i) {
    double diff = cv::norm(obj_descriptors[i].data, mat_descriptors.row(static_cast<int>(i)), cv::NORM_L2);
    EXPECT_NEAR(diff, 0.0, 1e-6) << "Mat vs Obj mismatch at descriptor " << i;
  }
}

TEST_F(LbdDescriptorTest, CustomBandParameters) {
  // Test with the numBand=7, widthBand=5 parameters used in match_test.cpp
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_, 7, 5);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    ASSERT_FALSE(descriptors[i].data.empty());
    EXPECT_EQ(descriptors[i].data.cols, 7 * 8);
    double norm = cv::norm(descriptors[i].data, cv::NORM_L2);
    EXPECT_NEAR(norm, 1.0, 1e-4) << "Custom-param descriptor " << i << " has L2 norm " << norm;
  }
}

TEST_F(LbdDescriptorTest, BruteForceMatcherSelfMatch) {
  // A set of descriptors matched against itself should produce near-zero
  // distance for the best match. Note: some synthetic lines may produce
  // near-identical descriptors, so we check distance ≈ 0, not strictly
  // that the index is the diagonal.
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  FmBruteForce<float, FdLBD<float>> matcher;
  matcher.train(descriptors, descriptors);
  std::vector<DescriptorMatch<float>> best;
  matcher.best(best);

  ASSERT_EQ(best.size(), descriptors.size());
  for (size_t i = 0; i < best.size(); ++i) {
    // Distance must be (near) zero since the identical descriptor exists
    EXPECT_NEAR(best[i].distance, 0.0f, 1e-5f) << "Descriptor " << i << " best-match distance is not zero";
  }
}

TEST_F(LbdDescriptorTest, ElementsBoundedAfterClipping) {
  // After normalization + clipping to 0.4 + re-normalization, no element
  // should exceed 0.4 / norm, which is bounded by 1.0 (since re-normalization
  // scales to unit norm). In practice elements should all be <= ~0.6.
  FdcLBD<float, LineSegment<float>, short> fdc(gx_, gy_);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    double min_val = 0;
    double max_val = 0;
    cv::minMaxLoc(descriptors[i].data, &min_val, &max_val);
    // After clipping at 0.4 and re-normalizing (dividing by norm <= sqrt(n)*0.4),
    // the maximum possible element value is bounded
    EXPECT_LE(max_val, 1.0) << "Descriptor " << i << " has element > 1.0";
    EXPECT_GE(min_val, 0.0) << "Descriptor " << i << " has negative element";
  }
}

TEST_F(LbdDescriptorTest, NearConstantGradientNoNaN) {
  // A near-constant gradient image has E[X^2] - E[X]^2 ~ 0.
  // Without a std::max(0, ...) guard the sqrt() argument can be slightly
  // negative due to floating-point rounding, producing NaN.
  cv::Mat const_gx(kImageSize, kImageSize, CV_16S, cv::Scalar(1));
  cv::Mat const_gy(kImageSize, kImageSize, CV_16S, cv::Scalar(0));

  FdcLBD<float, LineSegment<float>, short> fdc(const_gx, const_gy);
  std::vector<FdLBD<float>> descriptors;
  const auto& lines = lines_;
  fdc.createList(lines, descriptors);

  for (size_t i = 0; i < descriptors.size(); ++i) {
    const float* ptr = descriptors[i].data.ptr<float>(0);
    int n = descriptors[i].data.cols;
    for (int j = 0; j < n; ++j) {
      EXPECT_FALSE(std::isnan(ptr[j])) << "NaN at descriptor " << i << " element " << j;
      EXPECT_FALSE(std::isinf(ptr[j])) << "Inf at descriptor " << i << " element " << j;
    }
  }
}
