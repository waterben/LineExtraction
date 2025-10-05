#include <geometry/line.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <lfd/FeatureFilter.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

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
