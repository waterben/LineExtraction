#include <gtest/gtest.h>
#include <utility/interpolate.hpp>
#include <opencv2/opencv.hpp>

using namespace lsfm;
using namespace cv;

class InterpolateTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test image
        test_img = Mat::zeros(5, 5, CV_32F);
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 5; ++x) {
                test_img.at<float>(y, x) = y * 5 + x;
            }
        }
        
        // Create a simple 2x2 gradient for interpolation tests
        gradient = Mat::zeros(2, 2, CV_32F);
        gradient.at<float>(0, 0) = 0.0f;
        gradient.at<float>(0, 1) = 10.0f;
        gradient.at<float>(1, 0) = 1.0f;
        gradient.at<float>(1, 1) = 11.0f;
        
        // Create a peak for sub-pixel maximum testing
        peak = Mat::zeros(3, 3, CV_32F);
        peak.at<float>(0, 0) = 1.0f; peak.at<float>(0, 1) = 2.0f; peak.at<float>(0, 2) = 1.0f;
        peak.at<float>(1, 0) = 2.0f; peak.at<float>(1, 1) = 5.0f; peak.at<float>(1, 2) = 2.0f;
        peak.at<float>(2, 0) = 1.0f; peak.at<float>(2, 1) = 2.0f; peak.at<float>(2, 2) = 1.0f;
        
        // Create test data array
        for (int i = 0; i < 10; ++i) {
            test_data[i] = i * 2.0f;
        }
    }
    
    Mat test_img, gradient, peak;
    float test_data[10];
};

TEST_F(InterpolateTest, ReadValXInBounds) {
    // Test reading values within bounds
    EXPECT_EQ(readValX(test_data, 10, 0), 0.0f);
    EXPECT_EQ(readValX(test_data, 10, 5), 10.0f);
    EXPECT_EQ(readValX(test_data, 10, 9), 18.0f);
}

TEST_F(InterpolateTest, ReadValXOutOfBounds) {
    // Test reading values out of bounds with border replicate
    EXPECT_EQ(readValX(test_data, 10, -1), 0.0f);  // Should clamp to first element
    EXPECT_EQ(readValX(test_data, 10, 10), 18.0f); // Should clamp to last element
    EXPECT_EQ(readValX(test_data, 10, 15), 18.0f); // Should clamp to last element
}

TEST_F(InterpolateTest, ReadValXWithBorderConstant) {
    float border_val = -1.0f;
    EXPECT_EQ(readValX(test_data, 10, -1, BORDER_CONSTANT, border_val), border_val);
    EXPECT_EQ(readValX(test_data, 10, 10, BORDER_CONSTANT, border_val), border_val);
}

TEST_F(InterpolateTest, ReadValMatInBounds) {
    // Test reading values within bounds from Mat
    EXPECT_EQ(readVal<float>(test_img, 0, 0), 0.0f);
    EXPECT_EQ(readVal<float>(test_img, 2, 3), 13.0f);  // 2*5 + 3
    EXPECT_EQ(readVal<float>(test_img, 4, 4), 24.0f);  // 4*5 + 4
}

TEST_F(InterpolateTest, ReadValMatOutOfBounds) {
    // Test reading values out of bounds with border replicate
    EXPECT_EQ(readVal<float>(test_img, -1, 0), 0.0f);   // Should clamp to (0,0)
    EXPECT_EQ(readVal<float>(test_img, 0, -1), 0.0f);   // Should clamp to (0,0)
    EXPECT_EQ(readVal<float>(test_img, 5, 4), 24.0f);   // Should clamp to (4,4)
    EXPECT_EQ(readVal<float>(test_img, 4, 5), 24.0f);   // Should clamp to (4,4)
}

TEST_F(InterpolateTest, ReadValMatWithBorderConstant) {
    float border_val = -99.0f;
    EXPECT_EQ(readVal<float>(test_img, -1, 0, BORDER_CONSTANT, border_val), border_val);
    EXPECT_EQ(readVal<float>(test_img, 0, -1, BORDER_CONSTANT, border_val), border_val);
    EXPECT_EQ(readVal<float>(test_img, 5, 4, BORDER_CONSTANT, border_val), border_val);
}

TEST_F(InterpolateTest, BilinearInterpolation) {
    // Test bilinear interpolation using LinearInterpolator
    typedef LinearInterpolator<float, float> LinearInterp;
    
    // Test exact pixel values
    EXPECT_FLOAT_EQ(LinearInterp::get(gradient, 0.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(LinearInterp::get(gradient, 1.0f, 1.0f), 11.0f);
    
    // Test interpolation between pixels
    EXPECT_NEAR(LinearInterp::get(gradient, 0.5f, 0.0f), 5.0f, 1e-6);  // Between 0 and 10
    EXPECT_NEAR(LinearInterp::get(gradient, 0.0f, 0.5f), 0.5f, 1e-6);  // Between 0 and 1
    EXPECT_NEAR(LinearInterp::get(gradient, 0.5f, 0.5f), 5.5f, 1e-6);  // Center interpolation
}

TEST_F(InterpolateTest, InterpolatorTypes) {
    // Test different interpolator types
    typedef NearestInterpolator<float, float> NearestInterp;
    typedef LinearInterpolator<float, float> LinearInterp;
    typedef CubicInterpolator<float, float> CubicInterp;
    
    // Test that all interpolators work
    EXPECT_NO_THROW(NearestInterp::get(peak, 1.0f, 1.0f));
    EXPECT_NO_THROW(LinearInterp::get(peak, 1.0f, 1.0f));
    EXPECT_NO_THROW(CubicInterp::get(peak, 1.0f, 1.0f));
    
    // Values should be reasonable
    float nearest_val = NearestInterp::get(peak, 1.0f, 1.0f);
    float linear_val = LinearInterp::get(peak, 1.0f, 1.0f);
    float cubic_val = CubicInterp::get(peak, 1.0f, 1.0f);
    
    EXPECT_FLOAT_EQ(nearest_val, 5.0f);  // Exact pixel value
    EXPECT_FLOAT_EQ(linear_val, 5.0f);   // Exact pixel value
    EXPECT_NEAR(cubic_val, 5.0f, 1.0f);  // Should be close
}

TEST_F(InterpolateTest, BorderHandling) {
    typedef LinearInterpolator<float, float> LinearInterp;
    
    // Test border handling with different border types
    EXPECT_NO_THROW(LinearInterp::get(gradient, -0.5f, -0.5f, cv::BORDER_CONSTANT, 0.0f));
    EXPECT_NO_THROW(LinearInterp::get(gradient, 2.5f, 2.5f, cv::BORDER_REPLICATE));
    
    // Test with fixed border interpolator - may throw for out of bounds access
    typedef FixedBorderInterpolator<LinearInterpolator<float, float>, cv::BORDER_CONSTANT, 42> FixedBorder;
    // Test that we can at least create and use the interpolator within bounds
    float result = 0.0f;
    EXPECT_NO_THROW(result = FixedBorder::get(gradient, 0.5f, 0.5f));
    EXPECT_GT(result, 0.0f);
}