#include <gtest/gtest.h>
#include <utility/range.hpp>

using namespace lsfm;

class RangeTest : public ::testing::Test {
protected:
    void SetUp() override {
        range_int = Range<int>(5, 15);
        range_float = Range<float>(2.5f, 7.5f);
        range_negative = Range<double>(-10.0, -3.0);
        range_reversed = Range<int>(20, 10);
    }
    
    Range<int> range_int;
    Range<float> range_float;
    Range<double> range_negative;
    Range<int> range_reversed;
};

TEST_F(RangeTest, Construction) {
    // Test default construction
    Range<int> default_range;
    EXPECT_EQ(default_range.lower, 0);
    EXPECT_EQ(default_range.upper, 0);
    
    // Test parameterized construction
    EXPECT_EQ(range_int.lower, 5);
    EXPECT_EQ(range_int.upper, 15);
    
    EXPECT_FLOAT_EQ(range_float.lower, 2.5f);
    EXPECT_FLOAT_EQ(range_float.upper, 7.5f);
}

TEST_F(RangeTest, Size) {
    // Test size calculation for positive range
    EXPECT_EQ(range_int.size(), 10);
    EXPECT_FLOAT_EQ(range_float.size(), 5.0f);
    
    // Test size for negative range
    EXPECT_DOUBLE_EQ(range_negative.size(), 7.0);
    
    // Test size for reversed range (should still be positive)
    EXPECT_EQ(range_reversed.size(), 10);
    
    // Test size for zero-width range
    Range<int> zero_range(5, 5);
    EXPECT_EQ(zero_range.size(), 0);
}

TEST_F(RangeTest, Swap) {
    Range<int> test_range(10, 30);
    EXPECT_EQ(test_range.lower, 10);
    EXPECT_EQ(test_range.upper, 30);
    
    test_range.swap();
    EXPECT_EQ(test_range.lower, 30);
    EXPECT_EQ(test_range.upper, 10);
    
    // Swap back
    test_range.swap();
    EXPECT_EQ(test_range.lower, 10);
    EXPECT_EQ(test_range.upper, 30);
}

TEST_F(RangeTest, NegativeRange) {
    // Test with negative values
    EXPECT_DOUBLE_EQ(range_negative.lower, -10.0);
    EXPECT_DOUBLE_EQ(range_negative.upper, -3.0);
    EXPECT_DOUBLE_EQ(range_negative.size(), 7.0);
}

TEST_F(RangeTest, FloatingPointPrecision) {
    Range<double> precise_range(1.123456789, 2.987654321);
    double expected_size = std::abs(2.987654321 - 1.123456789);
    EXPECT_DOUBLE_EQ(precise_range.size(), expected_size);
}

TEST_F(RangeTest, TypeConversions) {
    // Test with different numeric types
    Range<short> short_range(100, 200);
    EXPECT_EQ(short_range.size(), 100);
    
    Range<long> long_range(1000000L, 2000000L);
    EXPECT_EQ(long_range.size(), 1000000L);
    
    Range<unsigned int> uint_range(5u, 15u);
    EXPECT_EQ(uint_range.size(), 10u);
}

TEST_F(RangeTest, EdgeCases) {
    // Test with very small numbers
    Range<float> tiny_range(1e-6f, 2e-6f);
    EXPECT_NEAR(tiny_range.size(), 1e-6f, 1e-12f);
    
    // Test with very large numbers
    Range<double> large_range(1e12, 2e12);
    EXPECT_NEAR(large_range.size(), 1e12, 1e6);
    
    // Test crossing zero
    Range<int> crossing_zero(-5, 5);
    EXPECT_EQ(crossing_zero.size(), 10);
}