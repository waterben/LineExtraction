#include <gtest/gtest.h>
#include <utility/limit.hpp>
#include <cmath>

using namespace lsfm;

class LimitTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(LimitTest, DoubleLimitsDefaultTemplate) {
    // Test default template specialization for double
    EXPECT_EQ(LIMITS<double>::tau(), 1.0E-7);
    EXPECT_EQ(LIMITS<double>::min(), std::numeric_limits<double>::min());
    EXPECT_EQ(LIMITS<double>::eps(), std::numeric_limits<double>::epsilon());
}

TEST_F(LimitTest, FloatLimitsSpecialization) {
    // Test explicit float specialization
    EXPECT_EQ(LIMITS<float>::tau(), std::numeric_limits<float>::epsilon());
    EXPECT_EQ(LIMITS<float>::min(), std::numeric_limits<float>::min());
    EXPECT_EQ(LIMITS<float>::eps(), std::numeric_limits<float>::epsilon());
}

TEST_F(LimitTest, IntLimitsTemplate) {
    // For integer types, tau is typically 0
    EXPECT_EQ(LIMITS<int>::tau(), 0);
    EXPECT_EQ(LIMITS<int>::min(), std::numeric_limits<int>::min());
    EXPECT_EQ(LIMITS<int>::eps(), 0);
}

TEST_F(LimitTest, LongDoubleLimitsTemplate) {
    // Test template with long double
    EXPECT_EQ(LIMITS<long double>::tau(), static_cast<long double>(1.0E-7));
    EXPECT_EQ(LIMITS<long double>::min(), std::numeric_limits<long double>::min());
    EXPECT_EQ(LIMITS<long double>::eps(), std::numeric_limits<long double>::epsilon());
}

TEST_F(LimitTest, ValuesAreReasonable) {
    // Sanity checks that the values make sense
    EXPECT_GT(LIMITS<double>::tau(), 0.0);
    EXPECT_GT(LIMITS<float>::tau(), 0.0f);
    
    EXPECT_GT(LIMITS<double>::eps(), 0.0);
    EXPECT_GT(LIMITS<float>::eps(), 0.0f);
    
    // min() for floating point types should be very small positive
    EXPECT_GT(LIMITS<double>::min(), 0.0);
    EXPECT_GT(LIMITS<float>::min(), 0.0f);
}

TEST_F(LimitTest, FloatVsDefaultDifference) {
    // Float specialization should have different tau than default template
    EXPECT_NE(LIMITS<float>::tau(), static_cast<float>(1.0E-7));
    EXPECT_EQ(LIMITS<float>::tau(), std::numeric_limits<float>::epsilon());
}