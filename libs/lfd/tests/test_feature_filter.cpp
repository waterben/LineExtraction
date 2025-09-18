#include <gtest/gtest.h>
#include <lfd/FeatureFilter.hpp>
#include <vector>
#include <algorithm>

using namespace lsfm;

class FeatureFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test feature matches
        feature_matches.clear();
        feature_matches.emplace_back(0, 0, FS_NONE);
        feature_matches.emplace_back(1, 1, FS_MASKED);
        feature_matches.emplace_back(2, 2, FS_NONE);
        
        // Create test descriptor matches
        descriptor_matches.clear();
        descriptor_matches.emplace_back(0, 0, FS_NONE, 0.5f);
        descriptor_matches.emplace_back(1, 1, FS_NONE, 0.3f);
        descriptor_matches.emplace_back(2, 2, FS_MASKED, 0.8f);
        descriptor_matches.emplace_back(3, 3, FS_NONE, 0.1f);
        descriptor_matches.emplace_back(4, 4, FS_NONE, 0.9f);
    }
    
    std::vector<FeatureMatch<float>> feature_matches;
    std::vector<DescriptorMatch<float>> descriptor_matches;
};

TEST_F(FeatureFilterTest, FeatureMatchConstruction) {
    FeatureMatch<float> match1;
    EXPECT_EQ(match1.queryIdx, -1);
    EXPECT_EQ(match1.matchIdx, -1);
    EXPECT_EQ(match1.filterState, FS_NONE);
    
    FeatureMatch<float> match2(5, 3, FS_MASKED);
    EXPECT_EQ(match2.queryIdx, 5);
    EXPECT_EQ(match2.matchIdx, 3);
    EXPECT_EQ(match2.filterState, FS_MASKED);
}

TEST_F(FeatureFilterTest, DescriptorMatchConstruction) {
    DescriptorMatch<float> match1;
    EXPECT_EQ(match1.queryIdx, -1);
    EXPECT_EQ(match1.matchIdx, -1);
    EXPECT_EQ(match1.filterState, FS_NONE);
    EXPECT_EQ(match1.distance, std::numeric_limits<float>::max());
    
    DescriptorMatch<float> match2(2, 4, 0.7f);
    EXPECT_EQ(match2.queryIdx, 2);
    EXPECT_EQ(match2.matchIdx, 4);
    EXPECT_EQ(match2.filterState, FS_NONE);
    EXPECT_FLOAT_EQ(match2.distance, 0.7f);
    
    DescriptorMatch<float> match3(1, 3, FS_MASKED, 0.2f);
    EXPECT_EQ(match3.queryIdx, 1);
    EXPECT_EQ(match3.matchIdx, 3);
    EXPECT_EQ(match3.filterState, FS_MASKED);
    EXPECT_FLOAT_EQ(match3.distance, 0.2f);
}

TEST_F(FeatureFilterTest, DescriptorMatchComparison) {
    DescriptorMatch<float> match1(0, 0, 0.5f);
    DescriptorMatch<float> match2(1, 1, 0.3f);
    DescriptorMatch<float> match3(2, 2, 0.8f);
    
    // Lower distance should be "less than"
    EXPECT_TRUE(match2 < match1);
    EXPECT_TRUE(match1 < match3);
    EXPECT_FALSE(match1 < match2);
    EXPECT_FALSE(match3 < match1);
}

TEST_F(FeatureFilterTest, DescriptorMatchSorting) {
    auto sorted_matches = descriptor_matches;
    std::sort(sorted_matches.begin(), sorted_matches.end());
    
    // Should be sorted by distance (ascending)
    EXPECT_FLOAT_EQ(sorted_matches[0].distance, 0.1f); // queryIdx 3
    EXPECT_FLOAT_EQ(sorted_matches[1].distance, 0.3f); // queryIdx 1
    EXPECT_FLOAT_EQ(sorted_matches[2].distance, 0.5f); // queryIdx 0
    EXPECT_FLOAT_EQ(sorted_matches[3].distance, 0.8f); // queryIdx 2
    EXPECT_FLOAT_EQ(sorted_matches[4].distance, 0.9f); // queryIdx 4
    
    EXPECT_EQ(sorted_matches[0].queryIdx, 3);
    EXPECT_EQ(sorted_matches[1].queryIdx, 1);
    EXPECT_EQ(sorted_matches[2].queryIdx, 0);
    EXPECT_EQ(sorted_matches[3].queryIdx, 2);
    EXPECT_EQ(sorted_matches[4].queryIdx, 4);
}

TEST_F(FeatureFilterTest, FilterStateEnum) {
    // Test filter state enumeration values
    EXPECT_EQ(FS_NONE, 0);
    EXPECT_EQ(FS_MASKED, 1); // Should be different from FS_NONE
    
    // Test filter state usage
    FeatureMatch<float> none_match(0, 0, FS_NONE);
    FeatureMatch<float> masked_match(1, 1, FS_MASKED);
    
    EXPECT_EQ(none_match.filterState, FS_NONE);
    EXPECT_EQ(masked_match.filterState, FS_MASKED);
    EXPECT_NE(none_match.filterState, masked_match.filterState);
}

TEST_F(FeatureFilterTest, DescriptorMatchEquality) {
    DescriptorMatch<float> match1(0, 0, 0.5f);
    DescriptorMatch<float> match2(0, 0, 0.5f);
    DescriptorMatch<float> match3(1, 1, 0.5f);
    DescriptorMatch<float> match4(0, 0, 0.6f);
    
    // Same distance - neither is less than the other
    EXPECT_FALSE(match1 < match2);
    EXPECT_FALSE(match2 < match1);
    
    // Different indices but same distance
    EXPECT_FALSE(match1 < match3);
    EXPECT_FALSE(match3 < match1);
    
    // Different distances
    EXPECT_TRUE(match1 < match4);
    EXPECT_FALSE(match4 < match1);
}

TEST_F(FeatureFilterTest, MaskedFilterCounting) {
    int masked_count = 0;
    int none_count = 0;
    
    for (const auto& match : feature_matches) {
        if (match.filterState == FS_MASKED) {
            masked_count++;
        } else if (match.filterState == FS_NONE) {
            none_count++;
        }
    }
    
    EXPECT_EQ(masked_count, 1);
    EXPECT_EQ(none_count, 2);
}

TEST_F(FeatureFilterTest, DescriptorMatchMaskedFiltering) {
    std::vector<DescriptorMatch<float>> non_masked;
    
    for (const auto& match : descriptor_matches) {
        if (match.filterState != FS_MASKED) {
            non_masked.push_back(match);
        }
    }
    
    EXPECT_EQ(non_masked.size(), 4); // All except the one with FS_MASKED
    
    // Check that the masked one is not included
    for (const auto& match : non_masked) {
        EXPECT_NE(match.filterState, FS_MASKED);
    }
}

TEST_F(FeatureFilterTest, DescriptorMatchBestMatch) {
    // Find best match (lowest distance) that's not masked
    DescriptorMatch<float> best_match(-1, -1, std::numeric_limits<float>::max());
    
    for (const auto& match : descriptor_matches) {
        if (match.filterState != FS_MASKED && match < best_match) {
            best_match = match;
        }
    }
    
    EXPECT_EQ(best_match.queryIdx, 3);
    EXPECT_EQ(best_match.matchIdx, 3);
    EXPECT_FLOAT_EQ(best_match.distance, 0.1f);
    EXPECT_EQ(best_match.filterState, FS_NONE);
}

TEST_F(FeatureFilterTest, TypeTemplating) {
    // Test with double precision
    DescriptorMatch<double> double_match(0, 0, 0.123456789);
    EXPECT_DOUBLE_EQ(double_match.distance, 0.123456789);
    
    FeatureMatch<double> double_feature(5, 10, FS_MASKED);
    EXPECT_EQ(double_feature.queryIdx, 5);
    EXPECT_EQ(double_feature.matchIdx, 10);
    EXPECT_EQ(double_feature.filterState, FS_MASKED);
}

TEST_F(FeatureFilterTest, DistanceComparisonEdgeCases) {
    DescriptorMatch<float> zero_distance(0, 0, 0.0f);
    DescriptorMatch<float> small_distance(1, 1, 1e-7f);
    DescriptorMatch<float> max_distance(2, 2, std::numeric_limits<float>::max());
    
    EXPECT_TRUE(zero_distance < small_distance);
    EXPECT_TRUE(small_distance < max_distance);
    EXPECT_TRUE(zero_distance < max_distance);
}

TEST_F(FeatureFilterTest, InvalidMatchIndices) {
    FeatureMatch<float> invalid_match(-1, -1, FS_NONE);
    DescriptorMatch<float> invalid_desc_match(-1, -1, FS_NONE, 0.5f);
    
    EXPECT_EQ(invalid_match.queryIdx, -1);
    EXPECT_EQ(invalid_match.matchIdx, -1);
    EXPECT_EQ(invalid_desc_match.queryIdx, -1);
    EXPECT_EQ(invalid_desc_match.matchIdx, -1);
    EXPECT_FLOAT_EQ(invalid_desc_match.distance, 0.5f);
}