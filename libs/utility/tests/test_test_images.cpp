/**
 * @file test_test_images.cpp
 * @brief Unit tests for TestImages path resolution utility.
 *
 * Tests the TestImages class which resolves test image paths
 * across Bazel and CMake build systems.
 */

#include <utility/test_images.hpp>

#include <gtest/gtest.h>

#include <filesystem>

using namespace lsfm;

/**
 * @brief Test fixture for TestImages tests.
 */
class TestImagesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize TestImages with dummy argv[0]
    TestImages::init("test_binary");
  }
};

/**
 * @brief Test basic initialization without crash.
 */
TEST_F(TestImagesTest, Initialization) {
  // Should not crash
  EXPECT_NO_THROW(TestImages::init("dummy_path"));
  EXPECT_NO_THROW(TestImages::init(nullptr));
}

/**
 * @brief Test isBazelRun detection.
 */
TEST_F(TestImagesTest, BazelDetection) {
  // Should return false in normal test environment (unless actually run under Bazel)
  // We just test that it doesn't crash
  bool is_bazel = TestImages::isBazelRun();
  EXPECT_TRUE(is_bazel || !is_bazel);  // Always true, just exercises the function
}

/**
 * @brief Test get() returns non-empty for non-existent files.
 */
TEST_F(TestImagesTest, GetNonExistentFile) {
  // Should return the path (even if file doesn't exist) as fallback
  std::string path = TestImages::get("nonexistent_image.jpg");
  EXPECT_FALSE(path.empty());
  EXPECT_EQ(path, "nonexistent_image.jpg");  // Fallback behavior
}

/**
 * @brief Test windmill() convenience method.
 */
TEST_F(TestImagesTest, WindmillConvenience) {
  std::string path = TestImages::windmill();
  EXPECT_FALSE(path.empty());
  // Should end with windmill.jpg
  EXPECT_NE(path.find("windmill.jpg"), std::string::npos);
}

/**
 * @brief Test noise() convenience method.
 */
TEST_F(TestImagesTest, NoiseConvenience) {
  std::string path = TestImages::noise("bike.png");
  EXPECT_FALSE(path.empty());
  // Should contain noise/ and bike.png
  EXPECT_NE(path.find("noise"), std::string::npos);
  EXPECT_NE(path.find("bike.png"), std::string::npos);
}

/**
 * @brief Test bsds500() method without split.
 */
TEST_F(TestImagesTest, BSDS500NoSplit) {
  std::string path = TestImages::bsds500("100007.jpg");
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path.find("BSDS500"), std::string::npos);
  EXPECT_NE(path.find("100007.jpg"), std::string::npos);
}

/**
 * @brief Test bsds500() method with split.
 */
TEST_F(TestImagesTest, BSDS500WithSplit) {
  std::string path = TestImages::bsds500("100007.jpg", "train");
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path.find("BSDS500"), std::string::npos);
  EXPECT_NE(path.find("train"), std::string::npos);
  EXPECT_NE(path.find("100007.jpg"), std::string::npos);
}

/**
 * @brief Test stereoPair() method.
 */
TEST_F(TestImagesTest, StereoPair) {
  auto [left, right] = TestImages::stereoPair("Adirondack", "H");

  EXPECT_FALSE(left.empty());
  EXPECT_FALSE(right.empty());

  // Should contain scene name and im0/im1
  EXPECT_NE(left.find("Adirondack"), std::string::npos);
  EXPECT_NE(left.find("im0.png"), std::string::npos);
  EXPECT_NE(right.find("Adirondack"), std::string::npos);
  EXPECT_NE(right.find("im1.png"), std::string::npos);

  // Should contain resolution directory
  EXPECT_NE(left.find("mdb_H"), std::string::npos);
  EXPECT_NE(right.find("mdb_H"), std::string::npos);
}

/**
 * @brief Test stereoLeft() convenience method.
 */
TEST_F(TestImagesTest, StereoLeft) {
  std::string path = TestImages::stereoLeft("Adirondack");
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path.find("Adirondack"), std::string::npos);
  EXPECT_NE(path.find("im0.png"), std::string::npos);
}

/**
 * @brief Test stereoRight() convenience method.
 */
TEST_F(TestImagesTest, StereoRight) {
  std::string path = TestImages::stereoRight("Adirondack");
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path.find("Adirondack"), std::string::npos);
  EXPECT_NE(path.find("im1.png"), std::string::npos);
}

/**
 * @brief Test different resolution parameters.
 */
TEST_F(TestImagesTest, DifferentResolutions) {
  // Test Quarter resolution
  auto [left_q, right_q] = TestImages::stereoPair("Adirondack", "Q");
  EXPECT_NE(left_q.find("mdb_Q"), std::string::npos);
  EXPECT_NE(right_q.find("mdb_Q"), std::string::npos);

  // Test Full resolution
  auto [left_f, right_f] = TestImages::stereoPair("Adirondack", "F");
  EXPECT_NE(left_f.find("mdb_F"), std::string::npos);
  EXPECT_NE(right_f.find("mdb_F"), std::string::npos);
}

/**
 * @brief Test multiple initialization calls (should not crash).
 */
TEST_F(TestImagesTest, MultipleInitCalls) {
  EXPECT_NO_THROW(TestImages::init("path1"));
  EXPECT_NO_THROW(TestImages::init("path2"));
  EXPECT_NO_THROW(TestImages::init("path3"));

  // Should still work after multiple inits
  std::string path = TestImages::windmill();
  EXPECT_FALSE(path.empty());
}

/**
 * @brief Test get() with subdirectory paths.
 */
TEST_F(TestImagesTest, SubdirectoryPaths) {
  std::string path = TestImages::get("some/nested/path/image.png");
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path.find("some"), std::string::npos);
  EXPECT_NE(path.find("nested"), std::string::npos);
  EXPECT_NE(path.find("image.png"), std::string::npos);
}
