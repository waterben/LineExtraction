//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************

#include <algorithm/detector_profile.hpp>
#include <algorithm/image_analyzer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using namespace lsfm;

// =============================================================================
// ImageAnalyzer Tests
// =============================================================================

TEST(ImageAnalyzerTest, AnalyzeUniformImage) {
  // Uniform gray image → low contrast, low noise, low edge density
  cv::Mat uniform(100, 100, CV_8UC1, cv::Scalar(128));
  auto props = ImageAnalyzer::analyze(uniform);

  EXPECT_NEAR(props.contrast, 0.0, 0.01);
  EXPECT_NEAR(props.noise_level, 0.0, 0.01);
  EXPECT_NEAR(props.edge_density, 0.0, 0.01);
  EXPECT_NEAR(props.dynamic_range, 0.0, 0.02);
}

TEST(ImageAnalyzerTest, AnalyzeHighContrastImage) {
  // Checkerboard pattern → high contrast, high edge density
  cv::Mat checker(100, 100, CV_8UC1);
  for (int r = 0; r < 100; ++r) {
    for (int c = 0; c < 100; ++c) {
      checker.at<uchar>(r, c) = ((r / 10 + c / 10) % 2 == 0) ? 0 : 255;
    }
  }
  auto props = ImageAnalyzer::analyze(checker);

  EXPECT_GT(props.contrast, 0.5);
  EXPECT_GT(props.edge_density, 0.1);
  EXPECT_GT(props.dynamic_range, 0.7);
}

TEST(ImageAnalyzerTest, AnalyzeNoisyImage) {
  cv::Mat noisy(200, 200, CV_8UC1);
  cv::randn(noisy, cv::Scalar(128), cv::Scalar(50));
  auto props = ImageAnalyzer::analyze(noisy);

  // Noisy image should have noticeable noise level
  EXPECT_GT(props.noise_level, 0.1);
}

TEST(ImageAnalyzerTest, ThrowsOnEmptyImage) {
  cv::Mat empty;
  EXPECT_THROW(ImageAnalyzer::analyze(empty), std::invalid_argument);
}

TEST(ImageAnalyzerTest, AcceptsColorImage) {
  // Should auto-convert from BGR to grayscale
  cv::Mat color(100, 100, CV_8UC3, cv::Scalar(100, 150, 200));
  auto props = ImageAnalyzer::analyze(color);

  EXPECT_GE(props.contrast, 0.0);
  EXPECT_LE(props.contrast, 1.0);
}

TEST(ImageAnalyzerTest, PropertiesInRange) {
  // Gradient image: left=0, right=255
  cv::Mat gradient(100, 256, CV_8UC1);
  for (int r = 0; r < 100; ++r) {
    for (int c = 0; c < 256; ++c) {
      gradient.at<uchar>(r, c) = static_cast<uchar>(c);
    }
  }
  auto props = ImageAnalyzer::analyze(gradient);

  EXPECT_GE(props.contrast, 0.0);
  EXPECT_LE(props.contrast, 1.0);
  EXPECT_GE(props.noise_level, 0.0);
  EXPECT_LE(props.noise_level, 1.0);
  EXPECT_GE(props.edge_density, 0.0);
  EXPECT_LE(props.edge_density, 1.0);
  EXPECT_GE(props.dynamic_range, 0.0);
  EXPECT_LE(props.dynamic_range, 1.0);
}

// =============================================================================
// ProfileHints Tests
// =============================================================================

TEST(ProfileHintsTest, SuggestProfileFromUniform) {
  ImageProperties props{};
  props.contrast = 0.0;
  props.noise_level = 0.0;
  props.edge_density = 0.0;
  props.dynamic_range = 0.0;

  auto hints = props.suggest_profile();

  // All knobs should be in [0, 100]
  EXPECT_GE(hints.detail, 0.0);
  EXPECT_LE(hints.detail, 100.0);
  EXPECT_GE(hints.gap_tolerance, 0.0);
  EXPECT_LE(hints.gap_tolerance, 100.0);
  EXPECT_GE(hints.min_length, 0.0);
  EXPECT_LE(hints.min_length, 100.0);
  EXPECT_GE(hints.precision, 0.0);
  EXPECT_LE(hints.precision, 100.0);

  // Factors in [0.5, 2.0]
  EXPECT_GE(hints.contrast_factor, 0.5);
  EXPECT_LE(hints.contrast_factor, 2.0);
  EXPECT_GE(hints.noise_factor, 0.5);
  EXPECT_LE(hints.noise_factor, 2.0);
}

TEST(ProfileHintsTest, HighNoiseRaisesFactors) {
  ImageProperties props{};
  props.contrast = 0.3;
  props.noise_level = 0.8;
  props.edge_density = 0.2;
  props.dynamic_range = 0.5;

  auto hints = props.suggest_profile();

  // Noisy image → noise_factor should be elevated
  EXPECT_GT(hints.noise_factor, 1.0);
  // Noisy image → gap tolerance should be elevated
  EXPECT_GT(hints.gap_tolerance, 50.0);
}

TEST(ProfileHintsTest, ClampWorks) {
  ProfileHints hints;
  hints.detail = 150;
  hints.gap_tolerance = -10;
  hints.contrast_factor = 5.0;
  hints.noise_factor = 0.1;

  hints.clamp();

  EXPECT_DOUBLE_EQ(hints.detail, 100.0);
  EXPECT_DOUBLE_EQ(hints.gap_tolerance, 0.0);
  EXPECT_DOUBLE_EQ(hints.contrast_factor, 2.0);
  EXPECT_DOUBLE_EQ(hints.noise_factor, 0.5);
}

// =============================================================================
// DetectorProfile Construction Tests
// =============================================================================

TEST(DetectorProfileTest, DefaultConstruction) {
  DetectorProfile profile;

  EXPECT_DOUBLE_EQ(profile.detail(), 50.0);
  EXPECT_DOUBLE_EQ(profile.gap_tolerance(), 50.0);
  EXPECT_DOUBLE_EQ(profile.min_length(), 50.0);
  EXPECT_DOUBLE_EQ(profile.precision(), 50.0);
  EXPECT_DOUBLE_EQ(profile.contrast_factor(), 1.0);
  EXPECT_DOUBLE_EQ(profile.noise_factor(), 1.0);
}

TEST(DetectorProfileTest, ExplicitConstruction) {
  DetectorProfile profile(80, 30, 60, 90);

  EXPECT_DOUBLE_EQ(profile.detail(), 80.0);
  EXPECT_DOUBLE_EQ(profile.gap_tolerance(), 30.0);
  EXPECT_DOUBLE_EQ(profile.min_length(), 60.0);
  EXPECT_DOUBLE_EQ(profile.precision(), 90.0);
}

TEST(DetectorProfileTest, ClampedConstruction) {
  DetectorProfile profile(150, -20, 200, -50);

  EXPECT_DOUBLE_EQ(profile.detail(), 100.0);
  EXPECT_DOUBLE_EQ(profile.gap_tolerance(), 0.0);
  EXPECT_DOUBLE_EQ(profile.min_length(), 100.0);
  EXPECT_DOUBLE_EQ(profile.precision(), 0.0);
}

TEST(DetectorProfileTest, FromHints) {
  ProfileHints hints;
  hints.detail = 70;
  hints.gap_tolerance = 40;
  hints.min_length = 55;
  hints.precision = 85;
  hints.contrast_factor = 1.3;
  hints.noise_factor = 1.5;

  auto profile = DetectorProfile::from_hints(hints);

  EXPECT_DOUBLE_EQ(profile.detail(), 70.0);
  EXPECT_DOUBLE_EQ(profile.gap_tolerance(), 40.0);
  EXPECT_DOUBLE_EQ(profile.min_length(), 55.0);
  EXPECT_DOUBLE_EQ(profile.precision(), 85.0);
  EXPECT_DOUBLE_EQ(profile.contrast_factor(), 1.3);
  EXPECT_DOUBLE_EQ(profile.noise_factor(), 1.5);
}

TEST(DetectorProfileTest, FromImage) {
  cv::Mat img(200, 200, CV_8UC1);
  cv::randn(img, cv::Scalar(128), cv::Scalar(30));

  auto profile = DetectorProfile::from_image(img);

  // Just verify it doesn't crash and produces valid values
  EXPECT_GE(profile.detail(), 0.0);
  EXPECT_LE(profile.detail(), 100.0);
  EXPECT_GE(profile.gap_tolerance(), 0.0);
  EXPECT_LE(profile.gap_tolerance(), 100.0);
}

TEST(DetectorProfileTest, SettersClamp) {
  DetectorProfile profile;

  profile.set_detail(120);
  EXPECT_DOUBLE_EQ(profile.detail(), 100.0);

  profile.set_detail(-5);
  EXPECT_DOUBLE_EQ(profile.detail(), 0.0);

  profile.set_contrast_factor(3.0);
  EXPECT_DOUBLE_EQ(profile.contrast_factor(), 2.0);

  profile.set_noise_factor(0.1);
  EXPECT_DOUBLE_EQ(profile.noise_factor(), 0.5);
}

// =============================================================================
// DetectorProfile Name Resolution Tests
// =============================================================================

TEST(DetectorProfileTest, DetectorIdToName) {
  EXPECT_EQ(detector_id_to_name(DetectorId::LSD_CC), "LsdCC");
  EXPECT_EQ(detector_id_to_name(DetectorId::LSD_FGIOI), "LsdFGioi");
  EXPECT_EQ(detector_id_to_name(DetectorId::LSD_HOUGHP), "LsdHoughP");
}

TEST(DetectorProfileTest, DetectorNameToId) {
  EXPECT_EQ(detector_name_to_id("LsdCC"), DetectorId::LSD_CC);
  EXPECT_EQ(detector_name_to_id("lsdcc"), DetectorId::LSD_CC);
  EXPECT_EQ(detector_name_to_id("LSDCC"), DetectorId::LSD_CC);
  EXPECT_EQ(detector_name_to_id("LsdFGioi"), DetectorId::LSD_FGIOI);
}

TEST(DetectorProfileTest, InvalidNameThrows) {
  EXPECT_THROW(detector_name_to_id("UnknownDetector"), std::invalid_argument);
}

TEST(DetectorProfileTest, SupportedDetectors) {
  auto names = DetectorProfile::supported_detectors();
  EXPECT_EQ(names.size(), static_cast<size_t>(DetectorId::COUNT));
  EXPECT_EQ(names[0], "LsdCC");
  EXPECT_EQ(names.back(), "LsdHoughP");
}

// =============================================================================
// Parameter Generation Tests
// =============================================================================

TEST(DetectorProfileTest, ToParamsLsdCC) {
  DetectorProfile profile(50, 50, 50, 50);
  auto params = profile.to_params(DetectorId::LSD_CC);

  // Should have: grad_kernel_size, nms_th_low, nms_th_high,
  //              edge_min_pixels, edge_max_gap, split_error_distance
  EXPECT_EQ(params.size(), 6u);

  // Verify all params have non-empty names
  for (const auto& p : params) {
    EXPECT_FALSE(p.name.empty());
  }
}

TEST(DetectorProfileTest, ToParamsLsdFGioi) {
  DetectorProfile profile(50, 50, 50, 50);
  auto params = profile.to_params(DetectorId::LSD_FGIOI);

  // FGioi: quant_error, angle_th, density_th, log_eps, bins
  EXPECT_EQ(params.size(), 5u);
}

TEST(DetectorProfileTest, ToParamsByName) {
  DetectorProfile profile(50, 50, 50, 50);
  auto params_by_id = profile.to_params(DetectorId::LSD_CC);
  auto params_by_name = profile.to_params("LsdCC");

  EXPECT_EQ(params_by_id.size(), params_by_name.size());
  for (size_t i = 0; i < params_by_id.size(); ++i) {
    EXPECT_EQ(params_by_id[i].name, params_by_name[i].name);
  }
}

TEST(DetectorProfileTest, AllDetectorsProduceParams) {
  DetectorProfile profile(50, 50, 50, 50);

  for (int i = 0; i < static_cast<int>(DetectorId::COUNT); ++i) {
    auto id = static_cast<DetectorId>(i);
    auto params = profile.to_params(id);
    EXPECT_GT(params.size(), 0u) << "Detector " << detector_id_to_name(id) << " produced no params";
  }
}

// =============================================================================
// Parameter Mapping Monotonicity Tests
// =============================================================================

/// @brief Helper to find a parameter value by name.
static double find_param(const ParamConfig& params, const std::string& name) {
  for (const auto& p : params) {
    if (p.name == name) {
      return p.value.getDouble();
    }
  }
  return -1.0;
}

static int find_param_int(const ParamConfig& params, const std::string& name) {
  for (const auto& p : params) {
    if (p.name == name) {
      return static_cast<int>(p.value.getInt());
    }
  }
  return -1;
}

TEST(DetectorProfileTest, DetailIncreasesLowersNmsThreshold) {
  // More detail → lower NMS thresholds
  DetectorProfile low_detail(10, 50, 50, 50);
  DetectorProfile high_detail(90, 50, 50, 50);

  auto lo = low_detail.to_params(DetectorId::LSD_CC);
  auto hi = high_detail.to_params(DetectorId::LSD_CC);

  EXPECT_GT(find_param(lo, "nms_th_low"), find_param(hi, "nms_th_low"));
  EXPECT_GT(find_param(lo, "nms_th_high"), find_param(hi, "nms_th_high"));
}

TEST(DetectorProfileTest, GapToleranceIncreasesMaxGap) {
  DetectorProfile low_gap(50, 10, 50, 50);
  DetectorProfile high_gap(50, 90, 50, 50);

  auto lo = low_gap.to_params(DetectorId::LSD_CC);
  auto hi = high_gap.to_params(DetectorId::LSD_CC);

  EXPECT_LT(find_param_int(lo, "edge_max_gap"), find_param_int(hi, "edge_max_gap"));
}

TEST(DetectorProfileTest, MinLengthIncreasesMinPixels) {
  DetectorProfile short_len(50, 50, 10, 50);
  DetectorProfile long_len(50, 50, 90, 50);

  auto lo = short_len.to_params(DetectorId::LSD_CC);
  auto hi = long_len.to_params(DetectorId::LSD_CC);

  EXPECT_LT(find_param_int(lo, "edge_min_pixels"), find_param_int(hi, "edge_min_pixels"));
}

TEST(DetectorProfileTest, PrecisionDecreasesError) {
  DetectorProfile low_prec(50, 50, 50, 10);
  DetectorProfile high_prec(50, 50, 50, 90);

  auto lo = low_prec.to_params(DetectorId::LSD_CC);
  auto hi = high_prec.to_params(DetectorId::LSD_CC);

  EXPECT_GT(find_param(lo, "split_error_distance"), find_param(hi, "split_error_distance"));
}

// =============================================================================
// Adaptive Factor Tests
// =============================================================================

TEST(DetectorProfileTest, NoiseFactorIncreasesThresholds) {
  DetectorProfile base(50, 50, 50, 50);
  DetectorProfile noisy(50, 50, 50, 50);
  noisy.set_noise_factor(1.8);

  auto base_params = base.to_params(DetectorId::LSD_CC);
  auto noisy_params = noisy.to_params(DetectorId::LSD_CC);

  // Noisy → higher NMS thresholds
  EXPECT_LT(find_param(base_params, "nms_th_low"), find_param(noisy_params, "nms_th_low"));
}

TEST(DetectorProfileTest, ContrastFactorAffectsParams) {
  DetectorProfile base(50, 50, 50, 50);
  DetectorProfile low_contrast(50, 50, 50, 50);
  low_contrast.set_contrast_factor(1.8);

  auto base_params = base.to_params(DetectorId::LSD_EDLZ);
  auto lc_params = low_contrast.to_params(DetectorId::LSD_EDLZ);

  // Low contrast factor → higher grad_th
  EXPECT_LT(find_param(base_params, "grad_th"), find_param(lc_params, "grad_th"));
}

// =============================================================================
// Extreme Knob Value Tests
// =============================================================================

TEST(DetectorProfileTest, ExtremeValuesAllDetectors) {
  // 0% everything
  DetectorProfile min_profile(0, 0, 0, 0);
  // 100% everything
  DetectorProfile max_profile(100, 100, 100, 100);

  for (int i = 0; i < static_cast<int>(DetectorId::COUNT); ++i) {
    auto id = static_cast<DetectorId>(i);

    auto min_params = min_profile.to_params(id);
    auto max_params = max_profile.to_params(id);

    // Both should produce valid non-empty param sets
    EXPECT_GT(min_params.size(), 0u) << "min profile empty for " << detector_id_to_name(id);
    EXPECT_GT(max_params.size(), 0u) << "max profile empty for " << detector_id_to_name(id);

    // All values should be finite
    for (const auto& p : min_params) {
      if (p.value.type() == Value::FLOAT) {
        EXPECT_TRUE(std::isfinite(p.value.getDouble()))
            << "Non-finite param " << p.name << " in " << detector_id_to_name(id);
      }
    }
    for (const auto& p : max_params) {
      if (p.value.type() == Value::FLOAT) {
        EXPECT_TRUE(std::isfinite(p.value.getDouble()))
            << "Non-finite param " << p.name << " in " << detector_id_to_name(id);
      }
    }
  }
}

// =============================================================================
// Integration: ImageAnalyzer → DetectorProfile
// =============================================================================

TEST(IntegrationTest, AnalyzeToProfile) {
  // Create a synthetic structured image
  cv::Mat img(200, 300, CV_8UC1, cv::Scalar(100));
  cv::line(img, cv::Point(10, 50), cv::Point(290, 50), cv::Scalar(200), 2);
  cv::line(img, cv::Point(50, 10), cv::Point(50, 190), cv::Scalar(30), 2);
  cv::rectangle(img, cv::Point(100, 80), cv::Point(200, 150), cv::Scalar(240), 2);

  auto profile = DetectorProfile::from_image(img);

  // Should produce valid params for all detectors
  for (int i = 0; i < static_cast<int>(DetectorId::COUNT); ++i) {
    auto id = static_cast<DetectorId>(i);
    auto params = profile.to_params(id);
    EXPECT_GT(params.size(), 0u) << "from_image profile empty for " << detector_id_to_name(id);
  }
}
