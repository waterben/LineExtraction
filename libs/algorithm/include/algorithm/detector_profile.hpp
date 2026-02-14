//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file detector_profile.hpp
/// @brief Intuitive high-level parameter profiles for LSD detectors.
///
/// Maps 4 user-facing percentage knobs (detail, gap_tolerance, min_length,
/// precision) to concrete detector parameters. Supports adaptive scaling
/// via ImageAnalyzer-produced hints.
///
/// Supported detectors:
///   LsdCC, LsdCP, LsdBurns, LsdFBW, LsdFGioi,
///   LsdEDLZ, LsdEL, LsdEP, LsdHoughP

#pragma once

#include <algorithm/image_analyzer.hpp>
#include <algorithm/search_strategy.hpp>
#include <opencv2/core.hpp>
#include <utility/value_manager.hpp>

#include <string>
#include <vector>

namespace lsfm {

/// @brief Known detector identifiers for parameter mapping.
enum class DetectorId : int {
  LSD_CC = 0,
  LSD_CP,
  LSD_BURNS,
  LSD_FBW,
  LSD_FGIOI,
  LSD_EDLZ,
  LSD_EL,
  LSD_EP,
  LSD_HOUGHP,
  COUNT  ///< Sentinel: number of detector types.
};

/// @brief Convert DetectorId to human-readable name string.
/// @param id Detector identifier.
/// @return Name string (e.g., "LsdCC").
std::string detector_id_to_name(DetectorId id);

/// @brief Parse a detector name string to DetectorId.
/// @param name Detector name (case-insensitive).
/// @return Corresponding DetectorId.
/// @throws std::invalid_argument if the name is not recognized.
DetectorId detector_name_to_id(const std::string& name);

/// @brief Intuitive high-level parameter profile for LSD detectors.
///
/// The user sets 4 percentage knobs (0–100%) and the profile translates
/// them to concrete detector parameters. Optional adaptive scaling
/// factors (from ImageAnalyzer) modulate the translation to account
/// for image characteristics such as noise and contrast.
///
/// @example
/// @code{cpp}
/// // Manual knob setting
/// DetectorProfile profile(70, 30, 40, 80);
/// auto params = profile.to_params(DetectorId::LSD_CC);
/// detector.value(params);
///
/// // Image-adaptive profile
/// cv::Mat img = cv::imread("photo.png", cv::IMREAD_GRAYSCALE);
/// auto profile2 = DetectorProfile::from_image(img);
/// profile2.apply(detector, DetectorId::LSD_CC);
/// @endcode
class DetectorProfile {
 public:
  /// @brief Construct a profile with explicit knob values.
  ///
  /// All knobs are clamped to [0, 100]. Adaptive factors default to 1.0.
  ///
  /// @param detail Detail level: 0 = coarse/salient, 100 = fine details.
  /// @param gap_tolerance Gap tolerance: 0 = strict, 100 = tolerant.
  /// @param min_length Minimum length: 0 = keep all, 100 = long only.
  /// @param precision Precision: 0 = rough/fast, 100 = sub-pixel accurate.
  explicit DetectorProfile(double detail = 50,
                           double gap_tolerance = 50,
                           double min_length = 50,
                           double precision = 50);

  /// @brief Construct a profile from ProfileHints (includes adaptive factors).
  /// @param hints Hints from ImageAnalyzer (knobs + adaptive factors).
  explicit DetectorProfile(const ProfileHints& hints);

  /// @brief Create a profile by analyzing an image.
  ///
  /// Runs ImageAnalyzer::analyze and uses suggest_profile() to derive
  /// both knob values and adaptive scaling factors.
  ///
  /// @param image Grayscale input image.
  /// @return Profile with image-adapted knob values and scaling factors.
  static DetectorProfile from_image(const cv::Mat& image);

  /// @brief Create a profile from explicit hints.
  /// @param hints ProfileHints with knobs and adaptive factors.
  /// @return Configured DetectorProfile.
  static DetectorProfile from_hints(const ProfileHints& hints);

  // ----- Knob accessors -----

  /// @brief Get the detail knob value [0, 100].
  double detail() const { return detail_; }

  /// @brief Set the detail knob value [0, 100].
  void set_detail(double v) { detail_ = std::clamp(v, 0.0, 100.0); }

  /// @brief Get the gap tolerance knob value [0, 100].
  double gap_tolerance() const { return gap_tolerance_; }

  /// @brief Set the gap tolerance knob value [0, 100].
  void set_gap_tolerance(double v) { gap_tolerance_ = std::clamp(v, 0.0, 100.0); }

  /// @brief Get the minimum length knob value [0, 100].
  double min_length() const { return min_length_; }

  /// @brief Set the minimum length knob value [0, 100].
  void set_min_length(double v) { min_length_ = std::clamp(v, 0.0, 100.0); }

  /// @brief Get the precision knob value [0, 100].
  double precision() const { return precision_; }

  /// @brief Set the precision knob value [0, 100].
  void set_precision(double v) { precision_ = std::clamp(v, 0.0, 100.0); }

  // ----- Adaptive factor accessors -----

  /// @brief Get the contrast scaling factor [0.5, 2.0].
  double contrast_factor() const { return contrast_factor_; }

  /// @brief Set the contrast scaling factor [0.5, 2.0].
  void set_contrast_factor(double v) { contrast_factor_ = std::clamp(v, 0.5, 2.0); }

  /// @brief Get the noise scaling factor [0.5, 2.0].
  double noise_factor() const { return noise_factor_; }

  /// @brief Set the noise scaling factor [0.5, 2.0].
  void set_noise_factor(double v) { noise_factor_ = std::clamp(v, 0.5, 2.0); }

  // ----- Parameter generation -----

  /// @brief Generate concrete parameters for a specific detector.
  ///
  /// Translates the 4 knobs (modulated by adaptive factors) into
  /// a NameValueVector suitable for ValueManager::value().
  ///
  /// @param detector Detector identifier.
  /// @return Vector of name-value pairs for that detector.
  ParamConfig to_params(DetectorId detector) const;

  /// @brief Generate concrete parameters by detector name string.
  /// @param detector_name Detector name (case-insensitive, e.g. "LsdCC").
  /// @return Vector of name-value pairs.
  /// @throws std::invalid_argument if detector name is not recognized.
  ParamConfig to_params(const std::string& detector_name) const;

  /// @brief Apply this profile to a ValueManager-based detector.
  ///
  /// Generates parameters for the specified detector and sets them
  /// on the given ValueManager instance.
  ///
  /// @param vm ValueManager-based detector to configure.
  /// @param detector Detector identifier for parameter mapping.
  void apply(ValueManager& vm, DetectorId detector) const;

  /// @brief Apply this profile by detector name string.
  /// @param vm ValueManager-based detector to configure.
  /// @param detector_name Detector name (case-insensitive).
  void apply(ValueManager& vm, const std::string& detector_name) const;

  /// @brief Get a list of all supported detector names.
  /// @return Vector of detector name strings.
  static std::vector<std::string> supported_detectors();

 private:
  double detail_ = 50;            ///< Detail knob [0, 100].
  double gap_tolerance_ = 50;     ///< Gap tolerance knob [0, 100].
  double min_length_ = 50;        ///< Minimum length knob [0, 100].
  double precision_ = 50;         ///< Precision knob [0, 100].
  double contrast_factor_ = 1.0;  ///< Adaptive contrast factor [0.5, 2.0].
  double noise_factor_ = 1.0;     ///< Adaptive noise factor [0.5, 2.0].

  // ----- Per-detector mapping functions -----

  /// @brief Linear interpolation helper: maps t in [0, 1] to [lo, hi].
  static double lerp(double t, double lo, double hi);

  /// @brief Map knobs to LsdCC parameters.
  ParamConfig map_lsd_cc() const;

  /// @brief Map knobs to LsdCP parameters.
  ParamConfig map_lsd_cp() const;

  /// @brief Map knobs to LsdBurns parameters.
  ParamConfig map_lsd_burns() const;

  /// @brief Map knobs to LsdFBW parameters.
  ParamConfig map_lsd_fbw() const;

  /// @brief Map knobs to LsdFGioi parameters.
  ParamConfig map_lsd_fgioi() const;

  /// @brief Map knobs to LsdEDLZ parameters.
  ParamConfig map_lsd_edlz() const;

  /// @brief Map knobs to LsdEL parameters.
  ParamConfig map_lsd_el() const;

  /// @brief Map knobs to LsdEP parameters.
  ParamConfig map_lsd_ep() const;

  /// @brief Map knobs to LsdHoughP parameters.
  ParamConfig map_lsd_houghp() const;
};

}  // namespace lsfm
