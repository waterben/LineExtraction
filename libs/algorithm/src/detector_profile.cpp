//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file detector_profile.cpp
/// @brief Implementation of DetectorProfile with per-detector parameter mapping.
///
/// Each map_lsd_*() function translates the 4 intuitive knobs (detail,
/// gap_tolerance, min_length, precision) into concrete detector parameters.
/// Adaptive factors (contrast_factor, noise_factor) scale thresholds.

#include <algorithm/detector_profile.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lsfm {

// ============================================================================
// Detector name ↔ id conversion
// ============================================================================

static const std::array<std::string, static_cast<size_t>(DetectorId::COUNT)> kDetectorNames = {
    "LsdCC", "LsdCP", "LsdBurns", "LsdFBW", "LsdFGioi", "LsdEDLZ", "LsdEL", "LsdEP", "LsdHoughP"};

std::string detector_id_to_name(DetectorId id) {
  auto idx = static_cast<size_t>(id);
  if (idx >= kDetectorNames.size()) {
    throw std::invalid_argument("Unknown DetectorId: " + std::to_string(idx));
  }
  return kDetectorNames.at(idx);
}

DetectorId detector_name_to_id(const std::string& name) {
  // Case-insensitive comparison
  auto to_lower = [](const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return r;
  };
  std::string lower = to_lower(name);
  for (size_t i = 0; i < kDetectorNames.size(); ++i) {
    if (to_lower(kDetectorNames[i]) == lower) {
      return static_cast<DetectorId>(i);
    }
  }
  throw std::invalid_argument("Unknown detector name: " + name);
}

// ============================================================================
// DetectorProfile construction
// ============================================================================

DetectorProfile::DetectorProfile(double detail, double gap_tolerance, double min_length, double precision)
    : detail_(std::clamp(detail, 0.0, 100.0)),
      gap_tolerance_(std::clamp(gap_tolerance, 0.0, 100.0)),
      min_length_(std::clamp(min_length, 0.0, 100.0)),
      precision_(std::clamp(precision, 0.0, 100.0)) {}

DetectorProfile::DetectorProfile(const ProfileHints& hints)
    : detail_(std::clamp(hints.detail, 0.0, 100.0)),
      gap_tolerance_(std::clamp(hints.gap_tolerance, 0.0, 100.0)),
      min_length_(std::clamp(hints.min_length, 0.0, 100.0)),
      precision_(std::clamp(hints.precision, 0.0, 100.0)),
      contrast_factor_(std::clamp(hints.contrast_factor, 0.5, 2.0)),
      noise_factor_(std::clamp(hints.noise_factor, 0.5, 2.0)) {}

DetectorProfile DetectorProfile::from_image(const cv::Mat& image) {
  auto props = ImageAnalyzer::analyze(image);
  auto hints = props.suggest_profile();
  return DetectorProfile(hints);
}

DetectorProfile DetectorProfile::from_hints(const ProfileHints& hints) { return DetectorProfile(hints); }

// ============================================================================
// Parameter generation
// ============================================================================

ParamConfig DetectorProfile::to_params(DetectorId detector) const {
  switch (detector) {
    case DetectorId::LSD_CC:
      return map_lsd_cc();
    case DetectorId::LSD_CP:
      return map_lsd_cp();
    case DetectorId::LSD_BURNS:
      return map_lsd_burns();
    case DetectorId::LSD_FBW:
      return map_lsd_fbw();
    case DetectorId::LSD_FGIOI:
      return map_lsd_fgioi();
    case DetectorId::LSD_EDLZ:
      return map_lsd_edlz();
    case DetectorId::LSD_EL:
      return map_lsd_el();
    case DetectorId::LSD_EP:
      return map_lsd_ep();
    case DetectorId::LSD_HOUGHP:
      return map_lsd_houghp();
    default:
      throw std::invalid_argument("Unknown DetectorId");
  }
}

ParamConfig DetectorProfile::to_params(const std::string& detector_name) const {
  return to_params(detector_name_to_id(detector_name));
}

void DetectorProfile::apply(ValueManager& vm, DetectorId detector) const { vm.value(to_params(detector)); }

void DetectorProfile::apply(ValueManager& vm, const std::string& detector_name) const {
  apply(vm, detector_name_to_id(detector_name));
}

std::vector<std::string> DetectorProfile::supported_detectors() {
  return {kDetectorNames.begin(), kDetectorNames.end()};
}

// ============================================================================
// Helpers
// ============================================================================

double DetectorProfile::lerp(double t, double lo, double hi) { return lo + t * (hi - lo); }

// ============================================================================
// Mapping tables
// ============================================================================
//
// Each map_lsd_*() function uses the 4 knobs (each 0–100, normalized to
// t ∈ [0,1]) to interpolate between parameter extremes.
//
// Convention:
//   detail↑  → lower thresholds, more sensitive → more detections
//   gap_tolerance↑ → larger max_gap, more lenient linking
//   min_length↑ → larger minimum pixel count
//   precision↑ → smaller error tolerances, higher quality
//
// Adaptive factors multiply threshold parameters:
//   contrast_factor > 1 → raise thresholds (low contrast image)
//   noise_factor > 1 → raise thresholds (noisy image)

ParamConfig DetectorProfile::map_lsd_cc() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  // Adaptive threshold scaling
  double th_scale = contrast_factor_ * noise_factor_;

  // nms_th_low: detail↑ → lower threshold (0.001 .. 0.020)
  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  // nms_th_high: detail↑ → lower threshold (0.004 .. 0.060)
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  // edge_min_pixels: min_length 0→3, 100→50
  int min_pix = static_cast<int>(std::round(lerp(tm, 3.0, 50.0)));
  // edge_max_gap: gap_tolerance 0→0, 100→5
  int max_gap = static_cast<int>(std::round(lerp(tg, 0.0, 5.0)));
  // split_error_distance: precision↑ → smaller error (0.5 .. 5.0)
  double split_err = lerp(1.0 - tp, 0.5, 5.0);
  // grad_kernel_size: precision↑ → larger kernel (3→5→7)
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);

  return {
      NV("grad_kernel_size", Value(kernel)), NV("nms_th_low", Value(nms_lo)),
      NV("nms_th_high", Value(nms_hi)),      NV("edge_min_pixels", Value(min_pix)),
      NV("edge_max_gap", Value(max_gap)),    NV("split_error_distance", Value(split_err)),
  };
}

ParamConfig DetectorProfile::map_lsd_cp() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  int min_pix = static_cast<int>(std::round(lerp(tm, 3.0, 50.0)));
  int max_gap = static_cast<int>(std::round(lerp(tg, 0.0, 5.0)));
  double split_err = lerp(1.0 - tp, 0.5, 5.0);
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);
  // edge_pattern_tolerance: precision↑ → less tolerance (1 .. 5)
  int pat_tol = std::max(1, static_cast<int>(std::round(lerp(1.0 - tp, 0.0, 5.0))));

  return {
      NV("grad_kernel_size", Value(kernel)),
      NV("nms_th_low", Value(nms_lo)),
      NV("nms_th_high", Value(nms_hi)),
      NV("edge_min_pixels", Value(min_pix)),
      NV("edge_max_gap", Value(max_gap)),
      NV("split_error_distance", Value(split_err)),
      NV("edge_pattern_tolerance", Value(pat_tol)),
  };
}

ParamConfig DetectorProfile::map_lsd_burns() const {
  double td = detail_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  // nms_th_low / nms_th_high: detail↑ → more sensitive
  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  // edge_min_pixels: min_length 0→2, 100→30
  int min_pix = static_cast<int>(std::round(lerp(tm, 2.0, 30.0)));
  // edge_partitions: precision↑ → more partitions (6 .. 36)
  int partitions = static_cast<int>(std::round(lerp(tp, 6.0, 36.0)));
  // grad_kernel_size
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);

  return {
      NV("grad_kernel_size", Value(kernel)), NV("nms_th_low", Value(nms_lo)),          NV("nms_th_high", Value(nms_hi)),
      NV("edge_min_pixels", Value(min_pix)), NV("edge_partitions", Value(partitions)),
  };
}

ParamConfig DetectorProfile::map_lsd_fbw() const {
  double td = detail_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  // edge_min_pixels: 0 means auto-compute; let's range 0..30
  int min_pix = static_cast<int>(std::round(lerp(tm, 0.0, 30.0)));
  // angle_th: precision↑ → tighter angle tolerance (5..45 degrees)
  double angle = lerp(1.0 - tp, 5.0, 45.0);
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);

  return {
      NV("grad_kernel_size", Value(kernel)), NV("nms_th_low", Value(nms_lo)), NV("nms_th_high", Value(nms_hi)),
      NV("edge_min_pixels", Value(min_pix)), NV("angle_th", Value(angle)),
  };
}

ParamConfig DetectorProfile::map_lsd_fgioi() const {
  double td = detail_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  // FGioi has its own internal gradient — no grad_kernel_size/NMS
  // quant_error: detail↑ → higher quant tolerance (more detections)
  double quant = lerp(td, 1.0, 4.0);
  // angle_th: precision↑ → tighter angle tolerance
  double angle = lerp(1.0 - tp, 10.0, 45.0);
  // density_th: detail↑ → lower density requirement → more detections
  double density = lerp(1.0 - td, 0.3, 0.9);
  // log_eps: detail↑ → lower NFA threshold → more detections
  double log_eps = lerp(1.0 - td, 0.0, 2.0);
  // bins: precision↑ → more bins
  int bins = static_cast<int>(std::round(lerp(tp, 512.0, 2048.0)));

  // Adaptive: noise → increase density requirement
  density = std::clamp(density * noise_factor_, 0.1, 0.95);

  // min_length is not directly exposed by FGioi, but log_eps indirectly
  // controls it. We additionally scale log_eps by min_length knob.
  log_eps = lerp(tm, log_eps * 0.5, log_eps * 2.0);
  log_eps = std::clamp(log_eps, 0.0, 5.0);

  return {
      NV("quant_error", Value(quant)), NV("angle_th", Value(angle)), NV("density_th", Value(density)),
      NV("log_eps", Value(log_eps)),   NV("bins", Value(bins)),
  };
}

ParamConfig DetectorProfile::map_lsd_edlz() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  // grad_th: detail↑ → lower threshold (3 .. 30)
  double grad_th = lerp(1.0 - td, 3.0, 30.0) * th_scale;
  // anchor_th: detail↑ → lower anchor threshold (0.5 .. 5)
  double anchor_th = lerp(1.0 - td, 0.5, 5.0) * th_scale;
  // scan_int: gap_tolerance↑ → larger interval (1..3)
  int scan_int = static_cast<int>(std::round(lerp(tg, 1.0, 3.0)));
  // min_len: min_length 0→5, 100→50
  int min_len = static_cast<int>(std::round(lerp(tm, 5.0, 50.0)));
  // fit_error: precision↑ → smaller error (0.5 .. 4.0)
  double fit_err = lerp(1.0 - tp, 0.5, 4.0);

  return {
      NV("grad_th", Value(grad_th)), NV("anchor_th", Value(anchor_th)), NV("scan_int", Value(scan_int)),
      NV("min_len", Value(min_len)), NV("fit_error", Value(fit_err)),
  };
}

ParamConfig DetectorProfile::map_lsd_el() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  // NMS thresholds
  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  // edge_min_pixels
  int min_pix = static_cast<int>(std::round(lerp(tm, 3.0, 50.0)));
  // edge_max_gap: gap_tolerance
  int max_gap = static_cast<int>(std::round(lerp(tg, 0.0, 8.0)));
  // split_error_distance: precision
  double split_err = lerp(1.0 - tp, 0.5, 5.0);
  // split_min_length: min_length
  int split_min = static_cast<int>(std::round(lerp(tm, 2.0, 20.0)));
  // grad_kernel_size
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);
  // nfa_precision: precision↑ → finer nfa precision (1/16 .. 1/4)
  double nfa_prec = lerp(1.0 - tp, 0.0625, 0.25);

  return {
      NV("grad_kernel_size", Value(kernel)),    NV("nms_th_low", Value(nms_lo)),
      NV("nms_th_high", Value(nms_hi)),         NV("edge_min_pixels", Value(min_pix)),
      NV("edge_max_gap", Value(max_gap)),       NV("split_error_distance", Value(split_err)),
      NV("split_min_length", Value(split_min)), NV("nfa_precision", Value(nfa_prec)),
  };
}

ParamConfig DetectorProfile::map_lsd_ep() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  int min_pix = static_cast<int>(std::round(lerp(tm, 3.0, 50.0)));
  int max_gap = static_cast<int>(std::round(lerp(tg, 0.0, 8.0)));
  double split_err = lerp(1.0 - tp, 0.5, 5.0);
  int split_min = static_cast<int>(std::round(lerp(tm, 2.0, 20.0)));
  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);
  // edge_pat_tol: precision↑ → less tolerance (0 .. 5)
  int pat_tol = static_cast<int>(std::round(lerp(1.0 - tp, 0.0, 5.0)));

  return {
      NV("grad_kernel_size", Value(kernel)),    NV("nms_th_low", Value(nms_lo)),
      NV("nms_th_high", Value(nms_hi)),         NV("edge_min_pixels", Value(min_pix)),
      NV("edge_max_gap", Value(max_gap)),       NV("split_error_distance", Value(split_err)),
      NV("split_min_length", Value(split_min)), NV("edge_pat_tol", Value(pat_tol)),
  };
}

ParamConfig DetectorProfile::map_lsd_houghp() const {
  double td = detail_ / 100.0;
  double tg = gap_tolerance_ / 100.0;
  double tm = min_length_ / 100.0;
  double tp = precision_ / 100.0;

  double th_scale = contrast_factor_ * noise_factor_;

  // NMS thresholds
  double nms_lo = lerp(1.0 - td, 0.001, 0.020) * th_scale;
  double nms_hi = lerp(1.0 - td, 0.004, 0.060) * th_scale;

  int kernel = (tp > 0.7) ? 7 : (tp > 0.3 ? 5 : 3);

  // hough_rho: precision↑ → finer resolution (0.5 .. 3.0)
  double rho = lerp(1.0 - tp, 0.5, 3.0);
  // hough_theta: precision↑ → finer angle (pi/360 .. pi/90)
  double theta = lerp(1.0 - tp, CV_PI / 360.0, CV_PI / 90.0);
  // hough_vote_th: detail↑ → lower vote threshold (50 .. 300)
  int vote_th = static_cast<int>(std::round(lerp(1.0 - td, 50.0, 300.0)));
  // edge_min_len: min_length 0→3, 100→50
  double edge_min = lerp(tm, 3.0, 50.0);
  // edge_max_gap: gap_tolerance 0→0, 100→10
  double edge_gap = lerp(tg, 0.0, 10.0);

  return {
      NV("grad_kernel_size", Value(kernel)), NV("nms_th_low", Value(nms_lo)),     NV("nms_th_high", Value(nms_hi)),
      NV("hough_rho", Value(rho)),           NV("hough_theta", Value(theta)),     NV("hough_vote_th", Value(vote_th)),
      NV("edge_min_len", Value(edge_min)),   NV("edge_max_gap", Value(edge_gap)),
  };
}

}  // namespace lsfm
