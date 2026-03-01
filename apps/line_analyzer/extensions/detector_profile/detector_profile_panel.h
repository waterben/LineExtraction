#pragma once

#include "analyzer.h"
#include "ui_detector_profile_panel.h"
#include <algorithm/detector_profile.hpp>

/// @brief Tool panel for high-level detector parameter tuning.
///
/// Exposes 4 percentage sliders (detail, gap_tolerance, min_length, precision)
/// and 2 adaptive factors (contrast, noise) that are translated by
/// DetectorProfile into concrete detector parameters. Supports automatic
/// profile derivation from the current source image via ImageAnalyzer.
class DetectorProfilePanel : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(DetectorProfilePanel)

  Ui::DetectorProfilePanel* ui{nullptr};
  Analyzer* ctrl{nullptr};
  const cv::Mat* srcImg{nullptr};

 public:
  explicit DetectorProfilePanel(QWidget* parent = nullptr);
  ~DetectorProfilePanel();

  void connectTools(Analyzer* w) override;

 public slots:
  /// @brief Derive slider values automatically from the current source image.
  void autoFromImage();

  /// @brief Apply the current profile sliders to the active detector.
  void applyToDetector();

  /// @brief Reset all sliders and factors to their default values.
  void resetDefaults();

  /// @brief Update image property labels from the current source image.
  void analyzeImage();

 private:
  /// @brief Resolve the current app detector name to a DetectorId.
  ///
  /// Maps display names like "LSD CC", "LSD CP QFSt Odd" etc. to the
  /// canonical DetectorId enum. Returns a pair of (success, id).
  ///
  /// @param name Display name from Analyzer.
  /// @return Pair of (found, DetectorId). found is false for unsupported detectors.
  static std::pair<bool, lsfm::DetectorId> resolveDetectorId(const QString& name);

  /// @brief Build a DetectorProfile from the current UI slider values.
  /// @return Configured DetectorProfile.
  lsfm::DetectorProfile buildProfile() const;

  /// @brief Update the status label with a message.
  /// @param msg Text to display.
  void setStatus(const QString& msg);
};
