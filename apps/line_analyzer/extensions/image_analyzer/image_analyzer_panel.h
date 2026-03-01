#pragma once

#include "analyzer.h"
#include "ui_image_analyzer_panel.h"

/// @brief Tool panel for analyzing image properties.
///
/// Displays image characteristics (contrast, noise, edge density, dynamic range)
/// and the suggested DetectorProfile hints computed by ImageAnalyzer from
/// libs/algorithm. Read-only panel — does not modify lines.
class ImageAnalyzerPanel : public LATool {
  Q_OBJECT

  Ui::ImageAnalyzerPanel* ui{nullptr};
  const cv::Mat* srcImg{nullptr};

 public:
  explicit ImageAnalyzerPanel(QWidget* parent = nullptr);
  ~ImageAnalyzerPanel();

  void connectTools(Analyzer* w) override;

 public slots:
  /// @brief Analyze the current source image and display results.
  void analyze();
};
