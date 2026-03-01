#pragma once

#include "analyzer.h"
#include "ui_continuity_optimizer.h"

/// @brief Unified line segment continuity optimizer.
///
/// Merges collinear line fragments and optionally bridges endpoint gaps
/// using gradient magnitude evidence. The gradient connection reuses the
/// merge constraints (max distance, angle error) so that a single set of
/// geometric parameters controls both operations.
class ContinuityOptimizer : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(ContinuityOptimizer)

  Ui::ContinuityOptimizer* ui{nullptr};
  const Analyzer::LineVector* lines{nullptr};
  const ImageSources* sources{nullptr};
  const cv::Mat* srcImg{nullptr};

 public:
  explicit ContinuityOptimizer(QWidget* parent = nullptr);
  ~ContinuityOptimizer();

  void connectTools(Analyzer* w) override;

 public slots:
  /// @brief Run the optimizer (merge, optionally with gradient connection).
  void run();

 private slots:
  /// @brief Enable/disable gradient parameter widgets.
  void onGradientToggled(bool checked);

 signals:
  void linesProcessed(const LineSegmentVector& lines);
};
