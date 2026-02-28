#pragma once

#include "controlwindow.h"
#include "ui_continuityoptimizer.h"

/// @brief Tool panel for merging near-collinear line segments.
///
/// Uses LineMerge from libs/algorithm to iteratively merge segment pairs
/// that satisfy proximity criteria (endpoint distance, angle, perpendicular
/// distance, parallel gap).
class ContinuityOptimizer : public LATool {
  Q_OBJECT

  Ui::ContinuityOptimizer* ui{nullptr};
  const ControlWindow::LineVector* lines{nullptr};

 public:
  explicit ContinuityOptimizer(QWidget* parent = nullptr);
  ~ContinuityOptimizer();

  void connectTools(ControlWindow* w) override;

 public slots:
  void runOptimizer();

 signals:
  void linesMerged(const LineSegmentVector& lines);
};
