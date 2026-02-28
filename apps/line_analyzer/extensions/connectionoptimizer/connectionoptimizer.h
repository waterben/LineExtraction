#pragma once

#include "controlwindow.h"
#include "ui_connectionoptimizer.h"

/// @brief Tool panel for connecting nearby line segment endpoints.
///
/// Uses LineConnect from libs/algorithm to join endpoints when gradient
/// magnitude along the connecting path exceeds a threshold.
class ConnectionOptimizer : public LATool {
  Q_OBJECT

  Ui::ConnectionOptimizer* ui{nullptr};
  const ControlWindow::LineVector* lines{nullptr};
  const ImageSources* sources{nullptr};
  const cv::Mat* srcImg{nullptr};

 public:
  explicit ConnectionOptimizer(QWidget* parent = nullptr);
  ~ConnectionOptimizer();

  void connectTools(ControlWindow* w) override;

 public slots:
  void runOptimizer();

 signals:
  void linesConnected(const LineSegmentVector& lines);
};
