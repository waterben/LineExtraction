#pragma once

#include "analyzer.h"
#include "ui_nfa_filter.h"

/// @brief NFA-based line segment filter.
///
/// Post-processing tool that computes the Number of False Alarms (NFA) for
/// each line segment and filters by either a log-epsilon threshold or by
/// keeping the top-N most meaningful segments. Three NFA variants are
/// supported: contrast (magnitude), binomial (orientation), and binomial 2.
class NfaFilter : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(NfaFilter)

  Ui::NfaFilter* ui{nullptr};
  Analyzer* ctrl{nullptr};
  const Analyzer::LineVector* lines{nullptr};
  const ImageSources* sources{nullptr};
  const cv::Mat* srcImg{nullptr};

 public:
  explicit NfaFilter(QWidget* parent = nullptr);
  ~NfaFilter();

  void connectTools(Analyzer* w) override;

 public slots:
  /// @brief Compute NFA values and apply the selected filter.
  void run();

 private slots:
  /// @brief Toggle epsilon / top-N parameter widgets.
  void onModeChanged(int index);

  /// @brief Enable/disable binomial-specific parameter widgets.
  void onVariantChanged(int index);

 signals:
  void linesProcessed(const LineSegmentVector& lines);
};
