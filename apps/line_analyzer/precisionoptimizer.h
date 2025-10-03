#ifndef PRECISIONOPTIMIZER_H
#define PRECISIONOPTIMIZER_H

#include "controlwindow.h"
#include "helpers.h"
#include "ui_precisionoptimizer.h"
#include <geometry/line_optimizer.hpp>

namespace Ui {
class PrecisionOptimizer;
}

class PrecisionOptimizer : public LATool {
  Q_OBJECT
  typedef ControlWindow::LineVector LineVector;
  typedef ControlWindow::Line Line;

  Ui::PrecisionOptimizer* ui;
  const ImageSources* sources;
  LineVector* lines;
  int lineSel;

 public:
  explicit PrecisionOptimizer(QWidget* parent = 0);
  ~PrecisionOptimizer();

  void connectTools(ControlWindow* w);

 public slots:
  void updateSources(const ImageSources& src);
  void updateMaxIter();

  void setLineSel(int sel);

  void optimizeLine();
  void optimizeAllLines();

 signals:
  void lineOptimized(int idx);
  void linesOptimized();

 private:
  template <class mat_type, class search_strategy_type, class stop_strategy_type>
  inline void optimize(const cv::Mat& mag,
                       double d_lower,
                       double d_upper,
                       double r_lower,
                       double r_upper,
                       double mean_param,
                       double derivative_prec,
                       typename lsfm::MeanHelper<double, cv::Point_>::func_type mean_op,
                       search_strategy_type search,
                       stop_strategy_type stop) {
    int idx = 0;
    for_each(lines->begin(), lines->end(), [&](Line& l) {
      std::cout << "line: " << idx++;
      float_type d = 0, a = 0;
      lsfm::LineOptimizer<mat_type, search_strategy_type, stop_strategy_type>::optimize(
          mag, l.modSegment(), d, a, d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search,
          stop);
      l.mod.angle += a;
      l.mod.distance += d;
    });
  }
};

#endif  // PRECISIONOPTIMIZER_H
