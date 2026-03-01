#include "precision_optimizer.h"

#include "help_button.hpp"

#include <QMessageBox>
#include <cmath>

using namespace lsfm;
using namespace std;

PrecisionOptimizer::PrecisionOptimizer(QWidget* parent)
    : LATool("Precision Optimizer", parent),
      ui(new Ui::PrecisionOptimizer),
      sources(nullptr),
      lines(nullptr),
      lineSel(-1) {
  setWindowTitle("Precision Optimizer");
  ui->setupUi(this);

  ui->cb_profile_interp->blockSignals(true);
  ui->cb_profile_interp->addItem("nearest");
  ui->cb_profile_interp->addItem("nearest_round");
  ui->cb_profile_interp->addItem("bilinear");
  ui->cb_profile_interp->addItem("bicubic");
  ui->cb_profile_interp->setCurrentIndex(2);
  ui->cb_profile_interp->blockSignals(false);

  ui->cb_search->blockSignals(true);
  ui->cb_search->addItem("BFGS");
  ui->cb_search->addItem("LBFGS");
  ui->cb_search->addItem("CG");
  ui->cb_search->blockSignals(false);

  ui->cb_stop->blockSignals(true);
  ui->cb_stop->addItem("Delta");
  ui->cb_stop->addItem("Grad_Norm");
  ui->cb_stop->blockSignals(false);

  // Tooltips explaining the optimizer and each parameter.
  setToolTip(
      tr("Optimize sub-pixel line localization by maximizing the mean "
         "gradient response along a line segment's profile."));
  ui->cb_data_source->setToolTip(tr("Gradient magnitude source to evaluate."));
  ui->cb_profile_interp->setToolTip(tr("Interpolation method for sampling the gradient along the profile."));
  ui->cb_search->setToolTip(tr("Optimization search strategy: BFGS, L-BFGS, or Conjugate Gradient."));
  ui->cb_stop->setToolTip(tr("Stopping criterion: Delta (function value change) or Gradient Norm."));
  ui->spin_range_rot->setToolTip(
      tr("Half-range in degrees to search around the current "
         "line orientation. E.g. \xc2\xb1"
         "1\xc2\xb0. Disabled when auto-rotation is active."));
  ui->chb_auto_rot->setToolTip(
      tr("Automatically compute the rotation range from the "
         "detector\xe2\x80\x99"
         "s gradient kernel size: "
         "\xc2\xb1"
         "atan(1 / \xe2\x8c\x8a"
         "kernel_size/2\xe2\x8c\x8b)\xc2\xb0"));
  ui->spin_range_prof->setToolTip(
      tr("Half-range in pixels to search perpendicular to "
         "the current line position."));
  ui->spin_line_dist->setToolTip(
      tr("Number of support pixels along the line used for "
         "computing the mean gradient response."));
  ui->chb_line_samples->setToolTip(
      tr("Interpret Line Distance as discrete sample count "
         "instead of pixel spacing."));
  ui->chb_fast_interp->setToolTip(tr("Use faster approximate interpolation (may reduce accuracy)."));
  ui->edit_stop_delta->setToolTip(
      tr("Convergence threshold for the stopping criterion. "
         "Smaller values = tighter convergence."));
  ui->chb_max_iter->setToolTip(tr("Enable a maximum iteration limit for the optimizer."));
  ui->spin_max_iter->setToolTip(tr("Maximum number of optimization iterations."));
  ui->edit_deri_delta->setToolTip(
      tr("Step size for numerical derivative computation. "
         "Smaller = more accurate but slower."));
  ui->pb_line->setToolTip(tr("Optimize the currently selected line segment."));
  ui->pb_all->setToolTip(tr("Optimize all detected line segments."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/precision_optimizer/README.md");
}

PrecisionOptimizer::~PrecisionOptimizer() { delete ui; }

void PrecisionOptimizer::connectTools(Analyzer* w) {
  ctrl = w;
  lines = &w->getLines();
  sources = &w->getSources();
  connect(this, SIGNAL(lineOptimized(int)), w, SLOT(updateLine(int)));
  connect(this, SIGNAL(linesOptimized()), w, SLOT(updateLines()));
  connect(w, SIGNAL(lineSelChanged(int)), this, SLOT(setLineSel(int)));
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(updateSources(const ImageSources&)));
  // Auto-rotation: recompute when the user switches detectors.
  connect(w, SIGNAL(detectorChanged(const QString&)), this, SLOT(updateAutoRotation()));
  // Auto-rotation: toggling the checkbox enables/disables manual spin box.
  connect(ui->chb_auto_rot, SIGNAL(toggled(bool)), this, SLOT(updateAutoRotation()));
}

void PrecisionOptimizer::updateSources(const ImageSources& src) {
  ui->cb_data_source->blockSignals(true);

  int isi = ui->cb_data_source->currentIndex(), magi = 0;
  QString iss = ui->cb_data_source->currentText();
  ui->cb_data_source->clear();
  int i = 0, idx = 0;
  for_each(src.begin(), src.end(), [&](const ImageSource& s) {
    if (s.name == "mag" || s.name == "qmag" || s.name == "nmag") {
      ui->cb_data_source->addItem(s.name, i);
      if (s.name == "mag") magi = idx;
      ++idx;
    }
    ++i;
  });

  if (isi != -1 && isi < ui->cb_data_source->count() && ui->cb_data_source->itemText(isi) == iss)
    ui->cb_data_source->setCurrentIndex(isi);
  else
    ui->cb_data_source->setCurrentIndex(magi);

  ui->cb_data_source->blockSignals(false);
  ui->cb_data_source->setDisabled(false);
}

void PrecisionOptimizer::updateMaxIter() { ui->spin_max_iter->setEnabled(ui->chb_max_iter->isChecked()); }

void PrecisionOptimizer::setLineSel(int sel) { lineSel = sel; }

void PrecisionOptimizer::updateAutoRotation() {
  bool auto_on = ui->chb_auto_rot->isChecked();
  ui->spin_range_rot->setEnabled(!auto_on);
  if (auto_on) computeAutoRotation();
}

void PrecisionOptimizer::computeAutoRotation() {
  if (ctrl == nullptr) return;

  DetectorPtr det = ctrl->getCurrentDetector();
  if (!det || !det->lsd) return;

  Value ks_val = det->lsd->value("grad_kernel_size");
  if (!ks_val.type()) return;  // parameter not registered

  int kernel_size = ks_val.getInt();
  int half = kernel_size / 2;
  if (half < 1) half = 1;

  double range_deg = std::atan(1.0 / half) * 180.0 / CV_PI;
  ui->spin_range_rot->setValue(range_deg);
}

template <class MT>
typename lsfm::MeanHelper<double, cv::Point_>::func_type getMean(int idx, bool sampled, bool fast) {
  switch (idx) {
    case 0:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<double, MT, lsfm::NearestInterpolator<double, MT> >::process;
        return lsfm::MeanSampled<double, MT, lsfm::NearestInterpolator<double, MT> >::process;
      }
      if (fast) return lsfm::FastMean<double, MT, lsfm::NearestInterpolator<double, MT> >::process;
      return lsfm::Mean<double, MT, lsfm::NearestInterpolator<double, MT> >::process;
    case 1:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<double, MT, lsfm::FastRoundNearestInterpolator<double, MT> >::process;
        return lsfm::MeanSampled<double, MT, lsfm::FastRoundNearestInterpolator<double, MT> >::process;
      }
      if (fast) return lsfm::FastMean<double, MT, lsfm::FastRoundNearestInterpolator<double, MT> >::process;
      return lsfm::Mean<double, MT, lsfm::FastRoundNearestInterpolator<double, MT> >::process;
    case 2:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
        return lsfm::MeanSampled<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
      }
      if (fast) return lsfm::FastMean<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
      return lsfm::Mean<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
    case 3:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<double, MT, lsfm::CubicInterpolator<double, MT> >::process;
        return lsfm::MeanSampled<double, MT, lsfm::CubicInterpolator<double, MT> >::process;
      }
      if (fast) return lsfm::FastMean<double, MT, lsfm::CubicInterpolator<double, MT> >::process;
      return lsfm::Mean<double, MT, lsfm::CubicInterpolator<double, MT> >::process;
  }
  if (sampled) {
    if (fast) return lsfm::FastMeanSampled<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
    return lsfm::MeanSampled<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
  }
  if (fast) return lsfm::FastMean<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
  return lsfm::Mean<double, MT, lsfm::LinearInterpolator<double, MT> >::process;
}

void PrecisionOptimizer::optimizeLine() {
  if (lineSel < 0 || lines == nullptr || sources == nullptr) return;

  // Lazy recompute: if auto-rotation is on, refresh the range now so that
  // per-parameter kernel-size edits (which don't emit detectorChanged) are
  // still picked up.
  if (ui->chb_auto_rot->isChecked()) computeAutoRotation();

  try {
    float_type d = 0, a = 0;
    double dr = ui->spin_range_prof->value(), ar = ui->spin_range_rot->value() / 180 * CV_PI;
    Line& l = (*lines)[static_cast<std::size_t>(lineSel)];

    const int sourceIndex = ui->cb_data_source->currentData().toInt();
    if (sourceIndex < 0) return;
    if (static_cast<std::size_t>(sourceIndex) >= sources->size()) return;

    ImageSource src = (*sources)[static_cast<std::size_t>(sourceIndex)];
    double stime = double(cv::getTickCount());
    if (src.data.type() == cv::DataType<float_type>::type) {
      MeanHelper<double, cv::Point_>::func_type meanp = getMean<float_type>(
          ui->cb_profile_interp->currentIndex(), ui->chb_line_samples->isChecked(), ui->chb_fast_interp->isChecked());

      switch (ui->cb_search->currentIndex()) {
        case 0:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<float_type, dlib::bfgs_search_strategy, dlib::objective_delta_stop_strategy>::
                  optimize<float_type>(
                      src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                      ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                      (ui->chb_max_iter->isChecked()
                           ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                                 static_cast<unsigned long>(ui->spin_max_iter->value()))
                                 .be_verbose()
                           : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<float_type, dlib::bfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 1:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<float_type, dlib::lbfgs_search_strategy, dlib::objective_delta_stop_strategy>::
                  optimize<float_type>(
                      src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                      ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                      (ui->chb_max_iter->isChecked()
                           ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                                 static_cast<unsigned long>(ui->spin_max_iter->value()))
                                 .be_verbose()
                           : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<float_type, dlib::lbfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 2:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<float_type, dlib::cg_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<float_type, dlib::cg_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
      }
    } else {
      MeanHelper<double, cv::Point_>::func_type meanp = getMean<int>(
          ui->cb_profile_interp->currentIndex(), ui->chb_line_samples->isChecked(), ui->chb_fast_interp->isChecked());

      switch (ui->cb_search->currentIndex()) {
        case 0:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::bfgs_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::bfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 1:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::lbfgs_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::lbfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 2:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::cg_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::cg_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
      }

      switch (ui->cb_search->currentIndex()) {
        case 0:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::bfgs_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::bfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 1:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::lbfgs_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::lbfgs_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
        case 2:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              lsfm::LineOptimizer<int, dlib::cg_search_strategy, dlib::objective_delta_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
            case 1:
              lsfm::LineOptimizer<int, dlib::cg_search_strategy, dlib::gradient_norm_stop_strategy>::optimize<
                  float_type>(
                  src.data, l.modSegment(), d, a, -dr, dr, -ar, ar, ui->spin_line_dist->value(),
                  ui->edit_deri_delta->text().toDouble(), meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                             .be_verbose()
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble()).be_verbose()));
              break;
          }
          break;
      }
    }
    std::cout << "Time for optimization: " << (double(cv::getTickCount()) - stime) * 1000 / cv::getTickFrequency()
              << "ms" << std::endl;

    l.mod.angle += a;
    l.mod.distance += d;
    emit lineOptimized(lineSel);
  } catch (const std::exception& ex) {
    std::cerr << "Line optimization failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Optimization Error"), tr("Line optimization failed:\n%1").arg(ex.what()));
  }
}

void PrecisionOptimizer::optimizeAllLines() {
  if (lines == nullptr || sources == nullptr) return;

  // Lazy recompute (see optimizeLine comment).
  if (ui->chb_auto_rot->isChecked()) computeAutoRotation();

  try {
    double dr = ui->spin_range_prof->value(), ar = ui->spin_range_rot->value() / 180 * CV_PI;

    const int sourceIndex = ui->cb_data_source->currentData().toInt();
    if (sourceIndex < 0) return;
    if (static_cast<std::size_t>(sourceIndex) >= sources->size()) return;

    ImageSource src = (*sources)[static_cast<std::size_t>(sourceIndex)];
    double stime = double(cv::getTickCount());
    if (src.data.type() == cv::DataType<float_type>::type) {
      MeanHelper<double, cv::Point_>::func_type meanp = getMean<float_type>(
          ui->cb_profile_interp->currentIndex(), ui->chb_line_samples->isChecked(), ui->chb_fast_interp->isChecked());

      switch (ui->cb_search->currentIndex()) {
        case 0:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<float_type, dlib::bfgs_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<float_type, dlib::bfgs_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
        case 1:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<float_type, dlib::lbfgs_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<float_type, dlib::lbfgs_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
        case 2:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<float_type, dlib::cg_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<float_type, dlib::cg_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
      }
    } else {
      MeanHelper<double, cv::Point_>::func_type meanp = getMean<int>(
          ui->cb_profile_interp->currentIndex(), ui->chb_line_samples->isChecked(), ui->chb_fast_interp->isChecked());

      switch (ui->cb_search->currentIndex()) {
        case 0:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<int, dlib::bfgs_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<int, dlib::bfgs_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::bfgs_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
        case 1:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<int, dlib::lbfgs_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<int, dlib::lbfgs_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::lbfgs_search_strategy(10),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
        case 2:
          switch (ui->cb_stop->currentIndex()) {
            case 0:
              optimize<int, dlib::cg_search_strategy, dlib::objective_delta_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                             static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::objective_delta_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
            case 1:
              optimize<int, dlib::cg_search_strategy, dlib::gradient_norm_stop_strategy>(
                  src.data, -dr, dr, -ar, ar, ui->spin_line_dist->value(), ui->edit_deri_delta->text().toDouble(),
                  meanp, dlib::cg_search_strategy(),
                  (ui->chb_max_iter->isChecked()
                       ? dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble(),
                                                           static_cast<unsigned long>(ui->spin_max_iter->value()))
                       : dlib::gradient_norm_stop_strategy(ui->edit_stop_delta->text().toDouble())));
              break;
          }
          break;
      }
    }
    std::cout << "Time for optimization: " << (double(cv::getTickCount()) - stime) * 1000 / cv::getTickFrequency()
              << "ms" << std::endl;


    emit linesOptimized();
  } catch (const std::exception& ex) {
    std::cerr << "Batch optimization failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Optimization Error"), tr("Batch line optimization failed:\n%1").arg(ex.what()));
  }
}
