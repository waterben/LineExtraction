#include "continuityoptimizer.h"

#include "help_button.hpp"
#include <algorithm/line_merge.hpp>

#include <QMessageBox>


using namespace lsfm;

ContinuityOptimizer::ContinuityOptimizer(QWidget* parent)
    : LATool("Continuity Optimizer", parent), ui(new Ui::ContinuityOptimizer), lines(nullptr) {
  ui->setupUi(this);
  ui->cb_mtype->addItem("Endpoints");
  ui->cb_mtype->addItem("Average");

  // Tooltips explaining the merge algorithm and each parameter.
  setToolTip(
      tr("Merge near-collinear line segments that likely belong to the "
         "same physical edge but were split during detection."));
  ui->spin_max_dist->setToolTip(
      tr("Maximum endpoint distance (px) between two segments to "
         "consider them for merging."));
  ui->spin_angle_error->setToolTip(
      tr("Maximum allowed angle difference (rad) between two "
         "candidate segments. Smaller values require segments to "
         "be more parallel."));
  ui->spin_distance_error->setToolTip(
      tr("Maximum perpendicular distance (px) between the "
         "endpoints of one segment and the supporting line of "
         "the other."));
  ui->spin_parallel_error->setToolTip(
      tr("Maximum lateral (parallel) gap tolerance. Controls how "
         "much overlap or overshoot is allowed along the line "
         "direction."));
  ui->cb_mtype->setToolTip(
      tr("Merge strategy: 'Endpoints' connects the outermost "
         "endpoints; 'Average' fits a new segment to the "
         "averaged geometry."));
  ui->pb_corun->setToolTip(tr("Run the merge algorithm on the currently detected lines."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/continuityoptimizer/README.md");
}

ContinuityOptimizer::~ContinuityOptimizer() { delete ui; }

void ContinuityOptimizer::connectTools(ControlWindow* w) {
  lines = &w->getLines();
  connect(this, SIGNAL(linesMerged(const LineSegmentVector&)), w, SLOT(setLines(const LineSegmentVector&)));
}

void ContinuityOptimizer::runOptimizer() {
  if (lines == nullptr) return;

  try {
    LineMerge<double> lm(ui->spin_max_dist->value(), ui->spin_angle_error->value(), ui->spin_distance_error->value(),
                         ui->spin_parallel_error->value(),
                         ui->cb_mtype->currentIndex() == 0 ? MergeType::STANDARD : MergeType::AVG);
    LineMerge<double>::LineSegmentVector ml, dl;
    dl.reserve(lines->size());
    for (const auto& l : *lines) {
      dl.emplace_back(l.modSegment());
    }

    lm.merge_lines(dl, ml);

    // Convert from algorithm type (Vec2) back to app type (cv::Point_)
    LineSegmentVector result;
    result.reserve(ml.size());
    for (const auto& seg : ml) {
      result.emplace_back(seg);
    }
    emit linesMerged(result);
  } catch (const std::exception& ex) {
    std::cerr << "Merge optimization failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Merge Error"), tr("Line merge failed:\n%1").arg(ex.what()));
  }
}
