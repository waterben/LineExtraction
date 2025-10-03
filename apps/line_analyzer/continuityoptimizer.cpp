#include "continuityoptimizer.h"

#include <utility/line_merge.hpp>

using namespace lsfm;

ContinuityOptimizer::ContinuityOptimizer(QWidget* parent)
    : LATool("Continuity Optimizer", parent), ui(new Ui::ContinuityOptimizer), lines(0) {
  ui->setupUi(this);
  ui->cb_mtype->addItem("Endpoints");
  ui->cb_mtype->addItem("Average");
}

ContinuityOptimizer::~ContinuityOptimizer() { delete ui; }

void ContinuityOptimizer::connectTools(ControlWindow* w) {
  lines = &w->getLines();
  connect(this, SIGNAL(linesMerged(const std::vector<lsfm::LineSegment2d>&)), w,
          SLOT(setLines(const std::vector<lsfm::LineSegment2d>&)));
}

void ContinuityOptimizer::runOptimizer() {
  if (lines == 0) return;

  LineMerge<double> lm(ui->spin_max_dist->value(), ui->spin_angle_error->value(), ui->spin_distance_error->value(),
                       ui->spin_parallel_error->value(),
                       ui->cb_mtype->currentIndex() == 0 ? MergeType::STANDARD : MergeType::AVG);
  LineMerge<double>::LineSegmentVector ml, dl;
  dl.reserve(lines->size());
  for_each(lines->begin(), lines->end(), [&](const ControlWindow::LineSegment& l) { dl.push_back(l.modSegment()); });

  lm.merge_lines(dl, ml);  //, ui->spin_max_dist->value(), ui->spin_angle_error->value(),
                           // ui->spin_distance_error->value(), ui->spin_parallel_error->value(),
                           // ui->cb_mtype->currentIndex() == 0 ? MergeType::STANDARD : MergeType::AVG);
  emit linesMerged(ml);
}
