#include "connectionoptimizer.h"

#include <utility/line_connect.hpp>

using namespace lsfm;

ConnectionOptimizer::ConnectionOptimizer(QWidget* parent)
    : LATool("Connection Optimizer", parent), ui(new Ui::ConnectionOptimizer), lines(0), srcImg(0) {
  ui->setupUi(this);
  // ui->cb_mtype->addItem("Endpoints");
  // ui->cb_mtype->addItem("Average");
}

ConnectionOptimizer::~ConnectionOptimizer() { delete ui; }

void ConnectionOptimizer::connectTools(ControlWindow* w) {
  lines = &w->getLines();
  srcImg = &w->getSrcImg();
  sources = &w->getSources();
  connect(this, SIGNAL(linesConnected(const std::vector<lsfm::LineSegment2d>&)), w,
          SLOT(setLines(const std::vector<lsfm::LineSegment2d>&)));
}

void ConnectionOptimizer::runOptimizer() {
  if (lines == 0) return;

  cv::Mat qmag;
  cv::Mat nmag;
  cv::Mat mag;
  auto c = sources->begin();
  while (c != sources->end()) {
    if (c->name == "qmag") {
      qmag = c->data;
    } else if (c->name == "nmag") {
      nmag = c->data;
    } else if (c->name == "mag") {
      mag = c->data;
    }
    c++;
  }

  //    LineConnect<double> lm(ui->spin_max_dist->value(), ui->spin_angle_error->value(),
  //    ui->spin_distance_error->value(), ui->spin_parallel_error->value(), ui->cb_mtype->currentIndex() == 0 ?
  //    MergeType::STANDARD : MergeType::AVG);
  LineConnect<double> lc(ui->connection_max_radius->value(), ui->connection_accuracy->value(),
                         ui->connection_threshold->value());
  LineConnect<double>::LineSegmentVector ml, dl;
  dl.reserve(lines->size());
  for_each(lines->begin(), lines->end(), [&](const ControlWindow::LineSegment& l) { dl.push_back(l.modSegment()); });

  if (!qmag.empty()) {
    lc.connect_lines(dl, ml, qmag);
  } else if (!nmag.empty()) {
    lc.connect_lines(dl, ml, nmag);
  } else if (!mag.empty()) {
    lc.connect_lines(dl, ml, mag);
  } else {
    lc.connect_lines(dl, ml, *srcImg, mag);
  }

  emit linesConnected(ml);
}
