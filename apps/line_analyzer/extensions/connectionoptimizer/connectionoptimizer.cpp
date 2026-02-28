#include "connectionoptimizer.h"

#include "help_button.hpp"
#include <algorithm/line_connect.hpp>

#include <QMessageBox>


using namespace lsfm;

ConnectionOptimizer::ConnectionOptimizer(QWidget* parent)
    : LATool("Connection Optimizer", parent),
      ui(new Ui::ConnectionOptimizer),
      lines(nullptr),
      sources(nullptr),
      srcImg(nullptr) {
  ui->setupUi(this);

  // Tooltips explaining the connection algorithm and each parameter.
  setToolTip(
      tr("Connect nearby line segment endpoints when the gradient "
         "magnitude along the connecting path is strong enough."));
  ui->connection_max_radius->setToolTip(
      tr("Maximum distance (px) between two endpoints to "
         "consider a connection. Larger values find more "
         "candidates but are slower."));
  ui->connection_accuracy->setToolTip(
      tr("Sampling step (px) along the connecting path. Smaller "
         "values sample more densely for a more accurate "
         "gradient check."));
  ui->connection_threshold->setToolTip(
      tr("Minimum average gradient magnitude along the "
         "connecting path. Connections below this threshold are "
         "rejected."));
  ui->pb_coprun->setToolTip(tr("Run the connection algorithm on the currently detected lines."));

  // Help button.
  addHelpButton(this, tr("Help \xe2\x80\x94 Connection Optimizer"),
                tr("<h3>Connection Optimizer</h3>"
                   "<p>Connects nearby line segment endpoints by sampling the "
                   "gradient magnitude along the shortest path between them.</p>"
                   "<h4>How it works</h4>"
                   "<ol>"
                   "<li>For each pair of endpoints within the search radius, "
                   "the algorithm samples the gradient magnitude along the "
                   "straight line connecting them.</li>"
                   "<li>If the average magnitude exceeds the threshold, the "
                   "segments are connected into a single longer segment.</li>"
                   "</ol>"
                   "<h4>Parameters</h4>"
                   "<ul>"
                   "<li><b>Max. Radius:</b> Search radius in pixels. Only "
                   "endpoint pairs closer than this are considered.</li>"
                   "<li><b>Accuracy:</b> Sampling step (px) along the connecting "
                   "path. Smaller = more samples = more accurate but slower.</li>"
                   "<li><b>Threshold:</b> Minimum average gradient magnitude. "
                   "Increase to reject weak connections.</li>"
                   "</ul>"));
}

ConnectionOptimizer::~ConnectionOptimizer() { delete ui; }

void ConnectionOptimizer::connectTools(ControlWindow* w) {
  lines = &w->getLines();
  srcImg = &w->getSrcImg();
  sources = &w->getSources();
  connect(this, SIGNAL(linesConnected(const LineSegmentVector&)), w, SLOT(setLines(const LineSegmentVector&)));
}

void ConnectionOptimizer::runOptimizer() {
  if (lines == nullptr || lines->empty()) return;

  try {
    cv::Mat qmag;
    cv::Mat nmag;
    cv::Mat mag;
    for (const auto& s : *sources) {
      if (s.name == "qmag") {
        qmag = s.data;
      } else if (s.name == "nmag") {
        nmag = s.data;
      } else if (s.name == "mag") {
        mag = s.data;
      }
    }

    // Need at least one magnitude source or a valid source image for fallback.
    if (qmag.empty() && nmag.empty() && mag.empty() && (srcImg == nullptr || srcImg->empty())) {
      return;
    }

    LineConnect<double> lc(ui->connection_max_radius->value(), ui->connection_accuracy->value(),
                           ui->connection_threshold->value());
    LineConnect<double>::LineSegmentVector ml, dl;
    dl.reserve(lines->size());
    for (const auto& l : *lines) {
      dl.emplace_back(l.modSegment());
    }

    if (!qmag.empty()) {
      lc.connect_lines(dl, ml, qmag);
    } else if (!nmag.empty()) {
      lc.connect_lines(dl, ml, nmag);
    } else if (!mag.empty()) {
      lc.connect_lines(dl, ml, mag);
    } else {
      lc.connect_lines(dl, ml, *srcImg, mag);
    }

    // Convert from algorithm type (Vec2) back to app type (cv::Point_)
    LineSegmentVector result;
    result.reserve(ml.size());
    for (const auto& seg : ml) {
      result.emplace_back(seg);
    }
    emit linesConnected(result);
  } catch (const std::exception& ex) {
    std::cerr << "Connection optimization failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Connection Error"), tr("Line connection failed:\n%1").arg(ex.what()));
  }
}
