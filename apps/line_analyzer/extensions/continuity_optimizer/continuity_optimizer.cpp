#include "continuity_optimizer.h"

#include "help_button.hpp"
#include <algorithm/line_continuity.hpp>

#include <QMessageBox>
#include <iostream>

using namespace lsfm;

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

ContinuityOptimizer::ContinuityOptimizer(QWidget* parent)
    : LATool("Continuity Optimizer", parent),
      ui(new Ui::ContinuityOptimizer),
      lines(nullptr),
      sources(nullptr),
      srcImg(nullptr) {
  ui->setupUi(this);

  // Merge type combo box
  ui->cb_mtype->addItem("Endpoints");
  ui->cb_mtype->addItem("Average");

  // Gradient section starts disabled
  ui->cb_gradient->setChecked(false);
  onGradientToggled(false);

  connect(ui->cb_gradient, &QCheckBox::toggled, this, &ContinuityOptimizer::onGradientToggled);
  connect(ui->pb_run, &QPushButton::clicked, this, &ContinuityOptimizer::run);

  // --- Panel tooltip ---
  setToolTip(
      tr("Optimize line segment continuity: merge collinear "
         "fragments and optionally bridge gaps using gradient "
         "magnitude evidence."));

  // --- Merge parameter tooltips ---
  ui->spin_max_dist->setToolTip(
      tr("Maximum endpoint distance (px) between two segments to "
         "consider them for merging or gradient connection."));
  ui->spin_angle_error->setToolTip(
      tr("Maximum allowed angle difference (deg) between two "
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

  // --- Gradient parameter tooltips ---
  ui->cb_gradient->setToolTip(
      tr("Enable gradient-based connection: after merging, also "
         "bridge nearby gaps where gradient magnitude along the "
         "connecting path is strong enough. Uses Max. Distance "
         "and Angle Error from the merge constraints."));
  ui->spin_accuracy->setToolTip(
      tr("Sampling step (px) along the connecting path. Smaller "
         "values sample more densely for a more accurate "
         "gradient check."));
  ui->spin_threshold->setToolTip(
      tr("Minimum average gradient magnitude along the "
         "connecting path. Connections below this threshold are "
         "rejected."));

  // --- Button tooltip ---
  ui->pb_run->setToolTip(
      tr("Run the optimizer: merge collinear segments, then "
         "optionally connect via gradient evidence."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/continuity_optimizer/README.md");
}

ContinuityOptimizer::~ContinuityOptimizer() { delete ui; }

// ---------------------------------------------------------------------------
// Tool wiring
// ---------------------------------------------------------------------------

void ContinuityOptimizer::connectTools(Analyzer* w) {
  lines = &w->getLines();
  srcImg = &w->getSrcImg();
  sources = &w->getSources();
  connect(this, SIGNAL(linesProcessed(const LineSegmentVector&)), w, SLOT(setLines(const LineSegmentVector&)));
}

// ---------------------------------------------------------------------------
// Gradient checkbox toggle
// ---------------------------------------------------------------------------

void ContinuityOptimizer::onGradientToggled(bool checked) {
  ui->label_accuracy->setEnabled(checked);
  ui->spin_accuracy->setEnabled(checked);
  ui->label_threshold->setEnabled(checked);
  ui->spin_threshold->setEnabled(checked);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

/// @brief Find best gradient magnitude source from image sources.
cv::Mat find_magnitude(const ImageSources& sources) {
  cv::Mat qmag;
  cv::Mat nmag;
  cv::Mat mag;
  for (const auto& s : sources) {
    if (s.name == "qmag") {
      qmag = s.data;
    } else if (s.name == "nmag") {
      nmag = s.data;
    } else if (s.name == "mag") {
      mag = s.data;
    }
  }
  if (!qmag.empty()) return qmag;
  if (!nmag.empty()) return nmag;
  return mag;
}

}  // namespace

// ---------------------------------------------------------------------------
// Slot
// ---------------------------------------------------------------------------

void ContinuityOptimizer::run() {
  if (lines == nullptr || lines->empty()) return;

  try {
    const double max_dist = ui->spin_max_dist->value();
    const double angle_error = ui->spin_angle_error->value();
    const double distance_error = ui->spin_distance_error->value();
    const double parallel_error = ui->spin_parallel_error->value();
    const auto mt = ui->cb_mtype->currentIndex() == 0 ? ContinuityMergeType::STANDARD : ContinuityMergeType::AVG;

    // Convert input
    LineContinuityOptimizer<double>::LineSegmentVector dl;
    dl.reserve(lines->size());
    for (const auto& l : *lines) {
      dl.emplace_back(l.modSegment());
    }

    LineContinuityOptimizer<double>::LineSegmentVector ml;

    if (ui->cb_gradient->isChecked()) {
      // Variant 2: gradient-assisted merge
      if (sources == nullptr || srcImg == nullptr) {
        QMessageBox::warning(this, tr("Gradient Unavailable"),
                             tr("No gradient data available. Run a detector first "
                                "to produce gradient sources, or disable the "
                                "gradient connection checkbox."));
        return;
      }

      const double accuracy = ui->spin_accuracy->value();
      const double threshold = ui->spin_threshold->value();

      LineContinuityOptimizer<double> opt(max_dist, angle_error, distance_error, parallel_error, mt, accuracy,
                                          threshold);

      cv::Mat magnitude = find_magnitude(*sources);
      if (!magnitude.empty()) {
        opt.optimize(dl, ml, magnitude);
      } else {
        opt.optimize(dl, ml, *srcImg, magnitude);
      }

      std::cout << "Continuity optimizer (gradient): " << lines->size() << " -> " << ml.size() << " segments"
                << std::endl;
    } else {
      // Variant 1: geometry-only merge
      LineContinuityOptimizer<double> opt(max_dist, angle_error, distance_error, parallel_error, mt);
      opt.optimize(dl, ml);

      std::cout << "Continuity optimizer (merge): " << lines->size() << " -> " << ml.size() << " segments" << std::endl;
    }

    // Convert output
    LineSegmentVector result;
    result.reserve(ml.size());
    for (const auto& seg : ml) {
      result.emplace_back(seg);
    }

    emit linesProcessed(result);
  } catch (const std::exception& ex) {
    std::cerr << "Continuity optimization failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Optimization Error"), tr("Continuity optimization failed:\n%1").arg(ex.what()));
  }
}
