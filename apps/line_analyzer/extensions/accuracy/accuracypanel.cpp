#include "accuracypanel.h"

#include "help_button.hpp"

#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <cstdlib>
#include <filesystem>
#include <iostream>

AccuracyPanel::AccuracyPanel(QWidget* parent) : LATool("Accuracy Measure", parent), ui(new Ui::AccuracyPanel) {
  ui->setupUi(this);

  // Tooltips for the panel and its controls.
  setToolTip(
      tr("Compare detected lines against ground truth to compute "
         "precision, recall, F1-score, and structural Average Precision (sAP)."));
  ui->edit_gt_path->setToolTip(tr("Path to the loaded ground truth CSV file."));
  ui->pb_browse->setToolTip(tr("Open a file dialog to select a ground truth CSV."));
  ui->pb_load_example->setToolTip(
      tr("Load the bundled example ground truth (example_gt.csv) and "
         "the matching example_lines.png image for quick evaluation."));
  ui->spin_threshold->setToolTip(
      tr("Maximum endpoint distance (px) for a detected segment "
         "to count as a true positive match against a GT segment."));
  ui->edit_image_name->setToolTip(
      tr("If the CSV contains multiple images, enter the image name "
         "to select the matching ground truth entry. Leave empty to "
         "auto-select (works when the CSV has only one entry)."));
  ui->pb_evaluate->setToolTip(tr("Run the evaluation on currently detected lines vs loaded GT."));
  ui->pb_clear->setToolTip(tr("Clear all results and unload the ground truth data."));

  // Tooltips for result labels.
  ui->lbl_precision->setToolTip(tr("TP / (TP + FP) \xe2\x80\x94 fraction of detected segments matching GT."));
  ui->lbl_recall->setToolTip(tr("TP / (TP + FN) \xe2\x80\x94 fraction of GT segments that were detected."));
  ui->lbl_f1->setToolTip(tr("Harmonic mean of precision and recall."));
  ui->lbl_sap->setToolTip(tr("Structural Average Precision \xe2\x80\x94 area under the precision-recall curve."));
  ui->lbl_counts->setToolTip(tr("True Positives / False Positives / False Negatives."));
  ui->lbl_gt_count->setToolTip(tr("Total number of ground truth segments loaded from the CSV."));

  // Help button.
  addHelpButton(this, tr("Help \xe2\x80\x94 Accuracy Measure"),
                tr("<h3>Accuracy Measure</h3>"
                   "<p>Evaluates the quality of detected line segments by "
                   "comparing them against ground truth annotations loaded "
                   "from a CSV file.</p>"
                   "<h4>Ground Truth</h4>"
                   "<ul>"
                   "<li><b>Browse:</b> Load a CSV with GT segment annotations "
                   "(x1, y1, x2, y2 columns, optionally per image name).</li>"
                   "</ul>"
                   "<h4>Settings</h4>"
                   "<ul>"
                   "<li><b>Match Threshold:</b> Maximum endpoint distance (px) "
                   "for a true positive match.</li>"
                   "<li><b>Image Name Filter:</b> Select GT entry by image name. "
                   "Leave empty to auto-select (single-entry CSV).</li>"
                   "</ul>"
                   "<h4>Results</h4>"
                   "<ul>"
                   "<li><b>Precision:</b> TP / (TP + FP).</li>"
                   "<li><b>Recall:</b> TP / (TP + FN).</li>"
                   "<li><b>F1 Score:</b> Harmonic mean of precision and recall.</li>"
                   "<li><b>sAP:</b> Structural Average Precision.</li>"
                   "<li><b>TP / FP / FN:</b> Match count breakdown.</li>"
                   "<li><b>GT Segments:</b> Total ground truth count.</li>"
                   "</ul>"));
}

AccuracyPanel::~AccuracyPanel() { delete ui; }

void AccuracyPanel::connectTools(ControlWindow* w) { ctrl = w; }

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void AccuracyPanel::browseGroundTruth() {
  QString path = QFileDialog::getOpenFileName(this, tr("Select Ground Truth CSV"), QString(),
                                              tr("CSV Files (*.csv);;All Files (*)"));
  if (path.isEmpty()) return;

  try {
    gt_entries = lsfm::GroundTruthLoader::load_csv(path.toStdString());
    ui->edit_gt_path->setText(path);

    int total_segs = 0;
    for (const auto& entry : gt_entries) {
      total_segs += static_cast<int>(entry.segments.size());
    }
    setStatus(QString("Loaded %1 entries, %2 segments.").arg(gt_entries.size()).arg(total_segs));
  } catch (const std::exception& ex) {
    setStatus(QString("Load failed: %1").arg(ex.what()));
    gt_entries.clear();
  }
}

void AccuracyPanel::loadExampleGroundTruth() {
  // Locate the bundled example ground truth CSV.
  std::string csv_path = findResourcePath("datasets/ground_truth/example_gt.csv");
  if (csv_path.empty()) {
    setStatus("Example ground truth not found.");
    QMessageBox::warning(this, tr("Example Ground Truth"),
                         tr("Could not locate the bundled example_gt.csv.\n"
                            "Make sure you are running from the workspace directory "
                            "or via 'bazel run'."));
    return;
  }

  try {
    gt_entries = lsfm::GroundTruthLoader::load_csv(csv_path);
    ui->edit_gt_path->setText(QString::fromStdString(csv_path));
    ui->edit_image_name->setText("example_lines.png");

    int total_segs = 0;
    for (const auto& entry : gt_entries) {
      total_segs += static_cast<int>(entry.segments.size());
    }
    setStatus(QString("Loaded example: %1 segments.").arg(total_segs));
  } catch (const std::exception& ex) {
    setStatus(QString("Load failed: %1").arg(ex.what()));
    gt_entries.clear();
    return;
  }

  // Also load the matching example image into the ControlWindow.
  if (ctrl != nullptr) {
    std::string img_path = findResourcePath("example_lines.png");
    if (!img_path.empty()) {
      ctrl->setImagePath(QString::fromStdString(img_path));
    }
  }
}

void AccuracyPanel::evaluate() {
  if (ctrl == nullptr) {
    setStatus("Not connected.");
    return;
  }
  if (gt_entries.empty()) {
    setStatus("No ground truth loaded.");
    return;
  }

  const auto& app_lines = ctrl->getLines();
  if (app_lines.empty()) {
    setStatus("No detected lines.");
    return;
  }

  const auto* gt_segs = findGroundTruth();
  if (gt_segs == nullptr || gt_segs->empty()) {
    setStatus("No matching ground truth found.");
    return;
  }

  try {
    // Convert app lines (cv::Point_ based) to Vec2-based LineSegment<double>.
    auto detected = convertLines(app_lines);

    double threshold = ui->spin_threshold->value();
    lsfm::AccuracyMeasure<double> measure(threshold);
    auto result = measure.evaluate(detected, *gt_segs);
    double sap = measure.structural_ap(detected, *gt_segs);

    // Display results.
    ui->lbl_precision->setText(QString::number(result.precision, 'f', 4));
    ui->lbl_recall->setText(QString::number(result.recall, 'f', 4));
    ui->lbl_f1->setText(QString::number(result.f1, 'f', 4));
    ui->lbl_sap->setText(QString::number(sap, 'f', 4));
    ui->lbl_counts->setText(
        QString("%1 / %2 / %3").arg(result.true_positives).arg(result.false_positives).arg(result.false_negatives));
    ui->lbl_gt_count->setText(QString::number(gt_segs->size()));

    setStatus(QString("Evaluated %1 detected vs %2 GT segments.").arg(detected.size()).arg(gt_segs->size()));
  } catch (const std::exception& ex) {
    std::cerr << "Evaluation failed: " << ex.what() << std::endl;
    setStatus(QString("Evaluation failed: %1").arg(ex.what()));
    QMessageBox::warning(this, tr("Evaluation Error"), tr("Accuracy evaluation failed:\n%1").arg(ex.what()));
  }
}

void AccuracyPanel::clearResults() {
  ui->lbl_precision->setText("-");
  ui->lbl_recall->setText("-");
  ui->lbl_f1->setText("-");
  ui->lbl_sap->setText("-");
  ui->lbl_counts->setText("- / - / -");
  ui->lbl_gt_count->setText("-");
  ui->edit_gt_path->clear();
  gt_entries.clear();
  setStatus("");
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

const std::vector<lsfm::LineSegment<double>>* AccuracyPanel::findGroundTruth() const {
  if (gt_entries.empty()) return nullptr;

  // If user specified an image name filter, use it.
  QString filter = ui->edit_image_name->text().trimmed();
  if (!filter.isEmpty()) {
    std::string filter_str = filter.toStdString();
    for (const auto& entry : gt_entries) {
      if (entry.image_name == filter_str) {
        return &entry.segments;
      }
    }
    return nullptr;
  }

  // If only one entry in the CSV, use it directly.
  if (gt_entries.size() == 1) {
    return &gt_entries.front().segments;
  }

  // No filter and multiple entries — cannot determine which to use.
  return nullptr;
}

std::vector<lsfm::LineSegment<double>> AccuracyPanel::convertLines(const ControlWindow::LineVector& app_lines) {
  std::vector<lsfm::LineSegment<double>> result;
  result.reserve(app_lines.size());
  for (const auto& line : app_lines) {
    // Use the converting copy constructor: LineSegment<double, cv::Point_> -> LineSegment<double, Vec2>.
    result.emplace_back(line.segment);
  }
  return result;
}

void AccuracyPanel::setStatus(const QString& msg) { ui->lbl_status->setText(msg); }

// ---------------------------------------------------------------------------
// Resource path resolution
// ---------------------------------------------------------------------------

std::string AccuracyPanel::findResourcePath(const std::string& relative_path) {
  namespace fs = std::filesystem;

  // Candidate base directories relative to cwd (mirrors ControlWindow::findResources).
  const std::vector<std::string> candidates = {
      "resources",
      "../resources",
      "../../resources",
      "../../../resources",
  };
  for (const auto& base : candidates) {
    fs::path p = fs::absolute(fs::path(base) / relative_path);
    if (fs::exists(p)) return p.string();
  }

  // Bazel: BUILD_WORKSPACE_DIRECTORY (set by `bazel run`).
  if (const char* ws_dir = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
    fs::path p = fs::path(ws_dir) / "resources" / relative_path;
    if (fs::exists(p)) return p.string();
  }

  // Bazel: RUNFILES_DIR (hermetic builds).
  if (const char* rd = std::getenv("RUNFILES_DIR")) {
    for (const auto& repo : {"_main", "line_extraction"}) {
      fs::path p = fs::path(rd) / repo / "resources" / relative_path;
      if (fs::exists(p)) return p.string();
    }
  }

  return {};
}
