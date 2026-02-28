#pragma once

#include "controlwindow.h"
#include "ui_accuracypanel.h"
#include <algorithm/accuracy_measure.hpp>
#include <algorithm/ground_truth.hpp>

#include <string>
#include <vector>

/// @brief Tool panel for evaluating line detection accuracy.
///
/// Loads ground truth line segments from a CSV file and compares them
/// against the currently detected lines from the ControlWindow.
/// Displays precision, recall, F1-score, sAP, and match counts.
///
/// A bundled example ground truth (example_gt.csv for example_lines.png)
/// can be loaded via the **Example** button for quick out-of-the-box testing.
class AccuracyPanel : public LATool {
  Q_OBJECT

  Ui::AccuracyPanel* ui{nullptr};
  ControlWindow* ctrl{nullptr};

  /// @brief Loaded ground truth entries (one per image in the CSV).
  std::vector<lsfm::GroundTruthEntry> gt_entries;

 public:
  explicit AccuracyPanel(QWidget* parent = nullptr);
  ~AccuracyPanel();

  void connectTools(ControlWindow* w) override;

 public slots:
  /// @brief Open a file dialog to select a ground truth CSV.
  void browseGroundTruth();

  /// @brief Load the bundled easy example ground truth (example_gt.csv).
  ///
  /// Searches standard resource locations (Bazel runfiles, relative paths)
  /// for the shipped example CSV. Also loads the matching example_lines.png
  /// image into the ControlWindow if found.
  void loadExampleGroundTruth();

  /// @brief Load the bundled challenge ground truth (example_challenge_gt.csv).
  ///
  /// The challenge scene has 8 shapes with varying contrast plus noise.
  /// Also loads the matching example_challenge.png into the ControlWindow.
  void loadChallengeGroundTruth();

  /// @brief Evaluate detected lines against the loaded ground truth.
  void evaluate();

  /// @brief Clear all result labels and loaded ground truth data.
  void clearResults();

 private:
  /// @brief Find ground truth segments for the current image.
  ///
  /// Uses the image-name filter field if non-empty, otherwise tries to match
  /// by the loaded filename. Falls back to the first entry if only one exists.
  ///
  /// @return Pointer to matching segments, or nullptr if not found.
  const std::vector<lsfm::LineSegment<double>>* findGroundTruth() const;

  /// @brief Convert app line segments to the Vec2-based type used by AccuracyMeasure.
  /// @param app_lines Lines from ControlWindow (cv::Point_ based).
  /// @return Converted line segments.
  static std::vector<lsfm::LineSegment<double>> convertLines(const ControlWindow::LineVector& app_lines);

  /// @brief Load a bundled example (CSV + image) by name.
  /// @param csv_relative CSV path relative to resources/ (e.g., "datasets/ground_truth/example_gt.csv").
  /// @param image_name Image filename in resources/ (e.g., "example_lines.png").
  /// @param label Human-readable label for status messages (e.g., "easy example").
  void loadBundledExample(const std::string& csv_relative, const std::string& image_name, const std::string& label);

  /// @brief Locate a resource file by searching standard paths.
  /// @param relative_path Path relative to the resources directory (e.g., "datasets/ground_truth/example_gt.csv").
  /// @return Resolved absolute path, or empty string if not found.
  static std::string findResourcePath(const std::string& relative_path);

  /// @brief Update the status label.
  /// @param msg Text to display.
  void setStatus(const QString& msg);
};
