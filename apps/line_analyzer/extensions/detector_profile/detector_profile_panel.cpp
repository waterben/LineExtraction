#include "detector_profile_panel.h"

#include "help_button.hpp"
#include <algorithm/image_analyzer.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QString>
#include <iostream>
#include <utility>

DetectorProfilePanel::DetectorProfilePanel(QWidget* parent)
    : LATool("Detector Profile", parent), ui(new Ui::DetectorProfilePanel) {
  ui->setupUi(this);

  // Tooltips for the panel and its controls.
  setToolTip(
      tr("High-level detector tuning via 4 percentage knobs and 2 "
         "adaptive factors. Use 'Auto from Image' to derive settings "
         "from the current image, then 'Apply to Detector' to push them."));
  ui->slider_detail->setToolTip(
      tr("Controls detection granularity (0-100%%). Higher values "
         "produce more segments, including smaller features."));
  ui->slider_gap_tolerance->setToolTip(
      tr("How tolerant the detector is of gaps in edge chains "
         "(0-100%%). Higher values bridge larger gaps."));
  ui->slider_min_length->setToolTip(
      tr("Minimum segment length percentage (0-100%%). Higher "
         "values discard shorter segments."));
  ui->slider_precision->setToolTip(
      tr("Sub-pixel precision emphasis (0-100%%). Higher values "
         "tighten fitting tolerances for more accurate endpoints."));
  ui->spin_contrast_factor->setToolTip(
      tr("Multiplier applied to contrast-dependent thresholds. "
         "Values > 1 raise thresholds; < 1 lower them."));
  ui->spin_noise_factor->setToolTip(
      tr("Multiplier for noise-related thresholds. Increase for "
         "noisier images; decrease for clean images."));
  ui->pb_auto->setToolTip(
      tr("Analyze the current source image and set all knobs and "
         "factors to values suggested by ImageAnalyzer."));
  ui->pb_apply->setToolTip(tr("Apply the current profile to the active detector."));
  ui->pb_reset->setToolTip(tr("Reset all knobs to 50%% and factors to 1.0."));

  // Image Properties group tooltips.
  ui->lbl_img_contrast->setToolTip(tr("Michelson contrast of the source image."));
  ui->lbl_img_noise->setToolTip(tr("Estimated noise standard deviation."));
  ui->lbl_img_edge_density->setToolTip(tr("Fraction of edge pixels relative to total pixels."));
  ui->lbl_img_dynamic_range->setToolTip(tr("Ratio of usable intensity range to theoretical maximum."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/detector_profile/README.md");
}

DetectorProfilePanel::~DetectorProfilePanel() { delete ui; }

void DetectorProfilePanel::connectTools(Analyzer* w) {
  ctrl = w;
  srcImg = &w->getSrcImg();
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(analyzeImage()));
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void DetectorProfilePanel::autoFromImage() {
  if (srcImg == nullptr || srcImg->empty()) {
    setStatus("No image loaded.");
    return;
  }

  try {
    double stime = double(cv::getTickCount());
    auto props = lsfm::ImageAnalyzer::analyze(*srcImg);
    auto hints = props.suggest_profile();
    double elapsed = (double(cv::getTickCount()) - stime) * 1000 / cv::getTickFrequency();

    // Push knob values to the UI (slider<->spinbox are already connected).
    ui->slider_detail->setValue(static_cast<int>(hints.detail));
    ui->slider_gap_tolerance->setValue(static_cast<int>(hints.gap_tolerance));
    ui->slider_min_length->setValue(static_cast<int>(hints.min_length));
    ui->slider_precision->setValue(static_cast<int>(hints.precision));
    ui->spin_contrast_factor->setValue(hints.contrast_factor);
    ui->spin_noise_factor->setValue(hints.noise_factor);

    std::cout << "Auto profile from image: detail=" << hints.detail << ", gap=" << hints.gap_tolerance
              << ", min_len=" << hints.min_length << ", precision=" << hints.precision
              << " (contrast_f=" << hints.contrast_factor << ", noise_f=" << hints.noise_factor << ") " << elapsed
              << "ms" << std::endl;
    setStatus("Profile derived from image.");
  } catch (const std::exception& ex) {
    std::cerr << "Auto profile failed: " << ex.what() << std::endl;
    setStatus(QString("Analyze failed: %1").arg(ex.what()));
  }
}

void DetectorProfilePanel::applyToDetector() {
  if (ctrl == nullptr) {
    setStatus("Not connected.");
    return;
  }

  DetectorPtr det = ctrl->getCurrentDetector();
  if (!det || !det->lsd) {
    setStatus("No active detector.");
    return;
  }

  QString det_name = ctrl->getCurrentDetectorName();
  auto [found, det_id] = resolveDetectorId(det_name);
  if (!found) {
    setStatus(QString("Unsupported detector: %1").arg(det_name));
    return;
  }

  try {
    auto profile = buildProfile();
    profile.apply(*det->lsd, det_id);
    ctrl->refreshDetectorOptions();
    std::cout << "Applied detector profile to " << det_name.toStdString() << " (detail=" << ui->slider_detail->value()
              << ", gap=" << ui->slider_gap_tolerance->value() << ", min_len=" << ui->slider_min_length->value()
              << ", precision=" << ui->slider_precision->value() << ", contrast_f=" << ui->spin_contrast_factor->value()
              << ", noise_f=" << ui->spin_noise_factor->value() << ")" << std::endl;
    setStatus(QString("Applied to %1.").arg(det_name));
  } catch (const std::exception& ex) {
    std::cerr << "Profile apply failed: " << ex.what() << std::endl;
    setStatus(QString("Apply failed: %1").arg(ex.what()));
  }
}

void DetectorProfilePanel::resetDefaults() {
  ui->slider_detail->setValue(50);
  ui->slider_gap_tolerance->setValue(50);
  ui->slider_min_length->setValue(50);
  ui->slider_precision->setValue(50);
  ui->spin_contrast_factor->setValue(1.0);
  ui->spin_noise_factor->setValue(1.0);
  setStatus("");
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::pair<bool, lsfm::DetectorId> DetectorProfilePanel::resolveDetectorId(const QString& name) {
  // The app uses display names like "LSD CC", "LSD CP", "LSD BURNS", etc.
  // Variant suffixes (e.g., "QFSt Odd", "SQ Odd", "RMG", "RCMG") use the
  // same base detector engine, so we strip to the first token after "LSD ".
  //
  // Names that have no DetectorId counterpart (e.g., "LSD ED", "LSD ES",
  // "LSD HOUGH") are reported as unsupported.
  static const struct {
    const char* prefix;
    lsfm::DetectorId id;
  } kMap[] = {
      {"LSD CC", lsfm::DetectorId::LSD_CC},         {"LSD CP", lsfm::DetectorId::LSD_CP},
      {"LSD BURNS", lsfm::DetectorId::LSD_BURNS},   {"LSD FBW", lsfm::DetectorId::LSD_FBW},
      {"LSD FGIOI", lsfm::DetectorId::LSD_FGIOI},   {"LSD EDLZ", lsfm::DetectorId::LSD_EDLZ},
      {"LSD EDTA", lsfm::DetectorId::LSD_EDLZ},  // EDTA maps to EDLZ profile
      {"LSD EL", lsfm::DetectorId::LSD_EL},         {"LSD EP", lsfm::DetectorId::LSD_EP},
      {"LSD HOUGHP", lsfm::DetectorId::LSD_HOUGHP},
  };

  for (const auto& entry : kMap) {
    if (name.startsWith(entry.prefix)) {
      return {true, entry.id};
    }
  }
  return {false, lsfm::DetectorId::LSD_CC};
}

lsfm::DetectorProfile DetectorProfilePanel::buildProfile() const {
  lsfm::DetectorProfile profile(
      static_cast<double>(ui->slider_detail->value()), static_cast<double>(ui->slider_gap_tolerance->value()),
      static_cast<double>(ui->slider_min_length->value()), static_cast<double>(ui->slider_precision->value()));
  profile.set_contrast_factor(ui->spin_contrast_factor->value());
  profile.set_noise_factor(ui->spin_noise_factor->value());
  return profile;
}

void DetectorProfilePanel::setStatus(const QString& msg) { ui->lbl_status->setText(msg); }

void DetectorProfilePanel::analyzeImage() {
  if (srcImg == nullptr || srcImg->empty()) {
    ui->lbl_img_contrast->setText(QString::fromUtf8("\xe2\x80\x94"));
    ui->lbl_img_noise->setText(QString::fromUtf8("\xe2\x80\x94"));
    ui->lbl_img_edge_density->setText(QString::fromUtf8("\xe2\x80\x94"));
    ui->lbl_img_dynamic_range->setText(QString::fromUtf8("\xe2\x80\x94"));
    return;
  }

  try {
    cv::Mat gray;
    if (srcImg->channels() != 1) {
      cv::cvtColor(*srcImg, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = *srcImg;
    }
    auto props = lsfm::ImageAnalyzer::analyze(gray);
    ui->lbl_img_contrast->setText(QString::number(props.contrast, 'f', 3));
    ui->lbl_img_noise->setText(QString::number(props.noise_level, 'f', 3));
    ui->lbl_img_edge_density->setText(QString::number(props.edge_density, 'f', 3));
    ui->lbl_img_dynamic_range->setText(QString::number(props.dynamic_range, 'f', 3));
  } catch (const std::exception& ex) {
    setStatus(QString("Image analysis failed: %1").arg(ex.what()));
  }
}
