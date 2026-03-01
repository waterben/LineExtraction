#include "image_analyzer_panel.h"

#include "help_button.hpp"
#include <algorithm/image_analyzer.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <iostream>

ImageAnalyzerPanel::ImageAnalyzerPanel(QWidget* parent)
    : LATool("Image Analyzer", parent), ui(new Ui::ImageAnalyzerPanel) {
  setWindowTitle("Image Analyzer");
  ui->setupUi(this);

  // Tooltips for all controls.
  setToolTip(
      tr("Analyze the loaded image to compute quantitative "
         "properties and suggest detector profile settings."));
  ui->lbl_contrast->setToolTip(
      tr("Michelson contrast: (max-min)/(max+min) of pixel "
         "intensities. Range 0\xe2\x80\x93"
         "1."));
  ui->lbl_noise->setToolTip(
      tr("Estimated noise standard deviation using the Median "
         "Absolute Deviation of high-frequency wavelet coefficients."));
  ui->lbl_edge_density->setToolTip(
      tr("Fraction of pixels classified as edges relative "
         "to the total number of pixels."));
  ui->lbl_dynamic_range->setToolTip(
      tr("Ratio of the effective intensity range to the "
         "theoretical maximum (typically 255)."));
  ui->lbl_detail->setToolTip(tr("Suggested Detail slider value for the Detector Profile."));
  ui->lbl_gap_tolerance->setToolTip(tr("Suggested Gap Tolerance slider value."));
  ui->lbl_min_length->setToolTip(tr("Suggested Min Length slider value."));
  ui->lbl_precision->setToolTip(tr("Suggested Precision slider value."));
  ui->lbl_contrast_factor->setToolTip(tr("Suggested contrast-dependent threshold multiplier."));
  ui->lbl_noise_factor->setToolTip(tr("Suggested noise-dependent threshold multiplier."));
  ui->pb_analyze->setToolTip(tr("Run image analysis and update all property fields."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/image_analyzer/README.md");
}

ImageAnalyzerPanel::~ImageAnalyzerPanel() { delete ui; }

void ImageAnalyzerPanel::connectTools(Analyzer* w) {
  srcImg = &w->getSrcImg();
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(analyze()));
}

void ImageAnalyzerPanel::analyze() {
  if (srcImg == nullptr || srcImg->empty()) return;

  try {
    cv::Mat gray;
    if (srcImg->channels() != 1) {
      cv::cvtColor(*srcImg, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = *srcImg;
    }
    auto props = lsfm::ImageAnalyzer::analyze(gray);

    std::cout << "Image analysis: contrast=" << props.contrast << ", noise=" << props.noise_level
              << ", edge_density=" << props.edge_density << ", dynamic_range=" << props.dynamic_range << std::endl;

    // Display image properties
    ui->lbl_contrast->setText(QString::number(props.contrast, 'f', 3));
    ui->lbl_noise->setText(QString::number(props.noise_level, 'f', 3));
    ui->lbl_edge_density->setText(QString::number(props.edge_density, 'f', 3));
    ui->lbl_dynamic_range->setText(QString::number(props.dynamic_range, 'f', 3));

    // Display suggested profile hints
    auto hints = props.suggest_profile();
    ui->lbl_detail->setText(QString::number(hints.detail, 'f', 1) + "%");
    ui->lbl_gap_tolerance->setText(QString::number(hints.gap_tolerance, 'f', 1) + "%");
    ui->lbl_min_length->setText(QString::number(hints.min_length, 'f', 1) + "%");
    ui->lbl_precision->setText(QString::number(hints.precision, 'f', 1) + "%");
    ui->lbl_contrast_factor->setText(QString::number(hints.contrast_factor, 'f', 2));
    ui->lbl_noise_factor->setText(QString::number(hints.noise_factor, 'f', 2));
  } catch (const std::exception& ex) {
    std::cerr << "Image analysis failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Analysis Error"), tr("Image analysis failed:\n%1").arg(ex.what()));
  }
}
