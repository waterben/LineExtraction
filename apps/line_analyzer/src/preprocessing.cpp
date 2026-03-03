#include "preprocessing.h"

#include "help_button.hpp"

PreProcessing::PreProcessing(QWidget* parent) : QMainWindow(parent), ui(new Ui::PreProcessing) {
  ui->setupUi(this);
  ui->cb_interp->addItem("nearest");
  ui->cb_interp->addItem("bilinear");
  ui->cb_interp->addItem("area");
  ui->cb_interp->addItem("bicubic");
  ui->cb_interp->addItem("lanczons");
  ui->cb_interp->setCurrentIndex(1);

  // Tooltips for all controls.
  setToolTip(tr("Pre-process the loaded image before line detection."));
  ui->chb_scale->setToolTip(
      tr("Enable image rescaling by the given factor before "
         "processing. Useful for downscaling high-res images."));
  ui->spin_scale->setToolTip(tr("Scale factor (e.g. 0.5 = half size, 2.0 = double)."));
  ui->label_interp->setToolTip(tr("Interpolation method used for resizing."));
  ui->cb_interp->setToolTip(
      tr("Resize interpolation: nearest, bilinear, area, "
         "bicubic, or Lanczos."));
  ui->chb_gray->setToolTip(
      tr("Convert color (BGR) images to single-channel "
         "grayscale. Required by most line detectors."));
  ui->chb_noise->setToolTip(
      tr("Add synthetic Gaussian noise to the image. "
         "Useful for testing detector robustness."));
  ui->spin_noise->setToolTip(tr("Standard deviation of the Gaussian noise (1-200)."));
  ui->chb_blur->setToolTip(
      tr("Apply Gaussian blur before detection. A small "
         "sigma (0.3-1.0) suppresses pixel noise."));
  ui->spin_blur->setToolTip(tr("Gaussian blur sigma in pixels (0.1-10.0)."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "README.md", "usage");
}

PreProcessing::~PreProcessing() { delete ui; }

void PreProcessing::scaleChange(int s) {
  ui->spin_scale->setEnabled(s != 0);
  ui->label_interp->setEnabled(s != 0);
  ui->cb_interp->setEnabled(s != 0);
}

void PreProcessing::blurChange(int s) { ui->spin_blur->setEnabled(s != 0); }

void PreProcessing::noiseChange(int s) { ui->spin_noise->setEnabled(s != 0); }
