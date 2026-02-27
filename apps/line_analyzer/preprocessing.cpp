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

  // Help button.
  addHelpButton(this, tr("Help \xe2\x80\x94 Image Pre-Processing"),
                tr("<h3>Image Pre-Processing</h3>"
                   "<p>Configure how the loaded image is transformed before "
                   "line detection.</p>"
                   "<h4>Parameters</h4>"
                   "<ul>"
                   "<li><b>Scale:</b> Resize by the given factor. "
                   "Useful for downscaling high-res images.</li>"
                   "<li><b>Interpolation:</b> Method used for resizing "
                   "(nearest, bilinear, area, bicubic, Lanczos).</li>"
                   "<li><b>Convert to Grayscale:</b> Convert color images to "
                   "single-channel grayscale, as required by most detectors.</li>"
                   "<li><b>Gaussian Noise:</b> Add synthetic noise with the "
                   "given standard deviation for robustness testing.</li>"
                   "<li><b>Gaussian Blur:</b> Blur with the given sigma. "
                   "A small blur (0.3\xe2\x80\x93"
                   "1.0) suppresses pixel noise.</li>"
                   "</ul>"));
}

PreProcessing::~PreProcessing() { delete ui; }

void PreProcessing::scaleChange(int s) {
  ui->spin_scale->setEnabled(s != 0);
  ui->label_interp->setEnabled(s != 0);
  ui->cb_interp->setEnabled(s != 0);
}

void PreProcessing::blurChange(int s) { ui->spin_blur->setEnabled(s != 0); }

void PreProcessing::noiseChange(int s) { ui->spin_noise->setEnabled(s != 0); }
