#include "analyseroptions.h"

#include "help_button.hpp"

AnalyserOptions::AnalyserOptions(QWidget* parent) : QMainWindow(parent), ui(new Ui::AnalyserOptions) {
  ui->setupUi(this);

  // Tooltips for all controls.
  setToolTip(tr("Configure image dimensions and scale for accuracy analysis."));
  ui->dsb_image_width->setToolTip(
      tr("Original full-resolution image width in pixels, "
         "before any downscaling was applied."));
  ui->dsb_image_height->setToolTip(
      tr("Original full-resolution image height in pixels, "
         "before any downscaling was applied."));
  ui->dsb_reduced_scale->setToolTip(
      tr("Factor by which the working image was downscaled "
         "from the original. Used to map coordinates back."));
  ui->dsb_offset->setToolTip(
      tr("Sub-pixel coordinate offset applied when mapping "
         "between scaled and original coordinates."));

  // Help button.
  addHelpButton(this, tr("Help \xe2\x80\x94 Analyser Options"),
                tr("<h3>Analyser Options</h3>"
                   "<p>Configure parameters for the 2D line analyzer "
                   "accuracy evaluation.</p>"
                   "<h4>Parameters</h4>"
                   "<ul>"
                   "<li><b>Orig. Image Width / Height:</b> The original image "
                   "dimensions before any scaling. Used to correctly map line "
                   "coordinates back to full resolution.</li>"
                   "<li><b>Reduced Scale:</b> The downscale factor applied to "
                   "the working image. Combined with image dimensions for "
                   "accuracy metrics at original scale.</li>"
                   "<li><b>Offset:</b> Sub-pixel coordinate offset applied "
                   "when mapping between scaled and original coordinates.</li>"
                   "</ul>"));
}

AnalyserOptions::~AnalyserOptions() { delete ui; }

/**
 * @brief   AnalyserOptions::getImageWidth
 *          Returns the image width
 * @return
 */
double AnalyserOptions::getImageWidth() { return ui->dsb_image_width->value(); }

/**
 * @brief   AnalyserOptions::getImageHeight
 *          Returns the image height
 * @return
 */
double AnalyserOptions::getImageHeight() { return ui->dsb_image_height->value(); }

/**
 * @brief   AnalyserOptions::getReducedScale
 *          Returns the reduced scale
 * @return
 */
double AnalyserOptions::getReducedScale() { return ui->dsb_reduced_scale->value(); }

/**
 * @brief   AnalyserOptions::getOffset
 *          Returns the offset
 * @return
 */
double AnalyserOptions::getOffset() { return ui->dsb_offset->value(); }
