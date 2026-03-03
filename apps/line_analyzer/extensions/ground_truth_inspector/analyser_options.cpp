#include "analyser_options.h"

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

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/ground_truth_inspector/README.md", "workflow");
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
