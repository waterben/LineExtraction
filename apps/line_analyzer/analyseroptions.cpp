#include "analyseroptions.h"

AnalyserOptions::AnalyserOptions(QWidget* parent) : QMainWindow(parent), ui(new Ui::AnalyserOptions) {
  ui->setupUi(this);
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
