#pragma once

#include "ui_analyser_options.h"

#include <QMainWindow>

/**
 * @brief   The AnalyserOptions class
 *          Preset Options for Ground Truth Data
 */
class AnalyserOptions : public QMainWindow {
  Q_OBJECT
  Q_DISABLE_COPY(AnalyserOptions)

 public:
  explicit AnalyserOptions(QWidget* parent = nullptr);
  ~AnalyserOptions();

  Ui::AnalyserOptions* ui; /* Analyser Options UI */

  double getImageWidth();   /* returns the width of image */
  double getImageHeight();  /* returns the height of image */
  double getReducedScale(); /* returns reduced scale */
  double getOffset();       /* returns offset */

 public slots:
};
