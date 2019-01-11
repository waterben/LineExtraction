#ifndef ANALYSEROPTIONS_H
#define ANALYSEROPTIONS_H

#include <QMainWindow>
#include "ui_analyseroptions.h"

/**
 * @brief   The AnalyserOptions class
 *          Preset Options for Ground Truth Data
 */
class AnalyserOptions : public QMainWindow
{
    Q_OBJECT

public:
    explicit AnalyserOptions(QWidget * parent = 0);
    ~AnalyserOptions();

    Ui::AnalyserOptions *ui;    /* Analyser Options UI */

    double getImageWidth();     /* returns the width of image */
    double getImageHeight();    /* returns the height of image */
    double getReducedScale();   /* returns reduced scale */
    double getOffset();         /* returns offset */

public slots:

};

#endif // ANALYSEROPTIONS_H
