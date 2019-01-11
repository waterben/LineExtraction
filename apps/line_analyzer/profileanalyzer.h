#ifndef PROFILEANALYZER_H
#define PROFILEANALYZER_H

#include <qplot/PlotWindow.h>
#include "latool.h"
#include "helpers.h"

namespace Ui {
class ProfileAnalyzer;
}

class ProfileAnalyzer : public LATool
{
    Q_OBJECT
    
    PlotWindow *plot;
    const ImageSources *sources;
    ControlWindow *cw;
    lsfm::LineSegment2d line;

    QVector<double> X, profile, std_dev, single_profile;
    double linePos, p_min, p_max, sp_min, sp_max;
    QCPGraph *profile_plot, *sprofile_plot, *std_dev_upper, *std_dev_lower;
    QCPItemLine *pa_indicator;
    
public:
    explicit ProfileAnalyzer(QWidget *parent = 0);
    ~ProfileAnalyzer();
    void connectTools(ControlWindow *w);

public slots:
    void updateSources(const ImageSources& src);
    void updateLine(const LineSegment& l);
    void updateLinePosSpin();
    void updateLinePosSlider();
    void updateX();
    void updatePlot();
    void updateProfile();
    void updateProfileLayer();
    void updateSProfile();
    void updateSProfileLayer();
    void fitProfile();

    
private:
    Ui::ProfileAnalyzer *ui;

    void createX();
    void createProfile();
    void createSProfile();

    void plotProfile();
    void plotSProfile();
};

#endif // PROFILEANALYZER_H
