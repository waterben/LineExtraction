#ifndef CONNECTIONOPTIMIZER_H
#define CONNECTIONOPTIMIZER_H

#include "controlwindow.h"
#include "ui_connectionoptimizer.h"

class ConnectionOptimizer : public LATool
{
    Q_OBJECT

    Ui::ConnectionOptimizer *ui;
    const ControlWindow::LineSegmentVector *lines;
    const ImageSources *sources;
    const cv::Mat *srcImg;

public:
    typedef std::vector<lsfm::LineSegment2d> LineVector;

    explicit ConnectionOptimizer(QWidget *parent = 0);
    ~ConnectionOptimizer();

    void connectTools(ControlWindow *w);


public slots:
    void runOptimizer();

signals :
    void linesConnected(const std::vector<lsfm::LineSegment2d>& lines);

};

#endif // CONTINUITYOPTIMIZER_H
