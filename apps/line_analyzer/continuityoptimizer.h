#ifndef CONTINUITYOPTIMIZER_H
#define CONTINUITYOPTIMIZER_H

#include "controlwindow.h"
#include "ui_continuityoptimizer.h"

class ContinuityOptimizer : public LATool
{
    Q_OBJECT
    
    Ui::ContinuityOptimizer *ui;
    const ControlWindow::LineSegmentVector *lines;
    
public:
    typedef std::vector<lsfm::LineSegment2d> LineVector;

    explicit ContinuityOptimizer(QWidget *parent = 0);
    ~ContinuityOptimizer();

    void connectTools(ControlWindow *w);
    

public slots:
    void runOptimizer();

signals :
    void linesMerged(const std::vector<lsfm::LineSegment2d>& lines);
    
};

#endif // CONTINUITYOPTIMIZER_H
