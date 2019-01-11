#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QColorDialog>
#include <qplot/PlotWindow.h>

#include "preprocessing.h"
#include "quiver.h"
#include "helpers.h"
#include "latool.h"


namespace Ui {
class ControlWindow;
}

class ControlWindow : public QMainWindow
{
    Q_OBJECT
    
    QFileDialog* file;
    PlotWindow *lplot;
    PreProcessing *pp;
	QColorDialog* cdia;
	Quiver *qo;
    cv::Mat img, src;
    int tools;
    
    ImageSources sources;
    QCPColorMap *imgMap;

    int lineSel;
    size_t inputSourcesSize;

    DetectorVector detectors;
    
    QCPItemLine *indicator;

public:

    struct LineMod {
        LineMod(float_type a = 0, float_type d = 0, float_type s = 0, float_type e = 0) : angle(a), distance(d), start(s), end(e) {}
        float_type angle, distance, start, end;

        inline bool modified() const {
            return angle != 0 || distance != 0 || start != 0 || end != 0;
        }

        inline void clear() {
            angle = 0;
            distance = 0;
            start = 0;
            end = 0;
        }
    };

    struct Line {
        Line(const LineSegment& l = LineSegment(), const LineMod& m = LineMod()) :
            segment(l), mod(m), line(0), normal(0) {}

        
        LineSegment segment;
        LineMod mod;
        
        inline LineSegment modSegment() const {
            LineSegment ret = segment;
            if (mod.modified()) {
                ret.translateOrtho(mod.distance);
                ret.rotate(mod.angle, ret.center());
                ret.translateStart(mod.start);
                ret.translateEnd(mod.end);
            }
            return ret;
        }

        inline void updateMod(const LineSegment& l) {
            mod.distance = l.originDist() - segment.originDist();
            mod.angle = l.angle() - segment.angle();
            mod.start = l.start() - segment.start();
            mod.end = l.end() - segment.end();
        }

        inline void freeze() {
            if (mod.modified()) {
                segment = modSegment();
                mod.clear();
            }
        }

        inline void reset() {
            mod.clear();
        }

        QCPItemLine *line, *normal;
    };

    typedef std::vector<Line> LineVector;

private:
    LineVector lines;
	LineVector quiver;
    
public:
    explicit ControlWindow(QWidget *parent = 0);
    ~ControlWindow();

    void addDetector(const DetectorPtr& detector);
    
    template<class TOOL>
    void addTool() {
        LATool *tool = new TOOL(this);
        addTool(tool);
    }

    void addTool(LATool *tool);
    const LineVector& getLines() const { return lines; }
    LineVector& getLines() { return lines; }
    int getLineSel() const { return lineSel;  }
    const ImageSources& getSources() const { return sources; }
    cv::Mat& getSrcImg() {return img; }
    PlotWindow* getPlotWindow() { return lplot;  }

public slots:
    // image stuff
    void selectImage();
    void openPreprocess();
    void loadImage();
    void loadSources();
    void updateSource(bool fit_axis = true);
    void updateSourceOptions();
    void sourceChanged();
    void fitImage();

    // detector stuff
    void selectDetector();
    void resetDetector();
    void resetAllDetectors();
    void setDetectorOption(int row, int col);
    void processData();
    
    // line stuff
    void updateLine(int idx);
    void updateLine();
    void updateLines();

    void updateLineLayers();
    void flipNormals();
    void flipEndpoints();
    void fitLine();
	void selectLineColor();
    
    void selectLine(int sel = -1);
    void selectLineByRow();
    void lplotSelChange();
    
    void readLineMod();
    
    void updateLineGeometry(Line &l);
    void updateLineTableData(int idx);
    void updateLineByCellChange(int row, int col);
    void freezeLine();
    void freezeAllLines();
    void setLines(const LineSegmentVector& lines);

	// quiver stuff
	void showQuiverControl();
	void toggleQuiver();
	void processQuiver();
	void gradientQuiver(cv::Mat mag, cv::Mat gx, cv::Mat gy);
	void angleQuiver(cv::Mat ang, cv::Mat ampl);
	void deleteQuivers(bool replot = true);

    // ui stuff
    void setRotSliderValue(double val);
    void setRotSpinValue(int val);
    void setOTransSliderValue(double val);
    void setOTransSpinValue(int val);
    void setSTransSliderValue(double val);
    void setSTransSpinValue(int val);
    void setETransSliderValue(double val);
    void setETransSpinValue(int val);
    void setTransPrec(int val);
    void setRotPrec(int val);
    void resetControlOptions();
    void resetControls();
    void resetAllControls();
    void updateRanges();

    void setIndicatorVisible(bool val);
    void setIndicatorPosition(double pos, double range = 1);
    void setIndicatorPosition(const LineSegment &l, double pos, double range = 1);

    void replot();
    
signals:
    void sourcesChanged(const ImageSources &src);
    void lineChanged(const LineSegment &l);
    void lineSelChanged(int sel);
    void linesUpdated(LineVector *lines);
    
private:
    Ui::ControlWindow *ui;
    void setControls(const LineMod* mod = nullptr);
    
    void clearLineSelected();
    void setLineSelected(Line &l);
    void processLineData();
    void clearLines();

	void connectQuiver();

    QPen pen, mPen, nPen, nmPen, selPen, nSelPen, iPen;

};

#endif // CONTROLWINDOW_H
