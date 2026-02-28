#pragma once

#include "helpers.h"
#include "latool.h"
#include "preprocessing.h"
#include "quiver.h"
#include <algorithm/preset_store.hpp>
#include <qplot/PlotWindow.h>

#include <QColorDialog>
#include <QComboBox>
#include <QFileDialog>
#include <QMainWindow>


namespace Ui {
class ControlWindow;
}

class ControlWindow : public QMainWindow {
  Q_OBJECT
  Q_DISABLE_COPY(ControlWindow)

  QFileDialog* file;
  PlotWindow* lplot;
  PreProcessing* pp;
  QColorDialog* cdia;
  Quiver* qo;
  cv::Mat img, src;
  int tools;

  ImageSources sources;
  QCPColorMap* imgMap;

  int lineSel;
  size_t inputSourcesSize;

  DetectorVector detectors;

  lsfm::PresetStore presetStore;
  QComboBox* cbPreset{nullptr};

  QCPItemLine* indicator;

 public:
  struct LineMod {
    LineMod(float_type a = 0, float_type d = 0, float_type s = 0, float_type e = 0)
        : angle(a), distance(d), start(s), end(e) {}
    float_type angle, distance, start, end;

    inline bool modified() const { return angle != 0 || distance != 0 || start != 0 || end != 0; }

    inline void clear() {
      angle = 0;
      distance = 0;
      start = 0;
      end = 0;
    }
  };

  struct Line {
    Line(const LineSegment& l = LineSegment(), const LineMod& m = LineMod())
        : segment(l), mod(m), line(nullptr), normal(nullptr) {}
    Line(const Line&) = default;
    Line& operator=(const Line&) = default;


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

    inline void reset() { mod.clear(); }

    QCPItemLine *line, *normal;
  };

  typedef std::vector<Line> LineVector;

 private:
  LineVector lines;
  LineVector quiver;

 public:
  explicit ControlWindow(QWidget* parent = nullptr);
  ~ControlWindow();

  void addDetector(const DetectorPtr& detector);

  template <class TOOL>
  void addTool() {
    LATool* tool = new TOOL(this);
    addTool(tool);
  }

  void addTool(LATool* tool);
  const LineVector& getLines() const { return lines; }
  LineVector& getLines() { return lines; }
  int getLineSel() const { return lineSel; }
  const ImageSources& getSources() const { return sources; }
  cv::Mat& getSrcImg() { return img; }
  PlotWindow* getPlotWindow() { return lplot; }

  /// @brief Set the image path and trigger a load.
  /// @param path Absolute path to the image file.
  void setImagePath(const QString& path);

  /// @brief Get the currently selected detector.
  /// @return Shared pointer to the current detector, or nullptr if none selected.
  DetectorPtr getCurrentDetector() const;

  /// @brief Get the display name of the currently selected detector.
  /// @return Detector name, or empty string if none selected.
  QString getCurrentDetectorName() const;

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

  /// @brief Refresh the detector parameter table from the current detector's ValueManager.
  void refreshDetectorOptions();

  /// @brief Apply the selected preset to the current detector.
  void applyPreset(int presetIndex);

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

  void updateLineGeometry(Line& l);
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
  void setIndicatorPosition(const LineSegment& l, double pos, double range = 1);

  void replot();

 signals:
  void sourcesChanged(const ImageSources& src);
  void lineChanged(const LineSegment& l);
  void lineSelChanged(int sel);
  void linesUpdated(LineVector* lines);

  /// @brief Emitted when the selected detector changes.
  /// @param name Display name of the newly selected detector.
  void detectorChanged(const QString& name);

 private:
  Ui::ControlWindow* ui;
  void setControls(const LineMod* mod = nullptr);

  void clearLineSelected();
  void setLineSelected(Line& l);
  void processLineData();
  void clearLines();

  void connectQuiver();

  /// @brief Resolve a display name like "LSD CC" to a DetectorId.
  /// @param name Detector display name from the combo box.
  /// @return Pair of (found, DetectorId). found is false for unsupported detectors.
  static std::pair<bool, lsfm::DetectorId> resolveDetectorId(const QString& name);

  /// @brief Find the presets JSON file path relative to the workspace.
  /// @return Absolute path to lsd_presets.json, or empty string if not found.
  static std::string findPresetsPath();

  /// @brief Update the preset combo box state for the current detector.
  void updatePresetCombo();

  QPen pen, mPen, nPen, nmPen, selPen, nSelPen, iPen;
};
