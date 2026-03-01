#pragma once

#include "analyser_options.h"
#include "precision_optimizer.h"
#include "ui_ground_truth_inspector.h"

#include <filesystem>
#include <string>

namespace Ui {
class LineAnalyser2D;
}

class LineAnalyser2D : public LATool {
  Q_OBJECT

  // Typedef
  typedef Analyzer::LineVector LineVector;
  typedef Analyzer::Line Line;

  // Correct Line Data
  struct CLData {
    const lsfm::LineSegment2d segment; /* Correct Line Segment */
    QCPItemLine* line;                 /* Pointer on line segment in Plot Window */
    double error;                      /* Computed error value */
    double angle_diff;                 /* Angle difference to GT Line */
    double length_diff;                /* Angle difference to GT Line */
    double dist_start;                 /* Distance from GT to CLine start point */
    double dist_end;                   /* Distance from GT to CLine end point */
    int posGT;                         /* Index position in gtlines */
    int posCL;                         /* Index position in clines */
    bool isDrawn = false;              /* Check if line is already drawn in plot window */

    CLData(const lsfm::LineSegment2d cline,
           double error,
           double angle_diff,
           double length_diff,
           double dist_start,
           double dist_end,
           int posGT,
           int posCL,
           bool isDrawn = false)
        : segment(cline),
          line(nullptr),
          error(error),
          angle_diff(angle_diff),
          length_diff(length_diff),
          dist_start(dist_start),
          dist_end(dist_end),
          posGT(posGT),
          posCL(posCL),
          isDrawn(isDrawn) {}
  };

  typedef std::vector<CLData> CLDataVector;

  // Ground Truth Data
  struct GTData {
    const lsfm::LineSegment2d segment; /* Ground Truth Line Segment */
    QCPItemLine* line;                 /* Pointer on GT line segment in Plot Window */
    CLDataVector clData;               /* CLines which fit to GT Segment */
    int posGT;                         /* Index position in gtlines */
    bool isDrawn = false;              /* Check if line is already drawn in plot window */

    GTData(const lsfm::LineSegment2d gtline, int posGT, bool isDrawn = false)
        : segment(gtline), line(nullptr), posGT(posGT), isDrawn(isDrawn) {}
  };

  typedef std::vector<GTData> GTDataVector;

  // Lines
  LineVector* lines;    /* Computed lines given from specific algorithms */
  GTDataVector gtLines; /* GT Lines */
  CLDataVector cLines;  /* CLines, each CLine fits to one GT Line */

  // Image
  cv::Mat* srcImg; /* Source Image, loaded from Analyzer */

  // Image Sources
  const ImageSources* sources; /* Different Image Sources (Gray, Magnitude, etc.) */
  QCPColorMap* imgMap;         /* Different ColorMaps (Hot, Cold, etc.) */

  // Analyzer (for loading bundled examples)
  Analyzer* ctrl{nullptr};

  // File
  QFileDialog* file; /* File Dialog for file loading */

  // Image View
  PlotWindow* lplot; /* Plot Window */

  // GUI
  Ui::LineAnalyser2D* ui; /* UI Information */

  // Precision Optimizer
  PrecisionOptimizer* po; /* Optimizer UI */

  // AnalyserOptions
  AnalyserOptions* ao; /* AnalyserOptions UI */

 public:
  explicit LineAnalyser2D(QWidget* parent = nullptr);
  ~LineAnalyser2D();

  void connectTools(Analyzer* w) override;

  double anglediff(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line);
  double miscalculation(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line);

 public slots:

 signals:

 private slots:

  // Button Function
  void clearOtherLines();
  void selectGroundTruthData();
  void loadGroundTruthData();
  void displayImageView();
  void computeCorrectLines();
  void openAnalyserOptions();
  void loadEasyExample();
  void loadHardExample();

  // Display Functions
  void displaySource();
  void displayImage();
  void displayGrid();
  void displayAllLines();
  void displayGTLines();
  void displayCorrectLines();
  void updateSourceOptions();

  void showCLines(int);
  void showCLines(int, int);
  void showSelectedCLine(int);
  void showSelectedCLine(int, int);

  void lplotSelChange();
  void manageGTandCLines();
  void manageThresholdSettings();

  // Debug Page
  void enableDebugSettings();
  void disableDebugSettings();
  void lockInGTLine();
  void lockInOtherLine();
  void processDebugData();
  void resetDebugSettings();

  // Analysis Page
  void enableAnalysisSettings();
  void disableAnalysisSettings();
  void displayOldCLines();
  void displayGTLinesAnalysis();
  void displayCLinesAnalysis();
  void updateLines(LineVector* line);
  void showCurLineReference(int);
  void showCurLineReference(int, int);
  void showOldLineReference(int);
  void showOldLineReference(int, int);
  void showOptimizer();
  void saveAnalysis();

 private:
  CLData* clDataTemp;
  GTData* gtDataTemp;
  lsfm::LineSegment2d lockedGTLine_storage;
  const lsfm::LineSegment2d* lockedGTLine;
  lsfm::LineSegment2d lockedOtherLine_storage;
  const lsfm::LineSegment2d* lockedOtherLine;
  bool displayMode;
  bool debugMode;
  int curSelMode;

  // QPens
  QPen red;
  QPen green;
  QPen blue;
  QPen yellow;
  QPen orange;
  QPen selPen;

  bool analysisMode;
  CLDataVector cLinesOld;
  void loadOldLineInfo();
  void loadCurLineInfo();
  void drawOldCLData();
  void manageOldLineVisibility(bool visibility);
  void markTableRow(int index);

  void initQPens();
  void initUISetup();
  void initPlotWindow();
  void initConnections();

  void addSourceItems();
  void addModeItems();
  void updateModes();
  void loadImageSources();
  void loadGroundTruthDataIntoTable();
  void loadGroundTruthDataFromFile(std::string file_path);
  void loadBundledExample(const std::string& txt_relative, const std::string& image_relative, const std::string& label);
  static std::string findResourcePath(const std::string& relative_path);
  void computeCorrectLinesHelper(double& angle, double& threshold, double& error);

  bool checkDistance(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line, double& distance);
  bool checkLineAngles(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line, double& angle);
  bool checkMiscalculation(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line, double& error);

  void setLayerVisibility(const char* name, bool visibility);
  void manageLayerVisibility(QCheckBox*& box, const char* name);
  void manageDebugSettings(bool disable);
  void manageCLineVisibility(bool visibility);
  void manageGTLineVisibility(bool visibility);

  void showSelectedCLineFromPlotWindow(int index);
  void displaySelCLine(CLData* clTemp);
  void loadCLineInfo();

  void redrawItems();
  void drawGTData();
  void drawCLData();
  void drawComputedLines();

  void enableImageSettings();
  void enableTestSettings();
  void resetTestSettings();

  void manageAnalysisSettings(bool disable);
  void loadSaveFileHeader(QTextStream& outStream, QString caption);
  void loadSaveFileContent(QTextStream& outStream, CLData& cl, int& i);
  void loadSaveFileFooter(QTextStream& outStream);
  void loadPercentageVarianceHeader(QTextStream& outStream);
  void loadPercentageVarianceContent(QTextStream& outStream, int& i);
  double calculatePercentage(int& i);

  // Templates
  template <class T>
  void loadLineInformationIntoTable(T& line);
  template <class T>
  void loadGTLineInformation(T& line);
  template <class T>
  void loadOtherLineInformation(T& line);
  template <class T>
  void drawLines(T& data, QPen& pen, QPen& selPen, const char* name, const QVariant& value);
  template <class T>
  void loadLineInfo(T& data, QTableWidget*& widget, int& row);
  template <class T>
  void loadErrorInfo(T& data, QTableWidget*& widget, int& row, bool add_at_front);
};
