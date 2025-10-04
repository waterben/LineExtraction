#include "lineanalyser2d.h"

#include <QTableWidgetItem>
#include <fstream>

LineAnalyser2D::LineAnalyser2D(QWidget* parent)
    : LATool("Line Analyser 2D", parent),
      file(new QFileDialog(this, tr("Open .txt File"))),
      lplot(new PlotWindow("Line Analyser 2D Image Viewer", this)),
      lines(0),
      sources(0),
      imgMap(0),
      displayMode(false),
      debugMode(false),
      analysisMode(false),
      curSelMode(0),
      clDataTemp(0),
      gtDataTemp(0),
      ao(new AnalyserOptions(this)),
      ui(new Ui::LineAnalyser2D) {
  setWindowTitle("Line Analyser 2D");
  ui->setupUi(this);

  file->setFileMode(QFileDialog::ExistingFile);
  file->setAcceptMode(QFileDialog::AcceptOpen);
  QStringList filter;
  filter << "(*.txt)";
  file->setNameFilters(filter);

  initQPens();
  initUISetup();
  initPlotWindow();
  initConnections();
}

LineAnalyser2D::~LineAnalyser2D() { delete ui; }

void LineAnalyser2D::connectTools(ControlWindow* w) {
  this->lines = &w->getLines();
  this->sources = &w->getSources();
  this->srcImg = &w->getSrcImg();
  this->po = new PrecisionOptimizer(w);
  this->po->connectTools(w);
  connect(w, SIGNAL(linesUpdated(LineVector*)), this, SLOT(updateLines(LineVector*)));
  connect(ui->pb_analysis_freez_all, SIGNAL(clicked(bool)), w, SLOT(freezeAllLines()));
}

/**
 * @brief   LineAnalyser2D::initQPens
 *          Initialise QPens
 */
void LineAnalyser2D::initQPens() {
  // QPens
  red = QPen(Qt::red);
  green = QPen(Qt::green);
  blue = QPen(Qt::blue);
  yellow = QPen(Qt::yellow);
  orange = QPen(QColor(255, 153, 0));
  selPen = QPen(Qt::red);
}

/**
 * @brief   LineAnalyser2D::initUISetup
 *          Setup UI
 */
void LineAnalyser2D::initUISetup() {
  // Settings
  ui->cb_image->setDisabled(true);
  ui->cb_grid->setDisabled(true);
  ui->cb_all_lines->setDisabled(true);
  ui->cb_correct_lines->setDisabled(true);
  ui->cb_gt_data->setDisabled(true);
  ui->cb_image_source->setDisabled(true);
  ui->cb_image_mode->setDisabled(true);
  ui->cb_gtline_plus_clines->setDisabled(true);
  // Tables
  ui->tw_gt_data->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tw_cl_data->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tw_debug_gtline_info->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tw_debug_lines_info->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tw_debug_result_info->setSelectionBehavior(QAbstractItemView::SelectRows);
  // Test Page
  ui->dsb_distance_threshold->setDisabled(true);
  ui->dsb_angle_threshold->setDisabled(true);
  ui->dsb_error_threshold->setDisabled(true);
  ui->pb_compute_clines->setDisabled(true);
  ui->cb_check_distance->setDisabled(true);
  ui->cb_check_angle->setDisabled(true);
  ui->cb_check_error_value->setDisabled(true);
  ui->cb_check_distance->setChecked(true);
  ui->cb_check_angle->setChecked(true);
  ui->cb_check_error_value->setChecked(true);
  // Debug Page
  ui->pb_debug_start->setDisabled(true);
  manageDebugSettings(true);
  // Analysis Page
  ui->pb_analysis_start->setDisabled(true);
  ui->tw_analysis_cur->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tw_analysis_old->setSelectionBehavior(QAbstractItemView::SelectRows);
  manageAnalysisSettings(true);
}

/**
 * @brief   LineAnalyser2D::initPlotWindow
 *          Initialise and Setup Plot Window
 */
void LineAnalyser2D::initPlotWindow() {
  // PlotWindow
  lplot->reverseY(true);
  lplot->keepAspectRatio(true);
  lplot->qplot->setInteraction(QCP::iSelectItems, true);
  lplot->qplot->setSelectionTolerance(5);
  // Set Up Layers
  lplot->qplot->addLayer("image", lplot->qplot->layer("grid"), QCustomPlot::limBelow);
  lplot->qplot->addLayer("gtlines", lplot->qplot->layer("main"), QCustomPlot::limAbove);
  lplot->qplot->addLayer("clines", lplot->qplot->layer("gtlines"), QCustomPlot::limAbove);
  lplot->qplot->addLayer("oldlines", lplot->qplot->layer("gtlines"), QCustomPlot::limAbove),
      lplot->qplot->addLayer("lines", lplot->qplot->layer("gtlines"), QCustomPlot::limAbove);
  // Layer Visibility
  lplot->qplot->layer("image")->setVisible(true);
  lplot->qplot->layer("grid")->setVisible(true);
  lplot->qplot->layer("gtlines")->setVisible(false);
  lplot->qplot->layer("lines")->setVisible(false);
  lplot->qplot->layer("clines")->setVisible(false);
  lplot->qplot->layer("oldlines")->setVisible(false);
}

/**
 * @brief   LineAnalyser2D::initConnections
 *          Initialise Connections (SIGNAL and SLOTS)
 */
void LineAnalyser2D::initConnections() {
  // Connect QPlotWindow
  connect(lplot->qplot, SIGNAL(selectionChangedByUser()), this, SLOT(lplotSelChange()));
  // Connect Tables with ImageView
  connect(ui->tw_gt_data, SIGNAL(cellClicked(int, int)), this, SLOT(showCLines(int, int)));
  connect(ui->tw_cl_data, SIGNAL(cellClicked(int, int)), this, SLOT(showSelectedCLine(int, int)));
  connect(ui->tw_gt_data->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(showCLines(int)));
  connect(ui->tw_cl_data->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(showSelectedCLine(int)));
  // Connect Analysis Tables with ImageView
  connect(ui->tw_analysis_cur, SIGNAL(cellClicked(int, int)), this, SLOT(showCurLineReference(int, int)));
  connect(ui->tw_analysis_old, SIGNAL(cellClicked(int, int)), this, SLOT(showOldLineReference(int, int)));
  connect(ui->tw_analysis_cur->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(showCurLineReference(int)));
  connect(ui->tw_analysis_old->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(showOldLineReference(int)));
}

/**
 * @brief   LineAnalyser2D::lplotSelChange
 *          SLOT-Function
 *          Checks User Mouse-Click Input in Plot Window
 */
void LineAnalyser2D::lplotSelChange() {
  auto items = lplot->qplot->selectedItems();

  if (items.size()) {
    QCPLayer* layer = items.front()->layer();
    int index = -1;

    if (layer->name() == "lines") {
      index = items.front()->property("lines").toInt();
      if (debugMode) loadLineInformationIntoTable(lines->at(index));
    } else if (layer->name() == "gtlines") {
      index = items.front()->property("gtlines").toInt();
      if (debugMode) loadLineInformationIntoTable(gtLines.at(index));
      showCLines(index);
    } else if (layer->name() == "clines") {
      index = items.front()->property("clines").toInt();
      if (debugMode) loadLineInformationIntoTable(cLines.at(index));
      if (analysisMode) markTableRow(index);
      showSelectedCLineFromPlotWindow(index);
    } else if (layer->name() == "oldlines") {
      index = items.front()->property("oldlines").toInt();
      if (analysisMode) markTableRow(index);
    }
  }
}

/**
 * @brief   LineAnalyser2D::showCLines
 *          Shows the CLines to a GT Line and marks the Table row
 * @param   row
 */
void LineAnalyser2D::showCLines(int row) { this->showCLines(row, 0); }

/**
 * @brief   LineAnalyser2D::loadCLineInfo
 *          Loads the CLine information from the Temp GTData
 */
void LineAnalyser2D::loadCLineInfo() {
  int row = 0;
  this->ui->tw_cl_data->setRowCount(static_cast<int>(gtDataTemp->clData.size()));
  this->ui->tw_cl_data->blockSignals(true);
  for_each(gtDataTemp->clData.begin(), gtDataTemp->clData.end(), [&](CLData& cld) {
    loadLineInfo(cld, ui->tw_cl_data, row);
    loadErrorInfo(cld, ui->tw_cl_data, row, false);
    ++row;
  });
  this->ui->tw_cl_data->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::showCLines
 *          Shows selected CLines in Plot Window and Table
 * @param   y   Table Row
 * @param   x   Table Colum
 */
void LineAnalyser2D::showCLines(int y, int x) {
  // (1) Load and Select Table Information
  displayMode = true;
  ui->tw_gt_data->selectRow(y);
  gtDataTemp = &gtLines.at(y);
  loadCLineInfo();
  // (2) Handle and Display Lines in Plot Window
  manageGTLineVisibility(true);
  if (gtDataTemp->isDrawn) gtDataTemp->line->setSelected(true);
  if (!ui->cb_gt_data->isChecked()) {
    ui->cb_gt_data->setChecked(true);
    setLayerVisibility("gtlines", true);
  }
  if (ui->cb_gtline_plus_clines->isChecked()) {
    manageCLineVisibility(false);
    for_each(gtDataTemp->clData.begin(), gtDataTemp->clData.end(), [&](CLData& cld) { cld.line->setVisible(true); });
    setLayerVisibility("clines", true);
  }
  if (ui->cb_gt_data->isChecked() || !ui->cb_gtline_plus_clines) lplot->replot();
}

/**
 * @brief   LineAnalyser2D::showSelectedCLine
 *          Shows the selected CLine
 * @param   row
 */
void LineAnalyser2D::showSelectedCLine(int row) { this->showSelectedCLine(row, 0); }

/**
 * @brief   LineAnalyser2D::showSelectedCLine
 *          Shows the selected CLine
 * @param   y   Table row
 * @param   x   Table colum
 */
void LineAnalyser2D::showSelectedCLine(int y, int x) {
  CLData* clTemp = &gtDataTemp->clData.at(y);
  displayMode = true;
  // (2) Handle and Display Lines in Plot Window
  if (ui->cb_gtline_plus_clines->isChecked()) displaySelCLine(clTemp);
}

/**
 * @brief   LineAnalyser2D::showSelectedCLineFromPlotWindow
 *          Shows the information about the selected line from the plot window
 * @param   index
 */
void LineAnalyser2D::showSelectedCLineFromPlotWindow(int index) {
  // (1) Load and Select Table Information
  CLData* clTemp = &cLines.at(index);
  gtDataTemp = &gtLines.at(clTemp->posGT);
  displayMode = true;
  loadCLineInfo();
  ui->tw_gt_data->selectRow(clTemp->posGT);
  ui->tw_cl_data->selectRow(clTemp->posCL);
  // (2) Handle and Display Lines in Plot Window
  if (ui->cb_gtline_plus_clines->isChecked()) displaySelCLine(clTemp);
}

/**
 * @brief   LineAnalyser2D::displaySelCLine
 *          Display cline to selected gt line
 * @param   clTemp
 */
void LineAnalyser2D::displaySelCLine(CLData* clTemp) {
  manageGTLineVisibility(true);
  manageCLineVisibility(false);
  for_each(gtDataTemp->clData.begin(), gtDataTemp->clData.end(), [&](CLData& cld) { cld.line->setVisible(true); });
  clTemp->line->setSelected(true);
  clTemp->line->setVisible(true);
  setLayerVisibility("gtlines", true);
  setLayerVisibility("clines", true);
}

/**
 * @brief   LineAnalyser2D::miscalculation
 *          Computes the error value
 *          (GTLine vs. Computed Line)
 * @param   gtline
 * @param   line
 * @return
 */
double LineAnalyser2D::miscalculation(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line) {
  double a = gtline.distance(line.startPoint());
  double b = gtline.distance(line.endPoint());

  if (a >= 0 && b >= 0) {
    return (a + b) / 2;
  }
  if (a < 0 && b < 0) {
    return fabs(a + b) / 2;
  }

  lsfm::Point2d p1, p2, s;
  lsfm::LineSegment2d temp;
  double s1, s2;

  p1 = gtline.normalLineDist(-a, line.startPoint());
  p2 = gtline.normalLineDist(-b, line.endPoint());
  gtline.intersection(line, s);
  temp = lsfm::LineSegment2d(s, p2);
  s2 = temp.length();
  temp = lsfm::LineSegment2d(p1, s);
  s1 = temp.length();

  return (fabs(a) * s1 + fabs(b) * s2) / (2 * (s1 + s2));
}

/**
 * @brief   LineAnalyser2D::checkDistance
 *          Checks the distance of a line to start- and endpoint against a given threshold
 * @param   gtline      GTLine
 * @param   line        Start-Endpoint
 * @param   distance    Threshold
 * @return
 */
bool LineAnalyser2D::checkDistance(const lsfm::LineSegment2d& gtline,
                                   const lsfm::LineSegment2d& line,
                                   double& distance) {
  if (!this->ui->cb_check_distance->isChecked())
    return true;
  else
    return fabs(gtline.distance(line.startPoint())) <= distance && fabs(gtline.distance(line.endPoint())) <= distance;
}

/**
 * @brief   LineAnalyser2D::anglediff
 *          Computes the angle difference from a gtline to a computed line
 * @param   gtline  GTLine
 * @param   line    Computed Line
 * @return
 */
double LineAnalyser2D::anglediff(const lsfm::LineSegment2d& gtline, const lsfm::LineSegment2d& line) {
  if (gtline.angle() < 0 && line.angle() < 0) return fabs(gtline.angle() - line.angle()) * 180 / CV_PI;

  if (gtline.angle() >= 0 && line.angle() >= 0) return fabs(gtline.angle() - line.angle()) * 180 / CV_PI;

  if (gtline.angle() < 0 && line.angle() > 0) return fabs((CV_PI + gtline.angle()) - line.angle()) * 180 / CV_PI;

  if (gtline.angle() > 0 && line.angle() < 0) return fabs(gtline.angle() - (CV_PI + line.angle())) * 180 / CV_PI;
}

/**
 * @brief   LineAnalyser2D::checkLineAngles
 *          Checks an angle against an given threshold
 * @param   gtline
 * @param   line
 * @param   angle
 * @return  true if computed angle <= threshold
 */
bool LineAnalyser2D::checkLineAngles(const lsfm::LineSegment2d& gtline,
                                     const lsfm::LineSegment2d& line,
                                     double& angle) {
  if (!this->ui->cb_check_angle->isChecked()) return true;

  if (gtline.angle() < 0 && line.angle() < 0) return fabs(gtline.angle() - line.angle()) <= angle ? true : false;

  if (gtline.angle() >= 0 && line.angle() >= 0) return fabs(gtline.angle() - line.angle()) <= angle ? true : false;

  if (gtline.angle() < 0 && line.angle() > 0)
    return fabs((CV_PI + gtline.angle()) - line.angle()) <= angle ? true : false;

  if (gtline.angle() > 0 && line.angle() < 0)
    return fabs(gtline.angle() - (CV_PI + line.angle())) <= angle ? true : false;
}

/**
 * @brief   LineAnalyser2D::checkMiscalculation
 * @param   gtline
 * @param   line
 * @param   error
 * @return
 */
bool LineAnalyser2D::checkMiscalculation(const lsfm::LineSegment2d& gtline,
                                         const lsfm::LineSegment2d& line,
                                         double& error) {
  if (!this->ui->cb_check_error_value->isChecked())
    return true;
  else
    return fabs(miscalculation(gtline, line)) <= error;
}

/**
 * @brief   LineAnalyser2D::clearOtherLines
 *          Sets the visibility of all layers to invisibile
 *          (Button function/signal "Clear All")
 */
void LineAnalyser2D::clearOtherLines() {
  if (ui->cb_all_lines->isChecked()) setLayerVisibility("lines", false);
  ui->cb_all_lines->setChecked(false);

  if (ui->cb_correct_lines->isChecked()) setLayerVisibility("clines", false);
  ui->cb_correct_lines->setChecked(false);

  if (ui->cb_gt_data->isChecked()) setLayerVisibility("gtlines", false);
  ui->cb_gt_data->setChecked(false);
}

/**
 * @brief   LineAnalyser2D::selectGroundTruthData
 *          Sets the file path
 */
void LineAnalyser2D::selectGroundTruthData() {
  if (file->exec()) {
    ui->le_file_path->setText(file->selectedFiles().front());
  }
}

/**
 * @brief   LineAnalyser2D::resetTestSettings
 *          Resets the settings of the load button is pressed twice
 *          (reset because of pointer issues)
 */
void LineAnalyser2D::resetTestSettings() {
  // Clear data
  cLines.clear();
  // Reset UI
  ui->dsb_distance_threshold->setDisabled(true);
  ui->dsb_angle_threshold->setDisabled(true);
  ui->dsb_error_threshold->setDisabled(true);
  ui->pb_compute_clines->setDisabled(true);
  ui->cb_gtline_plus_clines->setDisabled(true);
  ui->cb_check_angle->setDisabled(true);
  ui->cb_check_distance->setDisabled(true);
  ui->cb_check_error_value->setDisabled(true);
  if (ui->cb_correct_lines->isChecked()) ui->cb_correct_lines->setChecked(false);
  ui->cb_correct_lines->setDisabled(true);
  ui->tw_cl_data->setRowCount(0);
}

/**
 * @brief   LineAnalyser2D::openAnalyserOptions
 *          Open AnalyserOptions, preset settings
 */
void LineAnalyser2D::openAnalyserOptions() { ao->show(); }

/**
 * @brief   LineAnalyser2D::loadGroundTruthData
 *          Loads the Ground Truth Data from a given path
 */
void LineAnalyser2D::loadGroundTruthData() {
  if (!gtLines.empty()) gtLines.clear();

  if (!cLines.empty()) this->resetTestSettings();

  this->loadGroundTruthDataFromFile(ui->le_file_path->text().toStdString());
  this->loadGroundTruthDataIntoTable();

  if (lplot->isVisible()) this->redrawItems();
}

/**
 * @brief   LineAnalyser2D::loadGroundTruthDataFromFile
 *          Loads the GT Data from a given path,
 *          uniform data or already calculated data if an image was loaded
 * @param   file_path
 */
void LineAnalyser2D::loadGroundTruthDataFromFile(std::string file_path) {
  if (file_path.empty() || file_path == "None") return;

  std::ifstream txt_file(file_path);
  std::string tempLine;
  QStringList tempList;
  int posGT = 0;

  double imgHeight = ao->getImageHeight();
  double imgWidth = ao->getImageWidth();
  double div = ao->getReducedScale();
  double offset = ao->getOffset();

  // loads gt data from .txt file
  // uniform or calculated
  while (std::getline(txt_file, tempLine)) {
    tempList = QString::fromStdString(tempLine).split(",");
    lsfm::Point2d p1(tempList[0].toDouble(0), tempList[1].toDouble(0));
    lsfm::Point2d p2(tempList[2].toDouble(0), tempList[3].toDouble(0));

    if (this->ui->cb_uniform_data->isChecked() && !srcImg->empty()) {
      p1.x = ((imgWidth * p1.x) / div) - offset / div;
      p1.y = srcImg->rows - ((imgHeight * p1.y) / div) - offset / div;
      p2.x = ((imgWidth * p2.x) / div) - offset / div;
      p2.y = srcImg->rows - ((imgHeight * p2.y) / div) - offset / div;
    }

    // add gt data to vector
    gtLines.push_back(GTData(lsfm::LineSegment2d(p1, p2), posGT));
    ++posGT;
  }
}

/******************************************************* COMPUTE CORRECT LINES
 * ********************************************************/

/**
 * @brief   LineAnalyser2D::computeCorrectLines
 *          Compute the correct lines with users input
 */
void LineAnalyser2D::computeCorrectLines() {
  if (gtLines.empty() || lines->empty()) return;

  if (!cLines.empty()) {
    cLines.clear();
    for_each(gtLines.begin(), gtLines.end(), [&](GTData& gtd) { gtd.clData.clear(); });
  }

  // User Input
  double distance = this->ui->dsb_distance_threshold->value();
  double angle = this->ui->dsb_angle_threshold->value();
  double error = this->ui->dsb_error_threshold->value();

  // Calculate Lines with given Threshold
  this->computeCorrectLinesHelper(angle, distance, error);

  // draw lines
  this->redrawItems();
  this->ui->cb_correct_lines->setChecked(true);
  this->ui->cb_correct_lines->setDisabled(false);
  this->ui->cb_gtline_plus_clines->setDisabled(false);
  this->ui->pb_analysis_start->setDisabled(false);
  this->setLayerVisibility("clines", true);
}

/**
 * @brief   LineAnalyser2D::computeCorrectLinesHelper
 *          (Helper Function)
 *          Computes the correct lines
 * @param   angle
 * @param   threshold
 * @param   error
 */
void LineAnalyser2D::computeCorrectLinesHelper(double& angle, double& distance, double& error) {
  for_each(gtLines.begin(), gtLines.end(), [&](GTData& gtl) {
    int posCL = 0;
    const lsfm::LineSegment2d& gtline = gtl.segment;

    for_each(lines->begin(), lines->end(), [&](Line& tl) {
      const lsfm::LineSegment2d& line = tl.segment;

      if (checkDistance(gtline, line, distance))
        if (checkLineAngles(gtline, line, angle))
          if (checkMiscalculation(gtline, line, error)) {
            CLData clData(line, miscalculation(gtline, line), anglediff(gtline, line), gtline.length() - line.length(),
                          fabs(gtline.distance(line.startPoint())), fabs(gtline.distance(line.endPoint())), gtl.posGT,
                          posCL);

            cLines.push_back(clData);
            gtl.clData.push_back(clData);
            ++posCL;
          }
    });
  });
}

/********************************************************* LOADING DATA INTO TABLE
 * *********************************************************/

/**
 * @brief   LineAnalyser2D::loadGroundTruthDataIntoTable
 *          Loads the GT data into the UI table
 */
void LineAnalyser2D::loadGroundTruthDataIntoTable() {
  if (gtLines.empty()) return;

  int row = 0;
  this->ui->tw_gt_data->setRowCount(static_cast<int>(gtLines.size()));
  this->ui->tw_gt_data->blockSignals(true);

  // load gt data into table
  for_each(gtLines.begin(), gtLines.end(), [&](GTData& gtl) {
    loadLineInfo(gtl, ui->tw_gt_data, row);
    ++row;
  });
  this->ui->tw_gt_data->blockSignals(false);
}

template <class T>
void LineAnalyser2D::loadLineInfo(T& data, QTableWidget*& widget, int& row) {
  QTableWidgetItem* item = new QTableWidgetItem(QString::number(data.segment.angle() * 180 / CV_PI));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 0, item);
  item = new QTableWidgetItem(QString::number(data.segment.length()));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 1, item);
  item = new QTableWidgetItem(QString::number(data.segment.startPoint().x));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 2, item);
  item = new QTableWidgetItem(QString::number(data.segment.startPoint().y));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 3, item);
  item = new QTableWidgetItem(QString::number(data.segment.endPoint().x));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 4, item);
  item = new QTableWidgetItem(QString::number(data.segment.endPoint().y));
  item->setTextAlignment(Qt::AlignCenter);
  widget->setItem(row, 5, item);
}

template <class T>
void LineAnalyser2D::loadErrorInfo(T& data, QTableWidget*& widget, int& row, bool add_at_front) {
  if (add_at_front) {
    QTableWidgetItem* item = new QTableWidgetItem(QString::number(data.error));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 0, item);
    item = new QTableWidgetItem(QString::number(data.angle_diff));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 1, item);
    item = new QTableWidgetItem(QString::number(data.length_diff));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 2, item);
    item = new QTableWidgetItem(QString::number(data.dist_start));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 3, item);
    item = new QTableWidgetItem(QString::number(data.dist_end));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 4, item);
  } else {
    QTableWidgetItem* item = new QTableWidgetItem(QString::number(data.error));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 6, item);
    item = new QTableWidgetItem(QString::number(data.angle_diff));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 7, item);
    item = new QTableWidgetItem(QString::number(data.length_diff));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 8, item);
    item = new QTableWidgetItem(QString::number(data.dist_start));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 9, item);
    item = new QTableWidgetItem(QString::number(data.dist_end));
    item->setTextAlignment(Qt::AlignCenter);
    widget->setItem(row, 10, item);
  }
}

/********************************************************* DRAW FUNCTIONS
 * *********************************************************/

template <class T>
void LineAnalyser2D::drawLines(T& data, QPen& pen, QPen& selPen, const char* name, const QVariant& value) {
  data.line = lplot->line(data.segment, false);
  data.line->setHead(QCPLineEnding(QCPLineEnding::esSpikeArrow, 4, 8));
  data.line->setPen(pen);
  data.line->setSelectedPen(selPen);
  data.line->setSelectable(true);
  data.line->setProperty(name, value);
}

/**
 * @brief   LineAnalyser2D::drawComputedLines
 *          Draws the computed lines which are given from the controlwindow
 */
void LineAnalyser2D::drawComputedLines() {
  if (lines->empty()) return;

  lplot->qplot->setCurrentLayer("lines");
  int lineIndex = 0;
  lplot->hold(true);
  for_each(lines->begin(), lines->end(), [&](Line& tl) {
    drawLines(tl, blue, selPen, "lines", lineIndex);
    ++lineIndex;
  });
  lplot->hold(false);
}

/**
 * @brief   LineAnalyser2D::drawGTData
 *          Draws the GT data
 */
void LineAnalyser2D::drawGTData() {
  if (gtLines.empty()) return;

  lplot->qplot->setCurrentLayer("gtlines");
  int gtlineIndex = 0;
  lplot->hold(true);
  for_each(gtLines.begin(), gtLines.end(), [&](GTData& tl) {
    drawLines(tl, yellow, selPen, "gtlines", gtlineIndex);
    tl.isDrawn = true;
    ++gtlineIndex;
  });
  lplot->hold(false);
}

/**
 * @brief   LineAnalyser2D::drawCLData
 *          Draws the computed correct lines
 */
void LineAnalyser2D::drawCLData() {
  if (cLines.empty()) return;

  lplot->qplot->setCurrentLayer("clines");
  int clineIndex = 0;
  lplot->hold(true);
  for_each(cLines.begin(), cLines.end(), [&](CLData& cl) {
    drawLines(cl, green, selPen, "clines", clineIndex);
    gtLines.at(cl.posGT).clData.at(cl.posCL).line = cl.line;
    cl.isDrawn = true;
    ++clineIndex;
  });
  lplot->hold(false);
}

/**
 * @brief   LineAnalyser2D::redrawItems
 *          Redraw function, important if layers has been cleared
 */
void LineAnalyser2D::redrawItems() {
  lplot->qplot->clearItems();
  this->drawComputedLines();
  this->drawGTData();
  this->drawCLData();
  if (analysisMode) this->drawOldCLData();
  lplot->replot();
}

/********************************************************* DISPLAY FUNCTIONS
 * *********************************************************/

/**
 * @brief   LineAnalyser2D::loadImageSources
 *          Loads given image sources from controlwindow
 */
void LineAnalyser2D::loadImageSources() {
  if (sources->empty()) return;

  this->addSourceItems();
  this->addModeItems();
}

/**
 * @brief   LineAnalyser2D::addSourceItems
 *          Adds Source Items into UI
 */
void LineAnalyser2D::addSourceItems() {
  this->ui->cb_image_source->blockSignals(true);
  this->ui->cb_image_source->clear();
  for_each(sources->begin(), sources->end(), [&](const ImageSource& s) { this->ui->cb_image_source->addItem(s.name); });
  this->ui->cb_image_source->blockSignals(false);
  this->ui->cb_image_source->setDisabled(false);
}

/**
 * @brief   LineAnalyser2D::addModeItems
 *          Adds all Modes into UI
 */
void LineAnalyser2D::addModeItems() {
  this->ui->cb_image_mode->blockSignals(true);
  this->ui->cb_image_mode->clear();
  const ImageSource s = sources->at(ui->cb_image_source->currentIndex());
  for_each(s.modes.begin(), s.modes.end(), [&](const ImageMode& m) { this->ui->cb_image_mode->addItem(m.name); });
  this->ui->cb_image_mode->blockSignals(false);
  ui->cb_image_mode->setDisabled(false);
}

/**
 * @brief   LineAnalyser2D::updateModes
 *          Updates Modes if Source has been changed
 */
void LineAnalyser2D::updateModes() {
  this->ui->cb_image_mode->blockSignals(true);
  this->ui->cb_image_mode->clear();
  const ImageSource& s = sources->at(ui->cb_image_source->currentIndex());
  for_each(s.modes.begin(), s.modes.end(), [&](const ImageMode& m) { this->ui->cb_image_mode->addItem(m.name); });
  this->ui->cb_image_mode->setCurrentIndex(0);
  this->ui->cb_image_mode->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::updateSourceOptions
 *          Updated Source
 */
void LineAnalyser2D::updateSourceOptions() {
  if (!imgMap) return;
  const ImageSource& s = sources->at(ui->cb_image_source->currentIndex());
  const ImageMode& m = s.modes.at(ui->cb_image_mode->currentIndex());
  if (m.name != "RGB") imgMap->setGradient(m.grad);
  this->setLayerVisibility("image", true);
}

/**
 * @brief   LineAnalyser2D::enableImageSettings
 *          Enables image settings in UI
 */
void LineAnalyser2D::enableImageSettings() {
  // Enable Settings
  ui->cb_image->setDisabled(false);
  ui->cb_image->setChecked(true);
  ui->cb_grid->setDisabled(false);
  ui->cb_grid->setChecked(true);

  if (!lines->empty() && !ui->cb_all_lines->isEnabled()) this->ui->cb_all_lines->setDisabled(false);

  if (!gtLines.empty() && !ui->cb_gt_data->isEnabled()) this->ui->cb_gt_data->setDisabled(false);
}

/**
 * @brief   LineAnalyser2D::enableTestSettings
 *          Enables the Test UI Page
 */
void LineAnalyser2D::enableTestSettings() {
  ui->dsb_distance_threshold->setDisabled(false);
  ui->dsb_angle_threshold->setDisabled(false);
  ui->dsb_error_threshold->setDisabled(false);
  ui->pb_compute_clines->setDisabled(false);
  ui->cb_check_distance->setDisabled(false);
  ui->cb_check_angle->setDisabled(false);
  ui->cb_check_error_value->setDisabled(false);
}

/**
 * @brief   LineAnalyser2D::displayImageView
 *          Displays the QPlot Window with the src image
 */
void LineAnalyser2D::displayImageView() {
  if (srcImg->empty()) return;

  // Load RGB Image
  if (srcImg->channels() != 1) {
    lplot->qplot->setCurrentLayer("image");
    lplot->qplot->clearItems();
    lplot->setAxis(QCPRange(0, srcImg->cols - 1), QCPRange(0, srcImg->rows - 1));
    lplot->keepAspectRatio(true, static_cast<double>(srcImg->cols / srcImg->rows));
    lplot->hold(true);
    lplot->plot(*srcImg, false, QCPRange(), QCPRange(), QCPRange(), ImageMode("RGB").grad, QCPAxis::stLinear, false);
    lplot->hold(false);
    this->setLayerVisibility("image", true);
    lplot->show();
  } else if (srcImg->channels() == 1) {
    lplot->qplot->setCurrentLayer("image");
    lplot->qplot->clearItems();
    lplot->setAxis(QCPRange(0, srcImg->cols - 1), QCPRange(0, srcImg->rows - 1));
    lplot->keepAspectRatio(true, static_cast<double>(srcImg->cols / srcImg->rows));
    lplot->hold(true);
    lplot->plot(*srcImg, false, QCPRange(), QCPRange(), QCPRange(), ImageMode("Gray").grad, QCPAxis::stLinear, false);
    lplot->hold(false);
    this->setLayerVisibility("image", true);
    lplot->show();
  }

  this->redrawItems();
  this->loadImageSources();
  this->enableImageSettings();

  if (!lines->empty() && !gtLines.empty()) {
    this->ui->pb_debug_start->setDisabled(false);
    this->enableTestSettings();
  }

  if (!gtLines.empty()) {
    this->ui->cb_gt_data->setChecked(true);
    this->setLayerVisibility("gtlines", true);
  }
}

/**
 * @brief   LineAnalyser2D::displaySource
 *          Displays the selected Source
 */
void LineAnalyser2D::displaySource() {
  this->ui->cb_image_source->blockSignals(true);
  const ImageSource& s = sources->at(ui->cb_image_source->currentIndex());
  this->updateModes();
  const ImageMode& m = s.modes.at(ui->cb_image_mode->currentIndex());
  this->lplot->qplot->setCurrentLayer("image");
  this->lplot->qplot->clearPlottables();
  // Plot Image
  this->lplot->hold(true);
  this->imgMap = lplot->plot(s.data, false, QCPRange(), QCPRange(), s.range, m.grad, QCPAxis::stLinear, false);
  this->lplot->hold(false);
  this->updateModes();
  this->setLayerVisibility("image", true);
  // Update UI
  if (!this->ui->cb_image->isChecked()) this->ui->cb_image->setChecked(true);
  this->ui->cb_image_source->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::manageGTLineVisibility
 *          Manage the gt line visibility
 * @param   visibility
 */
void LineAnalyser2D::manageGTLineVisibility(bool visibility) {
  for_each(gtLines.begin(), gtLines.end(), [&](GTData& gtd) {
    if (gtd.isDrawn) {
      gtd.line->setVisible(visibility);
      gtd.line->setSelected(false);
    }
  });
}

/**
 * @brief   LineAnalyser2D::manageCLineVisibility
 *          Manage the visibility of clines
 * @param   visibility
 */
void LineAnalyser2D::manageCLineVisibility(bool visibility) {
  for_each(cLines.begin(), cLines.end(), [&](CLData& cld) {
    if (cld.isDrawn) {
      cld.line->setVisible(visibility);
      cld.line->setSelected(false);
    }
  });
}

/**
 * @brief   LineAnalyser2D::manageGTandCLines
 *          Manage the CheckBox and gt- / cline
 */
void LineAnalyser2D::manageGTandCLines() {
  if (!ui->cb_gtline_plus_clines->isChecked()) {
    manageCLineVisibility(true);
    manageGTLineVisibility(true);
    lplot->replot();
    if (!ui->cb_correct_lines->isChecked()) ui->cb_correct_lines->setChecked(true);
  }
}

/**
 * @brief   LineAnalyser2D::manageLayerVisibility
 *          Manage Layer Visibility of Plot Window
 * @param   box
 * @param   name
 */
void LineAnalyser2D::manageLayerVisibility(QCheckBox*& box, const char* name) {
  if (displayMode) {
    manageCLineVisibility(true);
    manageGTLineVisibility(true);
    displayMode = false;
  }

  if (box->isChecked())
    setLayerVisibility(name, true);
  else if (!box->isChecked())
    setLayerVisibility(name, false);
}

/**
 * @brief   LineAnalyser2D::manageThresholdSettings
 *          Manage the threshold settings that it is not possible to select nothing
 */
void LineAnalyser2D::manageThresholdSettings() {
  if (!ui->cb_check_angle->isChecked() && !ui->cb_check_distance->isChecked() && !ui->cb_check_error_value->isChecked())
    ui->cb_check_distance->setChecked(true);
}

/**
 * @brief   LineAnalyser2D::displayImage
 *          Displays the image layer
 */
void LineAnalyser2D::displayImage() { manageLayerVisibility(ui->cb_image, "image"); }

/**
 * @brief   LineAnalyser2D::displayGrid
 *          Displays the grid
 */
void LineAnalyser2D::displayGrid() { manageLayerVisibility(ui->cb_grid, "grid"); }

/**
 * @brief   LineAnalyser2D::displayAllLines
 *          Displays the computed lines from the controlwindow
 */
void LineAnalyser2D::displayAllLines() { manageLayerVisibility(ui->cb_all_lines, "lines"); }

/**
 * @brief   LineAnalyser2D::displayGTLines
 *          Displays the GT lines
 */
void LineAnalyser2D::displayGTLines() {
  if (ui->cb_gt_data->isChecked())
    ui->cb_analysis_gt_lines->setChecked(true);
  else
    ui->cb_analysis_gt_lines->setChecked(false);
  manageLayerVisibility(ui->cb_gt_data, "gtlines");
}

/**
 * @brief   LineAnalyser2D::displayCorrectLines
 *          Displays the Correct computed lines
 */
void LineAnalyser2D::displayCorrectLines() {
  if (ui->cb_correct_lines->isChecked())
    ui->cb_analysis_cur_lines->setChecked(true);
  else
    ui->cb_analysis_cur_lines->setChecked(false);
  manageLayerVisibility(ui->cb_correct_lines, "clines");
}

/**
 * @brief   LineAnalyser2D::displayGTLinesAnalysis
 *          Displays GT Lines when Checkbox in Analysis section is triggered
 */
void LineAnalyser2D::displayGTLinesAnalysis() {
  if (ui->cb_analysis_gt_lines->isChecked())
    ui->cb_gt_data->setChecked(true);
  else
    ui->cb_gt_data->setChecked(false);
  manageLayerVisibility(ui->cb_analysis_gt_lines, "gtlines");
}

/**
 * @brief   LineAnalyser2D::displayCLinesAnalysis
 *          Displays CLines when Checkbox in Analysis section is triggered
 */
void LineAnalyser2D::displayCLinesAnalysis() {
  if (ui->cb_analysis_cur_lines->isChecked())
    ui->cb_correct_lines->setChecked(true);
  else
    ui->cb_correct_lines->setChecked(false);
  manageLayerVisibility(ui->cb_analysis_cur_lines, "clines");
}

/**
 * @brief   LineAnalyser2D::setLayerVisibility
 *          Manage the visibility of a layer
 * @param   name
 * @param   visibility
 */
void LineAnalyser2D::setLayerVisibility(const char* name, bool visibility) {
  lplot->qplot->layer(name)->setVisible(visibility);
  lplot->replot();
}

/********************************************************* DEBUG PAGE
 * *********************************************************/

void LineAnalyser2D::manageDebugSettings(bool disable) {
  ui->pb_debug_disable->setDisabled(disable);
  ui->pb_debug_lockin_gtline->setDisabled(disable);
  ui->pb_debug_lockin_lines->setDisabled(disable);
  ui->pb_debug_process->setDisabled(disable);
  ui->pb_debug_reset->setDisabled(disable);
  ui->tw_debug_gtline_info->setDisabled(disable);
  ui->tw_debug_lines_info->setDisabled(disable);
  ui->tw_debug_result_info->setDisabled(disable);
}

/**
 * @brief   LineAnalyser2D::enableDebugSettings
 *          Enables settings on debug page
 */
void LineAnalyser2D::enableDebugSettings() {
  if (debugMode) return;

  this->debugMode = true;
  this->manageDebugSettings(false);
}

/**
 * @brief   LineAnalyser2D::diableDebugSettings
 *          Disables settings on debug page
 */
void LineAnalyser2D::disableDebugSettings() {
  this->debugMode = false;
  this->resetDebugSettings();
  this->manageDebugSettings(true);
}

/**
 * @brief   LineAnalyser2D::loadLineInformationIntoTable
 *          Load selected line information into UI Table
 * @param   line
 */
template <class T>
void LineAnalyser2D::loadLineInformationIntoTable(T& line) {
  switch (curSelMode) {
    case 0:
      this->lockedGTLine = &line.segment;
      this->loadGTLineInformation(line);
      break;
    case 1:
      this->lockedOtherLine = &line.segment;
      this->loadOtherLineInformation(line);
      break;
    default:
      break;
  }
}

template <class T>
void LineAnalyser2D::loadGTLineInformation(T& line) {
  int row = 0;
  this->ui->tw_debug_gtline_info->setRowCount(1);
  this->ui->tw_debug_gtline_info->blockSignals(true);
  loadLineInfo(line, ui->tw_debug_gtline_info, row);
  this->ui->tw_debug_gtline_info->blockSignals(false);
}

template <class T>
void LineAnalyser2D::loadOtherLineInformation(T& line) {
  int row = 0;
  this->ui->tw_debug_lines_info->setRowCount(1);
  this->ui->tw_debug_lines_info->blockSignals(true);
  loadLineInfo(line, ui->tw_debug_lines_info, row);
  this->ui->tw_debug_lines_info->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::lockInGTLine
 */
void LineAnalyser2D::lockInGTLine() {
  if (this->curSelMode != 2 && this->lockedGTLine != 0) {
    this->curSelMode = 1;
    this->ui->pb_debug_lockin_gtline->setDisabled(true);
  }
}

/**
 * @brief   LineAnalyser2D::lockInOtherLine
 */
void LineAnalyser2D::lockInOtherLine() {
  if (this->curSelMode != 0 && this->lockedOtherLine != 0) {
    this->curSelMode = 2;
    this->ui->pb_debug_lockin_lines->setDisabled(true);
  }
}

/**
 * @brief   LineAnalyser2D::processDebugData
 *          Compute miscalculation from selected gt line and other line
 */
void LineAnalyser2D::processDebugData() {
  if (lockedGTLine == 0 || lockedOtherLine == 0) return;

  int row = 0;
  this->ui->tw_debug_result_info->setRowCount(1);
  this->ui->tw_debug_result_info->blockSignals(true);

  CLData tmp(*lockedOtherLine, miscalculation(*lockedGTLine, *lockedOtherLine),
             anglediff(*lockedGTLine, *lockedOtherLine), lockedGTLine->length() - lockedOtherLine->length(),
             lockedGTLine->distance(lockedOtherLine->startPoint()), lockedGTLine->distance(lockedOtherLine->endPoint()),
             -1, -1);

  loadErrorInfo(tmp, ui->tw_debug_result_info, row, true);

  this->ui->tw_debug_result_info->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::resetDebugSettings
 *          Reset all debug settings in UI
 */
void LineAnalyser2D::resetDebugSettings() {
  if (lockedGTLine == 0 && lockedOtherLine == 0) return;

  this->curSelMode = 0;
  this->lockedGTLine = 0;
  this->lockedOtherLine = 0;
  this->ui->tw_debug_gtline_info->setRowCount(0);
  this->ui->tw_debug_lines_info->setRowCount(0);
  this->ui->tw_debug_result_info->setRowCount(0);
  this->ui->pb_debug_lockin_gtline->setDisabled(false);
  this->ui->pb_debug_lockin_lines->setDisabled(false);
}

/********************************************************* ANALYSIS PAGE
 * *********************************************************/

/**
 * @brief   LineAnalyser2D::manageAnalysisSettings
 *          Disable or Enable Analysis Settings
 * @param   disable
 */
void LineAnalyser2D::manageAnalysisSettings(bool disable) {
  if (disable) {
    ui->tw_analysis_cur->setRowCount(0);
    ui->tw_analysis_old->setRowCount(0);
  }
  ui->pb_analysis_optimizer->setDisabled(disable);
  ui->pb_analysis_disable->setDisabled(disable);
  ui->pb_analysis_save->setDisabled(disable);
  ui->tw_analysis_cur->setDisabled(disable);
  ui->tw_analysis_old->setDisabled(disable);
  ui->cb_analysis_cur_lines->setDisabled(disable);
  ui->cb_analysis_old_lines->setDisabled(disable);
  ui->cb_analysis_gt_lines->setDisabled(disable);
  ui->pb_analysis_freez_all->setDisabled(disable);
}

/**
 * @brief   LineAnalyser2D::enableAnalysisSettings
 *          Enables Analysis Settings
 */
void LineAnalyser2D::enableAnalysisSettings() {
  analysisMode = true;
  ui->tw_analysis_cur->setDisabled(false);
  ui->tw_analysis_old->setDisabled(false);
  ui->cb_analysis_cur_lines->setDisabled(false);
  ui->cb_analysis_gt_lines->setDisabled(false);
  ui->pb_analysis_disable->setDisabled(false);
  ui->pb_analysis_optimizer->setDisabled(false);
  ui->pb_analysis_freez_all->setDisabled(false);
  if (ui->cb_correct_lines->isChecked()) ui->cb_analysis_cur_lines->setChecked(true);
  if (ui->cb_gt_data->isChecked()) ui->cb_analysis_gt_lines->setChecked(true);
}

/**
 * @brief   LineAnalyser2D::disableAnalysisSettings
 *          Disables all Analysis Settings
 */
void LineAnalyser2D::disableAnalysisSettings() {
  analysisMode = false;
  setLayerVisibility("oldlines", false);
  if (ui->cb_analysis_old_lines->isChecked()) ui->cb_analysis_old_lines->setChecked(false);
  manageAnalysisSettings(true);
}

/**
 * @brief   LineAnalyser2D::loadOldLineInfo
 *          If there are information about old lines they will be loaded into the GUI's table
 */
void LineAnalyser2D::loadOldLineInfo() {
  if (cLinesOld.empty()) return;
  int row = 0;
  this->ui->tw_analysis_old->setRowCount(static_cast<int>(cLinesOld.size()));
  this->ui->tw_analysis_old->blockSignals(true);
  for_each(cLinesOld.begin(), cLinesOld.end(), [&](CLData& cl) {
    loadLineInfo(cl, ui->tw_analysis_old, row);
    loadErrorInfo(cl, ui->tw_analysis_old, row, false);
    ++row;
  });
  this->ui->tw_analysis_old->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::loadCurLineInfo
 *          Loads the Current correct line information into the GUI's table
 */
void LineAnalyser2D::loadCurLineInfo() {
  if (cLines.empty()) return;
  int row = 0;
  this->ui->tw_analysis_cur->setRowCount(static_cast<int>(cLines.size()));
  this->ui->tw_analysis_cur->blockSignals(true);
  for_each(cLines.begin(), cLines.end(), [&](CLData& cl) {
    loadLineInfo(cl, ui->tw_analysis_cur, row);
    loadErrorInfo(cl, ui->tw_analysis_cur, row, false);
    ++row;
  });
  this->ui->tw_analysis_cur->blockSignals(false);
}

/**
 * @brief   LineAnalyser2D::drawOldCLData
 *          Drawss the saved CLines onto a new layer
 */
void LineAnalyser2D::drawOldCLData() {
  if (cLinesOld.empty()) return;

  lplot->qplot->setCurrentLayer("oldlines");
  int clineIndex = 0;
  lplot->hold(true);
  for_each(cLinesOld.begin(), cLinesOld.end(), [&](CLData& cl) {
    drawLines(cl, orange, selPen, "oldlines", clineIndex);
    cl.isDrawn = true;
    ++clineIndex;
  });
  lplot->hold(false);
}

/**
 * @brief   LineAnalyser2D::manageOldLineVisibility
 *          Manages the visibility of the old CLines
 * @param   visibility
 */
void LineAnalyser2D::manageOldLineVisibility(bool visibility) {
  for_each(cLinesOld.begin(), cLinesOld.end(), [&](CLData& cl) {
    if (cl.isDrawn) {
      cl.line->setVisible(visibility);
      cl.line->setSelected(false);
    }
  });
}

/**
 * @brief   LineAnalyser2D::displayOldCLines
 *          Displays the old CLines in given plot window
 */
void LineAnalyser2D::displayOldCLines() { manageLayerVisibility(ui->cb_analysis_old_lines, "oldlines"); }

/**
 * @brief   LineAnalyser2D::updateLines
 *          Updates all lines, computed new correct lines
 *          and loads all information in given GUI tables
 * @param   line
 */
void LineAnalyser2D::updateLines(LineVector* line) {
  lines = line;
  if (!ui->pb_analysis_save->isEnabled()) ui->pb_analysis_save->setDisabled(false);
  if (!ui->cb_analysis_old_lines->isEnabled()) ui->cb_analysis_old_lines->setDisabled(false);
  this->ui->tw_analysis_cur->setRowCount(0);
  this->ui->tw_analysis_old->setRowCount(0);
  if (!cLinesOld.empty()) cLinesOld.clear();
  for_each(cLines.begin(), cLines.end(), [&](CLData& cl) { cLinesOld.push_back(cl); });
  computeCorrectLines();
  redrawItems();
  loadOldLineInfo();
  loadCurLineInfo();
}

/**
 * @brief   LineAnalyser2D::markTableRow
 *          Marks the table row for the selected line from given plot window
 * @param   index
 */
void LineAnalyser2D::markTableRow(int index) {
  if (static_cast<int>(cLines.size()) == static_cast<int>(cLinesOld.size())) {
    ui->tw_analysis_cur->selectRow(index);
    ui->tw_analysis_old->selectRow(index);
  }
}

/**
 * @brief   LineAnalyser2D::showCurLineReference
 * @param   row
 */
void LineAnalyser2D::showCurLineReference(int row) { showCurLineReference(row, 0); }

/**
 * @brief   LineAnalyser2D::showOldLineReference
 * @param   row
 */
void LineAnalyser2D::showOldLineReference(int row) { showOldLineReference(row, 0); }

/**
 * @brief   LineAnalyser2D::showCurLineReference
 * @param   row
 * @param   col
 */
void LineAnalyser2D::showCurLineReference(int row, int col) {
  if (clDataTemp) clDataTemp->line->setSelected(false);
  manageCLineVisibility(true);
  clDataTemp = &cLines.at(row);
  clDataTemp->line->setSelected(true);
  ui->cb_correct_lines->setChecked(true);
  ui->cb_analysis_cur_lines->setChecked(true);
  setLayerVisibility("clines", true);
}

/**
 * @brief   LineAnalyser2D::showOldLineReference
 * @param   row
 * @param   col
 */
void LineAnalyser2D::showOldLineReference(int row, int col) {
  if (clDataTemp) clDataTemp->line->setSelected(false);
  manageOldLineVisibility(true);
  clDataTemp = &cLinesOld.at(row);
  clDataTemp->line->setSelected(true);
  ui->cb_analysis_old_lines->setChecked(true);
  setLayerVisibility("oldlines", true);
}

/**
 * @brief   LineAnalyser2D::showOptimizer
 *          Opens the Precision Optimizer
 */
void LineAnalyser2D::showOptimizer() { po->show(); }

/**
 * @brief   LineAnalyser2D::loadSaveFileHeader
 *          Load the header information for the output file
 * @param   outStream
 * @param   caption
 */
void LineAnalyser2D::loadSaveFileHeader(QTextStream& outStream, QString caption) {
  outStream << "\\begin{table}" << "\n";
  outStream << "\t" << "\\caption{" << caption << "}" << "\n";
  outStream << "\t" << "\\begin{tabular}{|c|c|c|c|c|c|c||c|}" << "\n";
  outStream << "\t" << "\\hline" << "\n";
  outStream << "\t" << "\t";
  outStream << "\\textbf{Nr.}" << " & ";
  outStream << "\\textbf{angle}" << " & ";
  outStream << "\\textbf{length}" << " & ";
  outStream << "\\textbf{start\\_x}" << " & ";
  outStream << "\\textbf{start\\_y}" << " & ";
  outStream << "\\textbf{end\\_x}" << " & ";
  outStream << "\\textbf{end\\_y}" << " & ";
  outStream << "\\textbf{error}" << "\t" << "\\\\" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
}

/**
 * @brief   LineAnalyser2D::loadSaveFileContent
 *          Load the content information for the output file
 * @param   outStream
 * @param   cl
 * @param   i
 */
void LineAnalyser2D::loadSaveFileContent(QTextStream& outStream, CLData& cl, int& i) {
  outStream << "\t" << "\t";
  outStream << i << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.angle() * 180 / CV_PI << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.length() << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.startPoint().x << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.startPoint().y << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.endPoint().x << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.segment.endPoint().y << "$" << "\t" << "&" << "\t";
  outStream << "$" << cl.error << "$" << "\t" << "\\\\" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
}

/**
 * @brief   LineAnalyser2D::loadSaveFileFooter
 *          Load the footer information for the output file
 * @param   outStream
 */
void LineAnalyser2D::loadSaveFileFooter(QTextStream& outStream) {
  outStream << "\t" << "\\end{tabular}" << "\n";
  outStream << "\\end{table}" << "\n";
}

/**
 * @brief   LineAnalyser2D::loadPercentageVarianceHeader
 *          Writes the percentage variance header in a file
 * @param   outStream
 */
void LineAnalyser2D::loadPercentageVarianceHeader(QTextStream& outStream) {
  outStream << "\\begin{table}" << "\n";
  outStream << "\t" << "\\caption{Percentage Variance}" << "\n";
  outStream << "\t" << "\\begin{tabular}{|c|c|c|c|}" << "\n";
  outStream << "\t" << "\\hline" << "\n";
  outStream << "\t" << "\t";
  outStream << "\\textbf{Nr.}" << " & ";
  outStream << "\\textbf{Error Prev.}" << " & ";
  outStream << "\\textbf{Error Opti.}" << " & ";
  outStream << "\\textbf{Percent}" << "\t" << "\\\\" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
}

/**
 * @brief   LineAnalyser2D::loadPercentageVarianceContent
 *          Wirtes the percentage variance into a file
 * @param   outStream
 * @param   i
 */
void LineAnalyser2D::loadPercentageVarianceContent(QTextStream& outStream, int& i) {
  outStream << "\t" << "\t";
  outStream << i + 1 << "\t" << "&" << "\t";
  outStream << "$" << cLinesOld.at(i).error << "$" << "\t" << "&" << "\t";
  outStream << "$" << cLines.at(i).error << "$" << "\t" << "&" << "\t";
  outStream << "$" << calculatePercentage(i) << "$" << "\t" << "\\\\" << "\n";
  outStream << "\t" << "\t" << "\\hline" << "\n";
}

/**
 * @brief   LineAnalyser2D::calculatePercentage
 *          Calculate the Percentage Variance of two error values
 * @param   i
 * @return
 */
double LineAnalyser2D::calculatePercentage(int& i) {
  return ((cLinesOld.at(i).error - cLines.at(i).error) / cLinesOld.at(i).error) * 100;
}

/**
 * @brief   LineAnalyser2D::saveAnalysis
 *          Saves the old and current CLine information in LaTeX Code
 *          which can be copied from a .txt file.
 */
void LineAnalyser2D::saveAnalysis() {
  if (cLines.empty() || cLinesOld.empty()) return;

  QString filename = "";
  filename = file->getSaveFileName(this, "Save File", "", ".txt");
  if (filename != "") {
    int index = 1;
    QFile f(filename);
    QTextStream outStream(&f);
    f.open(QIODevice::WriteOnly);
    loadSaveFileHeader(outStream, "Previous Line Information");
    for_each(cLinesOld.begin(), cLinesOld.end(), [&](CLData& cl) {
      loadSaveFileContent(outStream, cl, index);
      ++index;
    });
    loadSaveFileFooter(outStream);
    index = 1;
    loadSaveFileHeader(outStream, "Optimized Line Information");
    for_each(cLines.begin(), cLines.end(), [&](CLData& cl) {
      loadSaveFileContent(outStream, cl, index);
      ++index;
    });
    loadSaveFileFooter(outStream);
    if (static_cast<int>(cLinesOld.size()) == static_cast<int>(cLines.size())) {
      index = 0;
      loadPercentageVarianceHeader(outStream);
      for_each(cLines.begin(), cLines.end(), [&](CLData& cl) {
        loadPercentageVarianceContent(outStream, index);
        ++index;
      });
      loadSaveFileFooter(outStream);
    }
    f.close();
  }
}
