#include <qplot/PlotWindow.h>

#include <iomanip>
#include <set>
using namespace std;

PlotWindow::PlotWindow(const char* title, QWidget* parent) : QMainWindow(parent) {
  if (objectName().isEmpty()) setObjectName(QStringLiteral("plotWindow"));

  actionInsert_Plot = new QAction(this);
  actionInsert_Plot->setObjectName(QStringLiteral("actionInsert_Plot"));
  actionSave_Document = new QAction(this);
  actionSave_Document->setObjectName(QStringLiteral("actionSave_Document"));
  centralWidget = new QWidget(this);
  centralWidget->setObjectName(QStringLiteral("centralWidget"));
  gridLayout = new QGridLayout(centralWidget);
  gridLayout->setSpacing(6);
  gridLayout->setContentsMargins(11, 11, 11, 11);
  gridLayout->setObjectName(QStringLiteral("gridLayout"));
  qplot = new QCustomPlot(centralWidget);
  qplot->setObjectName(QStringLiteral("plot"));
  QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  sizePolicy.setHorizontalStretch(0);
  sizePolicy.setVerticalStretch(0);
  sizePolicy.setHeightForWidth(qplot->sizePolicy().hasHeightForWidth());
  qplot->setSizePolicy(sizePolicy);

  gridLayout->addWidget(qplot, 0, 0, 1, 1);

  verticalScrollBar = new QScrollBar(centralWidget);
  verticalScrollBar->setObjectName(QStringLiteral("verticalScrollBar"));
  verticalScrollBar->setOrientation(Qt::Vertical);

  gridLayout->addWidget(verticalScrollBar, 0, 1, 1, 1);

  horizontalScrollBar = new QScrollBar(centralWidget);
  horizontalScrollBar->setObjectName(QStringLiteral("horizontalScrollBar"));
  horizontalScrollBar->setOrientation(Qt::Horizontal);

  gridLayout->addWidget(horizontalScrollBar, 1, 0, 1, 1);

  setCentralWidget(centralWidget);
  statusBar = new QStatusBar(this);
  statusBar->setObjectName(QStringLiteral("statusBar"));
  setStatusBar(statusBar);

  setWindowTitle(QApplication::translate("PlotWindow", title, 0));
  actionInsert_Plot->setText(QApplication::translate("PlotWindow", "Insert Plot", 0));
  actionSave_Document->setText(QApplication::translate("PlotWindow", "Save Document...", 0));

  QMetaObject::connectSlotsByName(this);

  // create connection between axes and scroll bars:
  connect(horizontalScrollBar, SIGNAL(valueChanged(int)), this, SLOT(horzScrollBarChanged(int)));
  connect(verticalScrollBar, SIGNAL(valueChanged(int)), this, SLOT(vertScrollBarChanged(int)));
  connect(qplot->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxisChanged(QCPRange)));
  connect(qplot->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxisChanged(QCPRange)));

  reset();
}

PlotWindow::~PlotWindow() {}

void PlotWindow::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  if (keep_aspect_) doAspectRatio();
}

void PlotWindow::reset() {
  resize(640, 480);

  keep_aspect_ = false;
  ratio_ = 4.0 / 3.0;
  hold_ = false;
  scale_x = 100;
  scale_y = 100;
  border_x = 0;
  border_y = 0;

  qplot->clearFocus();
  qplot->clearItems();
  qplot->clearMask();
  qplot->clearGraphs();
  qplot->clearPlottables();

  horizontalScrollBar->setRange(0, 500);
  verticalScrollBar->setRange(0, 500);

  // initialize axis range (and scroll bar positions via signals we just connected):
  qplot->xAxis->setRange(0, 5);
  qplot->yAxis->setRange(0, 5);

  horizontalScrollBar->setValue(
      qRound(qplot->xAxis->range().center() * scale_x));  // adjust position of scroll bar slider
  horizontalScrollBar->setPageStep(qRound(qplot->xAxis->range().size() * scale_x));  // adjust size of scroll bar slider

  verticalScrollBar->setValue(
      qRound(qplot->yAxis->range().center() * scale_y));  // adjust position of scroll bar slider
  verticalScrollBar->setPageStep(qRound(qplot->yAxis->range().size() * scale_y));  // adjust size of scroll bar slider

  qplot->axisRect()->setupFullAxesBox(true);
  qplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

void PlotWindow::setAxis(const QCPRange& r_x, const QCPRange& r_y) {
  setAxis(r_x.lower, r_x.upper, r_y.lower, r_y.upper, scale_x, scale_y, border_x, border_y);
}

void PlotWindow::setAxis(double start_x, double end_x, double start_y, double end_y) {
  setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y, border_x, border_y);
}

void PlotWindow::setAxis(double start_x, double end_x, double start_y, double end_y, double scale_x, double scale_y) {
  setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y, border_x, border_y);
}

void PlotWindow::setAxis(double start_x,
                         double end_x,
                         double start_y,
                         double end_y,
                         double scale_x,
                         double scale_y,
                         double border_x,
                         double border_y) {
  this->scale_x = scale_x;
  this->scale_y = scale_y;
  this->border_x = border_x;
  this->border_y = border_y;

  if (end_x < start_x) swap(start_x, end_x);

  if (end_y < start_y) swap(start_y, end_y);

  start_x -= border_x;
  end_x += border_x;
  start_y -= border_y;
  end_y += border_y;

  double ysc_start = start_y * scale_y, ysc_end = end_y * scale_y, xsc_start = start_x * scale_x,
         xsc_end = end_x * scale_x;

  int x_start = xsc_start < 0 ? qFloor(xsc_start) : qCeil(xsc_start);
  int x_end = xsc_end < 0 ? qFloor(xsc_end) : qCeil(xsc_end);
  int y_start = ysc_start < 0 ? qFloor(ysc_start) : qCeil(ysc_start);
  int y_end = ysc_end < 0 ? qFloor(ysc_end) : qCeil(ysc_end);

  // configure scroll bars:
  horizontalScrollBar->setRange(x_start, x_end);
  verticalScrollBar->setRange(y_start, y_end);

  // initialize axis range (and scroll bar positions via signals we just connected):
  qplot->xAxis->setRange(start_x, end_x);
  qplot->yAxis->setRange(start_y, end_y);
}

QCPGraph* PlotWindow::plot(
    const QVector<double>& X, const QVector<double>& Y, bool fit_axis, const QPen pen, const QBrush brush) {
  if (fit_axis) {
    auto res_Y = std::minmax_element(Y.begin(), Y.end());
    auto res_X = std::minmax_element(X.begin(), X.end());
    setAxis(*res_X.first, *res_X.second, *res_Y.first, *res_Y.second);
  }

  if (!hold_) {
    clearAxis();
  }
  QCPGraph* graph = qplot->addGraph();
  graph->setPen(pen);
  graph->setBrush(brush);
  graph->setData(X, Y);
  return graph;
}

QCPColorMap* PlotWindow::plot(const cv::Mat& mat,
                              bool fit_axis,
                              const QCPRange& prange_x,
                              const QCPRange& prange_y,
                              const QCPRange& drange,
                              const QCPColorGradient& grad,
                              QCPAxis::ScaleType st,
                              bool interpolate) {
  cv::Mat src = mat.clone();
  if (mat.type() == CV_8UC3)
    return plotRGB(mat, fit_axis, st, interpolate);
  else if (mat.channels() != 1) {
    std::vector<cv::Mat> channels(mat.channels());
    split(mat, channels);
    src = channels[0];
  }

  src.convertTo(src, CV_64FC1);

  QCPRange dr = drange;
  // no range, autodetect
  if (dr.size() == 0) {
    double rmin, rmax;
    minMaxIdx(src, &rmin, &rmax);
    // force symmetry
    if (rmin < 0) {
      rmax = std::max(rmax, -rmin);
      rmin = -rmax;
    } else
      rmin = 0;
    dr = QCPRange(rmin, rmax);
  }

  QCPColorMap* colorMap = new QCPColorMap(qplot->xAxis, qplot->yAxis);
  int nx = src.cols;
  int ny = src.rows;
  colorMap->data()->setSize(nx, ny);  // we want the color map to have nx * ny data points

  QCPRange prx = prange_x, pry = prange_y;
  // no range, autodetect
  if (prx.size() == 0) {
    prx = QCPRange(0, nx - 1);
  }
  if (pry.size() == 0) {
    pry = QCPRange(0, ny - 1);
  }

  colorMap->data()->setRange(prx, pry);
  // now we assign some data, by accessing the QCPColorMapData instance of the color map:
  for (int xIndex = 0; xIndex < nx; ++xIndex) {
    for (int yIndex = 0; yIndex < ny; ++yIndex) {
      colorMap->data()->setCell(xIndex, yIndex, src.at<double>(yIndex, xIndex));
    }
  }

  colorMap->setInterpolate(interpolate);
  colorMap->setDataScaleType(st);
  colorMap->setDataRange(dr);
  colorMap->setGradient(grad);

  if (!hold_) clearAxis();
  qplot->addPlottable(colorMap);

  if (fit_axis) setAxis(prx, pry);
  return colorMap;
}

QCPColorMap* PlotWindow::plotRGB(const cv::Mat& mat, bool fit_axis, QCPAxis::ScaleType st, bool interpolate) {
  if (mat.type() != CV_8UC3)
    return plot(mat, fit_axis, QCPRange(), QCPRange(), QCPRange(), QCPColorGradient::gpGrayscale, st, interpolate);

  QCPColorMap* colorMap = new QCPColorMap(qplot->xAxis, qplot->yAxis);
  int nx = mat.cols;
  int ny = mat.rows;
  colorMap->data()->setSize(nx, ny);  // we want the color map to have nx * ny data points

  QCPRange prx(0, nx - 1);
  QCPRange pry(0, ny - 1);
  std::set<unsigned int> tmp;

  unsigned int minVal = 0xffffffff, maxVal = 0;
  colorMap->data()->setRange(prx, pry);
  // now we assign some data, by accessing the QCPColorMapData instance of the color map:
  for (int xIndex = 0; xIndex < nx; ++xIndex) {
    for (int yIndex = 0; yIndex < ny; ++yIndex) {
      cv::Vec3b data = mat.at<cv::Vec3b>(yIndex, xIndex);
      unsigned int val = (data[2] << 16) | (data[1] << 8) | data[0];
      minVal = min(minVal, val);
      maxVal = max(maxVal, val);
      tmp.insert(val);
      colorMap->data()->setCell(xIndex, yIndex, val);
    }
  }

  QCPRange dr(minVal, maxVal);
  double diff = (maxVal - minVal);

  QCPColorGradient grad;
  grad.clearColorStops();
  grad.setColorInterpolation(QCPColorGradient::ciRGB);
  grad.setLevelCount(static_cast<int>(tmp.size()));

  for_each(tmp.begin(), tmp.end(), [&](unsigned int d) {
    grad.setColorStopAt((d - minVal) / diff, QColor((d >> 16) & 0x000000ff, (d >> 8) & 0x000000ff, d & 0x000000ff));
  });


  colorMap->setInterpolate(interpolate);
  colorMap->setDataScaleType(st);
  colorMap->setDataRange(dr);
  colorMap->setGradient(grad);

  if (!hold_) clearAxis();
  qplot->addPlottable(colorMap);

  if (fit_axis) setAxis(prx, pry);
  return colorMap;
}

void PlotWindow::horzScrollBarChanged(int value) {
  QCPRange r = qplot->xAxis->range();
  double v = value;
  v /= scale_x;
  v -= r.center();
  qplot->xAxis->blockSignals(true);
  qplot->xAxis->setRange(r.lower + v, r.upper + v);
  qplot->xAxis->blockSignals(false);
  qplot->replot();
}

void PlotWindow::vertScrollBarChanged(int value) {
  QCPRange r = qplot->yAxis->range();
  double v = value;
  v /= scale_y;
  v -= r.center();
  qplot->yAxis->blockSignals(true);
  qplot->yAxis->setRange(r.lower + v, r.upper + v);
  qplot->yAxis->blockSignals(false);
  qplot->replot();
}

void PlotWindow::xAxisChanged(QCPRange range) {
  horizontalScrollBar->blockSignals(true);
  horizontalScrollBar->setValue(qRound(range.center() * scale_x));   // adjust position of scroll bar slider
  horizontalScrollBar->setPageStep(qRound(range.size() * scale_x));  // adjust size of scroll bar slider
  horizontalScrollBar->blockSignals(false);
}

void PlotWindow::yAxisChanged(QCPRange range) {
  verticalScrollBar->blockSignals(true);
  verticalScrollBar->setValue(qRound(range.center() * scale_y));   // adjust position of scroll bar slider
  verticalScrollBar->setPageStep(qRound(range.size() * scale_y));  // adjust size of scroll bar slider
  verticalScrollBar->blockSignals(false);
}
