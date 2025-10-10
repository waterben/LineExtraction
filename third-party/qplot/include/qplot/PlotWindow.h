#pragma once

#include "qplot/qcustomplot.h"
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <geometry/line.hpp>
#include <opencv2/opencv.hpp>

#include <QMainWindow>
#include <vector>

class PlotWindow : public QMainWindow {
  Q_OBJECT
  Q_DISABLE_COPY(PlotWindow)

  QAction* actionInsert_Plot{nullptr};
  QAction* actionSave_Document{nullptr};
  QWidget* centralWidget{nullptr};
  QGridLayout* gridLayout{nullptr};
  QScrollBar* verticalScrollBar{nullptr};
  QScrollBar* horizontalScrollBar{nullptr};
  QStatusBar* statusBar{nullptr};

  bool hold_;
  bool keep_aspect_;
  double ratio_;

  void resizeEvent(QResizeEvent* event);

  inline void doAspectRatio() {
    double r = static_cast<double>(qplot->width()) / qplot->height();
    if (r < ratio_)
      qplot->yAxis->setScaleRatio(qplot->xAxis);
    else
      qplot->xAxis->setScaleRatio(qplot->yAxis);
  }

 public:
  explicit PlotWindow(const char* title = "", QWidget* parent = 0);
  ~PlotWindow();
  // direct access to custom plot
  QCustomPlot* qplot{nullptr};
  double scale_x{}, scale_y{};
  double border_x{}, border_y{};

  //! reset plot and set back to initial setup
  void reset();

  //! reverse y axis
  inline void reverseY(bool val) { qplot->yAxis->setRangeReversed(val); }

  //! set axis dimensions
  void setAxis(const QCPRange& r_x, const QCPRange& r_y);
  void setAxis(double start_x, double end_x, double start_y, double end_y);
  void setAxis(double start_x, double end_x, double start_y, double end_y, double scale_x, double scale_y);
  void setAxis(double start_x,
               double end_x,
               double start_y,
               double end_y,
               double scale_x,
               double scale_y,
               double border_x,
               double border_y);

  //! clear alls plotted data
  inline void clearAxis() {
    qplot->clearGraphs();
    qplot->clearPlottables();
    qplot->clearItems();
  }

  //! replot data (first add all graphs etc., than replot)
  inline void replot() {
    if (keep_aspect_) doAspectRatio();
    qplot->replot();
  }

  //! active hold to plot multiple graphs in plot window
  inline void hold(bool on = true) { hold_ = on; }

  //! set keep aspect ratio
  inline void keepAspectRatio(bool ar = true) { keep_aspect_ = ar; }

  //! set keep aspect ratio and ratio to keep
  inline void keepAspectRatio(bool ar, double ratio) {
    keep_aspect_ = ar;
    ratio_ = ratio;
  }

  //! check hold state
  inline bool isHold() const { return hold_; }

  //! plot X vector versus Y vector
  template <typename T>
  QCPGraph* plot(const std::vector<T>& X,
                 const std::vector<T>& Y,
                 bool fit_axis = false,
                 const QPen pen = QPen(Qt::blue),
                 const QBrush brush = QBrush());

  QCPGraph* plot(const QVector<double>& X,
                 const QVector<double>& Y,
                 bool fit_axis = false,
                 const QPen pen = QPen(Qt::blue),
                 const QBrush brush = QBrush());

  //! plot mat as QCPColorMap
  QCPColorMap* plot(const cv::Mat& mat,
                    bool fit_axis = false,
                    const QCPRange& prange_x = QCPRange(),
                    const QCPRange& prange_y = QCPRange(),
                    const QCPRange& drange = QCPRange(),
                    const QCPColorGradient& grad = QCPColorGradient::gpGrayscale,
                    QCPAxis::ScaleType st = QCPAxis::stLinear,
                    bool interpolate = false);
  QCPColorMap* plotRGB(const cv::Mat& mat,
                       bool fit_axis = false,
                       QCPAxis::ScaleType st = QCPAxis::stLinear,
                       bool interpolate = false);

  //! plot line
  template <typename T>
  QCPItemLine* line(T x1, T y1, T x2, T y2, bool fit_axis = false, const QPen pen = QPen(Qt::blue));

  //! plot line segement
  template <typename FT, template <class> class LPT>
  QCPItemLine* line(const lsfm::LineSegment<FT, LPT>& line, bool fit_axis = false, const QPen pen = QPen(Qt::blue));

 private slots:
  void horzScrollBarChanged(int value);
  void vertScrollBarChanged(int value);
  void xAxisChanged(QCPRange range);
  void yAxisChanged(QCPRange range);
};

template <typename T>
inline QCPGraph* PlotWindow::plot(
    const std::vector<T>& X, const std::vector<T>& Y, bool fit_axis, const QPen pen, const QBrush brush) {
  // simple convert to double
  std::vector<double> dX(X.begin(), X.end()), dY(Y.begin(), Y.end());

  if (fit_axis) {
    auto res_Y = std::minmax_element(dY.begin(), dY.end());
    auto res_X = std::minmax_element(dX.begin(), dX.end());
    setAxis(static_cast<double>(*res_X.first), static_cast<double>(*res_X.second), static_cast<double>(*res_Y.first),
            static_cast<double>(*res_Y.second));
  }

  if (!hold_) {
    clearAxis();
  }
  QCPGraph* graph = qplot->addGraph();
  graph->setPen(pen);
  graph->setBrush(brush);
  graph->setData(QVector<double>(dX.begin(), dX.end()), QVector<double>(dY.begin(), dY.end()));
  return graph;
}

template <>
inline QCPGraph* PlotWindow::plot(
    const std::vector<double>& X, const std::vector<double>& Y, bool fit_axis, const QPen pen, const QBrush brush) {
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
  graph->setData(QVector<double>(X.begin(), X.end()), QVector<double>(Y.begin(), Y.end()));
  return graph;
}

template <typename T>
inline QCPItemLine* PlotWindow::line(T x1, T y1, T x2, T y2, bool fit_axis, const QPen pen) {
  if (fit_axis) {
    double ymax = y1, ymin = y1, xmax = x1, xmin = x2;
    if (ymax < ymin) {
      T tmp = ymin;
      ymin = ymax;
      ymax = tmp;
    }
    if (xmax < xmin) {
      T tmp = xmin;
      xmin = xmax;
      xmax = tmp;
    }
    setAxis(xmin, xmax, ymin, ymax);
  }

  if (!hold_) {
    clearAxis();
  }

  QCPItemLine* l = new QCPItemLine(qplot);
  qplot->addItem(l);
  l->setPen(pen);
  l->start->setCoords(x1, y1);
  l->end->setCoords(x2, y2);
  return l;
}

template <typename FT, template <class> class LPT>
inline QCPItemLine* PlotWindow::line(const lsfm::LineSegment<FT, LPT>& l, bool fit_axis, const QPen pen) {
  lsfm::Vec<FT, 4> line_data = l.endPoints();
  return line(line_data[0], line_data[1], line_data[2], line_data[3], fit_axis, pen);
}
