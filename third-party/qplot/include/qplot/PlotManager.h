#pragma once

#include <qplot/PlotWindow.h>

#include <map>


class PlotManager {
  size_t id_count_;
  std::string active_;
  std::map<std::string, PlotWindow*> fmap_;
  QWidget* parent_;

  std::string new_name();

 public:
  PlotManager(QWidget* parent = 0) : id_count_(0), parent_(parent) {}
  ~PlotManager() { closeAll(); }

  // get active figure name
  inline std::string getName() const { return active_; }

  // get list of figure names
  std::vector<std::string> nameList() const;

  //! get figure pointer (create new if not exist), no name, get active_/create active_
  PlotWindow* getFigure(const std::string& name = std::string());
  // select figure, return true if figure exists
  bool setFigure(const std::string& name);

  // close figure and reset plot
  void close(const std::string& name = std::string());

  void closeAll();

  // set figure title
  inline void setTitle(const std::string& title) {
    PlotWindow* plot = getFigure();
    return plot->setWindowTitle(title.c_str());
  }
  inline void setTitle(const std::string& name, const std::string& title) {
    PlotWindow* plot = getFigure(name);
    return plot->setWindowTitle(title.c_str());
  }

  //! hold to plot multiple graphs in figure
  inline void hold(bool on = true) {
    PlotWindow* plot = getFigure();
    return plot->hold(on);
  }
  inline void hold(const std::string& name, bool on = true) {
    PlotWindow* plot = getFigure(name);
    return plot->hold(on);
  }

  //! check hold state
  inline bool isHold(const std::string& name = std::string()) {
    PlotWindow* plot = getFigure(name);
    return plot->isHold();
  }

  // set axis dimensions
  inline void setAxis(const QCPRange& r_x, const QCPRange& r_y) {
    PlotWindow* plot = getFigure();
    plot->setAxis(r_x, r_y);
  }
  inline void setAxis(double start_x, double end_x, double start_y, double end_y) {
    PlotWindow* plot = getFigure();
    plot->setAxis(start_x, end_x, start_y, end_y);
  }
  inline void setAxis(double start_x, double end_x, double start_y, double end_y, double scale_x, double scale_y) {
    PlotWindow* plot = getFigure();
    plot->setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y);
  }
  inline void setAxis(double start_x,
                      double end_x,
                      double start_y,
                      double end_y,
                      double scale_x,
                      double scale_y,
                      double border_x,
                      double border_y) {
    PlotWindow* plot = getFigure();
    plot->setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y, border_x, border_y);
  }

  inline void setAxis(const std::string& name, const QCPRange& r_x, const QCPRange& r_y) {
    PlotWindow* plot = getFigure(name);
    plot->setAxis(r_x, r_y);
  }
  inline void setAxis(const std::string& name, double start_x, double end_x, double start_y, double end_y) {
    PlotWindow* plot = getFigure(name);
    plot->setAxis(start_x, end_x, start_y, end_y);
  }
  inline void setAxis(const std::string& name,
                      double start_x,
                      double end_x,
                      double start_y,
                      double end_y,
                      double scale_x,
                      double scale_y) {
    PlotWindow* plot = getFigure(name);
    plot->setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y);
  }
  inline void setAxis(const std::string& name,
                      double start_x,
                      double end_x,
                      double start_y,
                      double end_y,
                      double scale_x,
                      double scale_y,
                      double border_x,
                      double border_y) {
    PlotWindow* plot = getFigure(name);
    plot->setAxis(start_x, end_x, start_y, end_y, scale_x, scale_y, border_x, border_y);
  }

  //! clear alls plotted data
  inline void clearAxis(const std::string& name = std::string()) {
    PlotWindow* plot = getFigure(name);
    plot->clearAxis();
  }

  inline void replot(const std::string& name = std::string()) {
    PlotWindow* plot = getFigure(name);
    plot->replot();
  }

  //! plot X vector versus Y vector
  template <typename T>
  inline void plot(const std::vector<T>& X,
                   const std::vector<T>& Y,
                   bool fit_axis = false,
                   const QPen pen = QPen(Qt::blue),
                   const QBrush brush = QBrush()) {
    PlotWindow* plot = getFigure();
    plot->plot(X, Y, fit_axis, pen, brush);
  }

  inline void plot(const QVector<double>& X,
                   const QVector<double>& Y,
                   bool fit_axis = false,
                   const QPen pen = QPen(Qt::blue),
                   const QBrush brush = QBrush()) {
    PlotWindow* plot = getFigure();
    plot->plot(X, Y, fit_axis, pen, brush);
  }

  //! plot line segement
  template <typename T>
  void plot(const lsfm::LineSegment<T>& line, bool fit_axis = false, const QPen pen = QPen(Qt::blue)) {
    PlotWindow* plot = getFigure();
    plot->plot(line, fit_axis, pen);
  }
  //! plot mat as QCPColorMap
  inline void plot(const cv::Mat& mat,
                   bool fit_axis = false,
                   const QCPRange& prange_x = QCPRange(),
                   const QCPRange& prange_y = QCPRange(),
                   const QCPRange& drange = QCPRange(),
                   const QCPColorGradient& grad = QCPColorGradient::gpGrayscale,
                   QCPAxis::ScaleType st = QCPAxis::stLinear,
                   bool interpolate = false) {
    PlotWindow* plot = getFigure();
    plot->plot(mat, fit_axis, prange_x, prange_y, drange, grad, st, interpolate);
  }

  //! plot X vector versus Y vector
  template <typename T>
  inline void plot(const std::string& name,
                   const std::vector<T>& X,
                   const std::vector<T>& Y,
                   bool fit_axis = false,
                   const QPen pen = QPen(Qt::blue),
                   const QBrush brush = QBrush()) {
    PlotWindow* plot = getFigure(name);
    plot->plot(X, Y, fit_axis, pen, brush);
  }

  inline void plot(const std::string& name,
                   const QVector<double>& X,
                   const QVector<double>& Y,
                   bool fit_axis = false,
                   const QPen pen = QPen(Qt::blue),
                   const QBrush brush = QBrush()) {
    PlotWindow* plot = getFigure(name);
    plot->plot(X, Y, fit_axis, pen, brush);
  }

  //! plot line
  template <typename T>
  inline void line(
      const std::string& name, T x1, T y1, T x2, T y2, bool fit_axis = false, const QPen pen = QPen(Qt::blue)) {
    PlotWindow* plot = getFigure(name);
    plot->line(x1, y1, x2, y2, fit_axis, pen);
  }

  //! plot line segement
  template <typename T>
  inline void line(const std::string& name,
                   const lsfm::LineSegment<T>& line,
                   bool fit_axis = false,
                   const QPen pen = QPen(Qt::blue)) {
    PlotWindow* plot = getFigure(name);
    plot->line(line, fit_axis, pen);
  }

  //! plot mat as QCPColorMap
  inline void plot(const std::string& name,
                   const cv::Mat& mat,
                   bool fit_axis = false,
                   const QCPRange& prange_x = QCPRange(),
                   const QCPRange& prange_y = QCPRange(),
                   const QCPRange& drange = QCPRange(),
                   const QCPColorGradient& grad = QCPColorGradient::gpGrayscale,
                   QCPAxis::ScaleType st = QCPAxis::stLinear,
                   bool interpolate = false) {
    PlotWindow* plot = getFigure(name);
    plot->plot(mat, fit_axis, prange_x, prange_y, drange, grad, st, interpolate);
  }
};
