#pragma once

#include "helpers.h"
#include "latool.h"
#include <qplot/PlotWindow.h>

namespace Ui {
class ProfileAnalyzer;
}

class ProfileAnalyzer : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(ProfileAnalyzer)

  PlotWindow* plot{nullptr};
  const ImageSources* sources{nullptr};
  ControlWindow* cw{nullptr};
  lsfm::LineSegment2d line{};

  QVector<double> X{};
  QVector<double> profile{};
  QVector<double> std_dev{};
  QVector<double> single_profile{};
  double linePos{0.0};
  double p_min{0.0};
  double p_max{0.0};
  double sp_min{0.0};
  double sp_max{0.0};
  QCPGraph* profile_plot{nullptr};
  QCPGraph* sprofile_plot{nullptr};
  QCPGraph* std_dev_upper{nullptr};
  QCPGraph* std_dev_lower{nullptr};
  QCPItemLine* pa_indicator{nullptr};

 public:
  explicit ProfileAnalyzer(QWidget* parent = nullptr);
  ~ProfileAnalyzer();
  void connectTools(ControlWindow* w);

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
  Ui::ProfileAnalyzer* ui{nullptr};

  void createX();
  void createProfile();
  void createSProfile();

  void plotProfile();
  void plotSProfile();
};
