#include "quiverpanel.h"

#include "help_button.hpp"
#include <qplot/PlotWindow.h>

#include <QMessageBox>
#include <cmath>
#include <iostream>
#include <limits>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

QuiverPanel::QuiverPanel(QWidget* parent)
    : LATool("Quivers", parent), ui(new Ui::QuiverPanel), colorDialog(new QColorDialog(this)), quiverPen(Qt::green) {
  ui->setupUi(this);
  setWindowTitle("Quivers");

  ui->cb_interpolate->addItem("CV_INTER_NN");
  ui->cb_interpolate->addItem("CV_INTER_LINEAR");
  ui->cb_interpolate->addItem("CV_INTER_CUBIC");
  ui->cb_interpolate->addItem("CV_INTER_AREA");
  ui->cb_interpolate->addItem("CV_INTER_LANCZOS4");
  ui->cb_interpolate->setCurrentIndex(3);

  ui->pb_color->setStyleSheet("background-color: rgb(0,255,0);");

  // Tooltips for all controls.
  setToolTip(
      tr("Display gradient vector fields (quivers) as arrows "
         "overlaid on the image."));
  ui->rb_gradient->setToolTip(tr("Display raw horizontal (Gx) and vertical (Gy) gradient components."));
  ui->rb_direction->setToolTip(tr("Display edge direction vectors (perpendicular to gradient)."));
  ui->rb_phase->setToolTip(tr("Display gradient phase angles as arrows."));
  ui->rb_phase_dir->setToolTip(tr("Overlay both phase and direction vectors."));
  ui->chb_threshold->setToolTip(
      tr("Only display arrows where gradient magnitude "
         "exceeds the threshold value."));
  ui->chb_scaling->setToolTip(tr("Scale arrow length proportionally to gradient magnitude."));
  ui->sb_threshold->setToolTip(tr("Minimum gradient magnitude for an arrow to be displayed."));
  ui->sb_interpolate->setToolTip(
      tr("Area in pixels over which gradient values are "
         "averaged for each arrow."));
  ui->cb_interpolate->setToolTip(tr("OpenCV interpolation method (NN, Linear, Cubic, Area, Lanczos4)."));
  ui->pb_color->setToolTip(tr("Choose the arrow color."));
  ui->chb_visibility->setToolTip(tr("Toggle arrow visibility on the image."));
  ui->pb_apply->setToolTip(tr("Apply the current settings and recompute the quiver overlay."));
  ui->pb_delete->setToolTip(tr("Remove all arrows from the image."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/quivers/README.md");
}

QuiverPanel::~QuiverPanel() { delete ui; }

// ---------------------------------------------------------------------------
// LATool interface
// ---------------------------------------------------------------------------

void QuiverPanel::connectTools(Analyzer* w) {
  ctrl = w;

  // Create the QCPCurve on the quiver layer.
  PlotWindow* lplot = ctrl->getPlotWindow();
  lplot->qplot->setCurrentLayer("quiver");
  quiverCurve = new QCPCurve(lplot->qplot->xAxis, lplot->qplot->yAxis);
  quiverCurve->setPen(quiverPen);
  quiverCurve->setScatterStyle(QCPScatterStyle::ssNone);
  quiverCurve->setSelectable(false);
  quiverCurve->setLayer("quiver");

  // Clear quivers when new lines are detected.
  connect(w, &Analyzer::sourcesChanged, this, [this](const ImageSources&) { deleteQuivers(); });
}

// ---------------------------------------------------------------------------
// Settings helper
// ---------------------------------------------------------------------------

QuiverPanel::Settings QuiverPanel::readSettings() const {
  Settings s;
  if (ui->rb_gradient->isChecked()) {
    s.data_mode = 0;
  } else if (ui->rb_direction->isChecked()) {
    s.data_mode = 1;
  } else if (ui->rb_phase->isChecked()) {
    s.data_mode = 2;
  } else if (ui->rb_phase_dir->isChecked()) {
    s.data_mode = 3;
  }
  s.interpolation_mode = ui->cb_interpolate->currentIndex();
  s.interpolation_pixels = ui->sb_interpolate->value();
  s.threshold = ui->sb_threshold->value();
  s.scale_use = ui->chb_scaling->isChecked();
  s.threshold_use = ui->chb_threshold->isChecked();
  s.visible = ui->chb_visibility->isChecked();
  return s;
}

// ---------------------------------------------------------------------------
// Arrow geometry helper
// ---------------------------------------------------------------------------

void QuiverPanel::appendArrow(double x1,
                              double y1,
                              double x2,
                              double y2,
                              double& t,
                              QVector<double>& ts,
                              QVector<double>& xs,
                              QVector<double>& ys) {
  constexpr double kArrowLen = 0.25;   // fraction of stem length
  constexpr double kArrowAngle = 0.4;  // half-angle in radians (~23 deg)
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Stem: (x1,y1) -> (x2,y2)
  ts.append(t++);
  xs.append(x1);
  ys.append(y1);
  ts.append(t++);
  xs.append(x2);
  ys.append(y2);
  // NaN separator
  ts.append(t++);
  xs.append(nan);
  ys.append(nan);

  // Arrowhead wings
  double dx = x2 - x1;
  double dy = y2 - y1;
  double len = std::sqrt(dx * dx + dy * dy);
  if (len < 1e-12) return;

  double wingLen = len * kArrowLen;
  double ux = dx / len;
  double uy = dy / len;
  double cosA = std::cos(kArrowAngle);
  double sinA = std::sin(kArrowAngle);

  // Left wing
  double wx1 = x2 - wingLen * (ux * cosA - uy * sinA);
  double wy1 = y2 - wingLen * (uy * cosA + ux * sinA);
  ts.append(t++);
  xs.append(wx1);
  ys.append(wy1);
  ts.append(t++);
  xs.append(x2);
  ys.append(y2);
  ts.append(t++);
  xs.append(nan);
  ys.append(nan);

  // Right wing
  double wx2 = x2 - wingLen * (ux * cosA + uy * sinA);
  double wy2 = y2 - wingLen * (uy * cosA - ux * sinA);
  ts.append(t++);
  xs.append(wx2);
  ys.append(wy2);
  ts.append(t++);
  xs.append(x2);
  ys.append(y2);
  ts.append(t++);
  xs.append(nan);
  ys.append(nan);
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void QuiverPanel::processQuiver() {
  if (ctrl == nullptr) return;
  const ImageSources& sources = ctrl->getSources();
  if (sources.empty()) return;

  try {
    double stime = double(cv::getTickCount());
    auto s = readSettings();

    // Update status labels.
    static const char* kModeNames[] = {"gx, gy", "dir", "phase", "ph, dir"};
    ui->l_qmode->setText(QString("Quiver Mode: %1").arg(kModeNames[s.data_mode]));
    ui->l_threshold_use->setText(QString("Use Threshold: %1").arg(s.threshold_use ? "true" : "false"));
    ui->l_threshold_val->setText(QString("Threshold Value: %1").arg(s.threshold));
    ui->l_scaling_use->setText(QString("Use Scaling: %1").arg(s.scale_use ? "true" : "false"));
    ui->l_scaling_val->setText(QString("Interpolating Pixels: %1").arg(s.interpolation_pixels));
    ui->l_interpolation_mode->setText(QString("Interpolation Mode: %1").arg(s.interpolation_mode));

    // Gather source matrices.
    cv::Mat mag, gx, gy, dir, phase, energy;
    for (const auto& is : sources) {
      if (is.name.compare(QString("mag"), Qt::CaseInsensitive) == 0) {
        mag = is.data;
      } else if (is.name.compare(QString("gx"), Qt::CaseInsensitive) == 0) {
        gx = is.data;
      } else if (is.name.compare(QString("gy"), Qt::CaseInsensitive) == 0) {
        gy = is.data;
      } else if (is.name.compare(QString("dir"), Qt::CaseInsensitive) == 0) {
        dir = is.data;
      } else if (is.name.compare(QString("phase"), Qt::CaseInsensitive) == 0) {
        phase = is.data;
      } else if (is.name.compare(QString("energy"), Qt::CaseInsensitive) == 0) {
        energy = is.data;
      }
    }

    // Compute arrow data into parametric vectors.
    QVector<double> ts, xs, ys;

    switch (s.data_mode) {
      case 0:
        gradientQuiver(mag, gx, gy, ts, xs, ys);
        break;
      case 1:
        angleQuiver(dir, mag, ts, xs, ys);
        break;
      case 2:
        angleQuiver(phase, energy, ts, xs, ys);
        break;
      case 3: {
        // Phase + direction: add both angles.
        cv::Mat combined;
        if (!dir.empty() && !phase.empty()) {
          cv::add(dir, phase, combined);
        }
        angleQuiver(combined, energy, ts, xs, ys);
        break;
      }
      default:
        break;
    }

    // Update the curve with all arrow data in a single call.
    quiverCurve->setPen(quiverPen);
    quiverCurve->setData(ts, xs, ys);

    PlotWindow* lplot = ctrl->getPlotWindow();
    lplot->qplot->layer("quiver")->setVisible(s.visible);
    lplot->replot();

    int numArrows = ts.size() / 9;  // 9 data points per arrow (3 segments * 3 points)
    double elapsed = (double(cv::getTickCount()) - stime) * 1000 / cv::getTickFrequency();
    std::cout << "Quiver computation: " << numArrows << " arrows, " << elapsed << "ms" << std::endl;
  } catch (const std::exception& ex) {
    std::cerr << "Quiver computation failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Quiver Error"), tr("Quiver computation failed:\n%1").arg(ex.what()));
  }
}

void QuiverPanel::toggleQuiver() {
  if (ctrl == nullptr) return;
  PlotWindow* lplot = ctrl->getPlotWindow();
  lplot->qplot->layer("quiver")->setVisible(ui->chb_visibility->isChecked());
  lplot->replot();
}

void QuiverPanel::deleteQuivers() {
  if (quiverCurve != nullptr) {
    quiverCurve->clearData();
  }
  if (ctrl != nullptr) {
    ctrl->getPlotWindow()->qplot->replot();
  }
}

void QuiverPanel::selectColor() {
  QColor c = colorDialog->getColor(quiverPen.color(), this, QString("Quiver Color"));
  if (!c.isValid()) return;
  quiverPen.setColor(c);

  ui->pb_color->setStyleSheet(QString("background-color: rgb(%1,%2,%3);").arg(c.red()).arg(c.green()).arg(c.blue()));
}

// ---------------------------------------------------------------------------
// Gradient quiver computation
// ---------------------------------------------------------------------------

void QuiverPanel::gradientQuiver(const cv::Mat& mag_in,
                                 const cv::Mat& gx_in,
                                 const cv::Mat& gy_in,
                                 QVector<double>& ts,
                                 QVector<double>& xs,
                                 QVector<double>& ys) {
  if (mag_in.empty()) {
    std::cout << "No mag for quiver computation available." << std::endl;
    return;
  }
  if (gx_in.empty() || gy_in.empty()) {
    std::cout << "No gx or gy for quiver computation available." << std::endl;
    return;
  }

  auto s = readSettings();
  double invInterp = 1.0 / s.interpolation_pixels;

  // Convert and downsample.
  cv::Mat mag, gx, gy;
  mag_in.convertTo(mag, CV_64FC1);
  gx_in.convertTo(gx, CV_64FC1);
  gy_in.convertTo(gy, CV_64FC1);

  cv::resize(mag, mag, cv::Size(), invInterp, invInterp, s.interpolation_mode);
  cv::resize(gx, gx, cv::Size(), invInterp, invInterp, s.interpolation_mode);
  cv::resize(gy, gy, cv::Size(), invInterp, invInterp, s.interpolation_mode);

  double gx_max = 0.0;
  double gy_max = 0.0;
  cv::minMaxIdx(gx, nullptr, &gx_max);
  cv::minMaxIdx(gy, nullptr, &gy_max);
  if (gx_max < 1e-12) gx_max = 1.0;
  if (gy_max < 1e-12) gy_max = 1.0;
  double inv_gx_max = 1.0 / gx_max;
  double inv_gy_max = 1.0 / gy_max;

  // Pre-compute constants.
  double halfInterp = s.interpolation_pixels / 2.0;
  double scale = s.interpolation_pixels / 2.0;

  // Conservative capacity estimate.
  ts.reserve(mag.rows * mag.cols * 9);
  xs.reserve(mag.rows * mag.cols * 9);
  ys.reserve(mag.rows * mag.cols * 9);

  double t = 0;
  for (int y = 0; y < gx.rows; ++y) {
    const auto* mag_row = mag.ptr<double>(y);
    const auto* gx_row = gx.ptr<double>(y);
    const auto* gy_row = gy.ptr<double>(y);

    for (int x = 0; x < gx.cols; ++x) {
      if (s.threshold_use && mag_row[x] <= s.threshold) continue;

      double gxv = gx_row[x] * inv_gx_max;
      double gyv = gy_row[x] * inv_gy_max;

      if (!s.scale_use) {
        double len = std::sqrt(gxv * gxv + gyv * gyv);
        if (len > 1e-12) {
          gxv /= len;
          gyv /= len;
        }
      }

      gxv *= scale;
      gyv *= scale;

      double cx = (static_cast<double>(x) * s.interpolation_pixels) - 0.5 + halfInterp;
      double cy = (static_cast<double>(y) * s.interpolation_pixels) - 0.5 + halfInterp;
      appendArrow(cx - gxv, cy - gyv, cx + gxv, cy + gyv, t, ts, xs, ys);
    }
  }
}

// ---------------------------------------------------------------------------
// Angle quiver computation
// ---------------------------------------------------------------------------

void QuiverPanel::angleQuiver(
    const cv::Mat& ang_in, const cv::Mat& ampl_in, QVector<double>& ts, QVector<double>& xs, QVector<double>& ys) {
  if (ang_in.empty() || ampl_in.empty()) {
    std::cout << "No phase or even or odd or energy for quiver computation available." << std::endl;
    return;
  }

  auto s = readSettings();
  double invInterp = 1.0 / s.interpolation_pixels;

  cv::Mat ang, ampl;
  ang_in.convertTo(ang, CV_64FC1);
  ampl_in.convertTo(ampl, CV_64FC1);

  double ampl_max = 0.0;
  cv::minMaxIdx(ampl, nullptr, &ampl_max);
  if (ampl_max < 1e-12) ampl_max = 1.0;
  double inv_ampl_max = 1.0 / ampl_max;

  cv::resize(ang, ang, cv::Size(), invInterp, invInterp, s.interpolation_mode);
  cv::resize(ampl, ampl, cv::Size(), invInterp, invInterp, s.interpolation_mode);

  // Pre-compute constants.
  double halfInterp = s.interpolation_pixels / 2.0;
  double scale = s.interpolation_pixels / 2.0;

  ts.reserve(ang.rows * ang.cols * 9);
  xs.reserve(ang.rows * ang.cols * 9);
  ys.reserve(ang.rows * ang.cols * 9);

  double t = 0;
  for (int y = 0; y < ang.rows; ++y) {
    const auto* ang_row = ang.ptr<double>(y);
    const auto* ampl_row = ampl.ptr<double>(y);

    for (int x = 0; x < ang.cols; ++x) {
      if (s.threshold_use && ampl_row[x] <= s.threshold) continue;

      double angle = ang_row[x];
      double xoff = std::cos(angle) * scale;
      double yoff = std::sin(angle) * scale;

      if (s.scale_use) {
        double e_val = ampl_row[x] * inv_ampl_max;
        xoff *= e_val;
        yoff *= e_val;
      }

      double cx = (static_cast<double>(x) * s.interpolation_pixels) - 0.5 + halfInterp;
      double cy = (static_cast<double>(y) * s.interpolation_pixels) - 0.5 + halfInterp;
      appendArrow(cx - xoff, cy - yoff, cx + xoff, cy + yoff, t, ts, xs, ys);
    }
  }
}
