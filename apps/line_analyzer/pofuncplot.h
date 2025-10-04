#pragma once

#include "controlwindow.h"
#include "ui_pofuncplot.h"
#include <imgproc/mean.hpp>
#include <qplot3d/qwt3d_function.h>
#include <qplot3d/qwt3d_surfaceplot.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace Ui {
class POFuncPlot;
}

class POFuncPlot : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(POFuncPlot)

  Ui::POFuncPlot* ui;

  template <class mat_type>
  struct LinePrecOpiFunction : public Qwt3D::Function {
    cv::Mat mag;
    LineSegment line;
    float_type mean_param, scale;
    typename lsfm::MeanHelper<float_type, cv::Point_>::func_type mean_op;

    LinePrecOpiFunction<mat_type>(
        Qwt3D::SurfacePlot& pw,
        const cv::Mat& m = cv::Mat(),
        const LineSegment& l = LineSegment(),
        float_type mp = 1,
        float_type sc = 1,
        typename lsfm::MeanHelper<float_type, cv::Point_>::func_type mop = lsfm::Mean<float_type, mat_type>::process)
        : Qwt3D::Function(pw), mag(m), line(l), mean_param(mp), scale(sc), mean_op(mop) {}

    double operator()(double u, double v) {
      LineSegment tmp = line;
      tmp.translateOrtho(u);
      tmp.rotate(v * CV_PI / 180, tmp.center());
      return mean_op(mag, tmp, mean_param) / scale;
    }
  };

  class FunctionPlot : public QMainWindow {
    Q_DISABLE_COPY(FunctionPlot)

    const ImageSources* src;
    Ui::POFuncPlot* po;
    Qwt3D::SurfacePlot* plot;
    LinePrecOpiFunction<int> lpoi;
    LinePrecOpiFunction<float_type> lpod;
    float_type z_scale;

   public:
    FunctionPlot(QWidget* parent, Ui::POFuncPlot* p)
        : QMainWindow(parent),
          src(nullptr),
          po(p),
          plot(new Qwt3D::SurfacePlot(this)),
          lpoi(*plot),
          lpod(*plot),
          z_scale(1) {
      setWindowTitle("PO Function Plot");

      setCentralWidget(plot);
      lpoi.setMinZ(0);
      lpod.setMinZ(0);

      for (unsigned i = 0; i != plot->coordinates()->axes.size(); ++i) {
        plot->coordinates()->axes[i].setMajors(3);
        plot->coordinates()->axes[i].setMinors(5);
      }

      // resetView();
      plot->setRotation(20, 0, 44.9);
    }

    ~FunctionPlot() override = default;


    void setSource(const ImageSources& s) { src = &s; }

    void resetView() {
      plot->setRotation(20, 0, 44.9);
      plot->setScale(1, 1, -1);
      plot->setShift(0, 0, 0);
      plot->setZoom(1);
      plot->setViewportShift(0, 0);
      plot->updateGL();
    }

    void fitProfile() {
      plot->setRotation(0, 0, 0);
      plot->setScale(1, 1, -1);
      plot->setShift(0, 0, 0);
      plot->setViewportShift(0, 0);
      plot->setZoom(1);

      plot->updateGL();
    }

    void fitRotation() {
      plot->setRotation(0, 0, 90);
      plot->setScale(1, 1, -1);
      plot->setShift(0, 0, 0);
      plot->setViewportShift(0, 0);
      plot->setZoom(1);

      plot->updateGL();
    }

    void updateStyles() {
      plot->setCoordinateStyle(static_cast<Qwt3D::COORDSTYLE>(po->cb_coord_style->currentData().toInt()));
      // plot->setShading(po->chb_flat->isChecked() ? Qwt3D::FLAT : Qwt3D::GOURAUD); -> no effect in new qt gl view

      plot->updateGL();
    }

    void updateData() {
      plot->setPlotStyle(static_cast<Qwt3D::PLOTSTYLE>(po->cb_plot_style->currentData().toInt()));
      plot->coordinates()->axes[Qwt3D::X1].setLabelString("Profile");
      plot->coordinates()->axes[Qwt3D::Y1].setLabelString("Angle");
      plot->coordinates()->axes[Qwt3D::Z1].setLabelString("Mean");
      plot->coordinates()->axes[Qwt3D::Z1].setLimits(0, z_scale);
      plot->coordinates()->axes[Qwt3D::Z2].setLimits(0, z_scale);
      plot->coordinates()->axes[Qwt3D::Z3].setLimits(0, z_scale);
      plot->coordinates()->axes[Qwt3D::Z4].setLimits(0, z_scale);

      plot->updateData();
      plot->updateGL();
    }

    template <class MT>
    typename lsfm::MeanHelper<float_type, cv::Point_>::func_type getMean(int idx, bool sampled, bool fast) {
      switch (idx) {
        case 0:
          if (sampled) {
            if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
            return lsfm::MeanSampled<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
          }
          if (fast) return lsfm::FastMean<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
          return lsfm::Mean<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
        case 1:
          if (sampled) {
            if (fast)
              return lsfm::FastMeanSampled<float_type, MT,
                                           lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
            return lsfm::MeanSampled<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
          }
          if (fast) return lsfm::FastMean<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
          return lsfm::Mean<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
        case 2:
          if (sampled) {
            if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
            return lsfm::MeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
          }
          if (fast) return lsfm::FastMean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
          return lsfm::Mean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
        case 3:
          if (sampled) {
            if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
            return lsfm::MeanSampled<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
          }
          if (fast) return lsfm::FastMean<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
          return lsfm::Mean<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
      }
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
        return lsfm::MeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
      }
      if (fast) return lsfm::FastMean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
      return lsfm::Mean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
    }

    void update(const LineSegment& l) {
      if (src == nullptr || src->empty()) return;

      const float_type rot_range = po->spin_range_rot->value();
      const float_type prof_range = po->spin_range_prof->value();
      const int subdiv = po->spin_subdiv->value();

      const int sourceIndex = po->cb_data_source->currentData().toInt();
      if (sourceIndex < 0) return;
      const auto index = static_cast<std::size_t>(sourceIndex);
      if (index >= src->size()) return;

      const cv::Mat& mag = src->at(index).data;

      const auto meshProfileSamples =
          static_cast<unsigned int>(std::max<long>(1L, std::lround(2.0 * prof_range * subdiv)));
      const auto meshRotationSamples =
          static_cast<unsigned int>(std::max<long>(1L, std::lround(2.0 * rot_range * subdiv)));

      if (mag.type() == cv::DataType<float_type>::type) {
        lpod.setDomain(-prof_range, prof_range, -rot_range, rot_range);
        lpod.mag = mag;
        lpod.line = l;
        lpod.mean_param = po->spin_line_dist->value();
        lpod.setMesh(meshProfileSamples, meshRotationSamples);
        lpod.mean_op = getMean<float_type>(po->cb_profile_interp->currentIndex(), po->chb_line_samples->isChecked(),
                                           po->chb_fast_interp->isChecked());
        lpod.scale = 1;
        z_scale = lpod(0, 0);
        lpod.scale = z_scale / 4;
        lpod.create();
      } else {
        lpoi.setDomain(-prof_range, prof_range, -rot_range, rot_range);
        lpoi.mag = mag;
        lpoi.line = l;
        lpoi.mean_param = po->spin_line_dist->value();
        lpoi.setMesh(meshProfileSamples, meshRotationSamples);
        lpoi.mean_op = getMean<int>(po->cb_profile_interp->currentIndex(), po->chb_line_samples->isChecked(),
                                    po->chb_fast_interp->isChecked());
        lpoi.scale = 1;
        z_scale = lpoi(0, 0);
        lpoi.scale = z_scale / 4;
        lpoi.create();
      }

      updateData();
    }
  };

  FunctionPlot* plot3d;
  LineSegment line;

 public:
  explicit POFuncPlot(QWidget* parent = 0);
  ~POFuncPlot();

  void connectTools(ControlWindow* w);

 public slots:
  void updateSources(const ImageSources& src);
  void updateLine(const LineSegment& l);
  void updatePlot();
  void autoUpdatePlot();
  void updateStyles();
  void updateData();
  void resetView();
  void fitProfile();
  void fitRotation();
};
