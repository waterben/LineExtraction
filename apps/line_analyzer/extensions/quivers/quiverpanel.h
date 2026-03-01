#pragma once

#include "analyzer.h"
#include "ui_quiverpanel.h"

#include <QColorDialog>

class QCPCurve;

/// @brief Extension panel for gradient vector field (quiver) visualization.
///
/// Displays gradient vectors as arrows overlaid on the plot. Supports
/// multiple visualization modes (gradient components, direction, phase)
/// with optional thresholding, scaling, and configurable interpolation.
///
/// Rendering uses a single QCPCurve plottable with parametric segments
/// (including arrowheads), which is dramatically faster than creating
/// individual QCPItemLine objects for each arrow.
class QuiverPanel : public LATool {
  Q_OBJECT
  Q_DISABLE_COPY(QuiverPanel)

  Ui::QuiverPanel* ui{nullptr};
  Analyzer* ctrl{nullptr};
  QColorDialog* colorDialog{nullptr};

  /// @brief Single QCPCurve used to draw all quiver arrows (stem + arrowhead).
  QCPCurve* quiverCurve{nullptr};

  /// @brief Pen for quiver arrows.
  QPen quiverPen;

 public:
  explicit QuiverPanel(QWidget* parent = nullptr);
  ~QuiverPanel() override;

  void connectTools(Analyzer* w) override;

 public slots:
  /// @brief Apply current settings and recompute the quiver overlay.
  void processQuiver();

  /// @brief Toggle quiver arrow visibility on the plot.
  void toggleQuiver();

  /// @brief Remove all quiver arrows from the plot.
  void deleteQuivers();

  /// @brief Open a color picker to choose the arrow color.
  void selectColor();

 private:
  /// @brief Compute gradient-based quiver arrows (Gx, Gy mode).
  ///
  /// @param mag Gradient magnitude matrix.
  /// @param gx  Horizontal gradient component.
  /// @param gy  Vertical gradient component.
  /// @param ts  Output parametric t values for QCPCurve.
  /// @param xs  Output x coordinates for QCPCurve.
  /// @param ys  Output y coordinates for QCPCurve.
  void gradientQuiver(const cv::Mat& mag,
                      const cv::Mat& gx,
                      const cv::Mat& gy,
                      QVector<double>& ts,
                      QVector<double>& xs,
                      QVector<double>& ys);

  /// @brief Compute angle-based quiver arrows (direction, phase modes).
  ///
  /// @param ang  Angle matrix (radians).
  /// @param ampl Amplitude matrix for scaling.
  /// @param ts   Output parametric t values for QCPCurve.
  /// @param xs   Output x coordinates for QCPCurve.
  /// @param ys   Output y coordinates for QCPCurve.
  void angleQuiver(
      const cv::Mat& ang, const cv::Mat& ampl, QVector<double>& ts, QVector<double>& xs, QVector<double>& ys);

  /// @brief Append a single arrow (stem + arrowhead) to the curve data vectors.
  ///
  /// @param x1 Start x of arrow stem.
  /// @param y1 Start y of arrow stem.
  /// @param x2 End x (tip) of arrow stem.
  /// @param y2 End y (tip) of arrow stem.
  /// @param t  Current parametric counter (updated in place).
  /// @param ts Output parametric t values.
  /// @param xs Output x coordinates.
  /// @param ys Output y coordinates.
  static void appendArrow(double x1,
                          double y1,
                          double x2,
                          double y2,
                          double& t,
                          QVector<double>& ts,
                          QVector<double>& xs,
                          QVector<double>& ys);

  /// @brief Read current UI settings into local variables.
  struct Settings {
    int data_mode{0};
    int interpolation_mode{1};
    int interpolation_pixels{1};
    int threshold{32};
    bool scale_use{true};
    bool threshold_use{true};
    bool visible{true};
  };
  Settings readSettings() const;
};
