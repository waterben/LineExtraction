#pragma once

#include "ui_quiver.h"

#include <QColorDialog>
#include <QMainWindow>
#include <QPen>

class Quiver : public QMainWindow {
  Q_OBJECT
  Q_DISABLE_COPY(Quiver)

  QColorDialog* color_dia;

 public:
  explicit Quiver(QWidget* parent = nullptr);
  ~Quiver();

  Ui::Quiver* ui;
  QPen qPen;

  int data_mode = 0;
  int interpolation_mode = 1;
  int interpolation_pixels = 1;
  int threshold = 32;

  bool scale_use = true;
  bool threshold_use = true;

 public slots:
  void applyChanges();
  void selectColor();

 signals:
  void computeQuivers();
};
