#include "quiver.h"

#include <iostream>

Quiver::Quiver(QWidget* parent) : QMainWindow(parent), ui(new Ui::Quiver), color_dia(new QColorDialog(this)) {
  ui->setupUi(this);
  this->setWindowTitle(QString("Quiver"));
  qPen = QPen(Qt::green);

  ui->cb_interpolate->addItem("CV_INTER_NN");
  ui->cb_interpolate->addItem("CV_INTER_LINEAR");
  ui->cb_interpolate->addItem("CV_INTER_CUBIC");
  ui->cb_interpolate->addItem("CV_INTER_AREA");
  ui->cb_interpolate->addItem("CV_INTER_LANCZOS4");
  ui->cb_interpolate->setCurrentIndex(3);

  ui->pb_color->setStyleSheet("background-color: rgb(0,255,0);");
}

Quiver::~Quiver() { delete ui; }

void Quiver::applyChanges() {
  // set
  if (ui->rb_gradient->isChecked()) {
    data_mode = 0;
  }
  if (ui->rb_direction->isChecked()) {
    data_mode = 1;
  }
  if (ui->rb_phase->isChecked()) {
    data_mode = 2;
  }
  if (ui->rb_phase_dir->isChecked()) {
    data_mode = 3;
  }

  interpolation_mode = ui->cb_interpolate->currentIndex();
  interpolation_pixels = ui->sb_interpolate->value();
  threshold = ui->sb_threshold->value();
  scale_use = ui->chb_scaling->isChecked();
  threshold_use = ui->chb_threshold->isChecked();

  // display
  if (ui->rb_gradient->isChecked()) {
    ui->l_qmode->setText(QString("Quiver Mode: gx, gy"));
  }
  if (ui->rb_direction->isChecked()) {
    ui->l_qmode->setText(QString("Quiver Mode: dir"));
  }
  if (ui->rb_phase->isChecked()) {
    ui->l_qmode->setText(QString("Quiver Mode: phase"));
  }
  if (ui->rb_phase_dir->isChecked()) {
    ui->l_qmode->setText(QString("Quiver Mode: ph, dir"));
  }

  if (ui->chb_threshold->isChecked()) {
    ui->l_threshold_use->setText(QString("Use Threshold: true"));
  } else {
    ui->l_threshold_use->setText(QString("Use Threshold: false"));
  }
  ui->l_threshold_val->setText(QString("Threshold Value: ").append(QString::number(ui->sb_threshold->value())));
  if (ui->chb_scaling->isChecked()) {
    ui->l_scaling_use->setText(QString("Use Scaling: true"));
  } else {
    ui->l_scaling_use->setText(QString("Use Scaling: false"));
  }
  ui->l_scaling_val->setText(QString("Interpolating Pixels: ").append(QString::number(ui->sb_interpolate->value())));
  ui->l_interpolation_mode->setText(
      QString("Interpolation Mode: ").append(QString::number(ui->cb_interpolate->currentIndex())));

  emit computeQuivers();
}

void Quiver::selectColor() {
  QColor c = color_dia->getColor(qPen.color(), this, QString("Quiver Color"));
  qPen.setColor(c);

  int red, green, blue;
  c.getRgb(&red, &green, &blue);

  QString style_set("background-color: rgb(");
  style_set.append(QString::number(red));
  style_set.append(",");
  style_set.append(QString::number(green));
  style_set.append(",");
  style_set.append(QString::number(blue));
  style_set.append(");");
  ui->pb_color->setStyleSheet(style_set);
}
