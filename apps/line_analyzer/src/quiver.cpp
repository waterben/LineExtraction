#include "quiver.h"

#include "help_button.hpp"

#include <iostream>

Quiver::Quiver(QWidget* parent)
    : QMainWindow(parent), color_dia(new QColorDialog(this)), ui(new Ui::Quiver), qPen(Qt::green) {
  ui->setupUi(this);
  this->setWindowTitle(QString("Quiver"));

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
  addHelpButton(this, "README.md", "visualization-options");
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
