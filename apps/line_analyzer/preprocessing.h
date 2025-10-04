#pragma once

#include "ui_preprocessing.h"

#include <QMainWindow>

class PreProcessing : public QMainWindow {
  Q_OBJECT
  Q_DISABLE_COPY(PreProcessing)

 public:
  explicit PreProcessing(QWidget* parent = 0);
  ~PreProcessing();

  Ui::PreProcessing* ui{nullptr};


 public slots:
  void scaleChange(int);
  void blurChange(int);
  void noiseChange(int);
};
