#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include "ui_preprocessing.h"

#include <QMainWindow>

class PreProcessing : public QMainWindow {
  Q_OBJECT


 public:
  explicit PreProcessing(QWidget* parent = 0);
  ~PreProcessing();

  Ui::PreProcessing* ui;


 public slots:
  void scaleChange(int);
  void blurChange(int);
  void noiseChange(int);
};

#endif  // PREPROCESSING_H
