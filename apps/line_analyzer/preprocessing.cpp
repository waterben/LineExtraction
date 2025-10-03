#include "preprocessing.h"

PreProcessing::PreProcessing(QWidget* parent) : QMainWindow(parent), ui(new Ui::PreProcessing) {
  ui->setupUi(this);
  ui->cb_interp->addItem("nearest");
  ui->cb_interp->addItem("bilinear");
  ui->cb_interp->addItem("area");
  ui->cb_interp->addItem("bicubic");
  ui->cb_interp->addItem("lanczons");
  ui->cb_interp->setCurrentIndex(1);
}

PreProcessing::~PreProcessing() { delete ui; }

void PreProcessing::scaleChange(int s) {
  ui->spin_scale->setEnabled(s != 0);
  ui->label_interp->setEnabled(s != 0);
  ui->cb_interp->setEnabled(s != 0);
}

void PreProcessing::blurChange(int s) { ui->spin_blur->setEnabled(s != 0); }

void PreProcessing::noiseChange(int s) { ui->spin_noise->setEnabled(s != 0); }
