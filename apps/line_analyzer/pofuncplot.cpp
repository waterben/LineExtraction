#include "pofuncplot.h"

#include <algorithm>

POFuncPlot::POFuncPlot(QWidget* parent)
    : LATool("PO Function Plot", parent), ui(new Ui::POFuncPlot), plot3d(nullptr), line() {
  setWindowTitle("PO Function Plot");
  ui->setupUi(this);

  plot3d = new FunctionPlot(parent, ui);

  ui->cb_profile_interp->blockSignals(true);
  ui->cb_profile_interp->addItem("nearest");
  ui->cb_profile_interp->addItem("nearest_round");
  ui->cb_profile_interp->addItem("bilinear");
  ui->cb_profile_interp->addItem("bicubic");
  ui->cb_profile_interp->setCurrentIndex(2);
  ui->cb_profile_interp->blockSignals(false);

  ui->cb_coord_style->blockSignals(true);
  ui->cb_coord_style->addItem("No Coord", QVariant(Qwt3D::NOCOORD));
  ui->cb_coord_style->addItem("Box", QVariant(Qwt3D::BOX));
  ui->cb_coord_style->addItem("Frame", QVariant(Qwt3D::FRAME));
  ui->cb_coord_style->setCurrentIndex(1);
  ui->cb_coord_style->blockSignals(false);

  ui->cb_plot_style->blockSignals(true);
  ui->cb_plot_style->addItem("No Plot", QVariant(Qwt3D::NOPLOT));
  ui->cb_plot_style->addItem("Wireframe", QVariant(Qwt3D::WIREFRAME));
  ui->cb_plot_style->addItem("Hidden Line", QVariant(Qwt3D::HIDDENLINE));
  ui->cb_plot_style->addItem("Filled", QVariant(Qwt3D::FILLED));
  ui->cb_plot_style->addItem("Filled Mesh", QVariant(Qwt3D::FILLEDMESH));
  ui->cb_plot_style->addItem("Points", QVariant(Qwt3D::POINTS));
  ui->cb_plot_style->setCurrentIndex(4);
  ui->cb_plot_style->blockSignals(false);
}

POFuncPlot::~POFuncPlot() {
  delete plot3d;
  delete ui;
}

void POFuncPlot::updateSources(const ImageSources& src) {
  ui->cb_data_source->blockSignals(true);

  int isi = ui->cb_data_source->currentIndex(), imag = 0;
  QString iss = ui->cb_data_source->currentText();
  ui->cb_data_source->clear();
  int i = 0, idx = 0;
  for_each(src.begin(), src.end(), [&](const ImageSource& s) {
    if (s.name == "mag" || s.name == "qmag" || s.name == "nmag") {
      ui->cb_data_source->addItem(s.name, i);
      if (s.name == "mag") imag = idx;
      ++idx;
    }
    ++i;
  });

  if (isi != -1 && isi < ui->cb_data_source->count() && ui->cb_data_source->itemText(isi) == iss)
    ui->cb_data_source->setCurrentIndex(isi);
  else
    ui->cb_data_source->setCurrentIndex(imag);

  ui->cb_data_source->blockSignals(false);
  ui->cb_data_source->setDisabled(false);

  plot3d->setSource(src);
}

void POFuncPlot::updateLine(const LineSegment& l) {
  line = l;
  if (ui->chb_auto->isChecked() && isVisible()) updatePlot();
}

void POFuncPlot::autoUpdatePlot() {
  if (ui->chb_auto->isChecked()) updatePlot();
}

void POFuncPlot::updatePlot() {
  if (plot3d->isHidden()) {
    plot3d->show();
    plot3d->resize(800, 600);
  }
  plot3d->update(line);
}

void POFuncPlot::updateStyles() { plot3d->updateStyles(); }

void POFuncPlot::resetView() { plot3d->resetView(); }

void POFuncPlot::fitProfile() { plot3d->fitProfile(); }

void POFuncPlot::fitRotation() { plot3d->fitRotation(); }

void POFuncPlot::updateData() { plot3d->updateData(); }

void POFuncPlot::connectTools(ControlWindow* w) {
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(updateSources(const ImageSources&)));
  connect(w, SIGNAL(lineChanged(const LineSegment&)), this, SLOT(updateLine(const LineSegment&)));
}
