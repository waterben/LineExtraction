#include "3d_profile_plot.h"

#include "help_button.hpp"

#include <QMessageBox>
#include <algorithm>

POFuncPlot::POFuncPlot(QWidget* parent)
    : LATool("3D Profile Plot", parent), ui(new Ui::POFuncPlot), plot3d(nullptr), line() {
  setWindowTitle("3D Profile Plot");
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
  ui->cb_coord_style->addItem("No Coord", QVariant(static_cast<uint>(Qwt3D::NOCOORD)));
  ui->cb_coord_style->addItem("Box", QVariant(static_cast<uint>(Qwt3D::BOX)));
  ui->cb_coord_style->addItem("Frame", QVariant(static_cast<uint>(Qwt3D::FRAME)));
  ui->cb_coord_style->setCurrentIndex(1);
  ui->cb_coord_style->blockSignals(false);

  ui->cb_plot_style->blockSignals(true);
  ui->cb_plot_style->addItem("No Plot", QVariant(static_cast<uint>(Qwt3D::NOPLOT)));
  ui->cb_plot_style->addItem("Wireframe", QVariant(static_cast<uint>(Qwt3D::WIREFRAME)));
  ui->cb_plot_style->addItem("Hidden Line", QVariant(static_cast<uint>(Qwt3D::HIDDENLINE)));
  ui->cb_plot_style->addItem("Filled", QVariant(static_cast<uint>(Qwt3D::FILLED)));
  ui->cb_plot_style->addItem("Filled Mesh", QVariant(static_cast<uint>(Qwt3D::FILLEDMESH)));
  ui->cb_plot_style->addItem("Points", QVariant(static_cast<uint>(Qwt3D::POINTS)));
  ui->cb_plot_style->setCurrentIndex(4);
  ui->cb_plot_style->blockSignals(false);

  // Tooltips for the panel and its controls.
  setToolTip(
      tr("3D surface plot of line precision optimization function. "
         "Shows how mean gradient response changes with profile "
         "offset (X) and rotation (Y)."));
  ui->cb_data_source->setToolTip(tr("Gradient magnitude source to evaluate the function on."));
  ui->cb_profile_interp->setToolTip(tr("Interpolation method for sampling along the line profile."));
  ui->spin_range_rot->setToolTip(tr("Half-range (degrees) for the rotation axis."));
  ui->spin_range_prof->setToolTip(tr("Half-range (pixels) for the profile offset axis."));
  ui->spin_subdiv->setToolTip(tr("Mesh subdivisions per unit range. Higher = finer surface."));
  ui->spin_line_dist->setToolTip(tr("Number of support pixels used to compute the mean response."));
  ui->chb_auto->setToolTip(tr("Automatically replot when the selected line changes."));
  ui->chb_line_samples->setToolTip(tr("Use sampled (discrete) points instead of continuous evaluation."));
  ui->chb_fast_interp->setToolTip(tr("Use fast approximate interpolation (may be less accurate)."));
  ui->cb_coord_style->setToolTip(tr("Coordinate frame style for the 3D plot."));
  ui->cb_plot_style->setToolTip(tr("Rendering style for the 3D surface."));

  // Help button — opens README in HelpViewer.
  addHelpButton(this, "extensions/3d_profile_plot/README.md");
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
  try {
    if (plot3d->isHidden()) {
      plot3d->show();
      plot3d->resize(800, 600);
    }
    plot3d->update(line);
  } catch (const std::exception& ex) {
    std::cerr << "Function plot failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Plot Error"), tr("Function plot failed:\n%1").arg(ex.what()));
  }
}

void POFuncPlot::updateStyles() { plot3d->updateStyles(); }

void POFuncPlot::resetView() { plot3d->resetView(); }

void POFuncPlot::fitProfile() { plot3d->fitProfile(); }

void POFuncPlot::fitRotation() { plot3d->fitRotation(); }

void POFuncPlot::updateData() {
  try {
    plot3d->updateData();
  } catch (const std::exception& ex) {
    std::cerr << "Function plot data update failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Plot Error"), tr("Plot data update failed:\n%1").arg(ex.what()));
  }
}

void POFuncPlot::connectTools(Analyzer* w) {
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(updateSources(const ImageSources&)));
  connect(w, SIGNAL(lineChanged(const LineSegment&)), this, SLOT(updateLine(const LineSegment&)));
}
