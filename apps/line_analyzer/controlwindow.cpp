#include "controlwindow.h"

#include "ui_controlwindow.h"
#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;


// TODOS:
// - Add false detector class to make it available for all lsd variants
// - fit endpoints of lines to corners
// - line connections (line chains)
// - line conjecture (find hidden or missing lines)
// - Add-, Split-, Merge- and Remove- (mit z.b. length thresh) Tools

ControlWindow::ControlWindow(QWidget* parent)
    : QMainWindow(parent),
      file(new QFileDialog(this, tr("Open Image"))),
      lplot(new PlotWindow("Line Plot", this)),
      pp(new PreProcessing(this)),
      cdia(new QColorDialog(this)),
      qo(new Quiver(this)),
      img{},
      src{},
      tools(0),
      sources{},
      imgMap(nullptr),
      lineSel(-1),
      inputSourcesSize(0),
      detectors{},
      indicator(nullptr),
      lines{},
      quiver{},
      ui(new Ui::ControlWindow),
      pen{},
      mPen{},
      nPen{},
      nmPen{},
      selPen{},
      nSelPen{},
      iPen{} {
  setWindowTitle("Line Analyzer");
  ui->setupUi(this);

  file->setFileMode(QFileDialog::ExistingFile);
  file->setAcceptMode(QFileDialog::AcceptOpen);
  QStringList filters;
  filters << "Image files (*.jpg *.jpeg *.jpe* *.jp2 *.png *.pbm *.pgm *.ppm *.tif *.tiff *.bmp *.dib)"
          << "JPEG files (*.jpg *.jpeg *.jpe* *.jp2)" << "PNG files (*.png)" << "PBM files (*.pbm *.pgm *.ppm)"
          << "TIFF files (*.tif *.tiff)" << "Windows bitmaps (*.bmp *.dib)";
  file->setNameFilters(filters);

  lplot->qplot->setInteraction(QCP::iSelectItems, true);
  lplot->qplot->setSelectionTolerance(5);
  lplot->keepAspectRatio(true);
  lplot->reverseY(true);
  lplot->qplot->addLayer("image", lplot->qplot->layer("grid"), QCustomPlot::limBelow);
  lplot->qplot->addLayer("lines", lplot->qplot->layer("main"), QCustomPlot::limAbove);
  lplot->qplot->addLayer("normals", lplot->qplot->layer("lines"), QCustomPlot::limAbove);
  lplot->qplot->addLayer("indicator", lplot->qplot->layer("normals"), QCustomPlot::limAbove);
  lplot->qplot->addLayer("quiver", lplot->qplot->layer("indicator"), QCustomPlot::limAbove);
  lplot->qplot->layer("indicator")->setVisible(false);
  connect(lplot->qplot, SIGNAL(selectionChangedByUser()), this, SLOT(lplotSelChange()));
  connectQuiver();

  QStringList lables;
  lables << "angle" << "distance" << "start" << "end" << "length" << "nx" << "ny" << "center X" << "center Y"
         << "start X" << "start Y" << "end X" << "end Y";
  ui->table_lines->setHorizontalHeaderLabels(lables);
  // ui->table_lines->resizeColumnsToContents();
  // ui->table_lines->horizontalHeader()->setStretchLastSection(true);

  selPen = QPen(Qt::red);
  nSelPen = QPen(QColor(255, 50, 50, 200));
  pen = QPen(Qt::blue);
  nPen = QPen(QColor(50, 50, 255, 200));
  mPen = QPen(QColor(0, 128, 255, 255));
  nmPen = QPen(QColor(50, 178, 255, 200));
  iPen = QPen(QColor(0, 255, 0, 200));

  indicator = 0;

  ui->le_image_filename->setText("../../images/office1_low.JPG");
  ui->pb_pre->setEnabled(true);
  ui->pb_load->setEnabled(true);
  ui->pb_line_color->setEnabled(true);

  updateRanges();
}

ControlWindow::~ControlWindow() { delete ui; }

void ControlWindow::selectImage() {
  if (file->exec()) {
    ui->le_image_filename->setText(file->selectedFiles().front());
    ui->pb_pre->setEnabled(true);
    ui->pb_load->setEnabled(true);
  }
}

void ControlWindow::openPreprocess() { pp->show(); }

void ControlWindow::loadImage() {
  sources.clear();
  ui->layout_image_options->setEnabled(false);

  string filename = ui->le_image_filename->text().toStdString();

  img = cv::imread(filename, cv::IMREAD_UNCHANGED);

  if (img.empty()) {
    cout << "Can not open " << filename << endl;
    ui->le_image_filename->setText("None");
    return;
  }

  float_type s = static_cast<float_type>(pp->ui->spin_scale->value());
  if (pp->ui->chb_scale->isChecked() && s != 1.0) {
    cv::resize(img, img, cv::Size(), s, s, pp->ui->cb_interp->currentIndex());
  }

  if (img.channels() != 1) {
    sources.push_back(ImageSource("Image", img));
    sources.back().modes.push_back(ImageMode("RGB"));

    cv::Mat tmp;
    cv::cvtColor(img, tmp, cv::COLOR_BGR2GRAY);

    sources.push_back(ImageSource("Image (grayscale)", tmp, Detector::imageModePresets, QCPRange(0, 255)));
    if (pp->ui->chb_gray->isChecked())
      src = tmp.clone();
    else
      src = img.clone();
  } else {
    sources.push_back(ImageSource("Image (grayscale)", img, Detector::imageModePresets, QCPRange(0, 255)));
    src = img.clone();
  }

  if (pp->ui->chb_noise->isChecked()) {
    lsfm::GaussianNoiseOperator noise(pp->ui->spin_noise->value());
    noise(src);
    if (src.channels() != 1) {
      sources.push_back(ImageSource("Noise", src.clone()));
      sources.back().modes.push_back(ImageMode("RGB"));

      cv::Mat tmp;
      cv::cvtColor(img, tmp, cv::COLOR_BGR2GRAY);

      sources.push_back(ImageSource("Noise (grayscale)", tmp, Detector::imageModePresets, QCPRange(0, 255)));
    } else
      sources.push_back(ImageSource("Noise (grayscale)", src.clone(), Detector::imageModePresets, QCPRange(0, 255)));
  }

  if (pp->ui->chb_blur->isChecked()) {
    cv::GaussianBlur(src, src, cv::Size(), pp->ui->spin_blur->value());
    if (src.channels() != 1) {
      sources.push_back(ImageSource("Blured", src.clone()));
      sources.back().modes.push_back(ImageMode("RGB"));

      cv::Mat tmp;
      cv::cvtColor(img, tmp, cv::COLOR_BGR2GRAY);

      sources.push_back(ImageSource("Blured (grayscale)", tmp, Detector::imageModePresets, QCPRange(0, 255)));
    } else
      sources.push_back(ImageSource("Blured (grayscale)", src.clone(), Detector::imageModePresets, QCPRange(0, 255)));
  }

  inputSourcesSize = sources.size();

  ui->layout_line_options->setDisabled(true);
  ui->pb_normal_flip->setDisabled(true);
  ui->pb_endpoint_flip->setDisabled(true);
  ui->pb_lines_freeze->setDisabled(true);
  ui->pb_lines_reset->setDisabled(true);

  ui->table_lines->blockSignals(true);
  ui->table_lines->setRowCount(0);
  ui->table_lines->clearSelection();
  ui->table_lines->scrollToTop();
  ui->table_lines->blockSignals(false);
  clearLines();

  ui->layout_image_options->setEnabled(true);
  ui->cb_image_source->setEnabled(true);
  ui->cb_image_mode->setEnabled(true);
  ui->pb_image_fit->setEnabled(true);
  if (detectors.size()) ui->pb_detector_process->setEnabled(true);
  loadSources();
  lplot->show();
  updateSource();
  lplot->keepAspectRatio(true, static_cast<float_type>(src.cols) / src.rows);
  lplot->replot();
}

void ControlWindow::loadSources() {
  ui->cb_image_source->blockSignals(true);
  ui->cb_image_mode->blockSignals(true);
  int isi = ui->cb_image_source->currentIndex();
  QString iss = ui->cb_image_source->currentText();
  ui->cb_image_source->clear();
  for_each(sources.begin(), sources.end(), [&](const ImageSource& s) { ui->cb_image_source->addItem(s.name); });

  if (isi != -1 && isi < ui->cb_image_source->count() && ui->cb_image_source->itemText(isi) == iss) {
    ui->cb_image_source->setCurrentIndex(isi);
    isi = ui->cb_image_mode->currentIndex();
    iss = ui->cb_image_mode->currentText();
  } else {
    ui->cb_image_source->setCurrentIndex(0);
    isi = -1;
  }
  ui->cb_image_mode->clear();
  if (sources.empty()) {
    ui->cb_image_source->blockSignals(false);
    ui->cb_image_mode->blockSignals(false);
    return;
  }

  const int sourceIdx = ui->cb_image_source->currentIndex();
  if (sourceIdx < 0) {
    ui->cb_image_source->blockSignals(false);
    ui->cb_image_mode->blockSignals(false);
    return;
  }
  const std::size_t sourceIndex{static_cast<std::size_t>(sourceIdx)};
  if (sourceIndex >= sources.size()) {
    ui->cb_image_source->blockSignals(false);
    ui->cb_image_mode->blockSignals(false);
    return;
  }

  const ImageModes& ms = sources[sourceIndex].modes;
  for_each(ms.begin(), ms.end(), [&](const ImageMode& m) { ui->cb_image_mode->addItem(m.name); });

  if (isi != -1 && isi < ui->cb_image_mode->count() && ui->cb_image_mode->itemText(isi) == iss)
    ui->cb_image_mode->setCurrentIndex(isi);
  else
    ui->cb_image_mode->setCurrentIndex(0);
  ui->cb_image_source->blockSignals(false);
  ui->cb_image_mode->blockSignals(false);
}

void ControlWindow::updateSource(bool fit_axis) {
  if (sources.empty()) return;

  const int sourceIdx = ui->cb_image_source->currentIndex();
  if (sourceIdx < 0) return;
  const std::size_t sourceIndex{static_cast<std::size_t>(sourceIdx)};
  if (sourceIndex >= sources.size()) return;

  const ImageSource& s = sources[sourceIndex];

  const int modeIdx = ui->cb_image_mode->currentIndex();
  if (modeIdx < 0) return;
  const std::size_t modeIndex{static_cast<std::size_t>(modeIdx)};
  if (modeIndex >= s.modes.size()) return;

  const ImageMode& m = s.modes[modeIndex];
  lplot->qplot->setCurrentLayer("image");
  lplot->qplot->clearPlottables();
  lplot->hold(true);
  imgMap = lplot->plot(s.data, fit_axis, QCPRange(), QCPRange(), s.range, m.grad, QCPAxis::stLinear,
                       ui->chb_image_interp->isChecked());
  lplot->hold(false);
}

void ControlWindow::updateSourceOptions() {
  if (!imgMap) return;
  const int sourceIdx = ui->cb_image_source->currentIndex();
  if (sourceIdx < 0) return;
  const std::size_t sourceIndex{static_cast<std::size_t>(sourceIdx)};
  if (sourceIndex >= sources.size()) return;

  const ImageSource& s = sources[sourceIndex];

  const int modeIdx = ui->cb_image_mode->currentIndex();
  if (modeIdx < 0) return;
  const std::size_t modeIndex{static_cast<std::size_t>(modeIdx)};
  if (modeIndex >= s.modes.size()) return;

  const ImageMode& m = s.modes[modeIndex];
  if (m.name != "RGB") imgMap->setGradient(m.grad);
  imgMap->setInterpolate(ui->chb_image_interp->isChecked());
  if (ui->chb_image_grid->isChecked() && lplot->qplot->layer("grid")->index() > lplot->qplot->layer("image")->index())
    lplot->qplot->moveLayer(lplot->qplot->layer("image"), lplot->qplot->layer("grid"), QCustomPlot::limAbove);
  else if (!ui->chb_image_grid->isChecked() &&
           lplot->qplot->layer("grid")->index() < lplot->qplot->layer("image")->index())
    lplot->qplot->moveLayer(lplot->qplot->layer("image"), lplot->qplot->layer("grid"), QCustomPlot::limBelow);
  lplot->qplot->layer("image")->setVisible(ui->chb_image_show->isChecked());
  // lplot->qplot->layer("quiver")->setVisible(ui->chb_quivers->isChecked());
  lplot->replot();
}


void ControlWindow::showQuiverControl() { qo->show(); }

void ControlWindow::connectQuiver() {
  QObject::connect(qo->ui->chb_visibility, SIGNAL(stateChanged(int)), this, SLOT(toggleQuiver()), Qt::AutoConnection);

  QObject::connect(qo->ui->pb_delete, SIGNAL(clicked()), this, SLOT(deleteQuivers()), Qt::AutoConnection);
  QObject::connect(qo, SIGNAL(computeQuivers()), this, SLOT(processQuiver()), Qt::AutoConnection);
}

void ControlWindow::processQuiver() {
  if (src.empty()) {
    return;
  }

  for_each(quiver.begin(), quiver.end(), [&](Line& l) { lplot->qplot->removeItem(l.line); });
  quiver.clear();

  cv::Mat mag, gx, gy, dir, phase, energy;
  for_each(sources.begin(), sources.end(), [&](ImageSource& is) {
    if (is.name.compare(QString("mag"), Qt::CaseInsensitive) == 0) {
      mag = is.data.clone();
    }
    if (is.name.compare(QString("gx"), Qt::CaseInsensitive) == 0) {
      gx = is.data.clone();
    }
    if (is.name.compare(QString("gy"), Qt::CaseInsensitive) == 0) {
      gy = is.data.clone();
    }
    if (is.name.compare(QString("dir"), Qt::CaseInsensitive) == 0) {
      dir = is.data;
    }
    if (is.name.compare(QString("phase"), Qt::CaseInsensitive) == 0) {
      phase = is.data.clone();
    }
    if (is.name.compare(QString("energy"), Qt::CaseInsensitive) == 0) {
      energy = is.data;
    }
  });

  if (qo->data_mode == 0) {
    gradientQuiver(mag, gx, gy);
  }
  if (qo->data_mode == 1) {
    angleQuiver(dir, mag);
  }
  if (qo->data_mode == 2) {
    angleQuiver(phase, energy);
  }
  if (qo->data_mode == 3) {
    angleQuiver(dir + phase, energy);
  }

  // draw the quivers
  lplot->hold(true);
  for_each(quiver.begin(), quiver.end(), [&](Line& q) {
    const LineSegment& qs = q.segment;
    lplot->qplot->setCurrentLayer("quiver");
    q.line = lplot->line(qs, false);
    q.line->setHead(QCPLineEnding(QCPLineEnding::esSpikeArrow, 4, 5));
    q.line->setPen(qo->qPen);
  });
  lplot->hold(false);
  lplot->qplot->layer("quiver")->setVisible(qo->ui->chb_visibility->isChecked());
  lplot->replot();
}

void ControlWindow::gradientQuiver(cv::Mat mag, cv::Mat gx, cv::Mat gy) {
  double mag_min = 0.0, mag_max = 0.0, gx_max = 0.0, gy_max = 0.0;
  if (mag.empty()) {
    cout << "No mag for quiver computation available." << endl;
    return;
  }
  mag.convertTo(mag, CV_64FC1);
  cv::minMaxIdx(mag, &mag_min, &mag_max);
  cv::resize(mag, mag, cv::Size(), 1.0 / qo->interpolation_pixels, 1.0 / qo->interpolation_pixels,
             qo->interpolation_mode);

  // gx, gy
  if (qo->data_mode == 0) {
    if (gx.empty() || gy.empty()) {
      cout << "No gx or gy for quiver computation available." << endl;
      return;
    }
    gx.convertTo(gx, CV_64FC1);
    gy.convertTo(gy, CV_64FC1);
    cv::minMaxIdx(gx, NULL, &gx_max);
    cv::minMaxIdx(gy, NULL, &gy_max);
    cv::resize(gx, gx, cv::Size(), 1.0 / qo->interpolation_pixels, 1.0 / qo->interpolation_pixels,
               qo->interpolation_mode);
    cv::resize(gy, gy, cv::Size(), 1.0 / qo->interpolation_pixels, 1.0 / qo->interpolation_pixels,
               qo->interpolation_mode);

    for (int y = 0; y < gx.rows; ++y) {
      for (int x = 0; x < gx.cols; ++x) {
        if (qo->threshold_use) {
          if (mag.at<float_type>(y, x) > qo->threshold) {
            float_type gxv, gyv;
            Line l;

            gxv = (gx.at<float_type>(y, x) / gx_max);
            gyv = (gy.at<float_type>(y, x) / gy_max);

            if (!qo->scale_use) {
              cv::Vec<float_type, 2> norm = cv::normalize(cv::Vec<float_type, 2>(gxv, gyv));
              gxv = norm[0];
              gyv = norm[1];
            }

            gxv = (gxv / 2.0) * qo->interpolation_pixels;
            gyv = (gyv / 2.0) * qo->interpolation_pixels;

            l = Line(LineSegment(lsfm::Vec4<float_type>(
                ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - gxv,
                ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - gyv,
                ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + gxv,
                ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + gyv)));

            quiver.push_back(l);
          }
        } else {
          float_type gxv, gyv;
          Line l;

          gxv = (gx.at<float_type>(y, x) / gx_max);
          gyv = (gy.at<float_type>(y, x) / gy_max);

          if (!qo->scale_use) {
            cv::Vec<float_type, 2> norm = cv::normalize(cv::Vec<float_type, 2>(gxv, gyv));
            gxv = norm[0];
            gyv = norm[1];
          }

          gxv = (gxv / 2.0) * qo->interpolation_pixels;
          gyv = (gyv / 2.0) * qo->interpolation_pixels;

          l = Line(LineSegment(lsfm::Vec4<float_type>(
              ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - gxv,
              ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - gyv,
              ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + gxv,
              ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + gyv)));

          quiver.push_back(l);
        }
      }
    }
  }
}

void ControlWindow::angleQuiver(cv::Mat ang, cv::Mat ampl) {
  double ang_min = 0.0, ang_max = 0.0, ampl_max = 0.0;

  if (ang.empty() || ampl.empty()) {
    cout << "No phase or even or odd or energy for quiver computation available." << endl;
    return;
  }
  ang.convertTo(ang, CV_64FC1);
  ampl.convertTo(ampl, CV_64FC1);
  cv::minMaxIdx(ang, &ang_min, &ang_max);
  cv::minMaxIdx(ampl, NULL, &ampl_max);
  cv::resize(ang, ang, cv::Size(), 1.0 / qo->interpolation_pixels, 1.0 / qo->interpolation_pixels,
             qo->interpolation_mode);
  cv::resize(ampl, ampl, cv::Size(), 1.0 / qo->interpolation_pixels, 1.0 / qo->interpolation_pixels,
             qo->interpolation_mode);

  for (int y = 0; y < ang.rows; ++y) {
    for (int x = 0; x < ang.cols; ++x) {
      if (qo->threshold_use) {
        if (ampl.at<double>(y, x) > qo->threshold) {
          float_type p_val, e_val;
          float_type xoff, yoff;
          Line l;

          p_val = ang.at<double>(y, x);
          e_val = ampl.at<double>(y, x) / ampl_max;

          xoff = (cos(p_val) / 2.0) * qo->interpolation_pixels;
          yoff = (sin(p_val) / 2.0) * qo->interpolation_pixels;

          if (qo->scale_use) {
            xoff *= e_val;
            yoff *= e_val;
          }

          l = Line(LineSegment(lsfm::Vec4<float_type>(
              ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - xoff,
              ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - yoff,
              ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + xoff,
              ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + yoff)));

          quiver.push_back(l);
        }
      } else {
        float_type p_val, e_val;
        float_type xoff, yoff;
        Line l;

        p_val = ang.at<double>(y, x);
        e_val = ampl.at<double>(y, x) / ampl_max;

        xoff = (cos(p_val) / 2.0) * qo->interpolation_pixels;
        yoff = (sin(p_val) / 2.0) * qo->interpolation_pixels;

        if (qo->scale_use) {
          xoff *= e_val;
          yoff *= e_val;
        }

        l = Line(LineSegment(lsfm::Vec4<float_type>(
            ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - xoff,
            ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) - yoff,
            ((double(x) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + xoff,
            ((double(y) * qo->interpolation_pixels) - 0.5) + (double(qo->interpolation_pixels) / 2.0) + yoff)));

        quiver.push_back(l);
      }
    }
  }
}

void ControlWindow::toggleQuiver() {
  lplot->qplot->layer("quiver")->setVisible(qo->ui->chb_visibility->isChecked());
  lplot->replot();
}

void ControlWindow::deleteQuivers(bool replot) {
  for_each(quiver.begin(), quiver.end(), [&](Line& l) { lplot->qplot->removeItem(l.line); });
  quiver.clear();

  if (replot) lplot->qplot->replot();
}


void ControlWindow::sourceChanged() {
  ui->cb_image_mode->blockSignals(true);
  ui->cb_image_mode->clear();
  if (sources.empty()) {
    ui->cb_image_mode->blockSignals(false);
    return;
  }

  const int sourceIdx = ui->cb_image_source->currentIndex();
  if (sourceIdx < 0) {
    ui->cb_image_mode->blockSignals(false);
    return;
  }
  const std::size_t sourceIndex{static_cast<std::size_t>(sourceIdx)};
  if (sourceIndex >= sources.size()) {
    ui->cb_image_mode->blockSignals(false);
    return;
  }

  const ImageModes& ms = sources[sourceIndex].modes;
  for_each(ms.begin(), ms.end(), [&](const ImageMode& m) { ui->cb_image_mode->addItem(m.name); });
  ui->cb_image_mode->setCurrentIndex(0);
  ui->cb_image_mode->blockSignals(false);
  updateSource(false);
  lplot->replot();
}

void ControlWindow::fitImage() {
  if (sources.empty()) return;
  lplot->setAxis(QCPRange(0, src.cols - 1), QCPRange(0, src.rows - 1));
  lplot->replot();
}

void ControlWindow::addDetector(const DetectorPtr& detector) {
  if (!detector) return;
  detectors.push_back(detector);
  detectors.back()->create();
  if (ui->cb_detector_select->currentIndex() == -1) {
    ui->cb_detector_select->setEnabled(true);
    ui->pb_detector_params_reset->setEnabled(true);
    ui->pb_detectors_params_reset->setEnabled(true);
    ui->table_detector_params->setEnabled(true);
  }
  ui->cb_detector_select->addItem(detector->name);
}

void ControlWindow::selectDetector() {
  const int detectorIdx = ui->cb_detector_select->currentIndex();
  if (detectorIdx < 0) return;
  const std::size_t detectorIndex{static_cast<std::size_t>(detectorIdx)};
  if (detectorIndex >= detectors.size()) return;

  DetectorPtr detector = detectors[detectorIndex];
  lsfm::ValueManager::NameValueVector options = detector->lsd->values();

  ui->table_detector_params->setColumnCount(static_cast<int>(options.size()));
  int col = 0;
  ui->table_detector_params->blockSignals(true);
  for_each(options.begin(), options.end(), [&](const lsfm::ValueManager::NameValuePair& option) {
    QTableWidgetItem* item = new QTableWidgetItem(QString(option.value.toString().c_str()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_detector_params->setItem(0, col, item);
    item = new QTableWidgetItem(QString(option.name.c_str()));
    item->setToolTip(option.description.c_str());
    ui->table_detector_params->setHorizontalHeaderItem(col, item);

    ++col;
  });
  ui->table_detector_params->blockSignals(false);
  ui->table_detector_params->resizeColumnsToContents();
  ui->table_detector_params->horizontalHeader()->setStretchLastSection(true);
}

void ControlWindow::resetDetector() {
  const int detectorIdx = ui->cb_detector_select->currentIndex();
  if (detectorIdx < 0) return;
  const std::size_t detectorIndex{static_cast<std::size_t>(detectorIdx)};
  if (detectorIndex >= detectors.size()) return;

  detectors[detectorIndex]->create();
  selectDetector();
}

void ControlWindow::resetAllDetectors() {
  for_each(detectors.begin(), detectors.end(), [](DetectorPtr d) { d->create(); });
  selectDetector();
}


void ControlWindow::setDetectorOption(int row, int col) {
  const int detectorIdx = ui->cb_detector_select->currentIndex();
  if (detectorIdx < 0) return;
  const std::size_t detectorIndex{static_cast<std::size_t>(detectorIdx)};
  if (detectorIndex >= detectors.size()) return;

  DetectorPtr detector = detectors[detectorIndex];
  if (col < 0) return;
  const std::size_t columnIndex{static_cast<std::size_t>(col)};

  lsfm::Value option = detector->lsd->value(columnIndex);
  lsfm::Value tmp;
  tmp.fromString(ui->table_detector_params->item(row, col)->text().toStdString());

  if (option.type() == lsfm::Value::FLOAT && tmp.type() == lsfm::Value::INT) tmp = static_cast<double>(tmp.getInt64());

  if (tmp.type() != option.type()) {
    ui->table_detector_params->blockSignals(true);
    ui->table_detector_params->item(row, col)->setText(option.toString().c_str());
    ui->table_detector_params->blockSignals(false);
  } else {
    detector->lsd->value(columnIndex, tmp);
    ui->table_detector_params->blockSignals(true);
    ui->table_detector_params->item(row, col)->setText(detector->lsd->value(columnIndex).toString().c_str());
    ui->table_detector_params->blockSignals(false);
  }
}

void ControlWindow::processData() {
  if (src.empty()) return;
  sources.resize(inputSourcesSize);
  const int detectorIdx = ui->cb_detector_select->currentIndex();
  if (detectorIdx < 0) return;
  const std::size_t detectorIndex{static_cast<std::size_t>(detectorIdx)};
  if (detectorIndex >= detectors.size()) return;

  DetectorPtr detector = detectors[detectorIndex];
  if (detector->colorInput() && src.channels() != 3) return;
  double stime = double(cv::getTickCount());
  LineSegmentVector l = detector->detect(src);
  std::cout << "Time for line detection: " << (double(cv::getTickCount()) - stime) * 1000 / cv::getTickFrequency()
            << "ms" << std::endl;

  ImageSources ds = detector->sources(src);
  sources.insert(sources.end(), ds.begin(), ds.end());
  loadSources();
  lplot->show();
  updateSource(false);
  deleteQuivers(false);
  setLines(l);
}

void ControlWindow::clearLines() {
  lines.clear();
  lplot->qplot->clearItems();
  indicator = 0;
}

void ControlWindow::setLines(const LineSegmentVector& ls) {
  clearLines();
  for_each(ls.begin(), ls.end(), [&](const LineSegment& line) { lines.push_back(Line(line)); });
  processLineData();
}

void ControlWindow::processLineData() {
  lplot->show();
  ui->table_lines->setRowCount(static_cast<int>(lines.size()));
  int row = 0;
  ui->table_lines->blockSignals(true);
  ui->table_lines->clearSelection();
  ui->table_lines->scrollToTop();

  lplot->qplot->setCurrentLayer("indicator");
  indicator = new QCPItemLine(lplot->qplot);
  lplot->qplot->addItem(indicator);
  indicator->setPen(iPen);
  indicator->setSelectable(false);

  lplot->hold(true);
  for_each(lines.begin(), lines.end(), [&](Line& tl) {
    const LineSegment& line = tl.segment;
    QTableWidgetItem* item = new QTableWidgetItem(QString::number(line.angle() * 180 / CV_PI));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 0, item);
    item = new QTableWidgetItem(QString::number(line.originDist()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 1, item);
    item = new QTableWidgetItem(QString::number(line.start()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 2, item);
    item = new QTableWidgetItem(QString::number(line.end()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 3, item);
    item = new QTableWidgetItem(QString::number(line.length()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 4, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item = new QTableWidgetItem(QString::number(line.normalX()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 5, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item = new QTableWidgetItem(QString::number(line.normalY()));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 6, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    auto p = line.centerPoint();
    item = new QTableWidgetItem(QString::number(p.x));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 7, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item = new QTableWidgetItem(QString::number(p.y));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 8, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    p = line.startPoint();
    item = new QTableWidgetItem(QString::number(p.x));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 9, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item = new QTableWidgetItem(QString::number(p.y));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 10, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    p = line.endPoint();
    item = new QTableWidgetItem(QString::number(p.x));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 11, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item = new QTableWidgetItem(QString::number(p.y));
    item->setTextAlignment(Qt::AlignCenter);
    ui->table_lines->setItem(row, 12, item);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);

    lplot->qplot->setCurrentLayer("lines");
    tl.line = lplot->line(line, false);
    if (ui->chb_endpoint_show->isChecked()) tl.line->setHead(QCPLineEnding(QCPLineEnding::esSpikeArrow, 4, 8));
    tl.line->setPen(pen);
    tl.line->setSelectable(true);
    tl.line->setSelectedPen(selPen);
    tl.line->setProperty("index", row);

    lplot->qplot->setCurrentLayer("normals");
    cv::Point_<float_type> p1 = line.normalLineDist(0, line.centerPoint()),
                           p2 = line.normalLineDist(ui->spin_normal_length->value(), line.centerPoint());
    tl.normal = lplot->line(p1.x, p1.y, p2.x, p2.y, false, nPen);
    tl.normal->setSelectable(true);
    tl.normal->setSelectedPen(nSelPen);
    tl.normal->setProperty("index", row);

    ++row;
  });
  lplot->hold(false);
  ui->table_lines->blockSignals(false);

  ui->layout_line_options->setDisabled(false);
  ui->pb_normal_flip->setDisabled(false);
  ui->pb_endpoint_flip->setDisabled(false);
  ui->pb_lines_freeze->setDisabled(false);
  ui->pb_lines_reset->setDisabled(false);
  emit sourcesChanged(sources);

  setControls();
  lineSel = -1;
  emit lineSelChanged(lineSel);
  lplot->replot();
}

void ControlWindow::updateLineGeometry(Line& l) {
  if (l.line == nullptr || l.normal == nullptr) return;
  bool modified = l.mod.modified();
  LineSegment seg = l.modSegment();

  cv::Point_<float_type> p1 = seg.startPoint(), p2 = seg.endPoint();
  l.line->start->setCoords(QPointF(p1.x, p1.y));
  l.line->end->setCoords(QPointF(p2.x, p2.y));
  if (ui->chb_endpoint_show->isChecked())
    l.line->setHead(QCPLineEnding(QCPLineEnding::esSpikeArrow, 4, 8));
  else
    l.line->setHead(QCPLineEnding(QCPLineEnding::esNone));
  l.line->setPen(modified ? mPen : pen);
  l.line->setSelectedPen(selPen);

  p1 = seg.normalLineDist(0, seg.centerPoint()),
  p2 = seg.normalLineDist(ui->spin_normal_length->value(), seg.centerPoint());
  l.normal->start->setCoords(QPointF(p1.x, p1.y));
  l.normal->end->setCoords(QPointF(p2.x, p2.y));
  l.normal->setPen(modified ? nmPen : nPen);
  l.normal->setSelectedPen(nSelPen);
}

void ControlWindow::updateLine(int idx) {
  if (idx < 0) return;
  const std::size_t index{static_cast<std::size_t>(idx)};
  if (index >= lines.size()) return;

  Line& l{lines[index]};
  updateLineGeometry(l);
  updateLineTableData(idx);
  if (idx == lineSel) {
    setControls(&l.mod);
    setLineSelected(l);
    emit lineChanged(l.modSegment());
  }
  lplot->replot();
}

void ControlWindow::updateLine() { updateLine(lineSel); }

void ControlWindow::updateLines() {
  int idx = 0;
  for_each(lines.begin(), lines.end(), [&](Line& l) {
    updateLineGeometry(l);
    updateLineTableData(idx++);
  });

  if (lineSel >= 0) {
    const std::size_t selected{static_cast<std::size_t>(lineSel)};
    if (selected < lines.size()) {
      setControls(&lines[selected].mod);
      emit lineChanged(lines[selected].modSegment());
    }
  }
  lplot->replot();
}

void ControlWindow::updateLineLayers() {
  lplot->qplot->layer("lines")->setVisible(ui->chb_lines_show->isChecked());
  lplot->qplot->layer("normals")->setVisible(ui->chb_lines_show->isChecked() && ui->chb_normal_show->isChecked());
  lplot->replot();
}


void ControlWindow::readLineMod() {
  if (lineSel < 0) return;
  const std::size_t index{static_cast<std::size_t>(lineSel)};
  if (index >= lines.size()) return;
  Line& l{lines[index]};
  l.mod.angle = ui->spin_line_rot->value() / 180 * CV_PI;
  l.mod.distance = ui->spin_line_otrans->value();
  l.mod.start = ui->spin_line_strans->value();
  l.mod.end = ui->spin_line_etrans->value();
}

void ControlWindow::updateLineTableData(int idx) {
  if (idx < 0) return;
  const std::size_t index{static_cast<std::size_t>(idx)};
  if (index >= lines.size()) return;

  LineSegment line{lines[index].modSegment()};
  ui->table_lines->blockSignals(true);
  ui->table_lines->item(idx, 0)->setText(QString::number(line.angle() * 180 / CV_PI));
  ui->table_lines->item(idx, 1)->setText(QString::number(line.originDist()));
  ui->table_lines->item(idx, 2)->setText(QString::number(line.start()));
  ui->table_lines->item(idx, 3)->setText(QString::number(line.end()));
  ui->table_lines->item(idx, 4)->setText(QString::number(line.length()));
  ui->table_lines->item(idx, 5)->setText(QString::number(line.normalX()));
  ui->table_lines->item(idx, 6)->setText(QString::number(line.normalY()));

  auto p{line.centerPoint()};
  ui->table_lines->item(idx, 7)->setText(QString::number(p.x));
  ui->table_lines->item(idx, 8)->setText(QString::number(p.y));


  p = line.startPoint();
  ui->table_lines->item(idx, 9)->setText(QString::number(p.x));
  ui->table_lines->item(idx, 10)->setText(QString::number(p.y));

  p = line.endPoint();
  ui->table_lines->item(idx, 11)->setText(QString::number(p.x));
  ui->table_lines->item(idx, 12)->setText(QString::number(p.y));

  ui->table_lines->blockSignals(false);
}


void ControlWindow::updateLineByCellChange(int row, int col) {
  if (col > 3) return;
  if (row < 0) return;
  const std::size_t index{static_cast<std::size_t>(row)};
  if (index >= lines.size()) return;

  Line& l{lines[index]};
  bool ok;
  float_type val = static_cast<float_type>(ui->table_lines->item(row, col)->text().toDouble(&ok));
  if (ok) switch (col) {
      case 0: {
        val -= l.segment.angle() * 180 / CV_PI;
        if (val <= ui->spin_range_rot->value() && val >= -ui->spin_range_rot->value()) {
          l.mod.angle = val * CV_PI / 180;
        } else {
          ok = false;
          val = l.segment.angle() * 180 / CV_PI;
        }

      } break;
      case 1: {
        val -= l.segment.originDist();
        if (val <= ui->spin_range_otrans->value() && val >= -ui->spin_range_otrans->value()) {
          l.mod.distance = val;
        } else {
          ok = false;
          val = l.segment.originDist();
        }
      } break;
      case 2: {
        val -= l.segment.start();
        if (val <= ui->spin_range_etrans->value() && val >= -ui->spin_range_etrans->value()) {
          l.mod.start = val;
        } else {
          ok = false;
          val = l.segment.start();
        }
      } break;
      case 3: {
        val -= l.segment.end();
        if (val <= ui->spin_range_etrans->value() && val >= -ui->spin_range_etrans->value()) {
          l.mod.end = val;
        } else {
          ok = false;
          val = l.segment.end();
        }
      } break;
    }
  if (ok) {
    updateLine(row);
  } else
    ui->table_lines->item(row, col)->setText(QString::number(val));
}

void ControlWindow::flipNormals() {
  for_each(lines.begin(), lines.end(), [&](Line& line) {
    line.segment.normalFlip();
    line.mod.distance *= -1;
    line.mod.start *= -1;
    line.mod.end *= -1;
  });
  updateLines();
}

void ControlWindow::flipEndpoints() {
  for_each(lines.begin(), lines.end(), [&](Line& line) {
    line.segment.endPointSwap();
    std::swap(line.mod.start, line.mod.end);
  });
  updateLines();
}

void ControlWindow::fitLine() {
  if (lineSel < 0) return;
  const std::size_t index{static_cast<std::size_t>(lineSel)};
  if (index >= lines.size()) return;

  auto l{lines[index].segment.endPoints()};
  lplot->setAxis(l[0], l[2], l[1], l[3], 1, 1, 2, 2);
  float_type w = abs(l[0] - l[2]), h = abs(l[1] - l[3]);
  if (w > h) {
    if (w / h > static_cast<float_type>(lplot->qplot->width() / lplot->qplot->height()))
      lplot->qplot->yAxis->setScaleRatio(lplot->qplot->xAxis);
    else
      lplot->qplot->xAxis->setScaleRatio(lplot->qplot->yAxis);
  } else {
    if (w / h > static_cast<float_type>(lplot->qplot->width() / lplot->qplot->height()))
      lplot->qplot->xAxis->setScaleRatio(lplot->qplot->yAxis);
    else
      lplot->qplot->xAxis->setScaleRatio(lplot->qplot->yAxis);
  }
  lplot->keepAspectRatio(false);
  lplot->replot();
  lplot->keepAspectRatio(true);
}

void ControlWindow::selectLineColor() {
  QColor c = cdia->getColor(pen.color(), this, QString("Line Color"));
  pen.setColor(c);
  nPen.setColor(c);

  for_each(lines.begin(), lines.end(), [&](Line& tl) { tl.line->setPen(pen); });
  updateLines();
}

void ControlWindow::clearLineSelected() {
  auto items = lplot->qplot->selectedItems();
  lplot->qplot->blockSignals(true);
  if (items.size())
    for_each(items.begin(), items.end(), [&](decltype(*items.begin())& item) { item->setSelected(false); });
  lplot->qplot->blockSignals(false);
  ui->pb_line_fit->setEnabled(false);
  lineSel = -1;
}

void ControlWindow::setLineSelected(Line& l) {
  lplot->qplot->blockSignals(true);
  l.line->setSelected(true);
  l.normal->setSelected(true);
  lplot->qplot->blockSignals(false);
  ui->pb_line_fit->setEnabled(true);
}

void ControlWindow::selectLineByRow() { selectLine(ui->table_lines->currentRow()); }

void ControlWindow::selectLine(int sel) {
  if (sel == lineSel) return;
  if (sel < 0) {
    clearLineSelected();
    lineSel = sel;
    emit lineSelChanged(lineSel);
    lplot->replot();
    return;
  }

  const std::size_t index{static_cast<std::size_t>(sel)};
  if (index >= lines.size()) return;
  clearLineSelected();
  if (sel > -1) {
    lineSel = sel;
    updateLine(sel);
    emit lineSelChanged(lineSel);
    return;
  }

  emit lineSelChanged(lineSel);
  lplot->replot();
}

void ControlWindow::lplotSelChange() {
  auto items = lplot->qplot->selectedItems();
  if (items.size()) {
    const int selection = items.front()->property("index").toInt();
    bool validSelection = selection >= 0;
    std::size_t index{};
    if (validSelection) {
      index = static_cast<std::size_t>(selection);
      validSelection = index < lines.size();
    }

    if (validSelection) {
      lineSel = selection;
      lplot->qplot->blockSignals(true);
      if (items.size() > 1)
        for_each(items.begin() + 1, items.end(), [&](decltype(*items.begin())& item) { item->setSelected(false); });
      lplot->qplot->blockSignals(false);

      ui->table_lines->blockSignals(true);
      ui->table_lines->selectRow(lineSel);
      ui->table_lines->blockSignals(false);

      setLineSelected(lines[index]);
      setControls(&lines[index].mod);
      emit lineChanged(lines[index].modSegment());
    } else {
      lineSel = -1;
      ui->pb_line_fit->setDisabled(true);
      ui->table_lines->blockSignals(true);
      ui->table_lines->clearSelection();
      ui->table_lines->blockSignals(false);
      setControls();
    }
  } else {
    lineSel = -1;
    ui->pb_line_fit->setDisabled(true);
    ui->table_lines->blockSignals(true);
    ui->table_lines->clearSelection();
    ui->table_lines->blockSignals(false);
    setControls();
  }
  emit lineSelChanged(lineSel);
}

void ControlWindow::freezeLine() {
  if (lineSel < 0) return;
  const std::size_t index{static_cast<std::size_t>(lineSel)};
  if (index >= lines.size()) return;
  lines[index].freeze();
  updateLine();
}

void ControlWindow::freezeAllLines() {
  for_each(lines.begin(), lines.end(), [&](Line& l) { l.freeze(); });
  updateLines();
  emit linesUpdated(&lines);
}

void ControlWindow::resetControls() {
  if (lineSel < 0) return;
  const std::size_t index{static_cast<std::size_t>(lineSel)};
  if (index >= lines.size()) return;
  lines[index].reset();
  updateLine();
}

void ControlWindow::resetAllControls() {
  for_each(lines.begin(), lines.end(), [&](Line& l) { l.reset(); });
  updateLines();
}

void ControlWindow::setControls(const LineMod* mod) {
  bool disable;
  LineMod tmp;
  if (mod == nullptr) {
    disable = true;
    mod = &tmp;
  } else
    disable = false;

  ui->spin_line_rot->blockSignals(true);
  ui->spin_line_rot->setValue(mod->angle * 180 / CV_PI);
  ui->spin_line_rot->blockSignals(false);
  ui->spin_line_rot->setDisabled(disable);
  ui->slider_line_rot->blockSignals(true);
  ui->slider_line_rot->setValue(static_cast<int>(ui->spin_prec_trans->value() * mod->angle * 180 / CV_PI));
  ui->slider_line_rot->blockSignals(false);
  ui->slider_line_rot->setDisabled(disable);


  ui->spin_line_otrans->blockSignals(true);
  ui->spin_line_otrans->setValue(mod->distance);
  ui->spin_line_otrans->blockSignals(false);
  ui->spin_line_otrans->setDisabled(disable);
  ui->slider_line_otrans->blockSignals(true);
  ui->slider_line_otrans->setValue(static_cast<int>(ui->spin_prec_trans->value() * mod->distance));
  ui->slider_line_otrans->blockSignals(false);
  ui->slider_line_otrans->setDisabled(disable);

  ui->spin_line_etrans->blockSignals(true);
  ui->spin_line_etrans->setValue(mod->end);
  ui->spin_line_etrans->blockSignals(false);
  ui->spin_line_etrans->setDisabled(disable);
  ui->slider_line_etrans->blockSignals(true);
  ui->slider_line_etrans->setValue(static_cast<int>(ui->spin_prec_trans->value() * mod->end));
  ui->slider_line_etrans->blockSignals(false);
  ui->slider_line_etrans->setDisabled(disable);

  ui->spin_line_strans->blockSignals(true);
  ui->spin_line_strans->setValue(mod->start);
  ui->spin_line_strans->blockSignals(false);
  ui->spin_line_strans->setDisabled(disable);
  ui->slider_line_strans->blockSignals(true);
  ui->slider_line_strans->setValue(static_cast<int>(ui->spin_prec_trans->value() * mod->start));
  ui->slider_line_strans->blockSignals(false);
  ui->slider_line_strans->setDisabled(disable);

  ui->pb_line_reset->setDisabled(disable);
  ui->pb_line_freeze->setDisabled(disable);
}

void ControlWindow::addTool(LATool* tool) {
  QPushButton* pb = new QPushButton(tool->name(), ui->group_tools);
  int row = tools / 4;
  int col = tools % 4;
  ui->layout_tools->addWidget(pb, row, col, 1, 1);
  connect(pb, SIGNAL(clicked()), tool, SLOT(show()));
  tool->connectTools(this);
  ++tools;
}

void ControlWindow::updateRanges() {
  int rotRange = ui->spin_range_rot->value();
  ui->spin_line_rot->blockSignals(true);
  ui->spin_line_rot->setMaximum(rotRange);
  ui->spin_line_rot->setMinimum(-rotRange);
  ui->spin_line_rot->blockSignals(false);
  int val = ui->spin_prec_rot->value();
  ui->slider_line_rot->blockSignals(true);
  ui->slider_line_rot->setMaximum(rotRange * val);
  ui->slider_line_rot->setMinimum(-rotRange * val);
  ui->slider_line_rot->blockSignals(false);

  int moveOrthoRange = ui->spin_range_otrans->value();
  val = ui->spin_prec_trans->value();
  ui->spin_line_otrans->blockSignals(true);
  ui->spin_line_otrans->setMaximum(moveOrthoRange);
  ui->spin_line_otrans->setMinimum(-moveOrthoRange);
  ui->spin_line_otrans->blockSignals(false);
  ui->slider_line_otrans->blockSignals(true);
  ui->slider_line_otrans->setMaximum(moveOrthoRange * val);
  ui->slider_line_otrans->setMinimum(-moveOrthoRange * val);
  ui->slider_line_otrans->blockSignals(false);

  int moveEndpointRange = ui->spin_range_etrans->value();
  ui->spin_line_strans->blockSignals(true);
  ui->spin_line_strans->setMaximum(moveEndpointRange);
  ui->spin_line_strans->setMinimum(-moveEndpointRange);
  ui->spin_line_strans->blockSignals(false);
  ui->slider_line_strans->blockSignals(true);
  ui->slider_line_strans->setMaximum(moveEndpointRange * val);
  ui->slider_line_strans->setMinimum(-moveEndpointRange * val);
  ui->slider_line_strans->blockSignals(false);

  ui->spin_line_etrans->blockSignals(true);
  ui->spin_line_etrans->setMaximum(moveEndpointRange);
  ui->spin_line_etrans->setMinimum(-moveEndpointRange);
  ui->spin_line_etrans->blockSignals(false);
  ui->slider_line_etrans->blockSignals(true);
  ui->slider_line_etrans->setMaximum(moveEndpointRange * val);
  ui->slider_line_etrans->setMinimum(-moveEndpointRange * val);
  ui->slider_line_etrans->blockSignals(false);
}

void ControlWindow::setRotSliderValue(double val) {
  ui->slider_line_rot->blockSignals(true);
  ui->slider_line_rot->setValue(static_cast<int>(ui->spin_prec_rot->value() * val));
  ui->slider_line_rot->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setRotSpinValue(int val) {
  ui->spin_line_rot->blockSignals(true);
  ui->spin_line_rot->setValue(static_cast<float_type>(val) / ui->spin_prec_rot->value());
  ui->spin_line_rot->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setOTransSliderValue(double val) {
  ui->slider_line_otrans->blockSignals(true);
  ui->slider_line_otrans->setValue(static_cast<int>(ui->spin_prec_trans->value() * val));
  ui->slider_line_otrans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setOTransSpinValue(int val) {
  ui->spin_line_otrans->blockSignals(true);
  ui->spin_line_otrans->setValue(static_cast<float_type>(val) / ui->spin_prec_trans->value());
  ui->spin_line_otrans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setSTransSliderValue(double val) {
  ui->slider_line_strans->blockSignals(true);
  ui->slider_line_strans->setValue(static_cast<int>(ui->spin_prec_trans->value() * val));
  ui->slider_line_strans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setSTransSpinValue(int val) {
  ui->spin_line_strans->blockSignals(true);
  ui->spin_line_strans->setValue(static_cast<float_type>(val) / ui->spin_prec_trans->value());
  ui->spin_line_strans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setETransSliderValue(double val) {
  ui->slider_line_etrans->blockSignals(true);
  ui->slider_line_etrans->setValue(static_cast<int>(ui->spin_prec_trans->value() * val));
  ui->slider_line_etrans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setETransSpinValue(int val) {
  ui->spin_line_etrans->blockSignals(true);
  ui->spin_line_etrans->setValue(static_cast<float_type>(val) / ui->spin_prec_trans->value());
  ui->spin_line_etrans->blockSignals(false);
  readLineMod();
  updateLine();
}

void ControlWindow::setTransPrec(int val) {
  int moveOrthoRange = ui->spin_range_otrans->value();
  ui->spin_line_otrans->setSingleStep(1.0 / val);
  ui->slider_line_otrans->blockSignals(true);
  ui->slider_line_otrans->setMaximum(moveOrthoRange * val);
  ui->slider_line_otrans->setMinimum(-moveOrthoRange * val);
  ui->slider_line_otrans->blockSignals(false);
  setOTransSliderValue(ui->spin_line_otrans->value());
  ui->spin_line_strans->setSingleStep(1.0 / val);
  ui->slider_line_strans->blockSignals(true);
  int moveEndpointRange = ui->spin_range_etrans->value();
  ui->slider_line_strans->setMaximum(moveEndpointRange * val);
  ui->slider_line_strans->setMinimum(-moveEndpointRange * val);
  ui->slider_line_strans->blockSignals(false);
  setSTransSliderValue(ui->spin_line_strans->value());
  ui->spin_line_etrans->setSingleStep(1.0 / val);
  ui->slider_line_etrans->blockSignals(true);
  ui->slider_line_etrans->setMaximum(moveEndpointRange * val);
  ui->slider_line_etrans->setMinimum(-moveEndpointRange * val);
  ui->slider_line_etrans->blockSignals(false);
  setETransSliderValue(ui->spin_line_etrans->value());
}

void ControlWindow::setRotPrec(int val) {
  int rotRange = ui->spin_range_rot->value();
  ui->spin_line_rot->setSingleStep(1.0 / val);
  ui->slider_line_rot->blockSignals(true);
  ui->slider_line_rot->setMaximum(rotRange * val);
  ui->slider_line_rot->setMinimum(-rotRange * val);
  ui->slider_line_rot->blockSignals(false);
  setRotSliderValue(ui->spin_line_rot->value());
}

void ControlWindow::resetControlOptions() {
  ui->spin_range_rot->blockSignals(true);
  ui->spin_range_rot->setValue(5);
  ui->spin_range_rot->blockSignals(false);
  ui->spin_range_otrans->blockSignals(true);
  ui->spin_range_otrans->setValue(2);
  ui->spin_range_otrans->blockSignals(false);
  ui->spin_range_etrans->blockSignals(true);
  ui->spin_range_etrans->setValue(10);
  ui->spin_range_etrans->blockSignals(false);
  updateRanges();
  ui->spin_prec_rot->setValue(100);
  ui->spin_prec_trans->setValue(100);
}

void ControlWindow::setIndicatorVisible(bool val) {
  lplot->qplot->layer("indicator")->setVisible(val);
  lplot->replot();
}

void ControlWindow::setIndicatorPosition(double pos, double range) {
  if (lineSel < 0) return;
  const std::size_t index{static_cast<std::size_t>(lineSel)};
  if (index >= lines.size()) return;
  setIndicatorPosition(lines[index].modSegment(), pos, range);
}

void ControlWindow::setIndicatorPosition(const LineSegment& l, double pos, double range) {
  if (!indicator) return;
  pos += l.start() > l.end() ? l.end() : l.start();
  cv::Point_<float_type> ppos = l.lineDist(pos, l.origin());
  cv::Point_<float_type> p1 = l.normalLineDist(-range, ppos), p2 = l.normalLineDist(range, ppos);
  indicator->start->setCoords(p1.x, p1.y);
  indicator->end->setCoords(p2.x, p2.y);
  lplot->replot();
}

void ControlWindow::replot() { lplot->replot(); }
