#include "profileanalyzer.h"

#include "controlwindow.h"
#include "help_button.hpp"
#include "ui_profileanalyzer.h"
#include <imgproc/mean.hpp>

#include <QMessageBox>


using namespace std;

ProfileAnalyzer::ProfileAnalyzer(QWidget* parent)
    : LATool("Profile Analyzer", parent),
      plot(new PlotWindow("Profile Plot", parent)),
      sources(nullptr),
      cw(nullptr),
      ui(new Ui::ProfileAnalyzer) {
  setWindowTitle("Line Profile Analyzer");
  ui->setupUi(this);

  ui->cb_interp->blockSignals(true);
  ui->cb_interp->addItem("nearest");
  ui->cb_interp->addItem("nearest_round");
  ui->cb_interp->addItem("bilinear");
  ui->cb_interp->addItem("bicubic");
  ui->cb_interp->setCurrentIndex(2);
  ui->cb_interp->blockSignals(false);

  plot->qplot->addLayer("std_dev", plot->qplot->layer("main"), QCustomPlot::limAbove);
  plot->qplot->setCurrentLayer("std_dev");
  std_dev_upper = plot->qplot->addGraph();
  std_dev_upper->setPen(Qt::NoPen);
  std_dev_upper->setBrush(QColor(0, 0, 255, 20));
  std_dev_lower = plot->qplot->addGraph();
  std_dev_lower->setPen(Qt::NoPen);
  std_dev_upper->setChannelFillGraph(std_dev_lower);

  plot->qplot->addLayer("profile", plot->qplot->layer("std_dev"), QCustomPlot::limAbove);
  plot->qplot->setCurrentLayer("profile");
  profile_plot = plot->qplot->addGraph();
  profile_plot->setPen(QPen(Qt::blue));


  plot->qplot->addLayer("sprofile", plot->qplot->layer("profile"), QCustomPlot::limAbove);
  plot->qplot->setCurrentLayer("sprofile");
  sprofile_plot = plot->qplot->addGraph();
  sprofile_plot->setPen(QPen(Qt::red, 2));

  pa_indicator = nullptr;

  createX();

  // Tooltips explaining the analyzer and its controls.
  setToolTip(
      tr("Visualize the gradient profile perpendicular to a selected "
         "line segment. The blue curve shows the mean gradient response "
         "and the shaded band shows the standard deviation."));
  ui->cb_data_source->setToolTip(tr("Image source to sample the profile from."));
  ui->cb_interp->setToolTip(tr("Interpolation method for sampling perpendicular to the line."));
  ui->spin_profile_range->setToolTip(
      tr("Half-range in pixels around the line center for "
         "profile evaluation."));
  ui->spin_line_dist->setToolTip(tr("Number of support pixels along the line for averaging."));
  ui->chb_line_dist->setToolTip(
      tr("Interpret Line Distance as discrete sample count "
         "instead of pixel spacing."));
  ui->spin_subdiv->setToolTip(
      tr("Evaluation points per pixel in the profile range. "
         "Higher = finer resolution."));
  ui->fast_interp->setToolTip(tr("Use faster approximate interpolation (may be less accurate)."));
  ui->chb_profile_show->setToolTip(tr("Toggle the mean gradient profile curve (blue)."));
  ui->chb_std_dev->setToolTip(
      tr("Toggle the \xc2\xb1"
         "1\xcf\x83 standard deviation band."));
  ui->chb_single->setToolTip(
      tr("Show a red curve for a single cross-section at "
         "the specified position along the line."));
  ui->spin_profile_pos->setToolTip(
      tr("Position along the line (0 = start, max = end) "
         "for the single-point profile."));
  ui->slider_profile_pos->setToolTip(tr("Drag to move the single-point sample position along the line."));
  ui->chb_profile_fit->setToolTip(tr("Automatically scale the plot axes to fit all visible data."));
  ui->pb_profile_fit->setToolTip(tr("Manually rescale the plot to fit all visible curves."));

  // Help button.
  addHelpButton(this, tr("Help \xe2\x80\x94 Profile Analyzer"),
                tr("<h3>Line Profile Analyzer</h3>"
                   "<p>Visualizes the gradient response profile perpendicular "
                   "to a selected line segment.</p>"
                   "<p><b>Blue curve:</b> Mean gradient response averaged along "
                   "the line length.<br/>"
                   "<b>Shaded band:</b> \xc2\xb1"
                   "1 standard deviation.<br/>"
                   "<b>Red curve:</b> Single-point profile at a specific "
                   "position along the line.</p>"
                   "<h4>Parameters</h4>"
                   "<ul>"
                   "<li><b>Select Source:</b> Gradient image to sample from.</li>"
                   "<li><b>Profile Range:</b> \xc2\xb1"
                   "pixels around the line center.</li>"
                   "<li><b>Line Distance:</b> Support pixels for averaging.</li>"
                   "<li><b>Use Line Distance as Line Samples:</b> Discrete mode.</li>"
                   "<li><b>Subdivisions:</b> Evaluation points per pixel.</li>"
                   "<li><b>Interpolation:</b> Sampling method.</li>"
                   "<li><b>Fast Interpolation:</b> Approximate sampling.</li>"
                   "</ul>"
                   "<h4>Display</h4>"
                   "<ul>"
                   "<li><b>Show Profile:</b> Toggle mean profile (blue).</li>"
                   "<li><b>Show Standard Deviation:</b> Toggle \xc2\xb1"
                   "\xcf\x83 band.</li>"
                   "<li><b>Show single Profile on Line:</b> Single-point "
                   "cross-section (red).</li>"
                   "<li><b>Profile Position:</b> Where along the line to sample.</li>"
                   "<li><b>Auto Fit to Range:</b> Auto-scale plot axes.</li>"
                   "<li><b>Fit Profile:</b> Manual axis rescale.</li>"
                   "</ul>"));
}

ProfileAnalyzer::~ProfileAnalyzer() {
  delete ui;
  delete plot;
}

void ProfileAnalyzer::connectTools(ControlWindow* w) {
  cw = w;
  sources = &w->getSources();
  connect(w, SIGNAL(sourcesChanged(const ImageSources&)), this, SLOT(updateSources(const ImageSources&)));
  connect(w, SIGNAL(lineChanged(const LineSegment&)), this, SLOT(updateLine(const LineSegment&)));
}

void ProfileAnalyzer::updateSources(const ImageSources& src) {
  ui->cb_data_source->blockSignals(true);

  int isi = ui->cb_data_source->currentIndex(), imag = 0;
  QString iss = ui->cb_data_source->currentText();
  ui->cb_data_source->clear();
  int i = 0, idx = 0;
  for_each(src.begin(), src.end(), [&](const ImageSource& s) {
    if (s.data.channels() == 1) {
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

  sources = &src;
}

void ProfileAnalyzer::plotProfile() {
  if (ui->chb_std_dev->isChecked()) {
    plot->qplot->layer("std_dev")->setVisible(true);
    QVector<double> upper, lower;
    upper.resize(profile.size());
    lower.resize(profile.size());
    int end = profile.size();
    p_max = std::numeric_limits<double>::lowest();
    p_min = std::numeric_limits<double>::max();
    for (int i = 0; i != end; ++i) {
      p_max = max(p_max, upper[i] = profile[i] + std_dev[i]);
      p_min = min(p_min, lower[i] = profile[i] - std_dev[i]);
    }
    std_dev_upper->setData(X, upper);
    std_dev_lower->setData(X, lower);
  } else {
    plot->qplot->layer("std_dev")->setVisible(false);
    auto mm = std::minmax_element(profile.begin(), profile.end());
    p_min = *mm.first;
    p_max = *mm.second;
  }

  profile_plot->setData(X, profile);
}

void ProfileAnalyzer::plotSProfile() {
  sprofile_plot->setData(X, single_profile);
  auto mm = std::minmax_element(single_profile.begin(), single_profile.end());
  sp_min = *mm.first;
  sp_max = *mm.second;

  cw->setIndicatorPosition(line, ui->spin_profile_pos->value(), ui->spin_profile_range->value());
}

void ProfileAnalyzer::updateLine(const LineSegment& l) {
  line = l;
  ui->spin_profile_pos->blockSignals(true);
  ui->spin_profile_pos->setMaximum(line.length());
  ui->spin_profile_pos->setValue(line.length() * linePos);
  ui->spin_profile_pos->blockSignals(false);
  if (isVisible()) updatePlot();
}

void ProfileAnalyzer::updateLinePosSlider() {
  linePos = ui->slider_profile_pos->value() / 1000.0;
  ui->spin_profile_pos->blockSignals(true);
  ui->spin_profile_pos->setValue(line.length() * linePos);
  ui->spin_profile_pos->blockSignals(false);
  updateSProfile();
}

void ProfileAnalyzer::updateLinePosSpin() {
  linePos = ui->spin_profile_pos->value() / line.length();
  ui->slider_profile_pos->blockSignals(true);
  ui->slider_profile_pos->setValue(static_cast<int>(linePos * 1000));
  ui->slider_profile_pos->blockSignals(false);
  updateSProfile();
}

void ProfileAnalyzer::updateProfileLayer() {
  plot->qplot->layer("profile")->setVisible(ui->chb_profile_show->isChecked());
  plot->qplot->layer("std_dev")->setVisible(ui->chb_profile_show->isChecked());
  ui->chb_std_dev->setEnabled(ui->chb_profile_show->isChecked());
  updateProfile();
}

void ProfileAnalyzer::updateSProfileLayer() {
  bool en = ui->chb_single->isChecked();
  ui->label_profile_pos->setEnabled(en);
  ui->spin_profile_pos->setEnabled(en);
  ui->slider_profile_pos->setEnabled(en);
  plot->qplot->layer("sprofile")->setVisible(en);
  cw->setIndicatorVisible(en);
  updateSProfile();
}


void ProfileAnalyzer::fitProfile() {
  if (!ui->chb_single->isChecked() && !ui->chb_profile_show->isChecked()) return;

  if (ui->chb_single->isChecked() && ui->chb_profile_show->isChecked())
    plot->setAxis(X.front(), X.back(), min(sp_min, p_min), max(sp_max, p_max));
  else if (ui->chb_single->isChecked())
    plot->setAxis(X.front(), X.back(), sp_min, sp_max);
  else
    plot->setAxis(X.front(), X.back(), p_min, p_max);

  plot->show();
  plot->replot();
}

void ProfileAnalyzer::updatePlot() {
  if (sources == nullptr) return;

  try {
    if (ui->chb_profile_show->isChecked()) {
      createProfile();
      plotProfile();
    }

    if (ui->chb_single->isChecked()) {
      createSProfile();
      plotSProfile();
    }

    if (ui->chb_profile_fit->isChecked())
      fitProfile();
    else {
      plot->show();
      plot->replot();
    }
  } catch (const std::exception& ex) {
    std::cerr << "Profile plot failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Profile Error"), tr("Profile plot failed:\n%1").arg(ex.what()));
  }
}

void ProfileAnalyzer::updateProfile() {
  if (sources == nullptr) return;

  try {
    if (ui->chb_profile_show->isChecked()) {
      createProfile();
      plotProfile();
    }

    if (ui->chb_profile_fit->isChecked())
      fitProfile();
    else {
      plot->show();
      plot->replot();
    }
  } catch (const std::exception& ex) {
    std::cerr << "Profile update failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Profile Error"), tr("Profile update failed:\n%1").arg(ex.what()));
  }
}

void ProfileAnalyzer::updateSProfile() {
  if (sources == nullptr) return;

  try {
    if (ui->chb_single->isChecked()) {
      createSProfile();
      plotSProfile();
    }
    if (ui->chb_profile_fit->isChecked())
      fitProfile();
    else {
      plot->show();
      plot->replot();
    }
  } catch (const std::exception& ex) {
    std::cerr << "Single profile update failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("Profile Error"), tr("Single profile update failed:\n%1").arg(ex.what()));
  }
}

void ProfileAnalyzer::updateX() {
  createX();
  updatePlot();
}


void ProfileAnalyzer::createX() {
  int subdiv = ui->spin_subdiv->value();
  int steps = static_cast<int>(ui->spin_profile_range->value() * subdiv);
  float_type step = 1.0 / subdiv;
  X.clear();
  X.reserve(2 * steps + 1);
  for (int start = -steps; start <= steps; ++start) X.push_back(start * step);
}

template <class MT, class Func>
Func getMean(int idx, bool sampled, bool fast) {
  switch (idx) {
    case 0:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
        return lsfm::MeanSampled<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
      }
      if (fast) return lsfm::FastMean<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
      return lsfm::Mean<float_type, MT, lsfm::NearestInterpolator<float_type, MT> >::process;
    case 1:
      if (sampled) {
        if (fast)
          return lsfm::FastMeanSampled<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
        return lsfm::MeanSampled<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
      }
      if (fast) return lsfm::FastMean<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
      return lsfm::Mean<float_type, MT, lsfm::FastRoundNearestInterpolator<float_type, MT> >::process;
    case 2:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
        return lsfm::MeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
      }
      if (fast) return lsfm::FastMean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
      return lsfm::Mean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
    case 3:
      if (sampled) {
        if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
        return lsfm::MeanSampled<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
      }
      if (fast) return lsfm::FastMean<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
      return lsfm::Mean<float_type, MT, lsfm::CubicInterpolator<float_type, MT> >::process;
  }

  if (sampled) {
    if (fast) return lsfm::FastMeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
    return lsfm::MeanSampled<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
  }
  if (fast) return lsfm::FastMean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
  return lsfm::Mean<float_type, MT, lsfm::LinearInterpolator<float_type, MT> >::process;
}

template <class mat_type>
void create_profile(const cv::Mat_<mat_type>& src,
                    const LineSegment& line,
                    const QVector<double>& X,
                    QVector<double>& profile,
                    QVector<double>& std_dev,
                    float_type param,
                    int idx,
                    bool variance,
                    bool sampled,
                    bool fast) {
  profile.clear();
  profile.reserve(X.size());
  std_dev.clear();
  std_dev.reserve(X.size());

  LineSegment tmp = line;

  if (variance) {
    float_type var;
    for_each(X.begin(), X.end(), [&](double x) {
      tmp.translateOrtho(static_cast<float_type>(x));
      profile.push_back(getMean<mat_type, lsfm::MeanHelper<float_type, cv::Point_>::func_type_variance>(
          idx, sampled, fast)(var, src, tmp, param));
      std_dev.push_back(sqrt(var));
      tmp = line;
    });
  } else {
    for_each(X.begin(), X.end(), [&](float_type x) {
      tmp.translateOrtho(static_cast<float_type>(x));
      profile.push_back(
          getMean<mat_type, lsfm::MeanHelper<float_type, cv::Point_>::func_type>(idx, sampled, fast)(src, tmp, param));
      tmp = line;
    });
  }
}

void ProfileAnalyzer::createProfile() {
  if (sources == nullptr || sources->empty()) return;
  bool variance = ui->chb_std_dev->isChecked();
  bool sampled = ui->chb_line_dist->isChecked();
  float_type param = ui->spin_line_dist->value();
  const int sourceIndex = ui->cb_data_source->currentData().toInt();
  if (sourceIndex < 0) return;
  if (static_cast<std::size_t>(sourceIndex) >= sources->size()) return;

  const cv::Mat& src = sources->at(static_cast<std::size_t>(sourceIndex)).data;
  switch (src.type()) {
    case CV_8S:
      create_profile(reinterpret_cast<const cv::Mat_<char>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_8U:
      create_profile(reinterpret_cast<const cv::Mat_<unsigned char>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_16S:
      create_profile(reinterpret_cast<const cv::Mat_<short>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_16U:
      create_profile(reinterpret_cast<const cv::Mat_<unsigned short>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_32S:
      create_profile(reinterpret_cast<const cv::Mat_<int>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_32F:
      create_profile(reinterpret_cast<const cv::Mat_<float>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
    case CV_64F:
      create_profile(reinterpret_cast<const cv::Mat_<double>&>(src), line, X, profile, std_dev, param,
                     ui->cb_interp->currentIndex(), variance, sampled, ui->fast_interp->isChecked());
      break;
  }
}

template <class MT>
typename lsfm::InterpolationHelper<float_type, cv::Point_>::func_type_point getInterpolator(int idx) {
  switch (idx) {
    case 0:
      return lsfm::NearestInterpolator<float_type, MT>::get;
    case 1:
      return lsfm::FastRoundNearestInterpolator<float_type, MT>::get;
    case 2:
      return lsfm::LinearInterpolator<float_type, MT>::get;
    case 3:
      return lsfm::CubicInterpolator<float_type, MT>::get;
  }
  return lsfm::LinearInterpolator<float_type, MT>::get;
}


template <class mat_type>
void create_sprofile(const cv::Mat_<mat_type>& src,
                     const LineSegment& line,
                     const QVector<double>& X,
                     QVector<double>& sprofile,
                     float_type pos,
                     int idx) {
  typename lsfm::InterpolationHelper<float_type, cv::Point_>::func_type_point interp = getInterpolator<mat_type>(idx);

  sprofile.clear();
  sprofile.reserve(X.size());


  LineSegment tmp = line;
  for_each(X.begin(), X.end(), [&](float_type x) {
    tmp.translateOrtho(x);
    sprofile.push_back(interp(src, tmp.lineDist(pos, tmp.start() > tmp.end() ? tmp.endPoint() : tmp.startPoint())));
    tmp = line;
  });
}

void ProfileAnalyzer::createSProfile() {
  if (sources == nullptr || sources->empty()) return;
  float_type pos = ui->spin_profile_pos->value();
  const int sourceIndex = ui->cb_data_source->currentData().toInt();
  if (sourceIndex < 0) return;
  if (static_cast<std::size_t>(sourceIndex) >= sources->size()) return;

  const cv::Mat& src = sources->at(static_cast<std::size_t>(sourceIndex)).data;
  switch (src.type()) {
    case CV_8S:
      create_sprofile(reinterpret_cast<const cv::Mat_<char>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_8U:
      create_sprofile(reinterpret_cast<const cv::Mat_<unsigned char>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_16S:
      create_sprofile(reinterpret_cast<const cv::Mat_<short>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_16U:
      create_sprofile(reinterpret_cast<const cv::Mat_<unsigned short>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_32S:
      create_sprofile(reinterpret_cast<const cv::Mat_<int>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_32F:
      create_sprofile(reinterpret_cast<const cv::Mat_<float>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
    case CV_64F:
      create_sprofile(reinterpret_cast<const cv::Mat_<double>&>(src), line, X, single_profile, pos,
                      ui->cb_interp->currentIndex());
      break;
  }
}
