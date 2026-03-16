#include "nfa_filter.h"

#include "help_button.hpp"
#include <edge/nfa.hpp>
#include <opencv2/imgproc.hpp>

#include <QMessageBox>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

using namespace lsfm;

// ---------------------------------------------------------------------------
// NFA variant indices (must match combo box order)
// ---------------------------------------------------------------------------

namespace {
constexpr int VARIANT_CONTRAST = 0;
constexpr int VARIANT_BINOM = 1;
constexpr int VARIANT_BINOM2 = 2;

constexpr int MODE_THRESHOLD = 0;
constexpr int MODE_TOP_N = 1;
}  // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

NfaFilter::NfaFilter(QWidget* parent)
    : LATool("NFA Filter", parent),
      ui(new Ui::NfaFilter),
      ctrl(nullptr),
      lines(nullptr),
      sources(nullptr),
      srcImg(nullptr) {
  ui->setupUi(this);

  // Populate combo boxes
  ui->cb_variant->addItem("Contrast (Magnitude)");
  ui->cb_variant->addItem("Binomial (Orientation)");
  ui->cb_variant->addItem("Binomial 2 (Orientation)");

  ui->cb_mode->addItem(QString::fromUtf8("Threshold (\xce\xb5)"));
  ui->cb_mode->addItem("Top-N");

  // Initial widget state: threshold mode active, top-N disabled
  ui->label_top_n->setEnabled(false);
  ui->spin_top_n->setEnabled(false);

  // Initial widget state: contrast selected, binomial params disabled
  onVariantChanged(VARIANT_CONTRAST);

  // Connections
  connect(ui->cb_mode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &NfaFilter::onModeChanged);
  connect(ui->cb_variant, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &NfaFilter::onVariantChanged);
  connect(ui->pb_run, &QPushButton::clicked, this, &NfaFilter::run);

  // --- Panel tooltip ---
  setToolTip(
      tr("Filter line segments by their Number of False Alarms (NFA). "
         "Segments below the statistical significance threshold are "
         "removed, or only the top-N most meaningful segments are kept."));

  // --- Variant tooltips ---
  ui->cb_variant->setToolTip(
      tr("NFA computation method:\n"
         "• Contrast — validates minimum gradient magnitude along the "
         "segment against a random model.\n"
         "• Binomial — tests whether the number of aligned gradient "
         "orientations exceeds random chance (binomial test).\n"
         "• Binomial 2 — like Binomial but with a different number-of-"
         "tests normalization (N = (W×H)²)."));

  // --- Mode tooltips ---
  ui->cb_mode->setToolTip(
      tr("Filter strategy:\n"
         "• Threshold — remove segments whose log₁₀ NFA is below ε.\n"
         "• Top-N — keep the N segments with the highest log₁₀ NFA "
         "(most meaningful)."));
  ui->spin_epsilon->setToolTip(
      tr("Log-epsilon threshold: segments with log₁₀(NFA) < this value "
         "are removed. Higher values are stricter.\n"
         "  0 → keep segments with ≤ 1 expected false alarm\n"
         "  1 → keep segments with ≤ 0.1 expected false alarms\n"
         " -1 → keep segments with ≤ 10 expected false alarms"));
  ui->spin_top_n->setToolTip(
      tr("Number of line segments to keep. Segments are ranked by "
         "log₁₀(NFA) — higher values indicate more meaningful lines."));

  // --- Binomial parameter tooltips ---
  ui->spin_precision->setToolTip(
      tr("Angle tolerance τ (degrees): a pixel's gradient direction is "
         "considered aligned with the line normal if the angular "
         "difference is less than τ. Default: 22.5° (π/8)."));
  ui->spin_p->setToolTip(
      tr("Alignment probability under the null hypothesis. For an "
         "angle tolerance of τ, p = τ / 180. Default: 0.125 (1/8)."));

  // --- Button tooltip ---
  ui->pb_run->setToolTip(tr("Compute NFA for all line segments and apply the selected filter."));

  // Help button
  addHelpButton(this, "extensions/nfa_filter/README.md");
}

NfaFilter::~NfaFilter() { delete ui; }

// ---------------------------------------------------------------------------
// Tool wiring
// ---------------------------------------------------------------------------

void NfaFilter::connectTools(Analyzer* w) {
  ctrl = w;
  lines = &w->getLines();
  srcImg = &w->getSrcImg();
  sources = &w->getSources();
  connect(this, SIGNAL(linesProcessed(const LineSegmentVector&)), w, SLOT(setLines(const LineSegmentVector&)));
}

// ---------------------------------------------------------------------------
// UI state toggles
// ---------------------------------------------------------------------------

void NfaFilter::onModeChanged(int index) {
  const bool threshold = (index == MODE_THRESHOLD);
  ui->label_epsilon->setEnabled(threshold);
  ui->spin_epsilon->setEnabled(threshold);
  ui->label_top_n->setEnabled(!threshold);
  ui->spin_top_n->setEnabled(!threshold);
}

void NfaFilter::onVariantChanged(int index) {
  const bool binom = (index != VARIANT_CONTRAST);
  ui->label_params_header->setEnabled(binom);
  ui->label_precision->setEnabled(binom);
  ui->spin_precision->setEnabled(binom);
  ui->label_p->setEnabled(binom);
  ui->spin_p->setEnabled(binom);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

/// @brief Find gradient magnitude source from image sources.
cv::Mat find_magnitude(const ImageSources& sources) {
  cv::Mat qmag, nmag, mag;
  for (const auto& s : sources) {
    if (s.name == "qmag")
      qmag = s.data;
    else if (s.name == "nmag")
      nmag = s.data;
    else if (s.name == "mag")
      mag = s.data;
  }
  if (!qmag.empty()) return qmag;
  if (!nmag.empty()) return nmag;
  return mag;
}

/// @brief Find gradient component sources (gx, gy) from image sources.
std::pair<cv::Mat, cv::Mat> find_gradient(const ImageSources& sources) {
  cv::Mat gx, gy;
  for (const auto& s : sources) {
    if (s.name == "gx")
      gx = s.data;
    else if (s.name == "gy")
      gy = s.data;
  }
  return {gx, gy};
}

/// @brief Compute gradient magnitude from source image via Sobel.
cv::Mat compute_magnitude(const cv::Mat& src) {
  cv::Mat gray;
  if (src.channels() > 1)
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  else
    gray = src;

  cv::Mat gx, gy, mag;
  cv::Sobel(gray, gx, CV_64F, 1, 0);
  cv::Sobel(gray, gy, CV_64F, 0, 1);
  cv::magnitude(gx, gy, mag);
  return mag;
}

/// @brief Compute gradient components from source image via Sobel.
std::pair<cv::Mat, cv::Mat> compute_gradient(const cv::Mat& src) {
  cv::Mat gray;
  if (src.channels() > 1)
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  else
    gray = src;

  cv::Mat gx, gy;
  cv::Sobel(gray, gx, CV_64F, 1, 0);
  cv::Sobel(gray, gy, CV_64F, 0, 1);
  return {gx, gy};
}

/// @brief Compute contrast-based NFA for a single line segment.
///
/// Rasterizes the segment via cv::LineIterator, finds the minimum magnitude
/// along the path, and computes log_nfa using the precomputed histogram.
/// @param seg Line segment to evaluate
/// @param mag Gradient magnitude image
/// @param nl Number of possible alignments (Nl)
/// @return -log10(NFA) value
double nfa_contrast(const ::LineSegment& seg, const cv::Mat& mag, int nl) {
  auto ps = seg.startPoint();
  auto pe = seg.endPoint();
  cv::Point p1(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y)));
  cv::Point p2(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y)));

  // Clamp to image bounds
  p1.x = std::max(0, std::min(p1.x, mag.cols - 1));
  p1.y = std::max(0, std::min(p1.y, mag.rows - 1));
  p2.x = std::max(0, std::min(p2.x, mag.cols - 1));
  p2.y = std::max(0, std::min(p2.y, mag.rows - 1));

  cv::LineIterator it(mag, p1, p2);
  if (it.count < 2) return -1;

  // Find minimum magnitude along the segment
  double min_val = std::numeric_limits<double>::max();
  for (int i = 0; i < it.count; ++i, ++it) {
    double v = 0;
    switch (mag.depth()) {
      case CV_8U:
        v = mag.at<uchar>(it.pos());
        break;
      case CV_16S:
        v = mag.at<short>(it.pos());
        break;
      case CV_32S:
        v = mag.at<int>(it.pos());
        break;
      case CV_32F:
        v = mag.at<float>(it.pos());
        break;
      case CV_64F:
        v = mag.at<double>(it.pos());
        break;
      default:
        v = mag.at<double>(it.pos());
        break;
    }
    if (v < min_val) min_val = v;
  }

  // Compute H(u): fraction of non-zero pixels with magnitude >= min_val
  int total = 0;
  int count_ge = 0;
  for (int r = 0; r < mag.rows; ++r) {
    for (int c = 0; c < mag.cols; ++c) {
      double v = 0;
      switch (mag.depth()) {
        case CV_8U:
          v = mag.at<uchar>(r, c);
          break;
        case CV_16S:
          v = mag.at<short>(r, c);
          break;
        case CV_32S:
          v = mag.at<int>(r, c);
          break;
        case CV_32F:
          v = mag.at<float>(r, c);
          break;
        case CV_64F:
          v = mag.at<double>(r, c);
          break;
        default:
          v = mag.at<double>(r, c);
          break;
      }
      if (v > 0) ++total;
      if (v >= min_val) ++count_ge;
    }
  }

  if (total == 0) return -1;
  double hu = static_cast<double>(count_ge) / total;

  // log_nfa = -log10(Nl * Hu^l)
  size_t l = static_cast<size_t>(it.count);
  return NfaContrast<double, double, cv::Point>::log_nfa(l, hu, nl);
}

/// @brief Compute binomial-based NFA for a single line segment.
///
/// Rasterizes the segment via cv::LineIterator, counts pixels whose
/// gradient direction is aligned with the line normal, and computes
/// the binomial NFA.
/// @param seg Line segment to evaluate
/// @param gx Gradient x-component image
/// @param gy Gradient y-component image
/// @param tau Angle tolerance in degrees
/// @param prob Alignment probability
/// @param log_nt Log10 of the number of tests
/// @return -log10(NFA) value
double nfa_binomial(
    const ::LineSegment& seg, const cv::Mat& gx, const cv::Mat& gy, double tau, double prob, double log_nt) {
  auto ps = seg.startPoint();
  auto pe = seg.endPoint();
  cv::Point p1(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y)));
  cv::Point p2(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y)));

  // Clamp to image bounds
  p1.x = std::max(0, std::min(p1.x, gx.cols - 1));
  p1.y = std::max(0, std::min(p1.y, gx.rows - 1));
  p2.x = std::max(0, std::min(p2.x, gx.cols - 1));
  p2.y = std::max(0, std::min(p2.y, gx.rows - 1));

  cv::LineIterator it(gx, p1, p2);
  int n = it.count;
  if (n < 2) return -1;

  // Line gradient angle (degrees, 0-360)
  double line_angle = seg.gradientAnglef();

  // Count aligned pixels
  int k = 0;
  for (int i = 0; i < n; ++i, ++it) {
    cv::Point pos = it.pos();
    double vx = 0, vy = 0;
    switch (gx.depth()) {
      case CV_16S:
        vx = gx.at<short>(pos);
        vy = gy.at<short>(pos);
        break;
      case CV_32F:
        vx = gx.at<float>(pos);
        vy = gy.at<float>(pos);
        break;
      case CV_64F:
        vx = gx.at<double>(pos);
        vy = gy.at<double>(pos);
        break;
      default:
        vx = gx.at<double>(pos);
        vy = gy.at<double>(pos);
        break;
    }

    double pixel_angle = cv::fastAtan2(static_cast<float>(vy), static_cast<float>(vx));
    double diff = std::abs(line_angle - pixel_angle);
    if (diff > 360) diff -= 360.0 * static_cast<int>(diff / 360);
    if (diff > 180) diff = 360.0 - diff;
    if (diff < tau) ++k;
  }

  return NfaBinom<short, double, cv::Point>::log_nfaNT(n, k, prob, log_nt);
}

}  // namespace

// ---------------------------------------------------------------------------
// Slot
// ---------------------------------------------------------------------------

void NfaFilter::run() {
  if (lines == nullptr || lines->empty()) return;

  try {
    const int variant = ui->cb_variant->currentIndex();
    const int mode = ui->cb_mode->currentIndex();
    const double log_eps = ui->spin_epsilon->value();
    const int top_n = ui->spin_top_n->value();
    const double tau = ui->spin_precision->value();
    const double p = ui->spin_p->value();

    // Collect current line segments
    ::LineSegmentVector segs;
    segs.reserve(lines->size());
    for (const auto& l : *lines) segs.push_back(l.modSegment());

    // Compute NFA values
    std::vector<double> nfa_values(segs.size(), -1);

    if (variant == VARIANT_CONTRAST) {
      // --- Contrast NFA ---
      cv::Mat mag = find_magnitude(*sources);
      if (mag.empty() && srcImg != nullptr && !srcImg->empty()) mag = compute_magnitude(*srcImg);
      if (mag.empty()) {
        QMessageBox::warning(this, tr("NFA Filter"),
                             tr("No gradient magnitude available. Run a detector first or load a source image."));
        return;
      }

      // Compute Nl: sum of l*(l-1)/2 for each segment
      int nl = 0;
      for (const auto& seg : segs) {
        auto ps = seg.startPoint();
        auto pe = seg.endPoint();
        cv::Point p1(static_cast<int>(std::round(ps.x)), static_cast<int>(std::round(ps.y)));
        cv::Point p2(static_cast<int>(std::round(pe.x)), static_cast<int>(std::round(pe.y)));
        p1.x = std::max(0, std::min(p1.x, mag.cols - 1));
        p1.y = std::max(0, std::min(p1.y, mag.rows - 1));
        p2.x = std::max(0, std::min(p2.x, mag.cols - 1));
        p2.y = std::max(0, std::min(p2.y, mag.rows - 1));
        cv::LineIterator it(mag, p1, p2);
        int len = it.count;
        nl += (len * (len - 1)) / 2;
      }
      if (nl == 0) nl = 1;

      for (size_t i = 0; i < segs.size(); ++i) {
        nfa_values[i] = nfa_contrast(segs[i], mag, nl);
      }

    } else {
      // --- Binomial NFA (variant 1 and 2) ---
      auto [gx, gy] = find_gradient(*sources);
      if (gx.empty() || gy.empty()) {
        if (srcImg != nullptr && !srcImg->empty()) {
          std::tie(gx, gy) = compute_gradient(*srcImg);
        }
      }
      if (gx.empty() || gy.empty()) {
        QMessageBox::warning(this, tr("NFA Filter"),
                             tr("No gradient data (gx, gy) available. Run a detector first or load a source image."));
        return;
      }

      double log_nt;
      if (variant == VARIANT_BINOM) {
        // NfaBinom: logNT = 2 * (log10(W) + log10(H))
        log_nt = 2.0 * (std::log10(static_cast<double>(gx.cols)) + std::log10(static_cast<double>(gx.rows)));
      } else {
        // NfaBinom2: N = (W*H)^2, logNT = log10(N)
        double wh = static_cast<double>(gx.cols) * static_cast<double>(gx.rows);
        log_nt = std::log10(wh * wh);
      }

      for (size_t i = 0; i < segs.size(); ++i) {
        nfa_values[i] = nfa_binomial(segs[i], gx, gy, tau, p, log_nt);
      }
    }

    // Compute statistics
    double nfa_min = *std::min_element(nfa_values.begin(), nfa_values.end());
    double nfa_max = *std::max_element(nfa_values.begin(), nfa_values.end());
    double nfa_mean =
        std::accumulate(nfa_values.begin(), nfa_values.end(), 0.0) / static_cast<double>(nfa_values.size());

    // Apply filter
    ::LineSegmentVector result;
    if (mode == MODE_THRESHOLD) {
      for (size_t i = 0; i < segs.size(); ++i) {
        if (nfa_values[i] >= log_eps) result.push_back(segs[i]);
      }
    } else {
      // Top-N: sort indices by NFA descending, keep top N
      std::vector<size_t> indices(segs.size());
      std::iota(indices.begin(), indices.end(), 0);
      std::sort(indices.begin(), indices.end(),
                [&nfa_values](size_t a, size_t b) { return nfa_values[a] > nfa_values[b]; });

      size_t keep = std::min(static_cast<size_t>(top_n), indices.size());
      result.reserve(keep);
      for (size_t i = 0; i < keep; ++i) {
        result.push_back(segs[indices[i]]);
      }
    }

    // Display statistics
    ui->label_stats->setText(tr("NFA: min=%1  max=%2  mean=%3\n%4 → %5 segments")
                                 .arg(nfa_min, 0, 'f', 2)
                                 .arg(nfa_max, 0, 'f', 2)
                                 .arg(nfa_mean, 0, 'f', 2)
                                 .arg(segs.size())
                                 .arg(result.size()));

    std::cout << "NFA filter (" << ui->cb_variant->currentText().toStdString() << ", "
              << ui->cb_mode->currentText().toStdString() << "): " << segs.size() << " -> " << result.size()
              << " segments" << std::endl;

    emit linesProcessed(result);

  } catch (const std::exception& ex) {
    std::cerr << "NFA filter failed: " << ex.what() << std::endl;
    QMessageBox::warning(this, tr("NFA Filter Error"), tr("NFA filtering failed:\n%1").arg(ex.what()));
  }
}
