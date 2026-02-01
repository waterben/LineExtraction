/// @file spe.cpp
/// @brief Sub-Pixel Estimation performance tests
#include "performance_test.hpp"
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <imgproc/derivative_gradient.hpp>


using namespace lsfm;


template <class FT>
using GradFT = DerivativeGradient<uchar, FT, FT, FT>;

template <class FT>
using NmsFT = NonMaximaSuppression<FT, FT, FT, FastNMS8<FT, FT, FT>>;


// =============================================================================
// SPE Performance Task Base
// =============================================================================

/// @brief Base class for SPE performance tasks
template <class FT>
class Entry : public CVPerformanceTaskBase {
 protected:
  GradFT<FT> grad_{};
  NmsFT<FT> nms_{};

  Entry(const std::string& n)
      : CVPerformanceTaskBase(n + std::string(sizeof(FT) == 4 ? " 32F" : " 64F")), grad_(), nms_() {}

  void prepareImpl(const cv::Mat& src) override {
    nms_ = NmsFT<FT>(0.004, 0.012, 2);
    grad_.process(src);
    nms_.process(grad_);
  }
};


// =============================================================================
// Nearest Neighbor Estimator
// =============================================================================

template <class FT>
class EntryNearest : public Entry<FT> {
  PixelEstimator<FT, cv::Point> pe_{};

 public:
  EntryNearest() : Entry<FT>("Nearest"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point> points;
    points.reserve(idxs.size());
    pe_.convert(idxs, points, this->grad_.magnitude(), this->nms_.directionMap());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntryNearest>(); }
};


// =============================================================================
// Linear Estimator
// =============================================================================

template <class FT>
class EntrySpeLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeLin() : Entry<FT>("SpeLin"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convert(idxs, points, this->grad_.magnitude(), this->nms_.directionMap());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeLin>(); }
};


// =============================================================================
// Quadratic Estimator
// =============================================================================

template <class FT>
class EntrySpeQuad : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>>
      pe_{};

 public:
  EntrySpeQuad() : Entry<FT>("SpeQuad"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convert(idxs, points, this->grad_.magnitude(), this->nms_.directionMap());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeQuad>(); }
};


// =============================================================================
// Center of Gravity Estimator
// =============================================================================

template <class FT>
class EntrySpeCog : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeCog() : Entry<FT>("SpeCog"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convert(idxs, points, this->grad_.magnitude(), this->nms_.directionMap());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeCog>(); }
};


// =============================================================================
// Sobel Estimator
// =============================================================================

template <class FT>
class EntrySpeSobel : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeSobel() : Entry<FT>("SpeSobel"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convert(idxs, points, this->grad_.magnitude(), this->nms_.directionMap());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeSobel>(); }
};


// =============================================================================
// Linear Estimator with Direction (Linear Interpolation)
// =============================================================================

template <class FT>
class EntrySpeLinDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeLinDirIpLin() : Entry<FT>("SpeLinDir IpLinear"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeLinDirIpLin>(); }
};


// =============================================================================
// Quadratic Estimator with Direction (Linear Interpolation)
// =============================================================================

template <class FT>
class EntrySpeQuadDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>>
      pe_{};

 public:
  EntrySpeQuadDirIpLin() : Entry<FT>("SpeQuadDir IpLinear"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeQuadDirIpLin>(); }
};


// =============================================================================
// Center of Gravity Estimator with Direction (Linear Interpolation)
// =============================================================================

template <class FT>
class EntrySpeCogDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeCogDirIpLin() : Entry<FT>("SpeCogDir IpLinear"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeCogDirIpLin>(); }
};


// =============================================================================
// Sobel Estimator with Direction (Linear Interpolation)
// =============================================================================

template <class FT>
class EntrySpeSobelDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>> pe_{};

 public:
  EntrySpeSobelDirIpLin() : Entry<FT>("SpeSobelDir IpLinear"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeSobelDirIpLin>(); }
};


// =============================================================================
// Linear Estimator with Direction (Cubic Interpolation)
// =============================================================================

template <class FT>
class EntrySpeLinDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, CubicInterpolator>> pe_{};

 public:
  EntrySpeLinDirIpCubic() : Entry<FT>("SpeLinDir IpCubic"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeLinDirIpCubic>(); }
};


// =============================================================================
// Quadratic Estimator with Direction (Cubic Interpolation)
// =============================================================================

template <class FT>
class EntrySpeQuadDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, CubicInterpolator>>
      pe_{};

 public:
  EntrySpeQuadDirIpCubic() : Entry<FT>("SpeQuadDir IpCubic"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeQuadDirIpCubic>(); }
};


// =============================================================================
// Center of Gravity Estimator with Direction (Cubic Interpolation)
// =============================================================================

template <class FT>
class EntrySpeCogDirIpCubic : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, CubicInterpolator>> pe_{};

 public:
  EntrySpeCogDirIpCubic() : Entry<FT>("SpeCogDir IpCubic"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeCogDirIpCubic>(); }
};


// =============================================================================
// Sobel Estimator with Direction (Cubic Interpolation)
// =============================================================================

template <class FT>
class EntrySpeSobelDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, CubicInterpolator>> pe_{};

 public:
  EntrySpeSobelDirIpCubic() : Entry<FT>("SpeSobelDir IpCubic"), pe_() {}

 protected:
  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    IndexVector idxs = this->nms_.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    pe_.convertDir(idxs, points, this->grad_.magnitude(), this->grad_.direction());
  }

 public:
  static CVPerformanceTaskPtr create() { return std::make_shared<EntrySpeSobelDirIpCubic>(); }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create SPE performance test with all tasks
template <class FT>
CVPerformanceTestPtr createSPEPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "SPE" + std::string(sizeof(FT) == 4 ? " 32F" : " 64F"));

  test->tasks.push_back(EntryNearest<FT>::create());
  test->tasks.push_back(EntrySpeLin<FT>::create());
  test->tasks.push_back(EntrySpeQuad<FT>::create());
  test->tasks.push_back(EntrySpeCog<FT>::create());
  test->tasks.push_back(EntrySpeSobel<FT>::create());

  test->tasks.push_back(EntrySpeLinDirIpLin<FT>::create());
  test->tasks.push_back(EntrySpeQuadDirIpLin<FT>::create());
  test->tasks.push_back(EntrySpeCogDirIpLin<FT>::create());
  test->tasks.push_back(EntrySpeSobelDirIpLin<FT>::create());

  test->tasks.push_back(EntrySpeLinDirIpCubic<FT>::create());
  test->tasks.push_back(EntrySpeQuadDirIpCubic<FT>::create());
  test->tasks.push_back(EntrySpeCogDirIpCubic<FT>::create());
  test->tasks.push_back(EntrySpeSobelDirIpCubic<FT>::create());

  return test;
}

bool addSPE() {
  addPerformanceTestCreator(createSPEPerformanceTest<float>);
  addPerformanceTestCreator(createSPEPerformanceTest<double>);
  std::cout << "Added SPE performance tests" << std::endl;
  return true;
}

// bool SPEAdded = addSPE();
