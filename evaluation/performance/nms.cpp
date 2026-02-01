/// @file nms.cpp
/// @brief Non-Maximum Suppression and Zero-Crossing performance tests
#include "performance_test.hpp"
#include <edge/nms.hpp>
#include <edge/zc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/laplace.hpp>


using namespace lsfm;


// =============================================================================
// NMS Performance Tasks
// =============================================================================

/// @brief NMS4 Fast (4-connected) performance task
class EntryNMS4Fast : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel_;
  NonMaximaSuppression<short, int, float, FastNMS4<short, int, float>> nms_;

 public:
  EntryNMS4Fast() : CVPerformanceTaskBase("NMS4 Fast") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.gx(), sobel_.gy(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()));
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Fast (8-connected) performance task
class EntryNMSFast : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel_;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms_;

 public:
  EntryNMSFast() : CVPerformanceTaskBase("NMS Fast") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.gx(), sobel_.gy(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()));
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Fast with direction input performance task
class EntryNMSFastDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel_;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms_;

 public:
  EntryNMSFastDir() : CVPerformanceTaskBase("NMS Fast Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
    sobel_.direction();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.direction(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()), sobel_.directionRange().lower,
                 sobel_.directionRange().upper);
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Precise Linear interpolation performance task
class EntryNMSPreciseLinear : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel_;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false>> nms_;

 public:
  EntryNMSPreciseLinear() : CVPerformanceTaskBase("NMS Precise Linear") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.gx(), sobel_.gy(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()));
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Precise Linear with direction input performance task
class EntryNMSPreciseLinearDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel_;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, LinearInterpolator, PolarCV>>
      nms_;

 public:
  EntryNMSPreciseLinearDir() : CVPerformanceTaskBase("NMS Precise Linear Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
    sobel_.direction();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.direction(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()), sobel_.directionRange().lower,
                 sobel_.directionRange().upper);
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Precise Cubic interpolation performance task
class EntryNMSPreciseCubic : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel_;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, CubicInterpolator>> nms_;

 public:
  EntryNMSPreciseCubic() : CVPerformanceTaskBase("NMS Precise Cubic") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.gx(), sobel_.gy(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()));
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief NMS Precise Cubic with direction input performance task
class EntryNMSPreciseCubicDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel_;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, CubicInterpolator, PolarCV>>
      nms_;

 public:
  EntryNMSPreciseCubicDir() : CVPerformanceTaskBase("NMS Precise Cubic Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.magnitude();
    sobel_.direction();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    nms_.process(sobel_.direction(), sobel_.magnitude(), sobel_.magnitudeThreshold(nms_.thresholdLow()),
                 sobel_.magnitudeThreshold(nms_.thresholdHigh()), sobel_.directionRange().lower,
                 sobel_.directionRange().upper);
    cv::Mat tmp = nms_.hysteresis();
    static_cast<void>(tmp);
  }
};


// =============================================================================
// Zero-Crossing Performance Tasks
// =============================================================================

/// @brief Zero-crossing without direction performance task
class EntryZCNoDir : public CVPerformanceTaskBase {
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, FastZC<short, short, float>> zc_;

 public:
  EntryZCNoDir() : CVPerformanceTaskBase("ZC No Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override { laplace_.process(src); }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()));
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing fast performance task
class EntryZCFast : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, FastZC<short, short, float>> zc_;

 public:
  EntryZCFast() : CVPerformanceTaskBase("ZC Fast") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.gx(), sobel_.gy(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()));
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing fast with direction performance task
class EntryZCFastDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, FastZC<short, short, float, NCC_BASIC, EZCMap8, PolarCV>> zc_;

 public:
  EntryZCFastDir() : CVPerformanceTaskBase("ZC Fast Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.direction();
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.direction(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()), sobel_.directionRange().lower,
                sobel_.directionRange().upper);
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing precise linear interpolation performance task
class EntryZCPreciseLinear : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, LinearInterpolator>> zc_;

 public:
  EntryZCPreciseLinear() : CVPerformanceTaskBase("ZC Precise Linear") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.gx(), sobel_.gy(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()));
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing precise linear with direction performance task
class EntryZCPreciseLinearDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, LinearInterpolator, PolarCV>>
      zc_;

 public:
  EntryZCPreciseLinearDir() : CVPerformanceTaskBase("ZC Precise Linear Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.direction();
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.direction(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()), sobel_.directionRange().lower,
                sobel_.directionRange().upper);
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing precise cubic interpolation performance task
class EntryZCPreciseCubic : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, CubicInterpolator>> zc_;

 public:
  EntryZCPreciseCubic() : CVPerformanceTaskBase("ZC Precise Cubic") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.gx(), sobel_.gy(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()));
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


/// @brief Zero-crossing precise cubic with direction performance task
class EntryZCPreciseCubicDir : public CVPerformanceTaskBase {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel_;
  LaplaceSimple<uchar, short> laplace_;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, CubicInterpolator, PolarCV>> zc_;

 public:
  EntryZCPreciseCubicDir() : CVPerformanceTaskBase("ZC Precise Cubic Dir") {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    sobel_.process(src);
    sobel_.direction();
    laplace_.process(src);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    zc_.process(sobel_.direction(), laplace_.laplace(), laplace_.laplaceThreshold(zc_.thresholdLow()),
                laplace_.laplaceThreshold(zc_.thresholdHigh()), sobel_.directionRange().lower,
                sobel_.directionRange().upper);
    cv::Mat tmp = zc_.hysteresis();
    static_cast<void>(tmp);
  }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create NMS performance test with all tasks
CVPerformanceTestPtr createNMSPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "NMS");

  test->tasks.push_back(std::make_shared<EntryNMS4Fast>());
  test->tasks.push_back(std::make_shared<EntryNMSFast>());
  test->tasks.push_back(std::make_shared<EntryNMSFastDir>());
  test->tasks.push_back(std::make_shared<EntryNMSPreciseLinear>());
  test->tasks.push_back(std::make_shared<EntryNMSPreciseLinearDir>());
  test->tasks.push_back(std::make_shared<EntryNMSPreciseCubic>());
  test->tasks.push_back(std::make_shared<EntryNMSPreciseCubicDir>());
  test->tasks.push_back(std::make_shared<EntryZCNoDir>());
  test->tasks.push_back(std::make_shared<EntryZCFast>());
  test->tasks.push_back(std::make_shared<EntryZCFastDir>());
  test->tasks.push_back(std::make_shared<EntryZCPreciseLinear>());
  test->tasks.push_back(std::make_shared<EntryZCPreciseLinearDir>());
  test->tasks.push_back(std::make_shared<EntryZCPreciseCubic>());
  test->tasks.push_back(std::make_shared<EntryZCPreciseCubicDir>());

  return test;
}

bool addNMS() {
  addPerformanceTestCreator(createNMSPerformanceTest);
  std::cout << "Added NMS performance test" << std::endl;
  return true;
}

// bool NMSAdded = addNMS();
