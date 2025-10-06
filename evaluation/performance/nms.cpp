#include "performance_test.hpp"
#include <edge/nms.hpp>
#include <edge/zc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/laplace.hpp>


using namespace lsfm;


class EntryNMS4Fast : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS4<short, int, float>> nms;

 public:
  EntryNMS4Fast() : PerformanceTaskDefault("NMS4 Fast") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.gx(), sobel.gy(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()));
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMS4Fast); }
};

class EntryNMSFast : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms;

 public:
  EntryNMSFast() : PerformanceTaskDefault("NMS Fast") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.gx(), sobel.gy(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()));
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSFast); }
};

class EntryNMSFastDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms;

 public:
  EntryNMSFastDir() : PerformanceTaskDefault("NMS Fast Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    sobel.direction();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.direction(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()), sobel.directionRange().lower,
                  sobel.directionRange().upper);
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSFastDir); }
};

class EntryNMSPreciseLinear : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false>> nms;

 public:
  EntryNMSPreciseLinear() : PerformanceTaskDefault("NMS Precise Linear") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.gx(), sobel.gy(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()));
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSPreciseLinear); }
};

class EntryNMSPreciseLinearDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, LinearInterpolator, PolarCV>>
      nms;

 public:
  EntryNMSPreciseLinearDir() : PerformanceTaskDefault("NMS Precise Linear Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    sobel.direction();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.direction(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()), sobel.directionRange().lower,
                  sobel.directionRange().upper);
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSPreciseLinearDir); }
};

class EntryNMSPreciseCubic : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, CubicInterpolator>> nms;

 public:
  EntryNMSPreciseCubic() : PerformanceTaskDefault("NMS Precise Cubic") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.gx(), sobel.gy(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()));
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSPreciseCubic); }
};

class EntryNMSPreciseCubicDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel;
  NonMaximaSuppression<short, float, float, PreciseNMS<short, float, false, float, EMap8, CubicInterpolator, PolarCV>>
      nms;

 public:
  EntryNMSPreciseCubicDir() : PerformanceTaskDefault("NMS Precise Cubic Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.magnitude();
    sobel.direction();
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      nms.process(sobel.direction(), sobel.magnitude(), sobel.magnitudeThreshold(nms.thresholdLow()),
                  sobel.magnitudeThreshold(nms.thresholdHigh()), sobel.directionRange().lower,
                  sobel.directionRange().upper);
      cv::Mat tmp = nms.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNMSPreciseCubicDir); }
};

class EntryZCNoDir : public PerformanceTaskDefault {
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, FastZC<short, short, float>> zc;

 public:
  EntryZCNoDir() : PerformanceTaskDefault("ZC No Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()));
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCNoDir); }
};

class EntryZCFast : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, FastZC<short, short, float>> zc;

 public:
  EntryZCFast() : PerformanceTaskDefault("ZC Fast") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.gx(), sobel.gy(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()));
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCFast); }
};

class EntryZCFastDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, FastZC<short, short, float, NCC_BASIC, EZCMap8, PolarCV>> zc;

 public:
  EntryZCFastDir() : PerformanceTaskDefault("ZC Fast Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.direction();
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.direction(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()), sobel.directionRange().lower,
                 sobel.directionRange().upper);
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCFastDir); }
};

class EntryZCPreciseLinear : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, LinearInterpolator>> zc;

 public:
  EntryZCPreciseLinear() : PerformanceTaskDefault("ZC Precise Linear") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.gx(), sobel.gy(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()));
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCPreciseLinear); }
};

class EntryZCPreciseLinearDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, LinearInterpolator, PolarCV>> zc;

 public:
  EntryZCPreciseLinearDir() : PerformanceTaskDefault("ZC Precise Linear Dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.direction();
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.direction(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()), sobel.directionRange().lower,
                 sobel.directionRange().upper);
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCPreciseLinearDir); }
};

class EntryZCPreciseCubic : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, CubicInterpolator>> zc;

 public:
  EntryZCPreciseCubic() : PerformanceTaskDefault("ZC Precise Cubic") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.gx(), sobel.gy(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()));
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCPreciseCubic); }
};

class EntryZCPreciseCubicDir : public PerformanceTaskDefault {
  DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, FastDirection> sobel;
  LaplaceSimple<uchar, short> laplace;
  ZeroCrossing<short, short, float, PreciseZC<short, short, float, NCC_BASIC, EZCMap8, CubicInterpolator, PolarCV>> zc;

 public:
  EntryZCPreciseCubicDir() : PerformanceTaskDefault("ZC Precise Cubic dir") {}
  virtual void run(const std::string& src_name, const cv::Mat& src, int runs, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    sobel.process(src);
    sobel.direction();
    laplace.process(src);
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      zc.process(sobel.direction(), laplace.laplace(), laplace.laplaceThreshold(zc.thresholdLow()),
                 laplace.laplaceThreshold(zc.thresholdHigh()), sobel.directionRange().lower,
                 sobel.directionRange().upper);
      cv::Mat tmp = zc.hysteresis();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryZCPreciseCubicDir); }
};

PerformanceTestPtr createNMSPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<PerformanceTest>();
  test->name = "NMS";
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

  test->tasks.push_back(EntryNMS4Fast::create());
  test->tasks.push_back(EntryNMSFast::create());
  test->tasks.push_back(EntryNMSFastDir::create());
  test->tasks.push_back(EntryNMSPreciseLinear::create());
  test->tasks.push_back(EntryNMSPreciseLinearDir::create());
  test->tasks.push_back(EntryNMSPreciseCubic::create());
  test->tasks.push_back(EntryNMSPreciseCubicDir::create());
  test->tasks.push_back(EntryZCNoDir::create());
  test->tasks.push_back(EntryZCFast::create());
  test->tasks.push_back(EntryZCFastDir::create());
  test->tasks.push_back(EntryZCPreciseLinear::create());
  test->tasks.push_back(EntryZCPreciseLinearDir::create());
  test->tasks.push_back(EntryZCPreciseCubic::create());
  test->tasks.push_back(EntryZCPreciseCubicDir::create());
  return test;
}

bool addNMS() {
  addPerformanceTestCreator(createNMSPerformanceTest);
  std::cout << "Added NMS performance test" << std::endl;
  return true;
}

// bool NMSAdded = addNMS();
