#include "performance_test.hpp"
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <imgproc/derivative_gradient.hpp>


using namespace lsfm;

template <class FT>
using GradFT = DerivativeGradient<uchar, FT, FT, FT>;

template <class FT>
using NmsFT = NonMaximaSuppression<FT, FT, FT, FastNMS8<FT, FT, FT>>;

template <class FT>
struct SpePerformaceData : public TaskData {
  SpePerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), nms(0.004, 0.012, 2) {
    if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    grad.process(src);
    nms.process(grad);
  }

  GradFT<FT> grad;
  NmsFT<FT> nms;
};

template <class FT>
struct SpePerformanceTest : public LegacyPerformanceTest {
  SpePerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
  virtual ~SpePerformanceTest() {}

 protected:
  // prepare task data
  std::unique_ptr<TaskData> prepareTaskData(const std::string& src_name, cv::Mat& src) override {
    prepareSource(src);
    return std::make_unique<SpePerformaceData<FT>>(src_name, src);
  }
};

template <class FT>
class Entry : public PerformanceTaskBase {
 protected:
  Entry(const std::string& n) : PerformanceTaskBase(n + std::string(sizeof(FT) == 4 ? " 32F" : " 64F")) {}

 public:
  virtual ~Entry() {}
  virtual void run(const TaskData& data, int runs, bool verbose) {
    const SpePerformaceData<FT>& pdata = dynamic_cast<const SpePerformaceData<FT>&>(data);
    process(pdata.name, pdata.src, pdata.grad, pdata.nms, runs, verbose);
  }
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) = 0;
};


template <class FT>
class EntryNearest : public Entry<FT> {
  PixelEstimator<FT, cv::Point> pe;

 public:
  EntryNearest() : Entry<FT>("Nearest") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convert(idxs, points, mag, n.directionMap());
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntryNearest); }
};

template <class FT>
class EntrySpeLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeLin() : Entry<FT>("SpeLin") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convert(idxs, points, mag, n.directionMap());
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeLin); }
};

template <class FT>
class EntrySpeQuad : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeQuad() : Entry<FT>("SpeQuad") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convert(idxs, points, mag, n.directionMap());
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeQuad); }
};

template <class FT>
class EntrySpeCog : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeCog() : Entry<FT>("SpeCog") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convert(idxs, points, mag, n.directionMap());
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeCog); }
};

template <class FT>
class EntrySpeSobel : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeSobel() : Entry<FT>("SpeSobel") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convert(idxs, points, mag, n.directionMap());
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeSobel); }
};

template <class FT>
class EntrySpeLinDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeLinDirIpLin() : Entry<FT>("SpeLinDir IpLinear") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeLinDirIpLin); }
};

template <class FT>
class EntrySpeQuadDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeQuadDirIpLin() : Entry<FT>("SpeQuadDir IpLinear") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeQuadDirIpLin); }
};

template <class FT>
class EntrySpeCogDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeCogDirIpLin() : Entry<FT>("SpeCogDir IpLinear") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeCogDirIpLin); }
};

template <class FT>
class EntrySpeSobelDirIpLin : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>> pe;

 public:
  EntrySpeSobelDirIpLin() : Entry<FT>("SpeSobelDir IpLinear") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeSobelDirIpLin); }
};

template <class FT>
class EntrySpeLinDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, CubicInterpolator>> pe;

 public:
  EntrySpeLinDirIpCubic() : Entry<FT>("SpeLinDir IpCubic") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeLinDirIpCubic); }
};

template <class FT>
class EntrySpeQuadDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, CubicInterpolator>> pe;

 public:
  EntrySpeQuadDirIpCubic() : Entry<FT>("SpeQuadDir IpCubic") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeQuadDirIpCubic); }
};

template <class FT>
class EntrySpeCogDirIpCubic : public Entry<FT> {
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, CubicInterpolator>> pe;

 public:
  EntrySpeCogDirIpCubic() : Entry<FT>("SpeCogDir IpCubic") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeCogDirIpCubic); }
};

template <class FT>
class EntrySpeSobelDirIpCubic : public Entry<FT> {
  PixelEstimator<float, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, CubicInterpolator>> pe;

 public:
  EntrySpeSobelDirIpCubic() : Entry<FT>("SpeSobelDir IpCubic") {}
  virtual void process(const std::string& src_name,
                       const cv::Mat& src,
                       const GradFT<FT>& g,
                       const NmsFT<FT>& n,
                       int runs,
                       bool verbose) {
    this->perf_measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->perf_measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    IndexVector idxs = n.hysteresis_edgels();
    std::vector<cv::Point_<FT>> points;
    points.reserve(idxs.size());
    cv::Mat mag = g.magnitude();
    cv::Mat dir = g.direction();
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      pe.convertDir(idxs, points, mag, dir);
      pm.durations.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  static PerformanceTaskPtr create() { return PerformanceTaskPtr(new EntrySpeSobelDirIpCubic); }
};

template <class FT>
PerformanceTestPtr createSPEPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<SpePerformanceTest<FT>>();
  test->name = "SPE" + std::string(sizeof(FT) == 4 ? " 32F" : " 64F");
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }


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
