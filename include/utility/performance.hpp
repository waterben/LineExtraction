#ifndef _PERFORMANCE_HPP_
#define _PERFORMANCE_HPP_
#ifdef __cplusplus

#  include <utility/task.hpp>

namespace lsfm {

struct PerformanceData : public TaskData {
  PerformanceData(const std::string& n, const cv::Mat& s);
  virtual ~PerformanceData() {}

  cv::Mat src_gray;
};

//! Performance results in millisec
struct PerformanceResult {
  PerformanceResult(double t = 0, double m = 0, double d = 0) : total(t), mean(m), stddev(d) {}
  double total;
  double mean;
  double stddev;
};

//! Performance measurements
struct PerformanceMeasure {
  PerformanceMeasure(const std::string& sn = std::string(),
                     const std::string& fn = std::string(),
                     double w = 0,
                     double h = 0)
      : sourceName(sn), filterName(fn), width(w), height(h) {}

  std::string sourceName;        // name of source image/file/set
  std::string filterName;        // name of performed filter/algorithm
  double width, height;          // image dimensions
  std::vector<uint64> measures;  // measurements

  PerformanceResult computeResult() const { return computeResult(measures); }
  static PerformanceResult computeResult(const std::vector<uint64>& data);
};

typedef std::vector<PerformanceMeasure> PerformanceMeasureVector;

//! Base class for performance task
struct PerformanceTaskBase : public TaskBase {
  PerformanceTaskBase(const std::string& taskName, int f = 0) : TaskBase(taskName, f) {}
  virtual ~PerformanceTaskBase() {}

  // data of performance task
  PerformanceMeasureVector measure;

  PerformanceMeasure accumulatedMeasure(const std::string& sourceName = std::string()) const {
    return accumulatedMeasure(measure, sourceName, name);
  }

  //! receive accumulated performance measure of all runs of this task
  static PerformanceMeasure accumulatedMeasure(const PerformanceMeasureVector& measure,
                                               const std::string sourceName = std::string(),
                                               const std::string filterName = std::string());
};

typedef std::shared_ptr<PerformanceTaskBase> PerformanceTaskPtr;
typedef std::vector<PerformanceTaskPtr> PerformanceTaskList;

//! Base class for performance task
struct PerformanceTaskDefault : public PerformanceTaskBase {
  PerformanceTaskDefault(const std::string& taskName, int f = 0) : PerformanceTaskBase(taskName, f) {}
  virtual ~PerformanceTaskDefault() {}

  virtual void run(const TaskData& data, int loops, bool verbose);
  virtual void run(const std::string& src_name, cv::Mat src, int loops, bool verbose){};
};

//! full performance test based on performance tasks and data providers
struct PerformanceTest : public TaskLoader {
  PerformanceTest(const std::string& testName = std::string())
      : TaskLoader(testName), showTotal(false), showMean(true), showStdDev(true), showMegaPixel(true) {}
  virtual ~PerformanceTest() {}

  PerformanceTaskList tasks;
  DataProviderList data;
  bool showTotal, showMean, showStdDev, showMegaPixel, visualResults;

  virtual void run(int runs = 10, bool verbose = true);

  virtual StringTable resultTable(bool fullReport = false);

 protected:
  // prepare task data and source, e.g. blur etc.
  virtual void prepareSource(cv::Mat& src) {}
  // prepare task data
  virtual std::unique_ptr<TaskData> prepareTaskData(const std::string& src_name, cv::Mat& src);

 private:
  struct DataProviderTaskMeasure {
    DataProviderTaskMeasure() {}
    DataProviderTaskMeasure(const std::string& p, const std::string& t, const PerformanceMeasureVector& r)
        : results(r), provider(p), task(t) {}
    PerformanceMeasureVector results;
    std::string provider, task;

    PerformanceMeasure accumulatedResult() const {
      return PerformanceTaskBase::accumulatedMeasure(results, provider, task);
    }
  };
  std::vector<DataProviderTaskMeasure> results_;
  size_t providerRecords_;

  void writeMeasure(const PerformanceMeasure& pm, StringTable& StringTable, size_t col, size_t row);
};

typedef std::shared_ptr<PerformanceTest> PerformanceTestPtr;
}  // namespace lsfm

#endif
#endif
