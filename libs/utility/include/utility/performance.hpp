#pragma once

#include <utility/task.hpp>

#include <utility>

namespace lsfm {

struct PerformanceData : public TaskData {
  PerformanceData(const std::string& n, const cv::Mat& s);
  virtual ~PerformanceData() = default;

  cv::Mat src_gray;
};

//! Performance results in millisec
struct PerformanceResult {
  PerformanceResult(double t = 0, double m = 0, double d = 0) : total(t), mean(m), stddev(d) {}
  double total{};
  double mean{};
  double stddev{};
};

//! Performance measurements
struct PerformanceMeasure {
  PerformanceMeasure(const std::string& sn = std::string(),
                     const std::string& fn = std::string(),
                     double w = 0,
                     double h = 0)
      : sourceName(sn), filterName(fn), width(w), height(h) {}

  std::string sourceName{};        // name of source image/file/set
  std::string filterName{};        // name of performed filter/algorithm
  double width{0.0};               // image dimensions
  double height{0.0};              // image dimensions
  std::vector<uint64> measures{};  // measurements

  PerformanceResult computeResult() const { return computeResult(measures); }
  static PerformanceResult computeResult(const std::vector<uint64>& data);
};

typedef std::vector<PerformanceMeasure> PerformanceMeasureVector;

//! Base class for performance task
struct PerformanceTaskBase : public TaskBase {
  PerformanceTaskBase(const std::string& taskName, int f = 0) : TaskBase(taskName, f) {}
  virtual ~PerformanceTaskBase() = default;

  // data of performance task
  PerformanceMeasureVector measure{};

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
  virtual ~PerformanceTaskDefault() = default;

  virtual void run(const TaskData& data, int loops, bool verbose);
  virtual void run(const std::string& /*src_name*/, const cv::Mat& /*src*/, int /*loops*/, bool /*verbose*/) {};
};

//! full performance test based on performance tasks and data providers
struct PerformanceTest : public TaskLoader {
  PerformanceTest(const std::string& testName = std::string()) : TaskLoader(testName) {}
  virtual ~PerformanceTest() = default;

  PerformanceTaskList tasks{};
  DataProviderList data{};
  bool showTotal{false};
  bool showMean{true};
  bool showStdDev{true};
  bool showMegaPixel{true};
  bool visualResults{false};

  virtual void run(int runs = 10, bool verbose = true);

  virtual StringTable resultTable(bool fullReport = false);

 protected:
  // prepare task data and source, e.g. blur etc.
  virtual void prepareSource(cv::Mat& /*src*/) {}
  // prepare task data
  virtual std::unique_ptr<TaskData> prepareTaskData(const std::string& src_name, cv::Mat& src);

 private:
  struct DataProviderTaskMeasure {
    DataProviderTaskMeasure() = default;
    DataProviderTaskMeasure(std::string p, std::string t, PerformanceMeasureVector r)
        : results(std::move(r)), provider(std::move(p)), task(std::move(t)) {}
    PerformanceMeasureVector results{};
    std::string provider{};
    std::string task{};

    PerformanceMeasure accumulatedResult() const {
      return PerformanceTaskBase::accumulatedMeasure(results, provider, task);
    }
  };
  std::vector<DataProviderTaskMeasure> results_{};
  size_t providerRecords_{0};

  void writeMeasure(const PerformanceMeasure& pm, StringTable& StringTable, size_t col, size_t row);
};

typedef std::shared_ptr<PerformanceTest> PerformanceTestPtr;
}  // namespace lsfm
