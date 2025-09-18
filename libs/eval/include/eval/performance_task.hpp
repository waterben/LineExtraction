#pragma once

#include <eval/data_provider.hpp>
#include <eval/task.hpp>
#include <eval/string_table.hpp>


namespace lsfm {

//! Performance results in millisec
struct PerformanceResult {
  double total{};
  double mean{};
  double stddev{};
};

/// @brief Performance measurements
struct PerformanceMeasure {
  /// Name of source image/file/set
  std::string source_name{};
  /// Name of performed task/filter/algorithm
  std::string task_name;
  // measurements in ns
  std::vector<uint64> measures;

  PerformanceResult computeResult() const { return computeResult(measures); }
  static PerformanceResult computeResult(const std::vector<uint64>& data);
};

//! PerformanceTask
template <typename InputDataT, typename PerformanceMeasureT>
struct PerformanceTask : public MeasureTask<InputDataT> {
  PerformanceTask(const std::string& pt_name, bool pt_verbose) : Task(pt_name, pt_verbose) {}
  virtual ~PerformanceTask() = default;

  using Base = Task<InputDataT>;
  using InputData = InputDataT;
  using Base::name;
  using Base::prepare;
  using Base::run;
  using Base::saveResults;

  using PerformanceMeasure = PerformanceMeasureT;
  using PerformanceMeasureVector = std::vector<PerformanceMeasure>;

  /// Data of performance task
  PerformanceMeasureVector measure;

  /// @brief Receive accumulated performance measure of all runs of this task
  virtual PerformanceMeasure accumulatedMeasure(const std::string& source_name = std::string()) const {
    return accumulatedMeasure(measure, source_name, name);
  }

  /// @brief Receive accumulated performance measure of all runs of this task
  static PerformanceMeasure accumulatedMeasure(const PerformanceMeasureVector& measure,
                                               const std::string source_name = std::string(),
                                               const std::string task_name = std::string()) {
    PerformanceMeasure ret{source_name, task_name};
    if (measure.empty()) return ret;

    for_each(measure.begin(), measure.end(), [&](const PerformanceMeasure& pd) {
      ret.measures.insert(ret.measures.end(), pd.measures.begin(), pd.measures.end());
    });
    return ret;
  }
};

//! full performance test based on performance tasks and data providers
template <typename PerformanceTaskT>
struct PerformanceTest : public TaskLoader<typename PerformanceTaskT::InputData> {
  PerformanceTest(const std::string& pt_name,
                  typename DataProvider<typename PerformanceTaskT::InputData>::Ptr pt_data_provider,
                  bool pt_verbose = false)
      : TaskLoader(test_name, pt_verbose),
        data_provider(pt_data_provider),
        show_total(false),
        show_mean(true),
        show_std_dev(true),
        show_mega_pixel(false) {}
  virtual ~PerformanceTest() {}

  using PerformanceTask = PerformanceTaskT;
  using InputData = typename PerformanceTask::InputData;
  using Base = TaskLoader<InputData>;
  using DataProvider = DataProvider<InputData>;


  DataProvider::Ptr data_provider{};
  bool show_total{};
  bool show_mean{};
  bool show_std_dev{};
  bool show_mega_pixel{};
  bool visual_results{};

  void run(std::size_t loops) override;

  StringTable resultTable(bool fullReport = false) override;

 protected:
  // prepare task data and source, e.g. blur etc.
  virtual void prepareSource(InputData& data) {}


 private:
  struct DataProviderTaskMeasure {
    std::string provider{}, task{};
    PerformanceMeasureVector results{};

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
