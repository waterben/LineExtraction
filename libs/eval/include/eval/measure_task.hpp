#pragma once

#include <eval/input_task.hpp>
#include <eval/measure.hpp>

#include <map>

namespace lsfm {

/// @brief Task that collects measurements during execution
///
/// Template parameters:
/// - InputDataT: Type of input data (must inherit from GenericInputData)
/// - MeasuresT: Type of measure to collect (must inherit from Measure)
template <typename InputDataT, typename MeasuresT>
class MeasureTask : public InputTask<InputDataT> {
 public:
  using InputData = InputDataT;
  using Measures = MeasuresT;
  using Result = typename Measures::Result;
  using Base = InputTask<InputDataT>;
  using MeasuresMap = std::map<std::string, Measures>;

  MeasureTask(const std::string& mt_name, bool mt_verbose = false) : Base(mt_name, mt_verbose) {}
  ~MeasureTask() override = default;

  using Base::prepare;
  using Base::run;
  using Base::saveVisualResults;

  using Task::name;
  using Task::verbose;

  void reset() override { measures_.clear(); }

  /// @brief Measure and record data after run (public for TaskRunner access)
  virtual void measure() = 0;

  /// @brief Access measures
  const MeasuresMap& measures() const { return measures_; }

 protected:
  /// Task measures from run by source name
  MeasuresMap measures_{};
};

/// @brief Task runner that collects measurements from measure tasks
template <typename MeasureTaskT>
class MeasureTaskRunner : public InputTaskRunner<MeasureTaskT> {
 public:
  using MeasureTaskType = MeasureTaskT;
  using InputData = typename MeasureTaskType::InputData;
  using Measures = typename MeasureTaskType::Measures;
  using Result = typename MeasureTaskType::Result;
  using Base = InputTaskRunner<MeasureTaskType>;
  using DataProviderPtrList = typename Base::DataProviderPtrList;

  MeasureTaskRunner(DataProviderPtrList tr_data_provider,
                    const std::string& tr_name,
                    const std::string& tr_target_path = std::string(),
                    bool tr_verbose = false,
                    bool tr_visual_results = false)
      : Base(std::move(tr_data_provider), tr_name, tr_target_path, tr_verbose, tr_visual_results) {}
  ~MeasureTaskRunner() override = default;

  using Base::data_provider;
  using Base::name;
  using Base::run;
  using Base::target_path;
  using Base::tasks;
  using Base::verbose;
  using Base::visual_results;

  /// @brief Run tasks - overridden to call measure() after each task run
  void run(std::size_t loops) override {
    std::cout << "Starting task sequence: " << this->name << std::endl;
    int64 start = cv::getTickCount();

    // Reset all tasks before starting
    for (auto& task : this->tasks) {
      task->reset();
    }

    for (auto& dp : this->data_provider) {
      if (!dp) {
        throw std::runtime_error("Empty/invalid data provider detected.");
      }
      // Rewind data provider to start
      dp->rewind();
      // Prepare input data for all tasks
      InputData data;
      while (dp->get(data)) {
        this->prepareInputData(data);
        for (auto& task : this->tasks) {
          // Prepare task by providing new data input
          task->prepare(data);
          // Run task the specified number of loops
          task->run(loops);
          // Record measurement after run
          task->measure();
          // Collect results if necessary
          this->collect(*task);
        }
      }
    }

    std::cout << "Task sequence " << this->name
              << " done: " << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s"
              << std::endl;
  }

  void collect(const Task& task) override {
    const auto& mtask = static_cast<const MeasureTaskType&>(task);
    for (const auto& [source_name, measure] : mtask.measures()) {
      results_.push_back({source_name, mtask.name, measure});
    }
  }

  void clear() override { results_.clear(); }

 protected:
  struct DataProviderTaskMeasure {
    std::string provider{};
    std::string task{};
    Measures measures{};
  };
  std::vector<DataProviderTaskMeasure> results_;
};

}  // namespace lsfm
