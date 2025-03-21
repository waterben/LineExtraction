#pragma once

#include <eval/input_task.hpp>


namespace lsfm {


///@brief Base task class
template <typename InputDataT, typename MeasuresT>
class MeasureTask : public InputTask<InputDataT> {
 public:
  using InputData = InputDataT;
  using Measures = MeasuresT;
  using Result = typename Measures::Result;
  using Base = InputTask<InputDataT>;

  MeasureTask(const std::string& mt_name, bool mt_verbose = false) : Base(mt_name, mt_verbose) {}
  virtual ~MeasureTask() {}

  using Base::prepare;
  using Base::run;
  using Base::saveVisualResults;

  void clear() override { measures_.clear(); }

  using Task::name;
  using Task::verbose;

  /// Access measures
  const Measures& measures() const { return measures_; }

  /// Access result via measures
  Result result() const { return measures_.computeResult(); }

 private:
  /// Task measures from run
  Measures measures_{};
};


/// @brief Task loader
template <typename MeasureTaskT>
class MeasureTaskLoader : public InputTaskLoader<MeasureTaskT> {
 public:
  using MeasureTask = MeasureTaskT;
  using InputData = typename MeasureTask::InputData;
  using Measures = typename MeasureTask::Measures;
  using Result = typename MeasureTask::Result;
  using Base = InputTaskLoader<MeasureTask>;


  MeasureTaskLoader(DataProviderPtr,
                    const std::string& tl_name,
                    const std::string& tl_target_path = std::string(),
                    bool tl_verbose = false,
                    bool tl_visual_results = false)
      : Base(tl_data_provider, tl_name, tl_target_path, tl_verbose, tl_visual_results) {}
  virtual ~MeasureTaskLoader() = default;

  using Base::run;
  using Base::visual_results;

  using Base::data_provider;
  using Base::name;
  using Base::target_path;
  using Base::tasks;
  using Base::verbose;

  void collect(const Task& task) override {
    const auto& mtask = static_cast<const MeasureTask&>(task);
    mtask;
  }

 protected:
  struct DataProviderTaskMeasure {
    std::string provider{};
    std::string task{};
    Measures measures{};
  };
  std::vector<DataProviderTaskMeasure> results_;
};


}  // namespace lsfm
