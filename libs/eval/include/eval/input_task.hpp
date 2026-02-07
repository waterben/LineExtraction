//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file input_task.hpp
/// @brief Input data handling task interface.
/// Provides interface for tasks that process input data.

#pragma once

#include <eval/data_provider.hpp>
#include <eval/task.hpp>
#include <opencv2/opencv.hpp>

#include <any>
#include <iostream>

namespace lsfm {

/// @brief Input task that processes typed input data
///
/// Bridges the type-erased ITask interface with strongly-typed input data.
/// The prepareAny() method handles type conversion from std::any.
template <typename InputDataT>
class InputTask : public Task {
 public:
  InputTask(const std::string& task_name, bool task_verbose = false) : Task(task_name, task_verbose) {}
  ~InputTask() override = default;

  using InputData = InputDataT;

  using Task::name;
  using Task::prepare;
  using Task::reset;
  using Task::run;
  using Task::saveVisualResults;
  using Task::verbose;

  /// @brief Prepare task with type-erased input (ITask interface)
  void prepareAny(const std::any& data) override {
    try {
      const auto& input = std::any_cast<const InputData&>(data);
      prepare(input);
    } catch (const std::bad_any_cast&) {
      // Try base class type
      Task::prepareAny(data);
    }
  }

  /// @brief Prepare task with generic input data (Task interface)
  void prepare(const GenericInputData& data) override {
    // Try to cast to our specific type if possible
    if (const auto* specific = dynamic_cast<const InputData*>(&data)) {
      prepare(*specific);
    }
  }

  /// @brief Prepare task for given typed input data
  virtual void prepare(const InputData& source) = 0;
};

/// @brief Task runner with data providers for input tasks
template <typename InputTaskT>
class InputTaskRunner : public TaskRunner {
 public:
  using InputTaskType = InputTaskT;
  using InputData = typename InputTaskType::InputData;
  using DataProviderType = DataProvider<InputData>;
  using DataProviderPtrList = typename DataProviderType::PtrList;
  using InputTaskPtr = std::shared_ptr<InputTaskType>;
  using InputTaskPtrList = std::vector<InputTaskPtr>;

  InputTaskRunner(DataProviderPtrList tr_data_provider,
                  const std::string& tr_name,
                  const std::string& tr_target_path = std::string(),
                  bool tr_verbose = false,
                  bool tr_visual_results = false)
      : TaskRunner(tr_name, tr_verbose, tr_visual_results),
        data_provider(std::move(tr_data_provider)),
        target_path(tr_target_path) {}
  ~InputTaskRunner() override = default;

  using TaskRunner::name;
  using TaskRunner::verbose;
  using TaskRunner::visual_results;

  DataProviderPtrList data_provider{};
  InputTaskPtrList input_tasks{};  ///< Typed tasks (shadows base class tasks)

  std::string target_path{};

  /// @brief Run tasks
  void run(std::size_t loops) override {
    std::cout << "Starting task sequence: " << name << std::endl;
    int64 start = cv::getTickCount();

    // Reset all tasks before starting
    for (auto& task : input_tasks) {
      task->reset();
    }

    for (auto& dp : data_provider) {
      if (!dp) {
        throw std::runtime_error("Empty/invalid data provider detected.");
      }
      // Rewind data provider to start
      dp->rewind();
      // Prepare input data for all tasks
      InputData data;
      while (dp->get(data)) {
        prepareInputData(data);
        for (auto& task : input_tasks) {
          // Prepare task by providing new data input
          task->prepare(data);
          // Run task the specified number of loops
          task->run(loops);
          // Collect results if necessary
          collect(*task);
        }
      }
    }

    std::cout << "Task sequence " << name
              << " done: " << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s"
              << std::endl;
  }

 protected:
  /// @brief Option to prepare data input in general for all tasks, e.g. blur etc.
  virtual void prepareInputData(InputData& data) { static_cast<void>(data); }
};

}  // namespace lsfm
