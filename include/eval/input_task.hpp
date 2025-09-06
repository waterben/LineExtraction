#pragma once

#include <eval/data_provider.hpp>
#include <eval/task.hpp>


namespace lsfm {


///@brief Base task class
template <typename InputDataT>
class InputTask : public Task {
 public:
  InputTask(const std::string& task_name, bool task_verbose = false) : Task(task_name, task_verbose) {}
  virtual ~InputTask() {}

  using InputData = InputDataT;

  void prepare(const GenericInputData& data) final {
    const InputData& input_data = static_cast<const InputData&>(data);
    prepare(input_data);
  }

  using Task::clear;
  using Task::run;
  using Task::saveVisualResults;

  using Task::name;
  using Task::verbose;

 protected:
  virtual void prepare(const InputData& source) = 0;
};


/// @brief Task loader
template <typename InputTaskT>
class InputTaskLoader : public TaskLoader {
 public:
  using InputTask = InputTaskT;
  using InputData = typename InputTask::InputData;
  using DataProvider = DataProvider<InputData>;
  using DataProviderPtrList = typename DataProvider::PtrList;


  InputTaskLoader(DataProviderPtrList tl_data_provider,
                  const std::string& tl_name,
                  const std::string& tl_target_path = std::string(),
                  bool tl_verbose = false,
                  bool tl_visual_results = false)
      : TaskLoader(tl_name, tl_verbose, tl_visual_results),
        data_provider(std::move(tl_data_provider)),
        target_path(tl_target_path) {}
  virtual ~InputTaskLoader() = default;

  using TaskLoader::name;
  using TaskLoader::tasks;
  using TaskLoader::verbose;
  using TaskLoader::visual_results;

  DataProviderPtrList data_provider{};

  std::string target_path{};


  /// @brief Run tasks
  void run(std::size_t loops) override {
    std::cout << "Starting task sequence: " << name << std::endl;
    uint64 start = cv::getTickCount();

    for (auto& dp : data_provider) {
      if (!dp) {
        throw std::runtime_error("Empty/invalid data provider detected.");
      }
      // Rewind data provider to start
      dp->rewind();
      // Prepare input data for all tasks
      InputData data;
      while (dp->get(data)) {
        prepare(data);
        for (auto& task : tasks) {
          // Clear measures of task
          task->clear();
          // Prepare task by providing new data input
          task->prepare(data);
          for (int i = 0; i < loop; ++i) {
            // Run task
            task->run(loops);
            // Collect results if necessary
            collect(*task);
          }
        }
      }
    }

    std::cout << "Task sequence  " << name
              << " done: " << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s"
              << std::endl;
  }

 protected:
  /// @brief Option to prepare data input in general for all tasks, e.g. blur etc.
  virtual void prepareInputData(InputData& data) {}
};


}  // namespace lsfm
