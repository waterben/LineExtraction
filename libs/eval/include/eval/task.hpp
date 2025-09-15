#pragma once

#include <utility/string_table.hpp>

#include <memory>
#include <string>


namespace lsfm {

/// @brief Generic input data struct
struct GenericInputData {
  /// Name of source
  std::string name{};
};

///@brief Base task class
struct Task {
  Task(const std::string& task_name, bool task_verbose = false) : name(task_name), verbose(task_verbose) {}
  virtual ~Task() {}

  using Ptr = std::shared_ptr<Task>;
  using PtrList = std::vector<Ptr>;


  /// @brief Prepare task by providing data input to process/copy required data for run (override has to downcast
  /// source)
  virtual void prepare(const GenericInputData& data) = 0;

  /// @brief Run eval task and collect data in measures
  virtual void run(std::size_t loops) = 0;

  /// @brief Save visual results, if any are available
  virtual void saveVisualResults(const std::string&) {}

  /// @brief Clears all, also collected data
  virtual void reset() {}

  /// Name of task
  const std::string name{};
  /// Verbose output
  const bool verbose{};
};


/// @brief Task loader
class TaskLoader {
 public:
  TaskLoader(const std::string& tl_name, bool tl_verbose = false, bool tl_visual_results = false)
      : name(tl_name), verbose{tl_verbose}, visual_results{tl_visual_results} {}
  virtual ~TaskLoader() = default;

  /// Name of task loader
  const std::string name{};
  /// Enable verbose output
  const bool verbose{};
  /// Enable visual output
  const bool visual_results{};

  /// @brief Run tasks
  virtual void run(std::size_t loops) = 0;

  /// @brief Collect result data from tasks
  virtual void collect(const Task&) {}

  /// @brief Clears internal results
  virtual void clear() {}

  /// @brief Get results
  virtual StringTable resultTable(bool fullReport = false) { return StringTable(); };

  /// @brief List of tasks to run
  typename Task::PtrList tasks{};
};


}  // namespace lsfm
