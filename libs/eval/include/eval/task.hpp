/// @file task.hpp
/// @brief Base task and task runner interfaces for evaluation framework.
/// Provides abstract base classes for implementing evaluation tasks that can be run in loops.

#pragma once

#include <utility/string_table.hpp>

#include <any>
#include <memory>
#include <string>

namespace lsfm {

/// @brief Generic input data struct for evaluation tasks.
/// Base class for all input data types containing common fields.
/// Extend this for domain-specific input data.
struct GenericInputData {
  GenericInputData() = default;
  /// @brief Construct with a name.
  /// @param n Name of the input data source
  explicit GenericInputData(const std::string& n) : name(n) {}
  virtual ~GenericInputData() = default;

  std::string name{};  ///< Name of data source
};

/// @brief Type-erased task interface for Python binding
///
/// This interface provides a stable ABI that can be easily wrapped
/// with pybind11. All methods use simple types (strings, size_t)
/// instead of templates.
///
/// Usage with pybind11:
/// @code
/// py::class_<ITask, std::shared_ptr<ITask>>(m, "ITask")
///     .def("prepare", &ITask::prepareAny)
///     .def("run", &ITask::run)
///     .def("name", &ITask::taskName);
/// @endcode
struct ITask {
  virtual ~ITask() = default;

  /// @brief Get task name
  virtual const std::string& taskName() const = 0;

  /// @brief Prepare task with type-erased input
  /// @param data Input data wrapped in std::any
  virtual void prepareAny(const std::any& data) = 0;

  /// @brief Run the task
  /// @param loops Number of iterations
  virtual void run(std::size_t loops) = 0;

  /// @brief Check if task is verbose
  virtual bool isVerbose() const = 0;

  /// @brief Reset task state
  virtual void reset() = 0;

  /// @brief Save visual results
  virtual void saveVisualResults(const std::string& target_path) = 0;
};

using ITaskPtr = std::shared_ptr<ITask>;
using ITaskPtrList = std::vector<ITaskPtr>;

/// @brief Base task class with prepare() support
///
/// Design principles:
/// - Implements ITask for Python binding compatibility
/// - Provides prepare() method as per thesis architecture
/// - Uses template method pattern for run loop
struct Task : public ITask {
  Task(const std::string& task_name, bool task_verbose = false) : name(task_name), verbose(task_verbose) {}
  ~Task() override = default;

  using Ptr = std::shared_ptr<Task>;
  using PtrList = std::vector<Ptr>;

  // ITask interface implementation
  const std::string& taskName() const override { return name; }
  bool isVerbose() const override { return verbose; }
  void reset() override {}
  void saveVisualResults(const std::string& /*target_path*/) override {}

  /// @brief Prepare task with type-erased input (for Python)
  void prepareAny(const std::any& data) override {
    try {
      const auto& input = std::any_cast<const GenericInputData&>(data);
      prepare(input);
    } catch (const std::bad_any_cast&) {
      // Subclasses may override to handle their specific types
    }
  }

  /// @brief Prepare task with generic input data
  /// @param data Input data to prepare for processing
  virtual void prepare(const GenericInputData& data) { static_cast<void>(data); }

  /// @brief Run eval task and collect data in measures
  void run(std::size_t loops) override = 0;

  /// Name of task
  std::string name{};
  /// Verbose output
  bool verbose{};
};

/// @brief Task runner that executes a collection of tasks
class TaskRunner {
 public:
  TaskRunner(const std::string& tr_name, bool tr_verbose = false, bool tr_visual_results = false)
      : name(tr_name), verbose{tr_verbose}, visual_results{tr_visual_results} {}
  virtual ~TaskRunner() = default;

  /// Name of task runner
  std::string name{};
  /// Enable verbose output
  bool verbose{};
  /// Enable visual output
  bool visual_results{};

  /// @brief Run tasks
  virtual void run(std::size_t loops) = 0;

  /// @brief Collect result data from tasks
  virtual void collect(const Task& /*task*/) {}

  /// @brief Clears internal results
  virtual void clear() {}

  /// @brief Get results
  virtual StringTable resultTable(bool fullReport = false) {
    static_cast<void>(fullReport);
    return StringTable();
  }

  /// @brief List of tasks to run
  typename Task::PtrList tasks{};
};

}  // namespace lsfm
