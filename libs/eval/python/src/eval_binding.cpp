/// @file eval_binding.cpp
/// @brief pybind11 bindings for the evaluation framework.
///
/// Binds the eval library classes: tasks, data providers, performance
/// measures, and the CVPerformanceTest orchestrator.

#include "eval_binding.hpp"

#include <cvnp/cvnp.h>
#include <eval/cv_data_provider.hpp>
#include <eval/cv_measure.hpp>
#include <eval/cv_performance_task.hpp>
#include <eval/performance_measure.hpp>
#include <eval/performance_task.hpp>
#include <eval/results.hpp>
#include <eval/task.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <utility/string_table.hpp>

#include <sstream>
#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Core types
// ============================================================================

void bind_eval_core_types(py::module_& m) {
  // --- StringTable ---
  py::class_<StringTable>(m, "StringTable",
                          "2D string table for evaluation results.\n\n"
                          "Supports row/column access, CSV export, and transposition.")
      .def(py::init<>(), "Construct an empty table.")
      .def(py::init<size_t, size_t>(), py::arg("rows"), py::arg("cols") = 1, "Construct table with given dimensions.")
      .def_property_readonly("rows", &StringTable::rows, "Number of rows.")
      .def_property_readonly("cols", &StringTable::cols, "Number of columns.")
      .def_property_readonly("size", &StringTable::size, "Total number of cells.")
      .def(
          "__getitem__",
          [](const StringTable& t, py::tuple pos) {
            size_t r = pos[0].cast<size_t>();
            size_t c = pos[1].cast<size_t>();
            return t(r, c);
          },
          "Get cell value at (row, col).")
      .def(
          "__setitem__",
          [](StringTable& t, py::tuple pos, const std::string& val) {
            size_t r = pos[0].cast<size_t>();
            size_t c = pos[1].cast<size_t>();
            t(r, c) = val;
          },
          "Set cell value at (row, col).")
      .def(
          "row", [](const StringTable& t, size_t r) { return t.row(r); }, py::arg("row"),
          "Get a row as list of strings.")
      .def(
          "col", [](const StringTable& t, size_t c) { return t.col(c); }, py::arg("col"),
          "Get a column as list of strings.")
      .def("transpose", &StringTable::transpose, "Return a transposed copy of this table.")
      .def("save_csv", &StringTable::saveCSV, py::arg("filename"), "Save table to CSV file.")
      .def(
          "to_list",
          [](const StringTable& t) {
            py::list rows;
            for (size_t r = 0; r < t.rows(); ++r) {
              rows.append(t.row(r));
            }
            return rows;
          },
          "Convert table to list of lists.")
      .def("__repr__", [](const StringTable& t) {
        std::ostringstream os;
        os << t;
        return os.str();
      });

  // --- GenericInputData ---
  py::class_<GenericInputData>(m, "GenericInputData",
                               "Base input data type for evaluation tasks.\n\n"
                               "Carries a name string identifying the data source.")
      .def(py::init<>(), "Construct empty input data.")
      .def(py::init<const std::string&>(), py::arg("name"), "Construct with name.")
      .def_readwrite("name", &GenericInputData::name, "Name of the data source.");

  // --- PerformanceResult ---
  py::class_<PerformanceResult>(m, "PerformanceResult",
                                "Timing results from a performance benchmark.\n\n"
                                "Contains total, mean, and stddev in milliseconds.")
      .def(py::init<>())
      .def_readwrite("total", &PerformanceResult::total, "Total time in milliseconds.")
      .def_readwrite("mean", &PerformanceResult::mean, "Mean time per iteration in milliseconds.")
      .def_readwrite("stddev", &PerformanceResult::stddev, "Standard deviation in milliseconds.")
      .def("__repr__", [](const PerformanceResult& r) {
        std::ostringstream os;
        os << "PerformanceResult(total=" << r.total << "ms, mean=" << r.mean << "ms, stddev=" << r.stddev << "ms)";
        return os.str();
      });

  // --- Task constants ---
  m.attr("TASK_SQR") = TASK_SQR;
  m.attr("TASK_RGB") = TASK_RGB;
  m.attr("TASK_NO_3") = TASK_NO_3;
  m.attr("TASK_NO_5") = TASK_NO_5;
}

// ============================================================================
// ITask / Task
// ============================================================================

/// @brief Trampoline for subclassing Task from Python.
///
/// Task has a pure-virtual run() method, so a trampoline is required to
/// allow Python subclasses to provide their own implementation.
class PyTask : public Task {
 public:
  using Task::Task;

  void run(std::size_t loops) override { PYBIND11_OVERRIDE_PURE(void, Task, run, loops); }

  void prepare(const GenericInputData& data) override { PYBIND11_OVERRIDE(void, Task, prepare, data); }

  void reset() override { PYBIND11_OVERRIDE(void, Task, reset); }

  void saveVisualResults(const std::string& target_path) override {
    PYBIND11_OVERRIDE(void, Task, saveVisualResults, target_path);
  }
};

/// @brief Trampoline for subclassing CVPerformanceTaskBase from Python.
class PyCVPerformanceTask : public CVPerformanceTaskBase {
 public:
  using CVPerformanceTaskBase::CVPerformanceTaskBase;

  void prepareImpl(const cv::Mat& src) override { PYBIND11_OVERRIDE(void, CVPerformanceTaskBase, prepareImpl, src); }

  void runImpl(const std::string& src_name, const cv::Mat& src) override {
    PYBIND11_OVERRIDE(void, CVPerformanceTaskBase, runImpl, src_name, src);
  }
};

void bind_eval_task(py::module_& m) {
  // --- ITask interface ---
  py::class_<ITask, std::shared_ptr<ITask>>(m, "ITask",
                                            "Type-erased task interface for evaluation.\n\n"
                                            "Designed for cross-language (C++/Python) benchmarking.")
      .def("task_name", &ITask::taskName, "Get task name.")
      .def("run", &ITask::run, py::arg("loops"), "Run the task for the given number of loops.")
      .def("is_verbose", &ITask::isVerbose, "Check if verbose output is enabled.")
      .def("reset", &ITask::reset, "Reset task state.")
      .def("save_visual_results", &ITask::saveVisualResults, py::arg("target_path"),
           "Save visual results to the target path.");

  // --- Task base ---
  py::class_<Task, ITask, PyTask, std::shared_ptr<Task>>(m, "Task",
                                                         "Base task with name and verbose flag.\n\n"
                                                         "Subclass and override run() to create custom tasks.\n\n"
                                                         "Example::\n\n"
                                                         "    class MyTask(le_eval.Task):\n"
                                                         "        def run(self, loops):\n"
                                                         "            for _ in range(loops):\n"
                                                         "                pass  # measured work\n")
      .def(py::init<const std::string&, bool>(), py::arg("name"), py::arg("verbose") = false,
           "Construct a task with a name and optional verbose flag.")
      .def_readwrite("name", &Task::name, "Task name.")
      .def_readwrite("verbose", &Task::verbose, "Verbose output flag.");
}

// ============================================================================
// Data providers
// ============================================================================

void bind_eval_data_provider(py::module_& m) {
  // --- CVData ---
  py::class_<CVData>(m, "CVData", "Image data container with name and source matrix.")
      .def(py::init<>())
      .def_readwrite("name", &CVData::name, "Image name.")
      .def_readwrite("src", &CVData::src, "Source image (cv::Mat).");

  // --- FileCVDataProvider ---
  py::class_<FileCVDataProvider, std::shared_ptr<FileCVDataProvider>>(
      m, "FileCVDataProvider",
      "File-system data provider for raw images.\n\n"
      "Scans directories for image files and provides them sequentially.")
      .def(py::init<const std::string&>(), py::arg("name"), "Construct with provider name.")
      .def(py::init<const std::filesystem::path&, const std::string&, bool>(), py::arg("path"), py::arg("name"),
           py::arg("recursive") = true, "Construct from directory path.")
      .def("parse",
           static_cast<void (FileCVDataProvider::*)(const std::filesystem::path&, bool)>(&FileCVDataProvider::parse),
           py::arg("folder"), py::arg("recursive") = true, "Scan directory for image files.")
      .def("rewind", &FileCVDataProvider::rewind, "Reset to start of dataset.")
      .def("clear", &FileCVDataProvider::clear, "Clear all loaded file paths.")
      .def_readonly("name", &FileCVDataProvider::name, "Provider name.");
}

// ============================================================================
// CV performance data providers
// ============================================================================

void bind_eval_cv_data_provider(py::module_& m) {
  // --- CVPerformanceData ---
  py::class_<CVPerformanceData, GenericInputData>(m, "CVPerformanceData",
                                                  "Image data for performance benchmarks.\n\n"
                                                  "Wraps source and grayscale images with automatic conversion.")
      .def(py::init<>())
      .def(py::init<const std::string&, const cv::Mat&>(), py::arg("name"), py::arg("source"),
           "Construct from name and source image.")
      .def_readwrite("src", &CVPerformanceData::src, "Source image (BGR or gray).")
      .def_readwrite("src_gray", &CVPerformanceData::src_gray, "Grayscale version of source.");

  // --- FileCVPerformanceDataProvider ---
  py::class_<FileCVPerformanceDataProvider, std::shared_ptr<FileCVPerformanceDataProvider>>(
      m, "FileCVPerformanceDataProvider",
      "File-system data provider for performance benchmarks.\n\n"
      "Scans directories and provides CVPerformanceData with auto grayscale conversion.")
      .def(py::init<const std::string&>(), py::arg("name"), "Construct with provider name.")
      .def(py::init<const std::filesystem::path&, const std::string&, bool>(), py::arg("path"), py::arg("name"),
           py::arg("recursive") = true, "Construct from directory path.")
      .def("parse",
           static_cast<void (FileCVPerformanceDataProvider::*)(const std::filesystem::path&, bool)>(
               &FileCVPerformanceDataProvider::parse),
           py::arg("folder"), py::arg("recursive") = true, "Scan directory for image files.")
      .def("rewind", &FileCVPerformanceDataProvider::rewind, "Reset to start of dataset.")
      .def("clear", &FileCVPerformanceDataProvider::clear, "Clear all loaded file paths.")
      .def_readonly("name", &FileCVPerformanceDataProvider::name, "Provider name.");

  // --- FileDataProvider (convenience alias) ---
  py::class_<FileDataProvider, FileCVPerformanceDataProvider, std::shared_ptr<FileDataProvider>>(
      m, "FileDataProvider",
      "Convenience data provider: construct from folder path and name.\n\n"
      "Equivalent to FileCVPerformanceDataProvider with simpler constructor.")
      .def(py::init<const std::string&, const std::string&>(), py::arg("folder"), py::arg("name"),
           "Construct from folder path and provider name.");
}

// ============================================================================
// Performance measures
// ============================================================================

void bind_eval_performance_measure(py::module_& m) {
  // --- Measure base ---
  py::class_<Measure>(m, "Measure",
                      "Base measurement with source/task names and metadata.\n\n"
                      "Extensible via metadata key-value pairs.")
      .def(py::init<>())
      .def_readwrite("source_name", &Measure::source_name, "Data source name.")
      .def_readwrite("task_name", &Measure::task_name, "Task name.")
      .def(
          "has_metadata", [](const Measure& measure, const std::string& key) { return measure.has_metadata(key); },
          py::arg("key"), "Check if metadata key exists.")
      .def("clear", &Measure::clear, "Clear all metadata.");

  // --- PerformanceMeasureBase ---
  py::class_<PerformanceMeasureBase, Measure>(m, "PerformanceMeasureBase",
                                              "Performance measure with timing durations.\n\n"
                                              "Stores raw tick durations and computes results.")
      .def(py::init<>())
      .def(py::init<const std::string&, const std::string&>(), py::arg("source_name"), py::arg("task_name"))
      .def_readwrite("durations", &PerformanceMeasureBase::durations, "Raw timing durations (ticks).")
      .def(
          "compute_result", [](const PerformanceMeasureBase& pm) { return pm.computeResult(defaultTimingStrategy()); },
          "Compute timing results using default (chrono) strategy.")
      .def("append_duration",
           static_cast<void (PerformanceMeasureBase::*)(std::uint64_t)>(&PerformanceMeasureBase::append),
           py::arg("duration"), "Append a timing duration.")
      .def("clear", &PerformanceMeasureBase::clear, "Clear durations and metadata.");

  // --- CVPerformanceMeasure ---
  py::class_<CVPerformanceMeasure, PerformanceMeasureBase>(
      m, "CVPerformanceMeasure",
      "OpenCV-based performance measure with image dimensions.\n\n"
      "Uses cv::getTickCount() for timing. Tracks image width/height\n"
      "for megapixel throughput computation.")
      .def(py::init<>())
      .def(py::init<const std::string&, const std::string&>(), py::arg("source_name"), py::arg("task_name"))
      .def(py::init<const std::string&, const std::string&, double, double>(), py::arg("source_name"),
           py::arg("task_name"), py::arg("width"), py::arg("height"))
      .def_readwrite("width", &CVPerformanceMeasure::width, "Image width.")
      .def_readwrite("height", &CVPerformanceMeasure::height, "Image height.")
      .def("compute_result",
           static_cast<PerformanceResult (CVPerformanceMeasure::*)() const>(&CVPerformanceMeasure::computeResult),
           "Compute timing results using OpenCV tick strategy.")
      .def("mega_pixels", &CVPerformanceMeasure::megaPixels, "Compute megapixels (width * height / 1e6).")
      .def("clear", &CVPerformanceMeasure::clear, "Clear all data.");

  // --- Free function ---
  m.def(
      "accumulate_measures",
      [](const CVPerformanceMeasureVector& measures, const std::string& source, const std::string& task) {
        return accumulateCVMeasures(measures, source, task);
      },
      py::arg("measures"), py::arg("source_name") = "", py::arg("task_name") = "",
      "Accumulate multiple CVPerformanceMeasures into one.");
}

// ============================================================================
// CVPerformanceTaskBase — the main subclassable task
// ============================================================================

void bind_eval_performance_task(py::module_& m) {
  py::class_<CVPerformanceTaskBase, Task, PyCVPerformanceTask, std::shared_ptr<CVPerformanceTaskBase>>(
      m, "CVPerformanceTask",
      "Base class for CV performance benchmark tasks.\n\n"
      "Subclass in Python and override prepare_impl() and run_impl()\n"
      "to benchmark custom algorithms. The timing loop is handled\n"
      "automatically: prepare_impl is called once (not timed), then\n"
      "run_impl is called in a timed loop.\n\n"
      "Example::\n\n"
      "    class MyTask(le_eval.CVPerformanceTask):\n"
      "        def prepare_impl(self, src):\n"
      "            self.data = preprocess(src)\n\n"
      "        def run_impl(self, name, src):\n"
      "            my_algorithm(self.data)\n")
      .def(py::init<const std::string&, int, bool>(), py::arg("name"), py::arg("flags") = 0, py::arg("verbose") = false)
      .def("rgb", &CVPerformanceTaskBase::rgb, "True if TASK_RGB flag is set.")
      .def("sqr", &CVPerformanceTaskBase::sqr, "True if TASK_SQR flag is set.")
      .def("border", &CVPerformanceTaskBase::border, "Get border size based on flags.")
      .def(
          "prepare_impl",
          [](CVPerformanceTaskBase& self, const cv::Mat& src) {
            // Call via the trampoline which has public access
            static_cast<PyCVPerformanceTask&>(self).prepareImpl(src);
          },
          py::arg("src"), "Override: prepare data before timed run (not timed).")
      .def(
          "run_impl",
          [](CVPerformanceTaskBase& self, const std::string& src_name, const cv::Mat& src) {
            static_cast<PyCVPerformanceTask&>(self).runImpl(src_name, src);
          },
          py::arg("source_name"), py::arg("src"), "Override: the timed operation.");
}

// ============================================================================
// CVPerformanceTest — the orchestrator
// ============================================================================

void bind_eval_performance_test(py::module_& m) {
  using DPList = std::vector<std::shared_ptr<DataProvider<CVPerformanceData>>>;

  py::class_<CVPerformanceTest, std::shared_ptr<CVPerformanceTest>>(
      m, "CVPerformanceTest",
      "Performance test orchestrator.\n\n"
      "Runs tasks over data providers, collects timing results,\n"
      "and generates a StringTable with total/mean/stddev.")
      .def(py::init<DPList, const std::string&, const std::string&, bool, bool>(), py::arg("data_providers"),
           py::arg("name"), py::arg("target_path") = "", py::arg("verbose") = false, py::arg("visual_results") = false)
      .def_readwrite("show_total", &CVPerformanceTest::show_total, "Include total in result table.")
      .def_readwrite("show_mean", &CVPerformanceTest::show_mean, "Include mean in result table.")
      .def_readwrite("show_std_dev", &CVPerformanceTest::show_std_dev, "Include std deviation in result table.")
      .def_readwrite("show_mega_pixel", &CVPerformanceTest::show_mega_pixel, "Include megapixel info.")
      .def(
          "add_task",
          [](CVPerformanceTest& test, std::shared_ptr<CVPerformanceTaskBase> task) {
            test.input_tasks.push_back(task);
          },
          py::arg("task"), "Add a performance task to the test.")
      .def("run", &CVPerformanceTest::run, py::arg("loops"), "Run all tasks over all data providers.")
      .def("result_table", &CVPerformanceTest::resultTable, py::arg("full_report") = false,
           "Get timing results as a StringTable.")
      .def("clear", &CVPerformanceTest::clear, "Clear accumulated results.");
}

}  // namespace python
}  // namespace lsfm
