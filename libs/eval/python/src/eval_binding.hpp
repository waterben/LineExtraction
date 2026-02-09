/// @file eval_binding.hpp
/// @brief Declarations for eval pybind11 bindings.
///
/// Provides binding functions for the evaluation framework: tasks, measures,
/// data providers, and performance testing infrastructure.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace lsfm {
namespace python {

/// @brief Bind core eval types (StringTable, GenericInputData, Measure, PerformanceResult).
void bind_eval_core_types(pybind11::module_& m);

/// @brief Bind ITask/Task, TaskRunner interfaces.
void bind_eval_task(pybind11::module_& m);

/// @brief Bind DataProvider, CVData, FileCVDataProvider.
void bind_eval_data_provider(pybind11::module_& m);

/// @brief Bind CVPerformanceData, FileCVPerformanceDataProvider, FileDataProvider.
void bind_eval_cv_data_provider(pybind11::module_& m);

/// @brief Bind PerformanceMeasure, CVPerformanceMeasure.
void bind_eval_performance_measure(pybind11::module_& m);

/// @brief Bind CVPerformanceTaskBase (the main task base class).
void bind_eval_performance_task(pybind11::module_& m);

/// @brief Bind CVPerformanceTest (the runner/orchestrator).
void bind_eval_performance_test(pybind11::module_& m);

}  // namespace python
}  // namespace lsfm
