/// @file module_le_eval.cpp
/// @brief PYBIND11_MODULE entry point for le_eval Python bindings.
///
/// Exposes the evaluation framework: tasks, data providers, performance
/// measures, and the CVPerformanceTest orchestrator.

#include "eval_binding.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(le_eval, m) {
  m.doc() =
      "LineExtraction evaluation framework — performance benchmarking,\n"
      "task management, and result collection.";

  lsfm::python::bind_eval_core_types(m);
  lsfm::python::bind_eval_task(m);
  lsfm::python::bind_eval_data_provider(m);
  lsfm::python::bind_eval_cv_data_provider(m);
  lsfm::python::bind_eval_performance_measure(m);
  lsfm::python::bind_eval_performance_task(m);
  lsfm::python::bind_eval_performance_test(m);
}
