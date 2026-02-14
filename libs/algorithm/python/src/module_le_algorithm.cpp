/// @file module_le_algorithm.cpp
/// @brief PYBIND11_MODULE entry point for le_algorithm Python bindings.
///
/// Exposes line merging, connecting, accuracy measurement,
/// ground truth loading, search strategies, and parameter optimization.

#include "algorithm_binding.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(le_algorithm, m) {
  m.doc() =
      "LineExtraction algorithm library — line merging, connecting,\n"
      "accuracy measurement, and parameter optimization.";

  // Import le_geometry so that LineSegment type bindings are available.
  py::module_::import("le_geometry");

  // Bind shared types first (enums, ground truth, search strategies, optimizer)
  lsfm::python::bind_ground_truth(m);
  lsfm::python::bind_search_strategy(m);
  lsfm::python::bind_param_optimizer(m);

  // Bind templated types for double (primary) and float
  lsfm::python::bind_algorithm<double>(m, "");
  lsfm::python::bind_algorithm<float>(m, "_f32");
}
