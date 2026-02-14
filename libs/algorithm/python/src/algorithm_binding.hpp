/// @file algorithm_binding.hpp
/// @brief Declarations for algorithm pybind11 bindings.
///
/// Provides binding functions for line merging, connecting, accuracy
/// measurement, parameter optimization, and search strategies.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace lsfm {
namespace python {

/// @brief Bind LineMerge class.
/// @param m pybind11 module.
/// @param suffix Type suffix (empty for double, "_f32" for float).
template <class FT>
void bind_line_merge(pybind11::module_& m, const std::string& suffix);

/// @brief Bind LineConnect class.
/// @param m pybind11 module.
/// @param suffix Type suffix.
template <class FT>
void bind_line_connect(pybind11::module_& m, const std::string& suffix);

/// @brief Bind AccuracyMeasure and AccuracyResult classes.
/// @param m pybind11 module.
/// @param suffix Type suffix.
template <class FT>
void bind_accuracy_measure(pybind11::module_& m, const std::string& suffix);

/// @brief Bind GroundTruthLoader and GroundTruthEntry.
void bind_ground_truth(pybind11::module_& m);

/// @brief Bind SearchStrategy, ParamRange, GridSearchStrategy, RandomSearchStrategy.
void bind_search_strategy(pybind11::module_& m);

/// @brief Bind ParamOptimizer, SearchResult, EvalResult.
void bind_param_optimizer(pybind11::module_& m);

/// @brief Bind all algorithm types for a given floating-point type.
/// @param m pybind11 module.
/// @param suffix Type suffix.
template <class FT>
void bind_algorithm(pybind11::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
