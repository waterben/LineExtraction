/// @file filter_binding.hpp
/// @brief pybind11 binding declarations for FilterI and related types.
///
/// Provides binding functions for the imgproc filter interface hierarchy:
/// - Range<T> template specializations
/// - Value and ValueManager
/// - FilterData and FilterResults
/// - FilterI<IT> (abstract, with trampoline for Python subclassing)
/// - GradientI<IT,GT,MT,DT> (abstract, with trampoline)
/// - LaplaceI<IT,LT> (abstract, with trampoline)

#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace lsfm {
namespace python {

/// @brief Bind Range, Value, ValueManager, FilterData, FilterResults to the module.
/// @param m The pybind11 module to add bindings to
void bind_core_types(py::module_& m);

/// @brief Bind FilterI<uchar> interface with trampoline.
/// @param m The pybind11 module to add bindings to
void bind_filter_interface(py::module_& m);

/// @brief Bind GradientI<uchar, short, int, float> interface with trampoline.
/// @param m The pybind11 module to add bindings to
void bind_gradient_interface(py::module_& m);

/// @brief Bind LaplaceI<uchar, int> interface with trampoline.
/// @param m The pybind11 module to add bindings to
void bind_laplace_interface(py::module_& m);

/// @brief Bind concrete gradient filter: DerivativeGradient (Sobel variant).
/// @param m The pybind11 module to add bindings to
void bind_derivative_gradient(py::module_& m);

}  // namespace python
}  // namespace lsfm
