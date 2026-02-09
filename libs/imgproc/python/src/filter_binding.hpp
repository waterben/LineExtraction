/// @file filter_binding.hpp
/// @brief pybind11 binding declarations for the imgproc filter hierarchy.
///
/// Provides both non-templated bindings (core types shared across all presets)
/// and templated binding functions that can be instantiated for different
/// pixel type combinations (e.g. uint8, uint16, float, double input images).
///
/// ## Preset system
///
/// Each preset is a combination of template parameters defining the full
/// processing pipeline types:
///
///   - IT  – Input image pixel type (uchar, ushort, float, double)
///   - GT  – Gradient / derivative type
///   - MT  – Magnitude type
///   - DT  – Direction type
///   - LT  – Laplacian type
///
/// A string suffix (e.g. "_f32") is appended to every Python class name
/// to disambiguate the preset.  The default (uchar) preset uses an empty
/// suffix so the Python names remain backwards-compatible.
///
/// ## Adding a new preset
///
/// 1. Add explicit template instantiations in filter_binding.cpp.
/// 2. Call bind_filter_preset<IT,GT,MT,DT,LT>(m, "_suffix") in the module.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Non-templated: shared core types (Range, Value, ValueManager, FilterData)
// ============================================================================

/// @brief Bind Range<T> specializations, Value, ValueManager, and FilterData.
/// @param m The pybind11 module to add bindings to
void bind_core_types(py::module_& m);

// ============================================================================
// Templated: filter interface and concrete class bindings
// ============================================================================

/// @brief Bind FilterI<IT> abstract interface with Python-subclassable trampoline.
/// @tparam IT Input image pixel type
/// @param m The pybind11 module
/// @param suffix Python class name suffix (e.g. "" or "_f32")
template <class IT>
void bind_filter_interface(py::module_& m, const std::string& suffix);

/// @brief Bind GradientI<IT,GT,MT,DT> abstract interface with trampoline.
/// @tparam IT Input image pixel type
/// @tparam GT Gradient / derivative type
/// @tparam MT Magnitude type
/// @tparam DT Direction type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class IT, class GT, class MT, class DT>
void bind_gradient_interface(py::module_& m, const std::string& suffix);

/// @brief Bind LaplaceI<IT,LT> abstract interface with trampoline.
/// @tparam IT Input image pixel type
/// @tparam LT Laplacian output type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class IT, class LT>
void bind_laplace_interface(py::module_& m, const std::string& suffix);

/// @brief Bind DerivativeGradient concrete classes (Sobel, Scharr, Prewitt).
/// @tparam IT Input image pixel type
/// @tparam GT Gradient / derivative type
/// @tparam MT Magnitude type
/// @tparam DT Direction type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class IT, class GT, class MT, class DT>
void bind_derivative_gradient(py::module_& m, const std::string& suffix);

/// @brief Convenience: bind all interfaces + concrete classes for one type preset.
///
/// Calls bind_filter_interface, bind_gradient_interface, bind_laplace_interface,
/// and bind_derivative_gradient with the given template parameters and suffix.
///
/// @tparam IT Input image pixel type
/// @tparam GT Gradient / derivative type
/// @tparam MT Magnitude type
/// @tparam DT Direction type
/// @tparam LT Laplacian output type (defaults to int)
/// @param m The pybind11 module
/// @param suffix Python class name suffix (empty for default / uchar preset)
template <class IT, class GT, class MT, class DT, class LT>
void bind_filter_preset(py::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
