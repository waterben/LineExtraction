/// @file edge_binding.hpp
/// @brief pybind11 binding declarations for the edge detection hierarchy.
///
/// Provides both non-templated bindings (EdgeSegment, enums, shared types) and
/// templated binding functions that can be instantiated for different magnitude
/// type (MT) / gradient type (GT) combinations.
///
/// ## Preset system
///
/// Each preset maps to a specific combination of template parameters defining
/// the EdgeSource, NMS, and ESD pipeline types:
///
///   - IT  – Input image pixel type (uchar, ushort, float, double)
///   - GT  – Gradient type
///   - MT  – Magnitude type
///   - DT  – Direction type
///
/// A string suffix (e.g. "_f32") is appended to every Python class name to
/// disambiguate the preset.  The default (uchar) preset uses an empty suffix
/// so the Python names remain backwards-compatible.
///
/// ## Adding a new preset
///
/// 1. Add explicit template instantiations in edge_binding.cpp.
/// 2. Call bind_edge_preset<IT,GT,MT,DT>(m, "_suffix") in the module.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Non-templated: shared core types (EdgeSegment, enums)
// ============================================================================

/// @brief Bind EdgeSegment, EdgeSegmentVector, IndexVector, and enums.
/// @param m The pybind11 module to add bindings to
void bind_edge_core_types(py::module_& m);

// ============================================================================
// Templated: edge detection class bindings
// ============================================================================

/// @brief Bind EdgeSourceI abstract interface with Python-accessible trampoline.
/// @param m The pybind11 module
/// @param suffix Python class name suffix (e.g. "" or "_f32")
void bind_edge_source_interface(py::module_& m, const std::string& suffix);

/// @brief Bind NonMaximaSuppression wrapper for a specific type preset.
/// @tparam GT Gradient element type
/// @tparam MT Magnitude element type
/// @tparam DT Direction/floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class GT, class MT, class DT>
void bind_nms(py::module_& m, const std::string& suffix);

/// @brief Bind EsdBase<MT> abstract interface.
/// @tparam MT Magnitude element type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class MT>
void bind_esd_base(py::module_& m, const std::string& suffix);

/// @brief Bind EsdDrawing<MT> concrete edge segment detector.
/// @tparam MT Magnitude element type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class MT>
void bind_esd_drawing(py::module_& m, const std::string& suffix);

/// @brief Bind EsdSimple<MT> concrete edge segment detector.
/// @tparam MT Magnitude element type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class MT>
void bind_esd_simple(py::module_& m, const std::string& suffix);

/// @brief Bind EsdLinking<MT> concrete edge segment detector.
/// @tparam MT Magnitude element type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class MT>
void bind_esd_linking(py::module_& m, const std::string& suffix);

/// @brief Bind EsdPattern<MT> concrete edge segment detector.
/// @tparam MT Magnitude element type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class MT>
void bind_esd_pattern(py::module_& m, const std::string& suffix);

/// @brief Bind EdgeSourceGRAD concrete implementations (Sobel, Scharr, Prewitt).
/// @tparam IT Input image pixel type
/// @tparam GT Gradient type
/// @tparam MT Magnitude type
/// @tparam DT Direction type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class IT, class GT, class MT, class DT>
void bind_edge_sources(py::module_& m, const std::string& suffix);

/// @brief Convenience: bind all edge interfaces + concrete classes for one preset.
///
/// Calls bind_edge_source_interface, bind_nms, bind_esd_base, the ESD
/// detectors, and bind_edge_sources with the given template parameters.
///
/// @tparam IT Input image pixel type
/// @tparam GT Gradient type
/// @tparam MT Magnitude type
/// @tparam DT Direction type
/// @param m The pybind11 module
/// @param suffix Python class name suffix (empty for default / uchar preset)
template <class IT, class GT, class MT, class DT>
void bind_edge_preset(py::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
