/// @file lsd_binding.hpp
/// @brief pybind11 binding declarations for the LSD (Line Segment Detector) hierarchy.
///
/// Provides both non-templated bindings (DataDescriptorEntry, Line/LineSegment
/// geometry types) and templated binding functions that can be instantiated for
/// different floating-point type (FT) presets.
///
/// ## Preset system
///
/// Each preset maps to a specific floating-point type:
///
///   - FT=float  (suffix "")    — default preset
///   - FT=double (suffix "_f64") — double-precision preset
///
/// A string suffix is appended to every Python class name to disambiguate
/// the preset.  The default (float) preset uses an empty suffix so the
/// Python names remain short and backwards-compatible.
///
/// ## Adding a new preset
///
/// 1. Add explicit template instantiations in lsd_binding.cpp.
/// 2. Call bind_lsd_preset<FT>(m, "_suffix") in the module.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Non-templated: shared core types (DataDescriptorEntry)
// ============================================================================

/// @brief Bind DataDescriptorEntry and related shared types.
/// @param m The pybind11 module to add bindings to
void bind_lsd_core_types(py::module_& m);

// ============================================================================
// Templated: geometry type bindings
// ============================================================================

/// @brief Bind Line<FT> geometry class.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix (e.g. "" or "_f64")
template <class FT>
void bind_line(py::module_& m, const std::string& suffix);

/// @brief Bind LineSegment<FT> geometry class (extends Line<FT>).
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_line_segment(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: detector base class bindings
// ============================================================================

/// @brief Bind LdBase<FT> abstract line detector interface.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_ld_base(py::module_& m, const std::string& suffix);

/// @brief Bind LsdBase<FT> abstract line segment detector interface.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_base(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: concrete detector bindings
// ============================================================================

/// @brief Bind LsdCC<FT> — connected component line segment detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_cc(py::module_& m, const std::string& suffix);

/// @brief Bind LsdCP<FT> — connected component with pattern detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_cp(py::module_& m, const std::string& suffix);

/// @brief Bind LsdBurns<FT> — Burns algorithm detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_burns(py::module_& m, const std::string& suffix);

/// @brief Bind LsdFBW<FT> — Fast region-growing detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_fbw(py::module_& m, const std::string& suffix);

/// @brief Bind LsdFGioi<FT> — FGioi/PLSD algorithm detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_fgioi(py::module_& m, const std::string& suffix);

/// @brief Bind LsdEDLZ<FT> — EDLines algorithm detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_edlz(py::module_& m, const std::string& suffix);

/// @brief Bind LsdEL<FT> — Edge Linking detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_el(py::module_& m, const std::string& suffix);

/// @brief Bind LsdEP<FT> — Edge Pattern detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_ep(py::module_& m, const std::string& suffix);

/// @brief Bind LsdHoughP<FT> — Probabilistic Hough Transform detector.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lsd_houghp(py::module_& m, const std::string& suffix);

/// @brief Convenience: bind all LSD types for one FT preset.
///
/// Calls bind_line, bind_line_segment, bind_ld_base, bind_lsd_base,
/// and all concrete detector binders with the given FT type.
///
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix (empty for default / float preset)
template <class FT>
void bind_lsd_preset(py::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
