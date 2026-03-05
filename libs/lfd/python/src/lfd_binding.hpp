/// @file lfd_binding.hpp
/// @brief pybind11 binding declarations for the LFD (Line Feature Descriptor) library.
///
/// Provides templated binding functions for descriptor types, creators,
/// matchers, and filters that can be instantiated for different
/// floating-point type (FT) presets.
///
/// ## Preset system
///
///   - FT=float  (suffix "")     — default preset
///   - FT=double (suffix "_f64") — double-precision preset
///
/// Line, LineSegment, and LSD types are provided by le_geometry / le_lsd
/// and imported at module load time — they are NOT re-registered here.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Non-templated: shared core types
// ============================================================================

/// @brief Bind FilterState constants and MatMap helper.
/// @param m The pybind11 module to add bindings to
void bind_lfd_core_types(py::module_& m);

// ============================================================================
// Templated: match types
// ============================================================================

/// @brief Bind FeatureMatch<FT> and DescriptorMatch<FT>.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_match_types(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: descriptor types
// ============================================================================

/// @brief Bind FdMat<FT>, FdLBD<FT>, FdOpenCVLBD<FT> descriptor data structures.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_descriptor_types(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: descriptor creators
// ============================================================================

/// @brief Bind FdcLBD, FdcGenericLR, FdcOpenCVLBD descriptor creators.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_descriptor_creators(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: filters
// ============================================================================

/// @brief Bind GlobalRotationFilter and StereoLineFilter.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_filters(py::module_& m, const std::string& suffix);

// ============================================================================
// Templated: matchers
// ============================================================================

/// @brief Bind FmBruteForce matcher for LBD, LR, and OpenCV LBD descriptors.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_matchers(py::module_& m, const std::string& suffix);

// ============================================================================
// Stereo matcher
// ============================================================================

/// @brief Bind StereoLineMatcher for a given floating-point type.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix
template <class FT>
void bind_lfd_stereo_matcher(py::module_& m, const std::string& suffix);

// ============================================================================
// Convenience: bind all LFD types for one FT preset
// ============================================================================

/// @brief Bind all LFD types for a given floating-point type.
/// @tparam FT Floating-point type
/// @param m The pybind11 module
/// @param suffix Python class name suffix (empty for float)
template <class FT>
void bind_lfd_preset(py::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
