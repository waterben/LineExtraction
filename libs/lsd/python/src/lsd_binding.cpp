/// @file lsd_binding.cpp
/// @brief Templated pybind11 bindings for the LSD (Line Segment Detector) hierarchy.
///
/// Implements the templated binding functions declared in lsd_binding.hpp
/// and provides explicit template instantiations for the standard presets:
///
///   | Suffix  | FT     |
///   |---------|--------|
///   | (none)  | float  |
///   | _f64    | double |

#include "lsd_binding.hpp"

#include <cvnp/cvnp.h>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <lsd/ld_base.hpp>
#include <lsd/lsd_base.hpp>
#include <lsd/lsd_burns.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>
#include <lsd/lsd_fbw.hpp>
#include <lsd/lsd_fgioi.hpp>
#include <lsd/lsd_hcv.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Core type bindings (non-templated, called once)
// ============================================================================

void bind_lsd_core_types(py::module_& m) {
  // --- DataDescriptorEntry ---
  py::class_<DataDescriptorEntry>(m, "DataDescriptorEntry",
                                  "Descriptor for an auxiliary image data layer.\n\n"
                                  "Each entry associates a name and description with\n"
                                  "a layer of processed image data (e.g., gradient,\n"
                                  "edge map, segment map).")
      .def(py::init<const std::string&, const std::string&>(), py::arg("name") = "", py::arg("description") = "",
           "Construct a data descriptor entry.")
      .def_readwrite("name", &DataDescriptorEntry::name, "Name of the data layer.")
      .def_readwrite("description", &DataDescriptorEntry::description, "Human-readable description.")
      .def("__repr__", [](const DataDescriptorEntry& e) {
        return "DataDescriptorEntry(name='" + e.name + "', description='" + e.description + "')";
      });

  // --- LSD flag constants ---
  // CC flags
  m.attr("CC_FIND_NEAR_COMPLEX") = static_cast<int>(CC_FIND_NEAR_COMPLEX);
  m.attr("CC_CORNER_RULE") = static_cast<int>(CC_CORNER_RULE);
  m.attr("CC_ADD_THICK_PIXELS") = static_cast<int>(CC_ADD_THICK_PIXELS);

  // Burns flags
  m.attr("BURNS_NMS") = static_cast<int>(BURNS_NMS);

  // FBW flags
  m.attr("FBW_NMS") = static_cast<int>(FBW_NMS);
  m.attr("FBW_PATAN") = static_cast<int>(FBW_PATAN);

  // EL flags
  m.attr("EL_USE_NFA") = static_cast<int>(EL_USE_NFA);
  m.attr("EL_USE_PRECISE_SPE") = static_cast<int>(EL_USE_PRECISE_SPE);

  // EP flags
  m.attr("EP_USE_PRECISE_SPE") = static_cast<int>(EP_USE_PRECISE_SPE);
}

// ============================================================================
// LdBase<FT> binding (abstract line detector)
// ============================================================================

/// @brief Type alias to avoid commas in PYBIND11_OVERRIDE macros.
template <class FT>
using LdBaseV2 = LdBase<FT, Vec2>;

/// @brief Type alias to avoid commas in PYBIND11_OVERRIDE macros.
template <class FT>
using LsdBaseV2 = LsdBase<FT, Vec2>;

/// @brief Trampoline enabling Python subclassing of LdBase<FT>.
template <class FT>
class PyLdBase : public LdBaseV2<FT> {
 public:
  using LdBaseV2<FT>::LdBaseV2;
  void detect(const cv::Mat& image) override { PYBIND11_OVERRIDE_PURE(void, LdBaseV2<FT>, detect, image); }
};

template <class FT>
void bind_ld_base(py::module_& m, const std::string& suffix) {
  using Base = LdBase<FT, Vec2>;
  const std::string cls = "LdBase" + suffix;

  py::class_<Base, ValueManager, PyLdBase<FT>>(m, cls.c_str(),
                                               ("Abstract base for line detectors" + suffix +
                                                ".\n\n"
                                                "Defines the interface for all line detection algorithms.\n"
                                                "Call detect(image) then access results via lines(),\n"
                                                "image_data_descriptor(), and image_data().")
                                                   .c_str())
      .def("detect", static_cast<void (Base::*)(const cv::Mat&)>(&Base::detect), py::arg("image"),
           "Detect lines in a grayscale input image.")
      .def("lines", &Base::lines, py::return_value_policy::reference_internal, "Get detected lines.")
      .def("image_data_descriptor", &Base::imageDataDescriptor, py::return_value_policy::reference_internal,
           "Get descriptors for auxiliary image data layers.")
      .def("image_data", static_cast<const typename Base::ImageData& (Base::*)() const>(&Base::imageData),
           py::return_value_policy::reference_internal, "Get all auxiliary image data from the last detection.")
      .def(
          "image_data_by_name", [](const Base& self, const std::string& name) { return self.imageData(name); },
          py::arg("name"), "Get a single auxiliary image data layer by name.");
}

// ============================================================================
// LsdBase<FT> binding (abstract line segment detector)
// ============================================================================

/// @brief Trampoline enabling Python subclassing of LsdBase<FT>.
template <class FT>
class PyLsdBase : public LsdBaseV2<FT> {
 public:
  using LsdBaseV2<FT>::LsdBaseV2;
  void detect(const cv::Mat& image) override { PYBIND11_OVERRIDE_PURE(void, LsdBaseV2<FT>, detect, image); }
};

template <class FT>
void bind_lsd_base(py::module_& m, const std::string& suffix) {
  using Base = LsdBase<FT, Vec2>;
  using LdB = LdBase<FT, Vec2>;
  const std::string cls = "LsdBase" + suffix;

  py::class_<Base, LdB, PyLsdBase<FT>>(m, cls.c_str(),
                                       ("Abstract base for line segment detectors" + suffix +
                                        ".\n\n"
                                        "Extends LdBase with line segment and endpoint access.\n"
                                        "Call detect(image) then access results via line_segments(),\n"
                                        "end_points(), or lines().")
                                           .c_str())
      .def("line_segments", &Base::lineSegments, py::return_value_policy::reference_internal,
           "Get detected line segments with full geometric information.")
      .def("end_points", &Base::endPoints, py::return_value_policy::reference_internal,
           "Get line endpoints as Vec4 vectors (x1, y1, x2, y2).")
      .def("lines", &Base::lines, py::return_value_policy::reference_internal,
           "Get detected lines (converted from segments).");
}

// ============================================================================
// LsdCC<FT>
// ============================================================================

template <class FT>
void bind_lsd_cc(py::module_& m, const std::string& suffix) {
  using Det = LsdCC<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdCC" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Connected component line segment detector" + suffix +
                         ".\n\n"
                         "Detects line segments by linking edge pixels into\n"
                         "connected components, then splitting and fitting.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min supporting pixels\n"
                         "    'edge_max_gap': Maximum gap in pixels\n"
                         "    'split_error_distance': Max split error\n"
                         "    'line_flags': Detection flags (CC_* constants)")
                            .c_str())
      .def(py::init<FT, FT, int, int, FT, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 10, py::arg("max_gap") = 0,
           py::arg("err_dist") = static_cast<FT>(2), py::arg("flags") = 0, "Construct CC detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdCP<FT>
// ============================================================================

template <class FT>
void bind_lsd_cp(py::module_& m, const std::string& suffix) {
  using Det = LsdCP<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdCP" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Connected component with pattern line segment detector" + suffix +
                         ".\n\n"
                         "Extends CC detection with pattern-based linking.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min supporting pixels\n"
                         "    'edge_max_gap': Maximum gap\n"
                         "    'split_error_distance': Max split error\n"
                         "    'edge_pat_tol': Pattern tolerance\n"
                         "    'line_flags': Detection flags (CP_* constants)")
                            .c_str())
      .def(py::init<FT, FT, int, int, FT, int, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 10, py::arg("max_gap") = 0,
           py::arg("err_dist") = static_cast<FT>(2), py::arg("pat_tol") = 2, py::arg("flags") = 0,
           "Construct CP detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdBurns<FT>
// ============================================================================

template <class FT>
void bind_lsd_burns(py::module_& m, const std::string& suffix) {
  using Det = LsdBurns<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdBurns" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Burns algorithm line segment detector" + suffix +
                         ".\n\n"
                         "Classic 'Extracting Straight Lines' algorithm using\n"
                         "gradient-based edge segmentation and CC analysis.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min supporting pixels\n"
                         "    'burns_partitions': Number of direction partitions\n"
                         "    'line_flags': Detection flags (BURNS_NMS)")
                            .c_str())
      .def(py::init<FT, FT, int, int, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 5, py::arg("part_num") = 12,
           py::arg("flags") = 0, "Construct Burns detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdFBW<FT>
// ============================================================================

template <class FT>
void bind_lsd_fbw(py::module_& m, const std::string& suffix) {
  using Det = LsdFBW<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdFBW" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Fast region-growing line segment detector (FBW)" + suffix +
                         ".\n\n"
                         "Detects line segments by growing aligned regions from\n"
                         "seed pixels, then fitting line segments.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min supporting pixels\n"
                         "    'angle_th': Gradient angle tolerance (degrees)\n"
                         "    'line_flags': Detection flags (FBW_NMS, FBW_PATAN)")
                            .c_str())
      .def(py::init<FT, FT, int, FT, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 0,
           py::arg("ang_th") = static_cast<FT>(22.5), py::arg("flags") = 0, "Construct FBW detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdFGioi<FT>
// ============================================================================

template <class FT>
void bind_lsd_fgioi(py::module_& m, const std::string& suffix) {
  using Det = LsdFGioi<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdFGioi" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("FGioi/PLSD probabilistic line segment detector" + suffix +
                         ".\n\n"
                         "Uses Number of False Alarms (NFA) principle for\n"
                         "statistical validation of line segments.\n\n"
                         "Parameters (via set_value):\n"
                         "    'quant_error': Gradient quantization error\n"
                         "    'angle_th': Gradient angle tolerance (degrees)\n"
                         "    'log_eps': NFA epsilon (logarithmic)\n"
                         "    'density_th': Min density of aligned region\n"
                         "    'bins': Number of gradient magnitude bins")
                            .c_str())
      .def(py::init<FT, FT, FT, FT, int>(), py::arg("quant") = static_cast<FT>(2),
           py::arg("ang_th") = static_cast<FT>(22.5), py::arg("log_eps") = static_cast<FT>(0),
           py::arg("density_th") = static_cast<FT>(0.7), py::arg("n_bins") = 1024,
           "Construct FGioi detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdEDLZ<FT> — non-copyable, needs special handling
// ============================================================================

template <class FT>
void bind_lsd_edlz(py::module_& m, const std::string& suffix) {
  using Det = LsdEDLZ<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdEDLZ" + suffix;

  // LsdEDLZ is non-copyable (raw pointer member), so we use
  // std::unique_ptr as the holder type.
  py::class_<Det, Base, std::unique_ptr<Det>>(m, cls.c_str(),
                                              ("EDLines line segment detector" + suffix +
                                               ".\n\n"
                                               "Uses Edge Drawing algorithm for edge detection\n"
                                               "followed by line segment fitting and NFA validation.\n\n"
                                               "Parameters (via set_value):\n"
                                               "    'grad_th': Gradient threshold\n"
                                               "    'anchor_th': Anchor threshold\n"
                                               "    'scan_int': Scan intervals\n"
                                               "    'min_len': Minimum line length\n"
                                               "    'fit_error': Line fit error threshold\n"
                                               "    'validate': Use NFA validation")
                                                  .c_str())
      .def(py::init<FT, FT, int, int, FT, bool>(), py::arg("gradient_threshold") = static_cast<FT>(10),
           py::arg("anchor_threshold") = static_cast<FT>(2), py::arg("scan_intervals") = 2,
           py::arg("min_line_len") = 15, py::arg("line_fit_err_threshold") = static_cast<FT>(2),
           py::arg("validate") = false, "Construct EDLines detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdEL<FT> — Edge Linking detector
// ============================================================================

template <class FT>
void bind_lsd_el(py::module_& m, const std::string& suffix) {
  using Det = LsdEL<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdEL" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Edge linking line segment detector" + suffix +
                         ".\n\n"
                         "Combines edge linking with NFA validation and\n"
                         "sub-pixel estimation for high-quality detection.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min edge segment pixels\n"
                         "    'split_error_distance': Split distance\n"
                         "    'split_min_len': Min segment length after split\n"
                         "    'nfa_log_eps': NFA epsilon (logarithmic)\n"
                         "    'line_flags': Flags (EL_USE_NFA, EL_USE_PRECISE_SPE)")
                            .c_str())
      .def(py::init<FT, FT, int, FT, int, FT, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 10, py::arg("dist") = static_cast<FT>(2),
           py::arg("min_len") = 5, py::arg("log_eps") = static_cast<FT>(0), py::arg("flags") = 0,
           "Construct EL detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdEP<FT> — Edge Pattern detector
// ============================================================================

template <class FT>
void bind_lsd_ep(py::module_& m, const std::string& suffix) {
  using Det = LsdEP<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdEP" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Edge pattern line segment detector" + suffix +
                         ".\n\n"
                         "Detects edge segments using directional patterns,\n"
                         "then splits and fits line segments.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'edge_min_pixels': Min edge segment pixels\n"
                         "    'split_error_distance': Split distance\n"
                         "    'split_min_len': Min segment length after split\n"
                         "    'edge_pat_tol': Pattern tolerance\n"
                         "    'edge_max_gap': Maximum gap\n"
                         "    'edge_mag_mul': Magnitude multiplier\n"
                         "    'edge_mag_th': Magnitude threshold\n"
                         "    'line_flags': Flags (EP_USE_PRECISE_SPE)")
                            .c_str())
      .def(py::init<FT, FT, int, FT, int, int, int, float, float, int>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("min_pix") = 10, py::arg("dist") = static_cast<FT>(2),
           py::arg("min_len") = 5, py::arg("pat_tol") = 2, py::arg("max_gap") = 3, py::arg("mag_mul") = 3.0f,
           py::arg("mag_th") = 5.0f, py::arg("flags") = 0, "Construct EP detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// LsdHoughP<FT> — Probabilistic Hough Transform
// ============================================================================

template <class FT>
void bind_lsd_houghp(py::module_& m, const std::string& suffix) {
  using Det = LsdHoughP<FT>;
  using Base = LsdBase<FT, Vec2>;
  const std::string cls = "LsdHoughP" + suffix;

  py::class_<Det, Base>(m, cls.c_str(),
                        ("Probabilistic Hough Transform line segment detector" + suffix +
                         ".\n\n"
                         "Wraps OpenCV's HoughLinesP with integrated gradient-based\n"
                         "edge detection and line segment output.\n\n"
                         "Parameters (via set_value):\n"
                         "    'nms_th_low': Lower gradient threshold\n"
                         "    'nms_th_high': Upper gradient threshold\n"
                         "    'hough_rho': Distance resolution (pixels)\n"
                         "    'hough_theta': Angle resolution (radians)\n"
                         "    'hough_vote_th': Min accumulator votes\n"
                         "    'edge_min_len': Min segment length\n"
                         "    'edge_max_gap': Max link gap")
                            .c_str())
      .def(py::init<FT, FT, double, double, int, double, double>(), py::arg("th_low") = static_cast<FT>(0.004),
           py::arg("th_high") = static_cast<FT>(0.012), py::arg("rho") = 1.5, py::arg("theta") = CV_PI / 180,
           py::arg("vote_threshold") = 150, py::arg("min_length") = 10.0, py::arg("max_gap") = 3.0,
           "Construct Probabilistic Hough detector with parameters.")
      .def("detect", static_cast<void (Det::*)(const cv::Mat&)>(&Det::detect), py::arg("image"),
           "Detect line segments in a grayscale image.")
      .def("line_segments", &Det::lineSegments, py::return_value_policy::reference_internal)
      .def("end_points", &Det::endPoints, py::return_value_policy::reference_internal)
      .def("lines", &Det::lines, py::return_value_policy::reference_internal)
      .def("image_data_descriptor", &Det::imageDataDescriptor, py::return_value_policy::reference_internal)
      .def("image_data", static_cast<const typename Base::ImageData& (Det::*)() const>(&Det::imageData),
           py::return_value_policy::reference_internal);
}

// ============================================================================
// Convenience preset binder
// ============================================================================

template <class FT>
void bind_lsd_preset(py::module_& m, const std::string& suffix) {
  bind_ld_base<FT>(m, suffix);
  bind_lsd_base<FT>(m, suffix);
  bind_lsd_cc<FT>(m, suffix);
  bind_lsd_cp<FT>(m, suffix);
  bind_lsd_burns<FT>(m, suffix);
  bind_lsd_fbw<FT>(m, suffix);
  bind_lsd_fgioi<FT>(m, suffix);
  bind_lsd_edlz<FT>(m, suffix);
  bind_lsd_el<FT>(m, suffix);
  bind_lsd_ep<FT>(m, suffix);
  bind_lsd_houghp<FT>(m, suffix);
}

// ============================================================================
// Explicit template instantiations for supported presets
// ============================================================================

// Default: float — suffix ""
template void bind_ld_base<float>(py::module_&, const std::string&);
template void bind_lsd_base<float>(py::module_&, const std::string&);
template void bind_lsd_cc<float>(py::module_&, const std::string&);
template void bind_lsd_cp<float>(py::module_&, const std::string&);
template void bind_lsd_burns<float>(py::module_&, const std::string&);
template void bind_lsd_fbw<float>(py::module_&, const std::string&);
template void bind_lsd_fgioi<float>(py::module_&, const std::string&);
template void bind_lsd_edlz<float>(py::module_&, const std::string&);
template void bind_lsd_el<float>(py::module_&, const std::string&);
template void bind_lsd_ep<float>(py::module_&, const std::string&);
template void bind_lsd_houghp<float>(py::module_&, const std::string&);
template void bind_lsd_preset<float>(py::module_&, const std::string&);

// Double (64-bit) — suffix "_f64"
template void bind_ld_base<double>(py::module_&, const std::string&);
template void bind_lsd_base<double>(py::module_&, const std::string&);
template void bind_lsd_cc<double>(py::module_&, const std::string&);
template void bind_lsd_cp<double>(py::module_&, const std::string&);
template void bind_lsd_burns<double>(py::module_&, const std::string&);
template void bind_lsd_fbw<double>(py::module_&, const std::string&);
template void bind_lsd_fgioi<double>(py::module_&, const std::string&);
template void bind_lsd_edlz<double>(py::module_&, const std::string&);
template void bind_lsd_el<double>(py::module_&, const std::string&);
template void bind_lsd_ep<double>(py::module_&, const std::string&);
template void bind_lsd_houghp<double>(py::module_&, const std::string&);
template void bind_lsd_preset<double>(py::module_&, const std::string&);

#undef COMMA

}  // namespace python
}  // namespace lsfm
