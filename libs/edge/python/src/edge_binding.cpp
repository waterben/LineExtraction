/// @file edge_binding.cpp
/// @brief Templated pybind11 bindings for the edge detection hierarchy.
///
/// Implements the templated binding functions declared in edge_binding.hpp
/// and provides explicit template instantiations for the standard presets:
///
///   | Suffix  | IT     | GT     | MT     | DT     |
///   |---------|--------|--------|--------|--------|
///   | (none)  | uchar  | short  | float  | float  |
///   | _16u    | ushort | float  | float  | float  |
///   | _f32    | float  | float  | float  | float  |
///   | _f64    | double | double | double | double |

#include "edge_binding.hpp"

#include <cvnp/cvnp.h>
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_segment.hpp>
#include <edge/edge_simple.hpp>
#include <edge/edge_source.hpp>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Core type bindings (non-templated, called once)
// ============================================================================

void bind_edge_core_types(py::module_& m) {
  // --- EdgeSegment flags ---
  // Cast anonymous enum values to int explicitly; pybind11 cannot
  // look up an anonymous (unregistered) enum type.
  m.attr("ES_NONE") = static_cast<int>(ES_NONE);
  m.attr("ES_REVERSE") = static_cast<int>(ES_REVERSE);
  m.attr("ES_CLOSED") = static_cast<int>(ES_CLOSED);

  // --- ESDirectionOptions enum ---
  py::enum_<ESDirectionOptions>(m, "ESDirectionOptions", "Direction options for edge sources.")
      .value("NONE", ESDO_NONE, "No direction enrichment.")
      .value("GXGY", ESDO_GXGY, "Use gradient components (gx, gy) for direction.")
      .value("DIR", ESDO_DIR, "Use pre-computed direction map.");

  // --- ESQuadratureOptions enum ---
  py::enum_<ESQuadratureOptions>(m, "ESQuadratureOptions", "Magnitude options for quadrature-based edge sources.")
      .value("MAG", ESQO_MAG, "Use odd response magnitude.")
      .value("ENERGY", ESQO_ENERGY, "Use energy response.")
      .value("PC", ESQO_PC, "Use phase congruency.");

  // --- EdgeSegment ---
  py::class_<EdgeSegment>(m, "EdgeSegment", "Edge segment referencing a range of points in a shared array.")
      .def(py::init<size_t, size_t, int, int>(), py::arg("begin") = 0, py::arg("end") = 0,
           py::arg("flags") = static_cast<int>(ES_NONE), py::arg("id") = 0,
           "Construct an edge segment with index range and flags.")
      .def_property("begin", static_cast<size_t (EdgeSegment::*)() const>(&EdgeSegment::begin),
                    static_cast<void (EdgeSegment::*)(size_t)>(&EdgeSegment::begin), "Start index in the point array.")
      .def_property("end", static_cast<size_t (EdgeSegment::*)() const>(&EdgeSegment::end),
                    static_cast<void (EdgeSegment::*)(size_t)>(&EdgeSegment::end),
                    "End index in the point array (exclusive).")
      .def(
          "size", [](const EdgeSegment& s) { return s.size(); }, "Number of supporting points.")
      .def("first", &EdgeSegment::first, "Index of the first point (respects reverse flag).")
      .def("last", &EdgeSegment::last, "Index of the last point (respects reverse flag).")
      .def_property("flags", static_cast<int (EdgeSegment::*)() const>(&EdgeSegment::flags),
                    static_cast<void (EdgeSegment::*)(int)>(&EdgeSegment::flags),
                    "Bitwise flags (ES_NONE, ES_REVERSE, ES_CLOSED).")
      .def_property("id", static_cast<int (EdgeSegment::*)() const>(&EdgeSegment::id),
                    static_cast<void (EdgeSegment::*)(int)>(&EdgeSegment::id), "Segment identifier.")
      .def("reverse", &EdgeSegment::reverse, "True if segment is marked as reversed.")
      .def("closed", &EdgeSegment::closed, "True if segment forms a closed loop.")
      .def("__repr__", [](const EdgeSegment& s) {
        std::string flags_str;
        if (s.reverse()) flags_str += "R";
        if (s.closed()) flags_str += "C";
        if (flags_str.empty()) flags_str = "-";
        return "EdgeSegment(begin=" + std::to_string(s.begin()) + ", end=" + std::to_string(s.end()) +
               ", size=" + std::to_string(s.size()) + ", flags=" + flags_str + ", id=" + std::to_string(s.id()) + ")";
      });
}

// ============================================================================
// EdgeSourceI interface binding (non-templated — virtual interface)
// ============================================================================

/// @brief Trampoline enabling Python subclassing of EdgeSourceI.
class PyEdgeSourceI : public EdgeSourceI {
 public:
  using EdgeSourceI::EdgeSourceI;

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, EdgeSourceI, process, img); }
  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, EdgeSourceI, results); }
  cv::Mat gx() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, EdgeSourceI, gx); }
  cv::Mat gy() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, EdgeSourceI, gy); }
  cv::Mat magnitude() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, EdgeSourceI, magnitude); }
  double magnitudeThreshold(double val) const override {
    PYBIND11_OVERRIDE_PURE(double, EdgeSourceI, magnitudeThreshold, val);
  }
  double magnitudeMax() const override { PYBIND11_OVERRIDE_PURE(double, EdgeSourceI, magnitudeMax); }
  cv::Mat direction() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, EdgeSourceI, direction); }
  Range<double> directionRange() const override { PYBIND11_OVERRIDE_PURE(Range<double>, EdgeSourceI, directionRange); }
  cv::Mat directionMap() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, EdgeSourceI, directionMap); }
  const IndexVector& seeds() const override { PYBIND11_OVERRIDE_PURE(const IndexVector&, EdgeSourceI, seeds); }
  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, EdgeSourceI, name); }
};

void bind_edge_source_interface(py::module_& m, const std::string& suffix) {
  const std::string cls = "EdgeSourceI" + suffix;

  // Only register the base class once (suffix == "")
  if (!suffix.empty()) return;

  py::class_<EdgeSourceI, ValueManager, PyEdgeSourceI>(
      m, cls.c_str(),
      "Abstract interface for edge source computation.\n\n"
      "Provides a unified interface for combining arbitrary edge response\n"
      "methods (Sobel, Laplace, quadrature) with different edgel extraction\n"
      "methods (NMS, zero-crossing).")
      .def("process", &EdgeSourceI::process, py::arg("img"), "Process an input image to compute edge responses.")
      .def("results", &EdgeSourceI::results, "Get all processed filter results.")
      .def("gx", &EdgeSourceI::gx, "Get the gradient in the X direction.")
      .def("gy", &EdgeSourceI::gy, "Get the gradient in the Y direction.")
      .def("magnitude", &EdgeSourceI::magnitude, "Get the edge magnitude map.")
      .def("magnitude_threshold", &EdgeSourceI::magnitudeThreshold, py::arg("val"),
           "Convert a relative threshold to absolute magnitude threshold.")
      .def("magnitude_max", &EdgeSourceI::magnitudeMax, "Get the maximum magnitude value.")
      .def("direction", &EdgeSourceI::direction, "Get the edge direction map.")
      .def("direction_range", &EdgeSourceI::directionRange, "Get the valid range of direction values.")
      .def("direction_map", &EdgeSourceI::directionMap, "Get the quantized direction map (CV_8S).")
      .def("seeds", &EdgeSourceI::seeds, "Get seed pixel indices for edge processing.")
      .def("name", &EdgeSourceI::name, "Get the name of this edge source.")
      .def("hysteresis", &EdgeSourceI::hysteresis, "Compute hysteresis-linked direction map.")
      .def("hysteresis_binary", &EdgeSourceI::hysteresis_binary, "Compute binary hysteresis map.")
      .def("hysteresis_edgels", &EdgeSourceI::hysteresis_edgels, "Compute hysteresis-linked edgel indices.");
}

// ============================================================================
// NonMaximaSuppression binding
// ============================================================================

template <class GT, class MT, class DT>
void bind_nms(py::module_& m, const std::string& suffix) {
  using NMSType = NonMaximaSuppression<GT, MT, DT, FastNMS8<GT, MT, DT>>;
  const std::string cls = "NonMaximaSuppression" + suffix;

  py::class_<NMSType, ValueManager>(m, cls.c_str(),
                                    ("Non-maximum suppression edge detector" + suffix +
                                     ".\n\n"
                                     "Wraps NMS with threshold management and hysteresis.\n"
                                     "Parameters (via set_value):\n"
                                     "    'nms_th_low': Lower threshold (default: 0.004)\n"
                                     "    'nms_th_high': Upper threshold (default: 0.012)\n"
                                     "    'nms_border': Border to skip (default: 1)")
                                        .c_str())
      .def(py::init<double, double, int>(), py::arg("low") = 0.004, py::arg("high") = 0.012, py::arg("border") = 1,
           "Construct NMS with threshold and border parameters.")
      .def("threshold_low", static_cast<double (NMSType::*)() const>(&NMSType::thresholdLow), "Get lower threshold.")
      .def("threshold_high", static_cast<double (NMSType::*)() const>(&NMSType::thresholdHigh), "Get upper threshold.")
      .def("set_threshold", &NMSType::threshold, py::arg("low"), py::arg("high"), "Set both thresholds.")
      .def("border", static_cast<int (NMSType::*)() const>(&NMSType::border), "Get border width.")
      .def("mag_max", &NMSType::magMax, "Get the maximum magnitude from last processing.")
      .def("direction_map", &NMSType::directionMap, "Get the quantized direction map (CV_8S).")
      .def("seeds", &NMSType::seeds, "Get seed pixel indices above the upper threshold.")
      .def(
          "process",
          [](NMSType& self, const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& mag, MT low, MT high) -> MT {
            return self.process(gx, gy, mag, low, high);
          },
          py::arg("gx"), py::arg("gy"), py::arg("mag"), py::arg("low"), py::arg("high"),
          "Run NMS on gradient components and magnitude with explicit thresholds.")
      .def("hysteresis", &NMSType::hysteresis, "Compute hysteresis thresholded edge map.")
      .def("hysteresis_binary", &NMSType::hysteresisBinary, py::arg("val") = static_cast<uchar>(255),
           "Compute binary hysteresis edge map.")
      .def("hysteresis_edgels", &NMSType::hysteresis_edgels, "Get all edge pixels after hysteresis.")
      .def("name", &NMSType::name, "Get the name of this NMS operator.");
}

// ============================================================================
// EsdBase binding (abstract segment detector interface)
// ============================================================================

template <class MT>
void bind_esd_base(py::module_& m, const std::string& suffix) {
  using Base = EsdBase<MT, index_type>;
  const std::string cls = "EsdBase" + suffix;

  py::class_<Base, ValueManager>(m, cls.c_str(), ("Abstract base for edge segment detectors" + suffix + ".").c_str())
      .def("detect", static_cast<void (Base::*)(const cv::Mat&, const cv::Mat&, const IndexVector&)>(&Base::detect),
           py::arg("dir"), py::arg("mag"), py::arg("seeds"),
           "Detect edge segments from direction map, magnitude, and seeds.")
      .def("detect", static_cast<void (Base::*)(const EdgeSourceI&)>(&Base::detect), py::arg("source"),
           "Detect edge segments using an EdgeSourceI.")
      .def("points", &Base::points, "Get the detected edge points (linear indices).")
      .def("segments", &Base::segments, "Get the detected edge segments.")
      .def("name", &Base::name, "Get the name of this detector.");
}

// ============================================================================
// EsdDrawing
// ============================================================================

template <class MT>
void bind_esd_drawing(py::module_& m, const std::string& suffix) {
  using Base = EsdBase<MT, index_type>;
  using Draw = EsdDrawing<MT, 8>;
  const std::string cls = "EsdDrawing" + suffix;

  py::class_<Draw, Base>(m, cls.c_str(),
                         ("Edge segment detector using drawing-based tracing" + suffix +
                          ".\n\n"
                          "Traces along gradient directions from seed points.\n"
                          "Parameters (via set_value):\n"
                          "    'edge_min_pixels': Minimum segment length (default: 10)\n"
                          "    'edge_mag_mul': Magnitude multiplicator (default: 3)\n"
                          "    'edge_mag_th': Magnitude threshold (default: 5)")
                             .c_str())
      .def(py::init<int, float, float>(), py::arg("min_pixels") = 10, py::arg("mag_mul") = 3.0f,
           py::arg("mag_th") = 5.0f, "Construct with minimum pixels, magnitude multiplier, and threshold.")
      .def("detect", static_cast<void (Draw::*)(const cv::Mat&, const cv::Mat&, const IndexVector&)>(&Draw::detect),
           py::arg("dir"), py::arg("mag"), py::arg("seeds"),
           "Detect edge segments from direction map, magnitude, and seeds.")
      .def("detect", static_cast<void (Draw::*)(const EdgeSourceI&)>(&Draw::detect), py::arg("source"),
           "Detect edge segments using an EdgeSourceI.")
      .def("name", &Draw::name)
      .def("points", &Draw::points)
      .def("segments", &Draw::segments);
}

// ============================================================================
// EsdSimple
// ============================================================================

template <class MT>
void bind_esd_simple(py::module_& m, const std::string& suffix) {
  using Base = EsdBase<MT, index_type>;
  using Simple = EsdSimple<MT, 8>;
  const std::string cls = "EsdSimple" + suffix;

  py::class_<Simple, Base>(m, cls.c_str(),
                           ("Simple edge segment detector with basic linking" + suffix +
                            ".\n\n"
                            "Parameters (via set_value):\n"
                            "    'edge_min_pixels': Minimum segment length (default: 10)")
                               .c_str())
      .def(py::init<int>(), py::arg("min_pixels") = 10, "Construct with minimum segment length.")
      .def("detect", static_cast<void (Simple::*)(const cv::Mat&, const cv::Mat&, const IndexVector&)>(&Simple::detect),
           py::arg("dir"), py::arg("mag"), py::arg("seeds"),
           "Detect edge segments from direction map, magnitude, and seeds.")
      .def("detect", static_cast<void (Simple::*)(const EdgeSourceI&)>(&Simple::detect), py::arg("source"),
           "Detect edge segments using an EdgeSourceI.")
      .def("name", &Simple::name)
      .def("points", &Simple::points)
      .def("segments", &Simple::segments);
}

// ============================================================================
// EsdLinking
// ============================================================================

template <class MT>
void bind_esd_linking(py::module_& m, const std::string& suffix) {
  using Base = EsdBase<MT, index_type>;
  using Link = EsdLinking<MT, 8, false>;
  const std::string cls = "EsdLinking" + suffix;

  py::class_<Link, Base>(m, cls.c_str(),
                         ("Edge segment detector with continuity-optimized linking" + suffix +
                          ".\n\n"
                          "Parameters (via set_value):\n"
                          "    'edge_min_pixels': Minimum segment length (default: 10)\n"
                          "    'edge_max_gap': Maximum gap in pixels (default: 3)\n"
                          "    'edge_mag_mul': Magnitude multiplier (default: 3)\n"
                          "    'edge_mag_th': Magnitude threshold (default: 5)")
                             .c_str())
      .def(py::init<int, int, float, float>(), py::arg("min_pixels") = 10, py::arg("max_gap") = 3,
           py::arg("mag_mul") = 3.0f, py::arg("mag_th") = 5.0f, "Construct with linking parameters.")
      .def("detect", static_cast<void (Link::*)(const cv::Mat&, const cv::Mat&, const IndexVector&)>(&Link::detect),
           py::arg("dir"), py::arg("mag"), py::arg("seeds"),
           "Detect edge segments from direction map, magnitude, and seeds.")
      .def("detect", static_cast<void (Link::*)(const EdgeSourceI&)>(&Link::detect), py::arg("source"),
           "Detect edge segments using an EdgeSourceI.")
      .def("name", &Link::name)
      .def("points", &Link::points)
      .def("segments", &Link::segments);
}

// ============================================================================
// EsdPattern
// ============================================================================

template <class MT>
void bind_esd_pattern(py::module_& m, const std::string& suffix) {
  using Base = EsdBase<MT, index_type>;
  using Pat = EsdPattern<MT, 8, false>;
  const std::string cls = "EsdPattern" + suffix;

  py::class_<Pat, Base>(m, cls.c_str(),
                        ("Pattern-based edge segment detector" + suffix +
                         ".\n\n"
                         "Parameters (via set_value):\n"
                         "    'edge_min_pixels': Minimum segment length (default: 10)\n"
                         "    'edge_max_gap': Maximum gap in pixels (default: 3)\n"
                         "    'edge_mag_mul': Magnitude multiplier (default: 3)\n"
                         "    'edge_mag_th': Magnitude threshold (default: 5)\n"
                         "    'edge_pat_tol': Pattern tolerance (default: 2)")
                            .c_str())
      .def(py::init<int, int, float, float, int>(), py::arg("min_pixels") = 10, py::arg("max_gap") = 3,
           py::arg("mag_mul") = 3.0f, py::arg("mag_th") = 5.0f, py::arg("pat_tol") = 2,
           "Construct with pattern detection parameters.")
      .def("detect", static_cast<void (Pat::*)(const cv::Mat&, const cv::Mat&, const IndexVector&)>(&Pat::detect),
           py::arg("dir"), py::arg("mag"), py::arg("seeds"),
           "Detect edge segments from direction map, magnitude, and seeds.")
      .def("detect", static_cast<void (Pat::*)(const EdgeSourceI&)>(&Pat::detect), py::arg("source"),
           "Detect edge segments using an EdgeSourceI.")
      .def("patterns", &Pat::patterns, "Get the extracted primitive patterns.")
      .def("pattern_segments", &Pat::patternSegments, "Get segments composed of linked patterns.")
      .def("name", &Pat::name)
      .def("points", &Pat::points)
      .def("segments", &Pat::segments);
}

// ============================================================================
// Concrete EdgeSource bindings (EdgeSourceGRAD<GRAD, NMS>)
// ============================================================================

template <class IT, class GT, class MT, class DT>
void bind_edge_sources(py::module_& m, const std::string& suffix) {
  using NMSImpl = NonMaximaSuppression<GT, MT, DT, FastNMS8<GT, MT, DT>>;
  using SobelGrad = DerivativeGradient<IT, GT, MT, DT, SobelDerivative, Magnitude, Direction>;
  using ScharrGrad = DerivativeGradient<IT, GT, MT, DT, ScharrDerivative, Magnitude, Direction>;
  using PrewittGrad = DerivativeGradient<IT, GT, MT, DT, PrewittDerivative, Magnitude, Direction>;

  using ESSobel = EdgeSourceGRAD<SobelGrad, NMSImpl>;
  using ESScharr = EdgeSourceGRAD<ScharrGrad, NMSImpl>;
  using ESPrewitt = EdgeSourceGRAD<PrewittGrad, NMSImpl>;

  // --- EdgeSourceSobel ---
  py::class_<ESSobel, EdgeSourceI>(m, ("EdgeSourceSobel" + suffix).c_str(),
                                   ("Gradient-based edge source using Sobel + NMS" + suffix +
                                    ".\n\n"
                                    "Combines Sobel gradient with non-maximum suppression.\n"
                                    "Parameters (via set_value):\n"
                                    "    'grad_kernel_size': Kernel size (1, 3, 5, 7)\n"
                                    "    'nms_th_low': NMS lower threshold\n"
                                    "    'nms_th_high': NMS upper threshold\n"
                                    "    'es_use_dir': Use direction for NMS (0 or 1)")
                                       .c_str())
      .def(py::init<bool>(), py::arg("use_dir") = false, "Construct with optional direction-based NMS.")
      .def("process", &ESSobel::process, py::arg("img"), "Process input image to compute edge responses.")
      .def("gx", &ESSobel::gx)
      .def("gy", &ESSobel::gy)
      .def("magnitude", &ESSobel::magnitude)
      .def("magnitude_threshold", &ESSobel::magnitudeThreshold, py::arg("val"))
      .def("magnitude_max", &ESSobel::magnitudeMax)
      .def("direction", &ESSobel::direction)
      .def("direction_range", &ESSobel::directionRange)
      .def("direction_map", &ESSobel::directionMap)
      .def("seeds", &ESSobel::seeds)
      .def("name", &ESSobel::name)
      .def("results", &ESSobel::results)
      .def("hysteresis", &ESSobel::hysteresis)
      .def("hysteresis_binary", &ESSobel::hysteresis_binary)
      .def("hysteresis_edgels", &ESSobel::hysteresis_edgels);

  // --- EdgeSourceScharr ---
  py::class_<ESScharr, EdgeSourceI>(m, ("EdgeSourceScharr" + suffix).c_str(),
                                    ("Gradient-based edge source using Scharr + NMS" + suffix + ".").c_str())
      .def(py::init<bool>(), py::arg("use_dir") = false)
      .def("process", &ESScharr::process, py::arg("img"))
      .def("gx", &ESScharr::gx)
      .def("gy", &ESScharr::gy)
      .def("magnitude", &ESScharr::magnitude)
      .def("magnitude_threshold", &ESScharr::magnitudeThreshold, py::arg("val"))
      .def("magnitude_max", &ESScharr::magnitudeMax)
      .def("direction", &ESScharr::direction)
      .def("direction_range", &ESScharr::directionRange)
      .def("direction_map", &ESScharr::directionMap)
      .def("seeds", &ESScharr::seeds)
      .def("name", &ESScharr::name)
      .def("results", &ESScharr::results)
      .def("hysteresis", &ESScharr::hysteresis)
      .def("hysteresis_binary", &ESScharr::hysteresis_binary)
      .def("hysteresis_edgels", &ESScharr::hysteresis_edgels);

  // --- EdgeSourcePrewitt ---
  py::class_<ESPrewitt, EdgeSourceI>(m, ("EdgeSourcePrewitt" + suffix).c_str(),
                                     ("Gradient-based edge source using Prewitt + NMS" + suffix + ".").c_str())
      .def(py::init<bool>(), py::arg("use_dir") = false)
      .def("process", &ESPrewitt::process, py::arg("img"))
      .def("gx", &ESPrewitt::gx)
      .def("gy", &ESPrewitt::gy)
      .def("magnitude", &ESPrewitt::magnitude)
      .def("magnitude_threshold", &ESPrewitt::magnitudeThreshold, py::arg("val"))
      .def("magnitude_max", &ESPrewitt::magnitudeMax)
      .def("direction", &ESPrewitt::direction)
      .def("direction_range", &ESPrewitt::directionRange)
      .def("direction_map", &ESPrewitt::directionMap)
      .def("seeds", &ESPrewitt::seeds)
      .def("name", &ESPrewitt::name)
      .def("results", &ESPrewitt::results)
      .def("hysteresis", &ESPrewitt::hysteresis)
      .def("hysteresis_binary", &ESPrewitt::hysteresis_binary)
      .def("hysteresis_edgels", &ESPrewitt::hysteresis_edgels);
}

// ============================================================================
// Convenience preset binder
// ============================================================================

template <class IT, class GT, class MT, class DT>
void bind_edge_preset(py::module_& m, const std::string& suffix) {
  bind_edge_source_interface(m, suffix);
  bind_nms<GT, MT, DT>(m, suffix);
  bind_esd_base<MT>(m, suffix);
  bind_esd_drawing<MT>(m, suffix);
  bind_esd_simple<MT>(m, suffix);
  bind_esd_linking<MT>(m, suffix);
  bind_esd_pattern<MT>(m, suffix);
  bind_edge_sources<IT, GT, MT, DT>(m, suffix);
}

// ============================================================================
// Explicit template instantiations for supported presets
// ============================================================================

// Default: 8-bit unsigned (uchar) — suffix ""
template void bind_nms<short, float, float>(py::module_&, const std::string&);
template void bind_esd_base<float>(py::module_&, const std::string&);
template void bind_esd_drawing<float>(py::module_&, const std::string&);
template void bind_esd_simple<float>(py::module_&, const std::string&);
template void bind_esd_linking<float>(py::module_&, const std::string&);
template void bind_esd_pattern<float>(py::module_&, const std::string&);
template void bind_edge_sources<uchar, short, float, float>(py::module_&, const std::string&);
template void bind_edge_preset<uchar, short, float, float>(py::module_&, const std::string&);

// 16-bit unsigned (ushort) — suffix "_16u"
template void bind_nms<float, float, float>(py::module_&, const std::string&);
template void bind_edge_sources<ushort, float, float, float>(py::module_&, const std::string&);
template void bind_edge_preset<ushort, float, float, float>(py::module_&, const std::string&);

// Float (32-bit) — suffix "_f32"
template void bind_edge_sources<float, float, float, float>(py::module_&, const std::string&);
template void bind_edge_preset<float, float, float, float>(py::module_&, const std::string&);

// Double (64-bit) — suffix "_f64"
template void bind_nms<double, double, double>(py::module_&, const std::string&);
template void bind_esd_base<double>(py::module_&, const std::string&);
template void bind_esd_drawing<double>(py::module_&, const std::string&);
template void bind_esd_simple<double>(py::module_&, const std::string&);
template void bind_esd_linking<double>(py::module_&, const std::string&);
template void bind_esd_pattern<double>(py::module_&, const std::string&);
template void bind_edge_sources<double, double, double, double>(py::module_&, const std::string&);
template void bind_edge_preset<double, double, double, double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
