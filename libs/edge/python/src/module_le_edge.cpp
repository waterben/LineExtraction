/// @file module_le_edge.cpp
/// @brief pybind11 module definition for le_edge.
///
/// This is the entry point for the le_edge Python extension module.
/// It registers all edge detection bindings: core types (EdgeSegment, enums),
/// the EdgeSourceI interface, NMS, edge segment detectors (Drawing, Simple,
/// Linking, Pattern), and concrete EdgeSource implementations for multiple
/// pixel type presets.
///
/// ## Presets
///
///   | Suffix | IT     | GT     | MT     | DT     | Python example               |
///   |--------|--------|--------|--------|--------|------------------------------|
///   | (none) | uchar  | short  | float  | float  | EdgeSourceSobel()            |
///   | _16u   | ushort | float  | float  | float  | EdgeSourceSobel_16u()        |
///   | _f32   | float  | float  | float  | float  | EdgeSourceSobel_f32()        |
///   | _f64   | double | double | double | double | EdgeSourceSobel_f64()        |

#include "edge_binding.hpp"
#include <opencv2/core.hpp>
#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

PYBIND11_MODULE(le_edge, m) {
  // Import le_imgproc so that ValueManager, Value, Range, FilterData
  // type bindings are available as base classes for edge types.
  py::module_::import("le_imgproc");

  m.doc() =
      "LineExtraction edge detection module.\n\n"
      "Provides Python bindings for edge detection interfaces and concrete\n"
      "implementations including edge sources (Sobel, Scharr, Prewitt + NMS),\n"
      "non-maximum suppression, and edge segment detectors (Drawing, Simple,\n"
      "Linking, Pattern).\n\n"
      "Multiple pixel type presets are available:\n\n"
      "  - Default (uint8):  EdgeSourceSobel, EsdDrawing, ...\n"
      "  - 16-bit unsigned:  EdgeSourceSobel_16u, EsdDrawing_16u, ...\n"
      "  - 32-bit float:     EdgeSourceSobel_f32, EsdDrawing_f32, ...\n"
      "  - 64-bit double:    EdgeSourceSobel_f64, EsdDrawing_f64, ...\n\n"
      "Core types:\n"
      "    EdgeSegment           - Edge segment referencing point indices\n"
      "    ES_NONE, ES_REVERSE, ES_CLOSED  - Segment flags\n"
      "    ESDirectionOptions    - Direction options enum\n"
      "    ESQuadratureOptions   - Quadrature options enum\n\n"
      "Example:\n"
      "    import numpy as np\n"
      "    import le_edge\n\n"
      "    # Load and process a grayscale image\n"
      "    img = np.zeros((100, 100), dtype=np.uint8)\n"
      "    img[30:70, 30:70] = 255\n\n"
      "    # Create edge source (Sobel + NMS)\n"
      "    es = le_edge.EdgeSourceSobel()\n"
      "    es.process(img)\n\n"
      "    # Detect edge segments\n"
      "    esd = le_edge.EsdDrawing()\n"
      "    esd.detect(es)\n"
      "    segments = esd.segments()\n"
      "    points = esd.points()";

  // ---- Core types (EdgeSegment, enums) ----
  lsfm::python::bind_edge_core_types(m);

  // ---- EdgeSourceI interface (registered once) ----
  lsfm::python::bind_edge_source_interface(m, "");

  // ---- NMS: one registration per unique C++ type ----
  // NMS<short,float,float>  -> default (uchar input)
  // NMS<float,float,float>  -> used by _16u and _f32 presets
  // NMS<double,double,double> -> _f64
  lsfm::python::bind_nms<short, float, float>(m, "");
  lsfm::python::bind_nms<float, float, float>(m, "_16u");
  lsfm::python::bind_nms<double, double, double>(m, "_f64");
  // Alias: _f32 uses same NMS C++ type as _16u
  m.attr("NonMaximaSuppression_f32") = m.attr("NonMaximaSuppression_16u");

  // ---- ESD types: one registration per unique MT ----
  // MT=float  -> default (also covers _16u and _f32)
  // MT=double -> _f64
  lsfm::python::bind_esd_base<float>(m, "");
  lsfm::python::bind_esd_drawing<float>(m, "");
  lsfm::python::bind_esd_simple<float>(m, "");
  lsfm::python::bind_esd_linking<float>(m, "");
  lsfm::python::bind_esd_pattern<float>(m, "");

  lsfm::python::bind_esd_base<double>(m, "_f64");
  lsfm::python::bind_esd_drawing<double>(m, "_f64");
  lsfm::python::bind_esd_simple<double>(m, "_f64");
  lsfm::python::bind_esd_linking<double>(m, "_f64");
  lsfm::python::bind_esd_pattern<double>(m, "_f64");

  // Aliases: _16u and _f32 ESD types are the same C++ type as default
  for (const char* suffix : {"_16u", "_f32"}) {
    m.attr(("EsdBase" + std::string(suffix)).c_str()) = m.attr("EsdBase");
    m.attr(("EsdDrawing" + std::string(suffix)).c_str()) = m.attr("EsdDrawing");
    m.attr(("EsdSimple" + std::string(suffix)).c_str()) = m.attr("EsdSimple");
    m.attr(("EsdLinking" + std::string(suffix)).c_str()) = m.attr("EsdLinking");
    m.attr(("EsdPattern" + std::string(suffix)).c_str()) = m.attr("EsdPattern");
  }

  // ---- Edge sources: all 4 presets are unique C++ types (different IT) ----
  lsfm::python::bind_edge_sources<uchar, short, float, float>(m, "");
  lsfm::python::bind_edge_sources<ushort, float, float, float>(m, "_16u");
  lsfm::python::bind_edge_sources<float, float, float, float>(m, "_f32");
  lsfm::python::bind_edge_sources<double, double, double, double>(m, "_f64");
}
