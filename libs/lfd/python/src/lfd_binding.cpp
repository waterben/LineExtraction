/// @file lfd_binding.cpp
/// @brief Templated pybind11 bindings for the LFD (Line Feature Descriptor) library.
///
/// Implements the binding functions declared in lfd_binding.hpp and provides
/// explicit template instantiations for the standard presets:
///
///   | Suffix  | FT     |
///   |---------|--------|
///   | (none)  | float  |
///   | _f64    | double |

#include "lfd_binding.hpp"

#include <cvnp/cvnp.h>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <imgproc/interpolate.hpp>
#include <lfd/FeatureDescriptor.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/FeatureDescriptorOpenCVLBD.hpp>
#include <lfd/FeatureFilter.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <lfd/GenericDescriptor.hpp>
#include <lfd/GlobalRotationFilter.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Convenience type aliases to keep binding code readable
// ============================================================================

/// @brief Default LineSegment vector type used throughout LFD.
template <class FT>
using LSVec = std::vector<LineSegment<FT>>;

/// @brief LBD descriptor type alias.
template <class FT>
using LBD = FdLBD<FT>;

/// @brief LBD descriptor vector.
template <class FT>
using LBDVec = std::vector<FdLBD<FT>>;

/// @brief LBD descriptor creator with float-type gradients matching Python usage.
template <class FT>
using LBDCreator = FdcLBD<FT, LineSegment<FT>, FT, RoundNearestInterpolator<FT, FT>>;

/// @brief Default GchGradImgInterpolate helper type with float-typed data.
template <class FT>
using GradImgHelper = GchGradImgInterpolate<FT,
                                            3,
                                            2,
                                            RotationAlign<FT>,
                                            FastRoundNearestInterpolator<FT, FT>,
                                            FastRoundNearestInterpolator<FT, FT>>;

/// @brief LR descriptor type for the default helper.
template <class FT>
using LRDsc = LRDescritpor<FT, GradImgHelper<FT>::dscSize>;

/// @brief LR descriptor vector.
template <class FT>
using LRDscVec = std::vector<LRDsc<FT>>;

/// @brief LR descriptor creator with default helper.
template <class FT>
using LRCreator = FdcGenericLR<FT, LineSegment<FT>, GradImgHelper<FT>>;

/// @brief BruteForce matcher for LBD descriptors.
template <class FT>
using BFMatcherLBD = FmBruteForce<FT, LBD<FT>>;

/// @brief BruteForce matcher for LR descriptors.
template <class FT>
using BFMatcherLR = FmBruteForce<FT, LRDsc<FT>>;

/// @brief OpenCV Binary LBD descriptor type alias.
template <class FT>
using OCVLBD = FdOpenCVLBD<FT>;

/// @brief OpenCV Binary LBD descriptor vector.
template <class FT>
using OCVLBDVec = std::vector<FdOpenCVLBD<FT>>;

/// @brief OpenCV Binary LBD descriptor creator.
template <class FT>
using OCVLBDCreator = FdcOpenCVLBD<FT, LineSegment<FT>>;

/// @brief BruteForce matcher for OpenCV Binary LBD descriptors.
template <class FT>
using BFMatcherOCVLBD = FmBruteForce<FT, OCVLBD<FT>>;

/// @brief DescriptorMatch type alias.
template <class FT>
using DMatch = DescriptorMatch<FT>;

/// @brief DescriptorMatch vector.
template <class FT>
using DMatchVec = std::vector<DMatch<FT>>;

/// @brief FeatureMatch type alias.
template <class FT>
using FMatch = FeatureMatch<FT>;

/// @brief FeatureMatch vector.
template <class FT>
using FMatchVec = std::vector<FMatch<FT>>;

/// @brief GlobalRotationFilter for LineSegment vectors.
template <class FT>
using GRFilter = GlobalRotationFilter<FT, LSVec<FT>>;

/// @brief StereoLineFilter for LineSegment vectors.
template <class FT>
using SLFilter = StereoLineFilter<FT, LSVec<FT>>;

/// @brief StereoLineMatcher for LineSegment vectors with default helpers.
template <class FT>
using SLMatcher = StereoLineMatcher<FT, LSVec<FT>, GradImgHelper<FT>>;

// ============================================================================
// Core type bindings (non-templated, called once)
// ============================================================================

void bind_lfd_core_types(py::module_& m) {
  m.attr("FS_NONE") = static_cast<int>(FS_NONE);
  m.attr("FS_MASKED") = static_cast<int>(FS_MASKED);
}

// ============================================================================
// Match types
// ============================================================================

template <class FT>
void bind_lfd_match_types(py::module_& m, const std::string& suffix) {
  // --- FeatureMatch ---
  {
    const std::string cls = "FeatureMatch" + suffix;
    py::class_<FMatch<FT>>(m, cls.c_str(), ("Feature match pair with query/match indices" + suffix + ".").c_str())
        .def(py::init<int, int, int>(), py::arg("query_idx") = -1, py::arg("match_idx") = -1,
             py::arg("filter_state") = 0, "Construct a feature match.")
        .def_readwrite("query_idx", &FMatch<FT>::queryIdx, "Query descriptor index.")
        .def_readwrite("match_idx", &FMatch<FT>::matchIdx, "Matched descriptor index.")
        .def_readwrite("filter_state", &FMatch<FT>::filterState, "Filter state (FS_NONE or FS_MASKED).")
        .def("__repr__", [](const FMatch<FT>& fm) {
          return "FeatureMatch(q=" + std::to_string(fm.queryIdx) + ", m=" + std::to_string(fm.matchIdx) +
                 ", fs=" + std::to_string(fm.filterState) + ")";
        });
  }

  // --- DescriptorMatch ---
  {
    const std::string cls = "DescriptorMatch" + suffix;
    py::class_<DMatch<FT>, FMatch<FT>>(m, cls.c_str(), ("Descriptor match with distance metric" + suffix + ".").c_str())
        .def(py::init<int, int, int, FT>(), py::arg("query_idx") = -1, py::arg("match_idx") = -1,
             py::arg("filter_state") = 0, py::arg("distance") = std::numeric_limits<FT>::max(),
             "Construct a descriptor match.")
        .def(py::init<int, int, FT>(), py::arg("query_idx"), py::arg("match_idx"), py::arg("distance"),
             "Construct with indices and distance.")
        .def_readwrite("distance", &DMatch<FT>::distance, "Distance between descriptors.")
        .def("__lt__", [](const DMatch<FT>& a, const DMatch<FT>& b) { return a < b; })
        .def("__repr__", [](const DMatch<FT>& dm) {
          return "DescriptorMatch(q=" + std::to_string(dm.queryIdx) + ", m=" + std::to_string(dm.matchIdx) +
                 ", d=" + std::to_string(dm.distance) + ")";
        });
  }
}

// ============================================================================
// Descriptor types
// ============================================================================

template <class FT>
void bind_lfd_descriptor_types(py::module_& m, const std::string& suffix) {
  // --- FdMat ---
  {
    const std::string cls = "FdMat" + suffix;
    py::class_<FdMat<FT>>(m, cls.c_str(),
                          ("Simple cv::Mat-based feature descriptor with L2 distance" + suffix + ".").c_str())
        .def(py::init<>(), "Default constructor.")
        .def(py::init<const cv::Mat&>(), py::arg("data"), "Construct from descriptor matrix.")
        .def_readwrite("data", &FdMat<FT>::data, "Descriptor data as numpy array.")
        .def("distance", &FdMat<FT>::distance, py::arg("other"), "Compute L2 distance to another descriptor.")
        .def("name", &FdMat<FT>::name, "Get descriptor type name.")
        .def("__repr__", [](const FdMat<FT>& d) { return "FdMat(cols=" + std::to_string(d.data.cols) + ")"; });
  }

  // --- FdLBD ---
  {
    const std::string cls = "FdLBD" + suffix;
    py::class_<LBD<FT>>(m, cls.c_str(), ("LBD (Line Band Descriptor) with L2 distance" + suffix + ".").c_str())
        .def(py::init<>(), "Default constructor.")
        .def(py::init<const cv::Mat&>(), py::arg("data"), "Construct from descriptor matrix.")
        .def_readwrite("data", &LBD<FT>::data, "Descriptor data as numpy array.")
        .def("distance", static_cast<FT (LBD<FT>::*)(const LBD<FT>&) const>(&LBD<FT>::distance), py::arg("other"),
             "Compute L2 distance to another LBD descriptor.")
        .def("name", &LBD<FT>::name, "Get descriptor type name ('LBD').")
        .def("__repr__", [](const LBD<FT>& d) { return "FdLBD(cols=" + std::to_string(d.data.cols) + ")"; });
  }

  // --- LRDescritpor ---
  {
    const std::string cls = "LRDescriptor" + suffix;
    constexpr int lr_size = GradImgHelper<FT>::dscSize * 2;
    py::class_<LRDsc<FT>>(m, cls.c_str(), ("Left-Right descriptor with L*R distance metric" + suffix + ".").c_str())
        .def(py::init<>(), "Default constructor.")
        .def_property_readonly(
            "data",
            [](LRDsc<FT>& self) {
              // Expose raw FT[cn*2] array as a 1-D numpy view (no copy).
              return py::array_t<FT>({lr_size}, {sizeof(FT)}, self.data, py::cast(self));
            },
            "Descriptor data as a numpy array view.")
        .def("distance", static_cast<FT (LRDsc<FT>::*)(const LRDsc<FT>&) const>(&LRDsc<FT>::distance), py::arg("other"),
             "Compute L*R distance to another LR descriptor.")
        .def("name", &LRDsc<FT>::name, "Get descriptor type name.")
        .def("__repr__",
             [](const LRDsc<FT>&) { return std::string("LRDescriptor(size=") + std::to_string(lr_size) + ")"; });
  }

  // --- FdOpenCVLBD ---
  {
    const std::string cls = "FdOpenCVLBD" + suffix;
    py::class_<OCVLBD<FT>>(m, cls.c_str(),
                           ("OpenCV Binary LBD descriptor with Hamming distance" + suffix +
                            ".\n\n"
                            "256-bit binary descriptor (32 bytes) from OpenCV's BinaryDescriptor.\n"
                            "Uses Hamming distance for matching.")
                               .c_str())
        .def(py::init<>(), "Default constructor.")
        .def(py::init<const cv::Mat&>(), py::arg("data"), "Construct from binary descriptor matrix (1x32, CV_8U).")
        .def_readwrite("data", &OCVLBD<FT>::data, "Binary descriptor data as numpy array (1x32, uint8).")
        .def("distance", static_cast<FT (OCVLBD<FT>::*)(const OCVLBD<FT>&) const>(&OCVLBD<FT>::distance),
             py::arg("other"), "Compute Hamming distance to another OpenCV LBD descriptor.")
        .def("name", &OCVLBD<FT>::name, "Get descriptor type name ('OpenCVLBD').")
        .def("__repr__", [](const OCVLBD<FT>& d) {
          return "FdOpenCVLBD(cols=" + std::to_string(d.data.cols) + ", empty=" + (d.data.empty() ? "true" : "false") +
                 ")";
        });
  }
}

// ============================================================================
// Descriptor creators
// ============================================================================

template <class FT>
void bind_lfd_descriptor_creators(py::module_& m, const std::string& suffix) {
  // --- FdcLBD ---
  {
    const std::string cls = "FdcLBD" + suffix;
    py::class_<LBDCreator<FT>>(m, cls.c_str(),
                               ("LBD descriptor creator from gradient images" + suffix +
                                ".\n\n"
                                "Computes LBD descriptors for line segments using gradient band\n"
                                "projection with Gaussian weighting.\n\n"
                                "Parameters (via set_value):\n"
                                "    'num_band': Number of bands (default 9)\n"
                                "    'width_band': Width of each band (default 7)")
                                   .c_str())
        .def(py::init<const cv::Mat&, const cv::Mat&, unsigned short, unsigned short>(), py::arg("dx"), py::arg("dy"),
             py::arg("num_band") = static_cast<unsigned short>(9),
             py::arg("width_band") = static_cast<unsigned short>(7), "Construct from gradient images.")
        .def(py::init([](const std::map<std::string, cv::Mat>& data, unsigned short nb, unsigned short wb) {
               MatMap mm(data.begin(), data.end());
               return std::make_unique<LBDCreator<FT>>(mm, nb, wb);
             }),
             py::arg("data"), py::arg("num_band") = static_cast<unsigned short>(9),
             py::arg("width_band") = static_cast<unsigned short>(7),
             "Construct from a dict of gradient matrices (keys: 'dx'/'gx', 'dy'/'gy').")
        .def(
            "create_mat",
            [](LBDCreator<FT>& self, const LSVec<FT>& lines) {
              cv::Mat dst;
              self.createMat(lines, dst);
              return dst;
            },
            py::arg("lines"), "Create descriptors as a matrix (rows = descriptors).")
        .def(
            "create_list",
            [](LBDCreator<FT>& self, const LSVec<FT>& lines) {
              LBDVec<FT> dst;
              self.createList(lines, dst);
              return dst;
            },
            py::arg("lines"), "Create descriptors as a list of FdLBD objects.")
        .def(
            "create_list_masked",
            [](LBDCreator<FT>& self, const LSVec<FT>& lines, const std::vector<int>& mask) {
              LBDVec<FT> dst;
              self.createList(lines, mask, dst);
              return dst;
            },
            py::arg("lines"), py::arg("mask"), "Create descriptors with inverted mask (!=0 means included).")
        .def(
            "create_single",
            [](LBDCreator<FT>& self, const LineSegment<FT>& line) {
              LBD<FT> dst;
              self.create(line, dst);
              return dst;
            },
            py::arg("line"), "Create a single LBD descriptor for one line segment.")
        .def("size", &LBDCreator<FT>::size, "Get descriptor dimension.")
        .def(
            "set_data",
            [](LBDCreator<FT>& self, const std::map<std::string, cv::Mat>& data) {
              MatMap mm(data.begin(), data.end());
              self.setData(mm);
            },
            py::arg("data"), "Update gradient data (keys: 'dx'/'gx', 'dy'/'gy').");
  }

  // --- FdcGenericLR (with GchGradImgInterpolate helper) ---
  {
    const std::string cls = "FdcGenericLR" + suffix;
    py::class_<LRCreator<FT>>(m, cls.c_str(),
                              ("Left-Right descriptor creator using gradient+image bands" + suffix +
                               ".\n\n"
                               "Creates LR descriptors by sampling features on both sides of\n"
                               "a line segment using gradient and image intensity bands.")
                                  .c_str())
        .def(py::init([](const std::map<std::string, cv::Mat>& data, FT pos, FT step_dir, FT lstep) {
               for (const char* key : {"gx", "gy", "img"}) {
                 if (data.find(key) == data.end())
                   throw std::invalid_argument(std::string("FdcGenericLR: missing required key '") + key + "'");
               }
               MatMap mm(data.begin(), data.end());
               return std::make_unique<LRCreator<FT>>(mm, pos, step_dir, lstep);
             }),
             py::arg("data"), py::arg("pos") = static_cast<FT>(-1), py::arg("step_dir") = static_cast<FT>(1),
             py::arg("lstep") = static_cast<FT>(1), "Construct from dict of matrices (keys: 'gx', 'gy', 'img').")
        .def(
            "create_list",
            [](LRCreator<FT>& self, const LSVec<FT>& lines) {
              LRDscVec<FT> dst;
              self.createList(lines, dst);
              return dst;
            },
            py::arg("lines"), "Create LR descriptors for all line segments.")
        .def(
            "create_list_masked",
            [](LRCreator<FT>& self, const LSVec<FT>& lines, const std::vector<int>& mask) {
              LRDscVec<FT> dst;
              self.createList(lines, mask, dst);
              return dst;
            },
            py::arg("lines"), py::arg("mask"), "Create LR descriptors with inverted mask.")
        .def(
            "create_mat",
            [](LRCreator<FT>& self, const LSVec<FT>& lines) {
              cv::Mat dst;
              self.createMat(lines, dst);
              return dst;
            },
            py::arg("lines"), "Create descriptors as a matrix (rows = descriptors).")
        .def("size", &LRCreator<FT>::size, "Get descriptor dimension.")
        .def(
            "set_data",
            [](LRCreator<FT>& self, const std::map<std::string, cv::Mat>& data) {
              MatMap mm(data.begin(), data.end());
              self.setData(mm);
            },
            py::arg("data"), "Update input data (keys: 'gx', 'gy', 'img').");
  }

  // --- FdcOpenCVLBD ---
  {
    const std::string cls = "FdcOpenCVLBD" + suffix;
    py::class_<OCVLBDCreator<FT>>(m, cls.c_str(),
                                  ("OpenCV Binary LBD descriptor creator" + suffix +
                                   ".\n\n"
                                   "Wraps OpenCV's BinaryDescriptor to compute 256-bit binary\n"
                                   "LBD descriptors from line segments. Accepts segments from\n"
                                   "any detector (e.g., our LSD) and converts them to KeyLine\n"
                                   "format internally.\n\n"
                                   "Uses Hamming distance for matching. Much more distinctive\n"
                                   "than the float-based LBD for most use cases.")
                                      .c_str())
        .def(py::init<const cv::Mat&>(), py::arg("image"), "Construct from a grayscale image (CV_8U).")
        .def(py::init([](const std::map<std::string, cv::Mat>& data) {
               if (data.find("img") == data.end() && data.find("image") == data.end())
                 throw std::invalid_argument("FdcOpenCVLBD: missing required key 'img' or 'image'");
               MatMap mm(data.begin(), data.end());
               return std::make_unique<OCVLBDCreator<FT>>(mm);
             }),
             py::arg("data"), "Construct from dict of matrices (key: 'img' or 'image').")
        .def(
            "create_list",
            [](OCVLBDCreator<FT>& self, const LSVec<FT>& lines) {
              OCVLBDVec<FT> dst;
              self.createAll(lines, dst);
              return dst;
            },
            py::arg("lines"),
            "Create binary LBD descriptors for all line segments.\n\n"
            "Note: OpenCV may drop some keylines (too short or near borders).\n"
            "Dropped segments receive empty descriptors (max distance).")
        .def(
            "create_single",
            [](OCVLBDCreator<FT>& self, const LineSegment<FT>& line) {
              FdOpenCVLBD<FT> dst;
              self.create(line, dst);
              return dst;
            },
            py::arg("line"), "Create a single binary LBD descriptor for one line segment.")
        .def("size", &OCVLBDCreator<FT>::size, "Get descriptor size in bytes (32 = 256 bits).")
        .def(
            "set_data",
            [](OCVLBDCreator<FT>& self, const std::map<std::string, cv::Mat>& data) {
              MatMap mm(data.begin(), data.end());
              self.setData(mm);
            },
            py::arg("data"), "Update image data (key: 'img' or 'image').");
  }
}

// ============================================================================
// Filters
// ============================================================================

template <class FT>
void bind_lfd_filters(py::module_& m, const std::string& suffix) {
  // --- GlobalRotationFilter ---
  {
    const std::string cls = "GlobalRotationFilter" + suffix;
    py::class_<GRFilter<FT>>(m, cls.c_str(),
                             ("Global rotation-based feature filter" + suffix +
                              ".\n\n"
                              "Estimates approximate global rotation between two line sets\n"
                              "using angle/length histograms, then filters matches by\n"
                              "rotation and length consistency.")
                                 .c_str())
        .def(py::init<>(), "Construct with default parameters.")
        .def("train", &GRFilter<FT>::train, py::arg("left"), py::arg("right"),
             "Estimate global rotation from two line sets.")
        .def("filter", static_cast<bool (GRFilter<FT>::*)(int, int) const>(&GRFilter<FT>::filter), py::arg("left_idx"),
             py::arg("right_idx"), "Check if a match should be rejected (True = reject).")
        .def(
            "create_matches",
            [](const GRFilter<FT>& self, size_t l_size, size_t r_size) {
              FMatchVec<FT> matches;
              self.create(l_size, r_size, matches);
              return matches;
            },
            py::arg("left_size"), py::arg("right_size"), "Generate all non-filtered match candidates.")
        .def(
            "create_matches_with_masks",
            [](const GRFilter<FT>& self, size_t l_size, size_t r_size) {
              FMatchVec<FT> matches;
              std::vector<int> lm, rm;
              self.create(l_size, r_size, matches, lm, rm);
              return py::make_tuple(matches, lm, rm);
            },
            py::arg("left_size"), py::arg("right_size"),
            "Generate candidates and inverted masks. Returns (matches, left_mask, right_mask).")
        .def(
            "filter_list",
            [](const GRFilter<FT>& self, FMatchVec<FT>& matches) {
              self.filterList(matches);
              return matches;
            },
            py::arg("matches"), "Apply filter to a list of matches (updates filter_state in place).");
  }

  // --- StereoLineFilter ---
  {
    const std::string cls = "StereoLineFilter" + suffix;
    py::class_<SLFilter<FT>>(m, cls.c_str(),
                             ("Stereo line filter with geometric constraints" + suffix +
                              ".\n\n"
                              "Filters stereo line matches using spatial binning, x-sorting,\n"
                              "angle similarity, and Y overlap constraints.\n\n"
                              "Parameters:\n"
                              "    height: Image height\n"
                              "    max_dis_px: Max endpoint distance (default 10000)\n"
                              "    angle_th: Max angle difference in degrees (default 45)\n"
                              "    min_y_overlap: Min Y overlap ratio 0-1 (default 0.5)")
                                 .c_str())
        .def(py::init<int, FT, FT, FT>(), py::arg("height"), py::arg("max_dis_px") = static_cast<FT>(10000),
             py::arg("angle_th") = static_cast<FT>(45), py::arg("min_y_overlap") = static_cast<FT>(0.5),
             "Construct with image height and optional thresholds.")
        .def("train", &SLFilter<FT>::train, py::arg("left"), py::arg("right"),
             "Train filter with left and right line sets.")
        .def("filter", static_cast<bool (SLFilter<FT>::*)(int, int) const>(&SLFilter<FT>::filter), py::arg("left_idx"),
             py::arg("right_idx"), "Check if a match should be rejected (True = reject).")
        .def(
            "create_matches",
            [](const SLFilter<FT>& self, size_t l_size, size_t r_size) {
              FMatchVec<FT> matches;
              self.create(l_size, r_size, matches);
              return matches;
            },
            py::arg("left_size"), py::arg("right_size"), "Generate all non-filtered match candidates.")
        .def(
            "create_matches_with_masks",
            [](const SLFilter<FT>& self, size_t l_size, size_t r_size) {
              FMatchVec<FT> matches;
              std::vector<int> lm, rm;
              self.create(l_size, r_size, matches, lm, rm);
              return py::make_tuple(matches, lm, rm);
            },
            py::arg("left_size"), py::arg("right_size"),
            "Generate candidates and inverted masks. Returns (matches, left_mask, right_mask).")
        .def(
            "filter_list",
            [](const SLFilter<FT>& self, FMatchVec<FT>& matches) {
              self.filterList(matches);
              return matches;
            },
            py::arg("matches"), "Apply filter to a list of matches.");
  }
}

// ============================================================================
// Matchers
// ============================================================================

template <class FT>
void bind_lfd_matchers(py::module_& m, const std::string& suffix) {
  // --- FmBruteForce for LBD ---
  {
    const std::string cls = "BruteForceLBD" + suffix;
    py::class_<BFMatcherLBD<FT>>(m, cls.c_str(),
                                 ("Brute force descriptor matcher for LBD descriptors" + suffix +
                                  ".\n\n"
                                  "Computes distances between all query and candidate LBD\n"
                                  "descriptors with support for k-NN, radius search, and\n"
                                  "filter-based matching.\n\n"
                                  "Parameters:\n"
                                  "    radius: Max distance (0 = no limit)\n"
                                  "    k: Number of nearest neighbors (0 = all)")
                                     .c_str())
        .def(py::init<FT, int>(), py::arg("radius") = static_cast<FT>(0), py::arg("k") = 0,
             "Construct with optional radius and k parameters.")
        // train (basic)
        .def(
            "train", [](BFMatcherLBD<FT>& self, const LBDVec<FT>& q, const LBDVec<FT>& m_dsc) { self.train(q, m_dsc); },
            py::arg("query"), py::arg("match"), "Compute distance graph between query and match descriptors.")
        // train with masks
        .def(
            "train_masked",
            [](BFMatcherLBD<FT>& self, const LBDVec<FT>& q, const LBDVec<FT>& m_dsc, const std::vector<int>& qm,
               const std::vector<int>& mm) { self.train(q, m_dsc, qm, mm); },
            py::arg("query"), py::arg("match"), py::arg("query_mask"), py::arg("match_mask"),
            "Train with inverted masks (!=0 means included).")
        // train with filter
        .def(
            "train_filtered",
            [](BFMatcherLBD<FT>& self, const LBDVec<FT>& q, const LBDVec<FT>& m_dsc, const GRFilter<FT>& ff) {
              self.train(q, m_dsc, static_cast<const FeatureFilter<FT>&>(ff));
            },
            py::arg("query"), py::arg("match"), py::arg("filter"), "Train with a GlobalRotationFilter applied.")
        .def(
            "train_filtered_stereo",
            [](BFMatcherLBD<FT>& self, const LBDVec<FT>& q, const LBDVec<FT>& m_dsc, const SLFilter<FT>& ff) {
              self.train(q, m_dsc, static_cast<const FeatureFilter<FT>&>(ff));
            },
            py::arg("query"), py::arg("match"), py::arg("filter"), "Train with a StereoLineFilter applied.")
        // match (train + return)
        .def(
            "match",
            [](BFMatcherLBD<FT>& self, const LBDVec<FT>& q, const LBDVec<FT>& m_dsc) { return self.match(q, m_dsc); },
            py::arg("query"), py::arg("match"), py::return_value_policy::copy, "Train and return all matches.")
        // best
        .def(
            "best",
            [](BFMatcherLBD<FT>& self) {
              DMatchVec<FT> result;
              self.best(result);
              return result;
            },
            "Get the single best match per query descriptor.")
        // knn
        .def(
            "knn",
            [](BFMatcherLBD<FT>& self, int k) {
              DMatchVec<FT> result;
              self.knn(result, k);
              return result;
            },
            py::arg("k"), "Get k nearest neighbors per query descriptor.")
        // radius
        .def(
            "radius",
            [](BFMatcherLBD<FT>& self, FT r) {
              DMatchVec<FT> result;
              self.radius(result, r);
              return result;
            },
            py::arg("radius"), "Get matches within radius for each query descriptor.")
        // static best match
        .def_static(
            "match_best",
            [](const LBDVec<FT>& q, const LBDVec<FT>& m_dsc) {
              DMatchVec<FT> result;
              BFMatcherLBD<FT>::match(q, m_dsc, result);
              return result;
            },
            py::arg("query"), py::arg("match"), "Find single best match per query (static, no graph stored).");
  }

  // --- FmBruteForce for LR descriptors ---
  {
    const std::string cls = "BruteForceLR" + suffix;
    py::class_<BFMatcherLR<FT>>(m, cls.c_str(),
                                ("Brute force descriptor matcher for LR descriptors" + suffix + ".").c_str())
        .def(py::init<FT, int>(), py::arg("radius") = static_cast<FT>(0), py::arg("k") = 0,
             "Construct with optional radius and k parameters.")
        .def(
            "train",
            [](BFMatcherLR<FT>& self, const LRDscVec<FT>& q, const LRDscVec<FT>& m_dsc) { self.train(q, m_dsc); },
            py::arg("query"), py::arg("match"), "Compute distance graph between query and match descriptors.")
        .def(
            "match",
            [](BFMatcherLR<FT>& self, const LRDscVec<FT>& q, const LRDscVec<FT>& m_dsc) {
              return self.match(q, m_dsc);
            },
            py::arg("query"), py::arg("match"), py::return_value_policy::copy, "Train and return all matches.")
        .def(
            "best",
            [](BFMatcherLR<FT>& self) {
              DMatchVec<FT> result;
              self.best(result);
              return result;
            },
            "Get the single best match per query descriptor.")
        .def(
            "knn",
            [](BFMatcherLR<FT>& self, int k) {
              DMatchVec<FT> result;
              self.knn(result, k);
              return result;
            },
            py::arg("k"), "Get k nearest neighbors per query descriptor.")
        .def(
            "radius",
            [](BFMatcherLR<FT>& self, FT r) {
              DMatchVec<FT> result;
              self.radius(result, r);
              return result;
            },
            py::arg("radius"), "Get matches within radius for each query descriptor.");
  }

  // --- FmBruteForce for OpenCV Binary LBD descriptors ---
  {
    const std::string cls = "BruteForceOpenCVLBD" + suffix;
    py::class_<BFMatcherOCVLBD<FT>>(m, cls.c_str(),
                                    ("Brute force Hamming matcher for OpenCV Binary LBD" + suffix +
                                     ".\n\n"
                                     "Matches 256-bit binary LBD descriptors using Hamming distance.")
                                        .c_str())
        .def(py::init<FT, int>(), py::arg("radius") = static_cast<FT>(0), py::arg("k") = 0,
             "Construct with optional radius and k parameters.")
        .def(
            "train",
            [](BFMatcherOCVLBD<FT>& self, const OCVLBDVec<FT>& q, const OCVLBDVec<FT>& m_dsc) { self.train(q, m_dsc); },
            py::arg("query"), py::arg("match"), "Compute Hamming distance graph between query and match descriptors.")
        .def(
            "match",
            [](BFMatcherOCVLBD<FT>& self, const OCVLBDVec<FT>& q, const OCVLBDVec<FT>& m_dsc) {
              return self.match(q, m_dsc);
            },
            py::arg("query"), py::arg("match"), py::return_value_policy::copy, "Train and return all matches.")
        .def(
            "best",
            [](BFMatcherOCVLBD<FT>& self) {
              DMatchVec<FT> result;
              self.best(result);
              return result;
            },
            "Get the single best match per query descriptor.")
        .def(
            "knn",
            [](BFMatcherOCVLBD<FT>& self, int k) {
              DMatchVec<FT> result;
              self.knn(result, k);
              return result;
            },
            py::arg("k"), "Get k nearest neighbors per query descriptor.")
        .def(
            "radius",
            [](BFMatcherOCVLBD<FT>& self, FT r) {
              DMatchVec<FT> result;
              self.radius(result, r);
              return result;
            },
            py::arg("radius"), "Get matches within radius for each query descriptor.")
        .def_static(
            "match_best",
            [](const OCVLBDVec<FT>& q, const OCVLBDVec<FT>& m_dsc) {
              DMatchVec<FT> result;
              BFMatcherOCVLBD<FT>::match(q, m_dsc, result);
              return result;
            },
            py::arg("query"), py::arg("match"), "Find single best match per query (static, no graph stored).");
  }
}

// ============================================================================
// Stereo matcher
// ============================================================================

template <class FT>
void bind_lfd_stereo_matcher(py::module_& m, const std::string& suffix) {
  const std::string cls = "StereoLineMatcher" + suffix;
  py::class_<SLMatcher<FT>>(m, cls.c_str(),
                            ("Stereo line matcher with geometric filtering and LR consistency" + suffix +
                             ".\n\n"
                             "Combines StereoLineFilter for candidate generation with brute-force\n"
                             "LR descriptor matching and left-right consistency check.\n\n"
                             "Parameters:\n"
                             "    height: Image height\n"
                             "    max_dist: Max disparity in pixels (default 10000)\n"
                             "    angle_th: Max angle difference in degrees (default 5)\n"
                             "    min_y_overlap: Min Y overlap ratio 0-1 (default 0.5)\n"
                             "    dist_th: Descriptor distance threshold, 0 = auto (default 0)\n"
                             "    radius: Radius for radius matching (default 0)\n"
                             "    k: Number of nearest neighbors (default 0)")
                                .c_str())
      // Constructor without descriptor creators (for use with pre-computed descriptors)
      .def(py::init([](int height, FT max_dist, FT angle_th, FT min_y_overlap, FT dist_th, FT radius, int k) {
             return std::make_unique<SLMatcher<FT>>(typename SLMatcher<FT>::FdcPtr(), typename SLMatcher<FT>::FdcPtr(),
                                                    height, max_dist, angle_th, min_y_overlap, dist_th, radius, k);
           }),
           py::arg("height"), py::arg("max_dist") = static_cast<FT>(10000), py::arg("angle_th") = static_cast<FT>(5),
           py::arg("min_y_overlap") = static_cast<FT>(0.5), py::arg("dist_th") = static_cast<FT>(0),
           py::arg("radius") = static_cast<FT>(0), py::arg("k") = 0,
           "Construct with filter parameters only (use match() with pre-computed descriptors).")
      // Constructor with LR descriptor creators (for automatic descriptor computation)
      .def(py::init([](LRCreator<FT>& cL, LRCreator<FT>& cR, int height, FT max_dist, FT angle_th, FT min_y_overlap,
                       FT dist_th, FT radius, int k) {
             return std::make_unique<SLMatcher<FT>>(cL, cR, height, max_dist, angle_th, min_y_overlap, dist_th, radius,
                                                    k);
           }),
           py::arg("creator_left"), py::arg("creator_right"), py::arg("height"),
           py::arg("max_dist") = static_cast<FT>(10000), py::arg("angle_th") = static_cast<FT>(5),
           py::arg("min_y_overlap") = static_cast<FT>(0.5), py::arg("dist_th") = static_cast<FT>(0),
           py::arg("radius") = static_cast<FT>(0), py::arg("k") = 0,
           "Construct with LR descriptor creators for automatic matching.")
      // match with pre-computed descriptors
      .def(
          "match",
          [](SLMatcher<FT>& self, const LSVec<FT>& left, const LSVec<FT>& right, const LRDscVec<FT>& dsc_left,
             const LRDscVec<FT>& dsc_right) {
            DMatchVec<FT> matches;
            self.match(left, right, dsc_left, dsc_right, matches);
            return matches;
          },
          py::arg("left"), py::arg("right"), py::arg("dsc_left"), py::arg("dsc_right"),
          "Match with pre-computed LR descriptors. Returns list of DescriptorMatch.")
      // match with automatic descriptor creation
      .def(
          "match_auto",
          [](SLMatcher<FT>& self, const LSVec<FT>& left, const LSVec<FT>& right) {
            DMatchVec<FT> matches;
            self.match(left, right, matches);
            return matches;
          },
          py::arg("left"), py::arg("right"),
          "Match with automatic descriptor computation (requires creators set in constructor).")
      // accessors
      .def("get_filter", &SLMatcher<FT>::getFilter, py::return_value_policy::reference_internal,
           "Get the internal StereoLineFilter.")
      .def(
          "get_descriptor_left", [](SLMatcher<FT>& self) { return self.getDescriptorLeft(); },
          "Get a copy of the left descriptors from the last match.")
      .def(
          "get_descriptor_right", [](SLMatcher<FT>& self) { return self.getDescriptorRight(); },
          "Get a copy of the right descriptors from the last match.");
}

// ============================================================================
// Convenience: bind all LFD types for one FT preset
// ============================================================================

template <class FT>
void bind_lfd_preset(py::module_& m, const std::string& suffix) {
  bind_lfd_match_types<FT>(m, suffix);
  bind_lfd_descriptor_types<FT>(m, suffix);
  bind_lfd_descriptor_creators<FT>(m, suffix);
  bind_lfd_filters<FT>(m, suffix);
  bind_lfd_matchers<FT>(m, suffix);
  bind_lfd_stereo_matcher<FT>(m, suffix);
}

// ============================================================================
// Explicit template instantiations for supported presets
// ============================================================================

// Default: float — suffix ""
template void bind_lfd_match_types<float>(py::module_&, const std::string&);
template void bind_lfd_descriptor_types<float>(py::module_&, const std::string&);
template void bind_lfd_descriptor_creators<float>(py::module_&, const std::string&);
template void bind_lfd_filters<float>(py::module_&, const std::string&);
template void bind_lfd_matchers<float>(py::module_&, const std::string&);
template void bind_lfd_stereo_matcher<float>(py::module_&, const std::string&);
template void bind_lfd_preset<float>(py::module_&, const std::string&);

// Double (64-bit) — suffix "_f64"
template void bind_lfd_match_types<double>(py::module_&, const std::string&);
template void bind_lfd_descriptor_types<double>(py::module_&, const std::string&);
template void bind_lfd_descriptor_creators<double>(py::module_&, const std::string&);
template void bind_lfd_filters<double>(py::module_&, const std::string&);
template void bind_lfd_matchers<double>(py::module_&, const std::string&);
template void bind_lfd_stereo_matcher<double>(py::module_&, const std::string&);
template void bind_lfd_preset<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
