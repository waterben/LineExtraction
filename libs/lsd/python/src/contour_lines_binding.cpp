/// @file contour_lines_binding.cpp
/// @brief Contour-to-line-segment conversion binding for Python.
///
/// Wraps the C++ RamerSplit + EigenFit pipeline so that ordered contour
/// point sequences (e.g. from OpenCV findContours or AI segmentation)
/// can be converted to LineSegment objects without running a full LSD
/// detector on an image.

#include "contour_lines_binding.hpp"

#include <edge/edge_segment.hpp>
#include <edge/fit.hpp>
#include <edge/split.hpp>
#include <geometry/base.hpp>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

/// @brief Convert a numpy (N,2) int32 contour to a vector of Vec2i.
static std::vector<Vec2i> numpy_to_points(py::array_t<std::int32_t> arr) {
  auto buf = arr.request();

  // Accept (N, 2) or (N, 1, 2) — the latter is OpenCV findContours format
  if (buf.ndim == 3 && buf.shape[1] == 1 && buf.shape[2] == 2) {
    arr = arr.reshape(std::vector<py::ssize_t>{buf.shape[0], 2});
    buf = arr.request();
  }

  if (buf.ndim != 2 || buf.shape[1] != 2) {
    throw std::invalid_argument("points must be an (N, 2) or (N, 1, 2) int32 array");
  }

  const auto n = static_cast<int>(buf.shape[0]);
  auto r = arr.unchecked<2>();

  std::vector<Vec2i> points;
  points.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    points.emplace_back(r(i, 0), r(i, 1));
  }
  return points;
}

/// @brief Run RamerSplit + EigenFit on contour points → LineSegments.
/// @tparam FT Floating-point precision for line fitting
template <class FT>
static std::vector<LineSegment<FT, Vec2>> contour_to_line_segments_impl(py::array_t<std::int32_t> points_array,
                                                                        FT split_distance,
                                                                        int min_pixels,
                                                                        bool closed) {
  auto points = numpy_to_points(points_array);
  auto n = points.size();

  if (n < static_cast<size_t>(std::max(2, min_pixels))) {
    return {};
  }

  EdgeSegmentVector input_segments;

  if (closed && n >= 4) {
    // For closed contours the first and last points may coincide, giving
    // RamerSplit a zero-length baseline.  Instead, split the loop into
    // two halves at the point farthest from the start.  Each half has a
    // well-defined baseline.

    // Ensure the contour actually closes (duplicate first point if needed)
    if (getX(points.back()) != getX(points.front()) || getY(points.back()) != getY(points.front())) {
      points.push_back(points.front());
      n = points.size();
    }

    // Find the point farthest from points[0]
    size_t far_idx = 0;
    FT max_dist_sq = 0;
    for (size_t i = 1; i < n; ++i) {
      FT dx = static_cast<FT>(getX(points[i]) - getX(points[0]));
      FT dy = static_cast<FT>(getY(points[i]) - getY(points[0]));
      FT d = dx * dx + dy * dy;
      if (d > max_dist_sq) {
        max_dist_sq = d;
        far_idx = i;
      }
    }

    // Two halves sharing endpoints at index 0 and far_idx
    input_segments.emplace_back(0, far_idx + 1);
    input_segments.emplace_back(far_idx, n);
  } else {
    input_segments.emplace_back(0, n);
  }

  // Split using Ramer-Douglas-Peucker
  RamerSplit<FT, Vec2i, true> splitter(split_distance, min_pixels);
  EdgeSegmentVector split_segments;
  splitter.apply(input_segments, points, split_segments);

  // Fit line segments using EigenFit
  std::vector<LineSegment<FT, Vec2>> result;
  result.reserve(split_segments.size());

  for (const auto& seg : split_segments) {
    if (seg.size() < 2) continue;

    Line<FT, Vec2> line;
    EigenFit<FT, Vec2i>::fit(points.data() + seg.begin(), points.data() + seg.end(), line);

    const Vec2i& first = points[seg.first()];
    const Vec2i& last = points[seg.last()];
    result.emplace_back(line, first, last);
  }

  return result;
}

void bind_contour_lines(py::module_& m) {
  m.def("fit_line_segments", &contour_to_line_segments_impl<float>, py::arg("points"), py::arg("split_distance") = 2.0f,
        py::arg("min_pixels") = 6, py::arg("closed") = true,
        "Convert contour points to fitted line segments using RamerSplit + EigenFit.\n\n"
        "This applies the same split-and-fit pipeline used internally by the LSD\n"
        "detectors, but operates directly on an ordered sequence of contour points\n"
        "rather than requiring a full image.\n\n"
        "Parameters\n"
        "----------\n"
        "points : numpy.ndarray\n"
        "    Contour points as an (N, 2) or (N, 1, 2) int32 array.\n"
        "    Points should be ordered along the contour boundary.\n"
        "split_distance : float\n"
        "    Ramer-Douglas-Peucker error distance threshold for splitting.\n"
        "    Larger values produce fewer, longer segments. Default: 2.0\n"
        "min_pixels : int\n"
        "    Minimum number of points required per line segment. Default: 6\n"
        "closed : bool\n"
        "    Whether the contour forms a closed loop. When True, the\n"
        "    wrap-around junction is handled to avoid artifacts. Default: True\n\n"
        "Returns\n"
        "-------\n"
        "list[LineSegment]\n"
        "    Fitted line segments with proper geometric attributes.\n\n"
        "Example\n"
        "-------\n"
        "    import le_lsd\n"
        "    import numpy as np\n\n"
        "    # Rectangle contour (from OpenCV findContours)\n"
        "    contour = np.array([[10,10],[90,10],[90,90],[10,90]], dtype=np.int32)\n"
        "    segments = le_lsd.fit_line_segments(contour, split_distance=2.0)\n"
        "    for seg in segments:\n"
        "        print(seg.start_point(), seg.end_point())\n");

  m.def("fit_line_segments_f64", &contour_to_line_segments_impl<double>, py::arg("points"),
        py::arg("split_distance") = 2.0, py::arg("min_pixels") = 6, py::arg("closed") = true,
        "Double-precision variant of fit_line_segments. See fit_line_segments for documentation.");
}

}  // namespace python
}  // namespace lsfm
