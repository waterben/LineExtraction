/// @file ndarray_converter.hpp
/// @brief Bidirectional cv::Mat <-> numpy ndarray converter for pybind11.
///
/// Provides pybind11 type_caster specialization that enables automatic
/// conversion between OpenCV cv::Mat and numpy arrays in Python bindings.
/// Based on the well-known cv::Mat/numpy interop pattern.

#pragma once

#include <opencv2/core/core.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <stdexcept>

namespace pybind11 {
namespace detail {

/// @brief pybind11 type_caster for cv::Mat <-> numpy.ndarray conversion.
///
/// Supports automatic bidirectional conversion:
///   - Python -> C++: numpy array is wrapped as cv::Mat (zero-copy when contiguous)
///   - C++ -> Python: cv::Mat data is copied into a new numpy array
template <>
struct type_caster<cv::Mat> {
 public:
  PYBIND11_TYPE_CASTER(cv::Mat, const_name("numpy.ndarray"));

  /// @brief Convert numpy ndarray to cv::Mat (Python -> C++).
  /// @param src Python object (expected: numpy ndarray)
  /// @return true if conversion succeeded
  bool load(handle src, bool /*implicit*/) {
    if (!isinstance<array>(src)) {
      return false;
    }

    auto arr = reinterpret_borrow<array>(src);
    int ndim = static_cast<int>(arr.ndim());

    int dtype = 0;
    if (arr.dtype().is(dtype::of<uint8_t>())) {
      dtype = CV_8U;
    } else if (arr.dtype().is(dtype::of<int8_t>())) {
      dtype = CV_8S;
    } else if (arr.dtype().is(dtype::of<uint16_t>())) {
      dtype = CV_16U;
    } else if (arr.dtype().is(dtype::of<int16_t>())) {
      dtype = CV_16S;
    } else if (arr.dtype().is(dtype::of<int32_t>())) {
      dtype = CV_32S;
    } else if (arr.dtype().is(dtype::of<float>())) {
      dtype = CV_32F;
    } else if (arr.dtype().is(dtype::of<double>())) {
      dtype = CV_64F;
    } else {
      return false;
    }

    // Determine shape
    std::vector<int> shape;
    for (int i = 0; i < ndim; ++i) {
      shape.push_back(static_cast<int>(arr.shape(i)));
    }

    int channels = 1;
    if (ndim == 3) {
      channels = shape[2];
      dtype = CV_MAKETYPE(dtype, channels);
    } else if (ndim > 3 || ndim < 1) {
      return false;
    }

    // Request contiguous array
    auto buf = arr.request();
    value = cv::Mat(ndim == 1 ? 1 : shape[0], ndim == 1 ? shape[0] : shape[1], dtype, buf.ptr).clone();

    return true;
  }

  /// @brief Convert cv::Mat to numpy ndarray (C++ -> Python).
  /// @param src The cv::Mat to convert
  /// @param policy Return value policy
  /// @param parent Parent object for reference counting
  /// @return Python numpy array
  static handle cast(const cv::Mat& src, return_value_policy /*policy*/, handle /*parent*/) {
    if (src.empty()) {
      return none().inc_ref();
    }

    // Determine numpy dtype from OpenCV depth
    std::string format;
    switch (src.depth()) {
      case CV_8U:
        format = format_descriptor<uint8_t>::format();
        break;
      case CV_8S:
        format = format_descriptor<int8_t>::format();
        break;
      case CV_16U:
        format = format_descriptor<uint16_t>::format();
        break;
      case CV_16S:
        format = format_descriptor<int16_t>::format();
        break;
      case CV_32S:
        format = format_descriptor<int32_t>::format();
        break;
      case CV_32F:
        format = format_descriptor<float>::format();
        break;
      case CV_64F:
        format = format_descriptor<double>::format();
        break;
      default:
        throw std::runtime_error("Unsupported cv::Mat depth for numpy conversion");
    }

    // Build shape and strides
    std::vector<ssize_t> shape;
    std::vector<ssize_t> strides;

    int channels = src.channels();

    if (channels == 1) {
      shape = {src.rows, src.cols};
      strides = {static_cast<ssize_t>(src.step[0]), static_cast<ssize_t>(src.step[1])};
    } else {
      shape = {src.rows, src.cols, channels};
      strides = {static_cast<ssize_t>(src.step[0]), static_cast<ssize_t>(src.step[1]),
                 static_cast<ssize_t>(src.elemSize1())};
    }

    // Copy data into numpy array
    cv::Mat contiguous;
    if (src.isContinuous()) {
      contiguous = src;
    } else {
      contiguous = src.clone();
    }

    return array(buffer_info(contiguous.data, static_cast<ssize_t>(contiguous.elemSize1()), format,
                             static_cast<ssize_t>(shape.size()), shape, strides))
        .release();
  }
};

}  // namespace detail
}  // namespace pybind11
