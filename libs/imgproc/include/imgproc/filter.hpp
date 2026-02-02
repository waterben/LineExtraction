/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// C by Benjamin Wassermann
//M*/

#pragma once

#include <opencv2/core/core.hpp>
#include <utility/range.hpp>
#include <utility/value_manager.hpp>

#include <map>


namespace lsfm {

/// @brief Container for filter output data with associated value range.
///
/// Stores a cv::Mat along with the expected range of values in that matrix,
/// useful for normalization and visualization of filter results.
struct FilterData {
  /// @brief Construct FilterData with optional matrix and range.
  /// @param d The output matrix data (default: empty matrix)
  /// @param lower The lower bound of the value range (default: 0)
  /// @param upper The upper bound of the value range (default: 0)
  FilterData(const cv::Mat& d = cv::Mat(), double lower = 0, double upper = 0) : data(d), range(lower, upper) {}

  /// @brief Construct FilterData from matrix and typed Range.
  /// @tparam FT The type of the Range bounds
  /// @param d The output matrix data
  /// @param r The value range to convert to double
  template <class FT>
  FilterData(const cv::Mat& d, const Range<FT>& r)
      : data(d), range(static_cast<double>(r.lower), static_cast<double>(r.upper)) {}

  cv::Mat data;         ///< The filter output matrix
  Range<double> range;  ///< The expected value range for data
};

/// @brief Map of named filter results.
///
/// Used to return multiple outputs from a filter (e.g., magnitude, direction).
typedef std::map<std::string, FilterData> FilterResults;

/// @brief Single named filter result pair.
typedef std::pair<std::string, FilterData> FilterResult;

/// @brief Abstract base interface for image filters.
///
/// Provides a common interface for all image processing filters in the library.
/// Derived classes implement specific filtering algorithms (gradient, laplacian, etc.).
/// Inherits from ValueManager for runtime parameter configuration.
/// @tparam IT Input image pixel type (uchar for 8-bit, short for 16-bit, float/double)
template <class IT>
class FilterI : public ValueManager {
  FilterI(const FilterI&);

 protected:
  FilterI() {}

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values

  virtual ~FilterI() {}

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  virtual IntensityRange intensityRange() const = 0;

  /// @brief Process the input image through the filter.
  /// @param img The input image to process
  virtual void process(const cv::Mat& img) = 0;

  /// @brief Get all filter outputs as named results.
  /// @return Map of output name to FilterData containing matrix and range
  virtual FilterResults results() const = 0;

  /// @brief Get the name identifier of this filter.
  /// @return String name of the filter (e.g., "sobel", "laplace")
  virtual std::string name() const = 0;
};
}  // namespace lsfm
