//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file filter.hpp
/// @brief Abstract base interface for image filters.

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
