//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file threshold.hpp
/// @brief Threshold computation and application for edge detection.
/// Provides global and adaptive thresholding methods for converting edge magnitude maps
/// to binary edge maps using various statistical estimators (Otsu, etc.).

#pragma once

#include <edge/threshold_estimator.hpp>
#include <opencv2/core.hpp>

#include <algorithm>


namespace lsfm {

/// @brief Abstract base class for threshold-based edge detection.
/// Defines the interface for converting magnitude maps to binary edge maps.
/// @tparam IT Image element type (typically uint8_t or float)
template <class IT>
class Threshold {
 protected:
  Threshold() {}

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  virtual ~Threshold() {}

  /// @brief Apply threshold to magnitude map.
  /// @param img Input magnitude map
  /// @return Binary edge map (IT type with 0 for non-edges, non-zero for edges)
  virtual cv::Mat_<IT> process(const cv::Mat& img) const = 0;

  /// @brief Get the name of this threshold method.
  /// @return String describing the threshold method
  virtual std::string name() const = 0;
};

/// @brief Global threshold using automatic estimation (e.g., Otsu's method).
/// Computes a single threshold value for the entire image and applies it globally.
/// @tparam IT Image element type
/// @tparam E Threshold estimator (default: ThresholdOtsu)
/// @tparam PARALLEL_FOR Whether to use parallel processing for large images
template <class IT, class E = ThresholdOtsu<IT, 512, float>, bool PARALLEL_FOR = false>
class GlobalThreshold : public Threshold<IT> {
  E th_{};

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct a global threshold with value range.
  /// @param r_max Maximum possible value (default: max value for type)
  /// @param r_min Minimum possible value (default: 0)
  GlobalThreshold(IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0) : th_(r_max, r_min) {}

  /// @brief Compute and apply global threshold to magnitude map.
  /// @param mag Input magnitude map
  /// @return Binary output map with edges marked as 255 (or IT max value)
  cv::Mat_<IT> process(const cv::Mat& mag) const {
    cv::Mat_<IT> ret(mag.size());

    if constexpr (PARALLEL_FOR) {
      auto global_th = th_.process(mag);
      int tiles_x = 3;
      int tiles_y = 3;
      int wx = mag.cols / tiles_x;
      int wy = mag.rows / tiles_y;
      int rx = mag.cols % tiles_x;
      int ry = mag.rows % tiles_y;
      // Parallel processing using cv::parallel_for_
      cv::parallel_for_(cv::Range(0, tiles_y * tiles_x), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
          int tile_y = i / tiles_x;
          int tile_x = i % tiles_x;

          int x_off = std::min(rx, tile_x);
          int y_off = std::min(ry, tile_y);

          // Define tile region
          cv::Rect tile_region(tile_x * wx + x_off, tile_y * wy + y_off, wx + (rx - tile_x > 0 ? 1 : 0),
                               wy + (ry - tile_y > 0 ? 1 : 0));
          cv::Mat mag_tile = mag(tile_region);
          cv::threshold(mag_tile, ret(tile_region), global_th, th_.max(), cv::THRESH_BINARY);
        }
      });
    } else {
      cv::threshold(mag, ret, th_.process(mag), th_.max(), cv::THRESH_BINARY);
    }


    return ret;
  }


  /// @brief Get the name of this threshold method.
  /// @return "global " + estimator name
  std::string name() const { return "global " + E::name(); }
};

/// @brief Local threshold using fixed window size.
/// Divides the image into rectangular tiles and applies independent
/// threshold estimation within each tile.
/// @tparam IT Image element type
/// @tparam E Threshold estimator type (default: ThresholdOtsu)
template <class IT, class E = ThresholdOtsu<IT, 512, float>>
class LocalThreshold : public Threshold<IT> {
 protected:
  E th_;
  int wx_, wy_;

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct a local threshold with window size.
  /// @param win_size_x Window width in pixels
  /// @param win_size_y Window height in pixels
  /// @param r_max Maximum magnitude value
  /// @param r_min Minimum magnitude value
  LocalThreshold(int win_size_x, int win_size_y, IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0)
      : th_(r_max, r_min), wx_(win_size_x), wy_(win_size_y) {}


  /// @brief Compute and apply local threshold to magnitude map.
  /// Divides the image into fixed-size windows and applies independent thresholding.
  /// Falls back to global thresholding if window covers the entire image.
  /// @param mag Input magnitude map
  /// @return Binary output map
  cv::Mat_<IT> process(const cv::Mat& mag) const {
    cv::Mat_<IT> ret(mag.size());
    if ((wx_ >= mag.cols && wy_ >= mag.rows) || (wx_ <= 0 && wy_ <= 0)) {
      cv::threshold(mag, ret, th_.process(mag), th_.max(), cv::THRESH_BINARY);
      return ret;
    }
    // 0 or negative means auto value. In this case we use mat width or height
    int wx = wx_ <= 0 ? mag.cols : wx_;
    int wy = wy_ <= 0 ? mag.rows : wy_;

    // Loop over the mag matrix in steps of wx_ and wy_
    for (int y = 0; y < mag.rows; y += wy) {
      for (int x = 0; x < mag.cols; x += wx) {
        // Compute width and height of the tile
        int tile_width = std::min(wx, mag.cols - x);
        int tile_height = std::min(wy, mag.rows - y);

        // Define tile region
        cv::Rect tile_region(x, y, tile_width, tile_height);

        // Extract tile
        cv::Mat mag_tile = mag(tile_region);
        cv::Mat ret_tile = ret(tile_region);

        cv::threshold(mag_tile, ret_tile, th_.process(mag_tile), th_.max(), cv::THRESH_BINARY);
      }
    }
    return ret;
  }


  /// @brief Get the name of this threshold method.
  /// @return "local window " + estimator name
  std::string name() const { return "local window " + E::name(); }
};

/// @brief Local threshold using tile-based decomposition.
/// Divides the image into a grid of tiles with optional overlap scaling,
/// and applies independent threshold estimation within each tile.
/// Supports parallel processing via cv::parallel_for_.
/// @tparam IT Image element type
/// @tparam E Threshold estimator type (default: ThresholdOtsu)
/// @tparam PARALLEL_FOR Whether to use parallel processing (default: true)
template <class IT, class E = ThresholdOtsu<IT, 512, float>, bool PARALLEL_FOR = true>
class LocalThresholdTiles : public Threshold<IT> {
  E th_;
  float scale_{2.};
  int tiles_x_, tiles_y_;

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct a tile-based local threshold.
  /// @param tiles_x Number of horizontal tiles
  /// @param tiles_y Number of vertical tiles
  /// @param r_max Maximum magnitude value
  /// @param scale Overlap scale factor (1.0 = no overlap, >1.0 = overlapping regions)
  /// @param r_min Minimum magnitude value
  LocalThresholdTiles(
      int tiles_x, int tiles_y, IT r_max = std::numeric_limits<IT>::max(), float scale = 1.0f, IT r_min = 0)
      : th_(r_max, r_min), scale_(scale), tiles_x_(tiles_x), tiles_y_(tiles_y) {}


  /// @brief Compute and apply tile-based local threshold.
  /// Splits image into tiles, optionally with overlap, and thresholds
  /// each tile independently using the estimator.
  /// @param mag Input magnitude map
  /// @return Binary output map
  cv::Mat_<IT> process(const cv::Mat& mag) const {
    cv::Mat_<IT> ret(mag.size());
    if (tiles_x_ <= 1 && tiles_y_ <= 1) {
      cv::threshold(mag, ret, th_.process(mag), th_.max(), cv::THRESH_BINARY);
      return ret;
    }

    int wx = mag.cols / tiles_x_;
    int wy = mag.rows / tiles_y_;
    int rx = mag.cols % tiles_x_;
    int ry = mag.rows % tiles_y_;

    if (scale_ <= 1.0) {
      if constexpr (PARALLEL_FOR) {
        // Parallel processing using cv::parallel_for_
        cv::parallel_for_(cv::Range(0, tiles_y_ * tiles_x_), [&](const cv::Range& range) {
          for (int i = range.start; i < range.end; ++i) {
            int tile_y = i / tiles_x_;
            int tile_x = i % tiles_x_;

            int x_off = std::min(rx, tile_x);
            int y_off = std::min(ry, tile_y);

            // Define tile region
            cv::Rect tile_region(tile_x * wx + x_off, tile_y * wy + y_off, wx + (rx - tile_x > 0 ? 1 : 0),
                                 wy + (ry - tile_y > 0 ? 1 : 0));
            cv::Mat mag_tile = mag(tile_region);
            cv::threshold(mag_tile, ret(tile_region), th_.process(mag_tile), th_.max(), cv::THRESH_BINARY);
          }
        });
      } else {
        int y_start = 0;
        for (int ty = 0; ty != tiles_y_; ++ty) {
          int wy_local = wy + (ry - ty > 0 ? 1 : 0);
          int x_start = 0;
          for (int tx = 0; tx != tiles_x_; ++tx) {
            int wx_local = wx + (rx - tx > 0 ? 1 : 0);
            // Define tile region
            cv::Rect tile_region(x_start, y_start, wx_local, wy_local);
            cv::Mat mag_tile = mag(tile_region);
            cv::threshold(mag_tile, ret(tile_region), th_.process(mag_tile), th_.max(), cv::THRESH_BINARY);
            x_start += wx_local;
          }
          y_start += wy_local;
        }
      }
    } else {
      cv::Rect image_bounds(0, 0, mag.cols, mag.rows);
      if constexpr (PARALLEL_FOR) {
        // Parallel processing using cv::parallel_for_
        cv::parallel_for_(cv::Range(0, tiles_y_ * tiles_x_), [&](const cv::Range& range) {
          for (int i = range.start; i < range.end; ++i) {
            int tile_y = i / tiles_x_;
            int tile_x = i % tiles_x_;

            int x_off = std::min(rx, tile_x);
            int y_off = std::min(ry, tile_y);

            // Define tile region
            cv::Rect tile_region(tile_x * wx + x_off, tile_y * wy + y_off, wx + (rx - tile_x > 0 ? 1 : 0),
                                 wy + (ry - tile_y > 0 ? 1 : 0));
            cv::Rect scaled_region = scaledRegion(image_bounds, tile_region, scale_);

            cv::threshold(mag(tile_region), ret(tile_region), th_.process(mag(scaled_region)), th_.max(),
                          cv::THRESH_BINARY);
          }
        });
      } else {
        int y_start = 0;
        for (int ty = 0; ty != tiles_y_; ++ty) {
          int wy_local = wy + (ry - ty > 0 ? 1 : 0);
          int x_start = 0;
          for (int tx = 0; tx != tiles_x_; ++tx) {
            int wx_local = wx + (rx - tx > 0 ? 1 : 0);
            // Define tile region
            cv::Rect tile_region(x_start, y_start, wx_local, wy_local);
            cv::Rect scaled_region = scaledRegion(image_bounds, tile_region, scale_);

            cv::threshold(mag(tile_region), ret(tile_region), th_.process(mag(scaled_region)), th_.max(),
                          cv::THRESH_BINARY);
            x_start += wx_local;
          }
          y_start += wy_local;
        }
      }
    }

    return ret;
  }


  /// @brief Get the name of this threshold method.
  /// @return "local tiles " + estimator name
  std::string name() const { return "local tiles " + E::name(); }

  /// @brief Compute a scaled region centered on the tile for overlap.
  /// @param image_bounds Full image rectangle
  /// @param tile_region Tile rectangle
  /// @param scale Scale factor for overlap
  /// @return Intersection of scaled region with image bounds
  static cv::Rect scaledRegion(const cv::Rect image_bounds, const cv::Rect& tile_region, float scale) {
    int newWidth = static_cast<int>(static_cast<float>(tile_region.width) * scale);
    int newHeight = static_cast<int>(static_cast<float>(tile_region.height) * scale);

    // Scale region at center
    cv::Rect scaled_region = cv::Rect(tile_region.x - (tile_region.width - newWidth) / 2,
                                      tile_region.y - (tile_region.height - newHeight) / 2, newWidth, newHeight);


    // Return valid range by calculating the intersection part of image bounds and scaled region
    return scaled_region & image_bounds;
  }
};

/// @brief Dynamic threshold using a sliding window.
/// Computes a local threshold for each pixel using a square neighborhood
/// defined by a radius. Supports parallel processing.
/// @tparam IT Image element type
/// @tparam E Threshold estimator type (default: ThresholdOtsu)
/// @tparam PARALLEL_FOR Whether to use parallel processing (default: true)
template <class IT, class E = ThresholdOtsu<IT, 512, float>, bool PARALLEL_FOR = true>
class DynamicThreshold : public Threshold<IT> {
 protected:
  E th_;
  int r_;

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct a dynamic threshold with sliding window radius.
  /// @param radius Neighborhood radius (window size = 2*radius+1)
  /// @param r_max Maximum magnitude value
  /// @param r_min Minimum magnitude value
  DynamicThreshold(int radius, IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0)
      : th_(r_max, r_min), r_(radius) {}


  /// @brief Compute per-pixel dynamic threshold.
  /// For each pixel, estimates a threshold from its local neighborhood.
  /// @param mag Input magnitude map
  /// @return Binary output map
  cv::Mat_<IT> process(const cv::Mat& mag) const {
    cv::Mat_<IT> ret(mag.size());

    if constexpr (PARALLEL_FOR) {
      // Parallel processing using cv::parallel_for_
      cv::parallel_for_(cv::Range(0, mag.rows * mag.cols), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
          int y = i / mag.cols;  // Row index
          int x = i % mag.cols;  // Column index
          // Define the window range, ensuring boundaries
          int x_start = std::max(0, x - r_);
          int y_start = std::max(0, y - r_);
          int x_end = std::min(mag.cols, x + r_ + 1);
          int y_end = std::min(mag.rows, y + r_ + 1);

          // Store the processed value in ret at (y, x)
          ret(y, x) = mag.at<IT>(y, x) >= th_.process(mag(cv::Range(y_start, y_end), cv::Range(x_start, x_end)))
                          ? th_.max()
                          : 0;
        }
      });
    } else {
      // Iterate over each pixel
      for (int y = 0; y < mag.rows; ++y) {
        for (int x = 0; x < mag.cols; ++x) {
          // Define the window range, ensuring boundaries
          int x_start = std::max(0, x - r_);
          int y_start = std::max(0, y - r_);
          int x_end = std::min(mag.cols, x + r_ + 1);
          int y_end = std::min(mag.rows, y + r_ + 1);

          // Store the processed value in ret at (y, x)
          ret(y, x) = mag.at<IT>(y, x) >= th_.process(mag(cv::Range(y_start, y_end), cv::Range(x_start, x_end)))
                          ? th_.max()
                          : 0;
        }
      }
    }

    return ret;
  }

  /// @brief Get the name of this threshold method.
  /// @return "dynamic " + estimator name
  std::string name() const { return "dynamic " + E::name(); }
};

}  // namespace lsfm
