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

#include <edge/threshold_estimator.hpp>
#include <opencv2/core.hpp>

#include <algorithm>


namespace lsfm {

template <class IT>
class Threshold {
 protected:
  Threshold() {}

 public:
  typedef IT img_type;

  virtual ~Threshold() {}

  virtual cv::Mat_<IT> process(const cv::Mat& img) const = 0;

  virtual std::string name() const = 0;
};

//! Global threshold class
template <class IT, class E = ThresholdOtsu<IT, 512, float>>
class GlobalThreshold : public Threshold<IT> {
  E th_{};

 public:
  typedef IT img_type;

  GlobalThreshold(IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0) : th_(r_max, r_min) {}


  //! Compute global threshold
  cv::Mat_<IT> process(const cv::Mat& mag) const {
    cv::Mat_<IT> ret(mag.size());
    cv::threshold(mag, ret, th_.process(mag), th_.max(), cv::THRESH_BINARY);
    return ret;
  }


  //! Get name of threshold method
  std::string name() const { return "global " + E::name(); }
};

//! Local threshold class using fixed window
template <class IT, class E = ThresholdOtsu<IT, 512, float>>
class LocalThreshold : public Threshold<IT> {
 protected:
  E th_;
  int wx_, wy_;

 public:
  typedef IT img_type;

  LocalThreshold(int win_size_x, int win_size_y, IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0)
      : th_(r_max, r_min), wx_(win_size_x), wy_(win_size_y) {}


  //! Compute local threshold
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


  //! Get name of threshold method
  std::string name() const { return "local window " + E::name(); }
};

//! Local dynamic threshold class using tiles
template <class IT, class E = ThresholdOtsu<IT, 512, float>, bool PARALLEL_FOR = true>
class LocalThresholdTiles : public Threshold<IT> {
  E th_;
  float scale_{2.};
  int tiles_x_, tiles_y_;

 public:
  typedef IT img_type;

  LocalThresholdTiles(
      int tiles_x, int tiles_y, IT r_max = std::numeric_limits<IT>::max(), float scale = 1.0f, IT r_min = 0)
      : th_(r_max, r_min), tiles_x_(tiles_x), tiles_y_(tiles_y), scale_(scale) {}


  //! Compute local threshold
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

            // Define tile region
            cv::Rect tile_region(tile_x * wx, tile_y * wy, wx + (rx - tile_x > 0 ? 1 : 0),
                                 wy + (ry - tile_y > 0 ? 1 : 0));
            cv::Mat mag_tile = mag(tile_region);
            cv::threshold(mag_tile, ret(tile_region), th_.process(mag_tile), th_.max(), cv::THRESH_BINARY);
          }
        });
      } else {
        int y_start = 0;
        for (int ty = 0; ty != tiles_y_; ++ty) {
          int wy_local = wy + (ry-- > 0 ? 1 : 0);
          int x_start = 0;
          int rx_local = rx;
          for (int tx = 0; tx != tiles_x_; ++tx) {
            int wx_local = wx + (rx_local-- > 0 ? 1 : 0);
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

            // Define tile region
            cv::Rect tile_region(tile_x * wx, tile_y * wy, wx + (rx - tile_x > 0 ? 1 : 0),
                                 wy + (ry - tile_y > 0 ? 1 : 0));
            cv::Rect scaled_region = scaledRegion(image_bounds, tile_region, scale_);

            cv::threshold(mag(tile_region), ret(tile_region), th_.process(mag(scaled_region)), th_.max(),
                          cv::THRESH_BINARY);
          }
        });
      } else {
        int y_start = 0;
        for (int ty = 0; ty != tiles_y_; ++ty) {
          int wy_local = wy + (ry-- > 0 ? 1 : 0);
          int x_start = 0;
          int rx_local = rx;
          for (int tx = 0; tx != tiles_x_; ++tx) {
            int wx_local = wx + (rx_local-- > 0 ? 1 : 0);
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


  //! Get name of threshold method
  std::string name() const { return "local tiles " + E::name(); }

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

//! Dynamic threshold class using radius to define sliding window size
template <class IT, class E = ThresholdOtsu<IT, 512, float>, bool PARALLEL_FOR = true>
class DynamicThreshold : public Threshold<IT> {
 protected:
  E th_;
  int r_;

 public:
  typedef IT img_type;

  DynamicThreshold(int radius, IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0)
      : th_(r_max, r_min), r_(radius) {}


  //! Compute dynamic threshold
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

  //! Get name of threshold method
  std::string name() const { return "dynamic " + E::name(); }
};

}  // namespace lsfm
