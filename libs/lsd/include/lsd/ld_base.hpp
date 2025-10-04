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
//M*/

#pragma once

#include <geometry/line.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <utility/value_manager.hpp>

namespace lsfm {

struct DataDescriptorEntry {
  DataDescriptorEntry(const std::string& n = std::string(), const std::string d = std::string())
      : name(n), description(d) {}

  std::string name{};
  std::string description{};
};

typedef std::vector<DataDescriptorEntry> DataDescriptor;

//! line detector base class
template <class FT, template <class> class LPT = Vec2>
class LdBase : public ValueManager {
 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;
  template <class A>
  using line_point_template = LPT<A>;

  typedef lsfm::Line<FT, LPT> Line;
  typedef std::vector<Line> LineVector;
  typedef std::vector<cv::Mat> ImageData;

  virtual ~LdBase() {}

  //! Detect lines in the input image.
  //! @param image      Input image. Possible image types depends on implementation.
  //!                  8Bit single channel should work with all variants
  virtual void detect(const cv::Mat& image) = 0;

  // interface helpers
  inline void detect(const cv::Mat& image, LineVector& l) {
    detect(image);
    l = lines();
  }
  inline void detect(const cv::Mat& image, LineVector& l, ImageData& id) {
    detect(image);
    l = lines();
    id = imageData();
  }

  //! Get detected lines as line vector.
  virtual const LineVector& lines() const { return lines_; }

  //! Get image data description. For every layer in image data, a DataDescriptorEntry is defined, giving the name
  //! and a description for the layer
  //! @return Image data descriptor
  virtual const DataDescriptor& imageDataDescriptor() const {
    static DataDescriptor id;
    return id;
  }


  //! Return additional image data as mat
  //! @return: A vector of cv::Mat with additinal image data like gradient
  //!          magnitude or gradient direction. Every layer entry in the vector represents one image
  //!          data source (as cv::Mat). Use imageDataDescriptor to get informations of the provided data and
  //!          data order that is included within the image_data vector.
  virtual const ImageData& imageData() const {
    static ImageData id;
    return id;
  }

  //! get single image data by name
  const cv::Mat imageData(const std::string& name) const {
    const DataDescriptor& data = imageDataDescriptor();
    typename DataDescriptor::const_iterator f =
        find_if(data.begin(), data.end(), [&name](const DataDescriptorEntry& e) { return (e.name == name); });
    return f != data.end() ? imageData()[f - data.begin()] : cv::Mat();
  }

 protected:
  LdBase() {}

  mutable LineVector lines_{};
  // int flags_;

  //! helper that should be used to clear internal data
  virtual void clearData() { lines_.clear(); }
};

}  // namespace lsfm
