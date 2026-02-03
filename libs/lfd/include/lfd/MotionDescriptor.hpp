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

/// @file MotionDescriptor.hpp
/// @brief Motion-based feature descriptor for temporal matching.
/// Provides descriptor type and creator based on feature midpoint for motion estimation.


#pragma once

#include <lfd/GenericDescriptor.hpp>

namespace lsfm {

/// @brief Motion-based feature descriptor using midpoint position.
/// Measures distance between feature midpoints for motion matching.
/// @tparam FT Float type
template <class FT>
struct MotionDescritpor {
  MotionDescritpor() {}

  lsfm::Vec2<FT> midPoint;  ///< Feature midpoint position

  /// @brief Compute squared distance to another descriptor.
  /// @param rhs The other descriptor
  /// @return Squared Euclidean distance between midpoints
  inline FT distance(const MotionDescritpor<FT>& rhs) const { return (midPoint - rhs.midPoint).squaredNorm(); }

  /// @brief Compute distance between two descriptors (static version).
  /// @param lhs First descriptor
  /// @param rhs Second descriptor
  /// @return Euclidean distance between midpoints
  static inline FT distance(const MotionDescritpor<FT>& lhs, const MotionDescritpor<FT>& rhs) {
    return cv::norm(lhs.midPoint - rhs.midPoint);
  }

  /// @brief Get descriptor size in bytes.
  /// @return Size of cv::Point2D
  static inline int size() { return sizeof(cv::Point_<FT>); }

  /// @brief Get descriptor type name.
  /// @return String "Motion"
  std::string name() const { return "Motion"; }
};

// Generic Feature Descriptor creator for gradient
template <class FT, class GT = LineSegment<FT>>
class FdcMotion : public FdcObj<FT, GT, MotionDescritpor<FT>> {
 public:
  typedef typename FdcObj<FT, GT, MotionDescritpor<FT>>::Ptr FdcPtr;
  typedef MotionDescritpor<FT> descriptor_type;

  FdcMotion(const MatMap& data = MatMap()) { this->setData(data); }

  static FdcPtr createFdc() { return FdcPtr(new FdcMotion<FT, GT>()); }

  using FdcObjI<FT, GT, descriptor_type>::create;

  //! create single descriptor from single geometric object
  virtual void create(const GT& input, descriptor_type& dst) { dst.midPoint = input.centerPoint(); }

  //! get size of single descriptor (cols in cv::Mat)
  virtual size_t size() const { return static_cast<size_t>(descriptor_type::size()); }

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) {}
};
}  // namespace lsfm
