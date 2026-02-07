//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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
