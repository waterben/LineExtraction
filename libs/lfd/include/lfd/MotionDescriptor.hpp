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

/// @brief Feature descriptor creator for motion-based descriptors.
/// Creates MotionDescritpor instances from geometric objects using midpoint position.
/// @tparam FT Float type for computations
/// @tparam GT Geometric type (default: LineSegment<FT>)
template <class FT, class GT = LineSegment<FT>>
class FdcMotion : public FdcObj<FT, GT, MotionDescritpor<FT>> {
 public:
  typedef typename FdcObj<FT, GT, MotionDescritpor<FT>>::Ptr FdcPtr;  ///< Shared pointer type for this creator
  typedef MotionDescritpor<FT> descriptor_type;                       ///< Descriptor type produced

  /// @brief Construct a motion descriptor creator.
  /// @param data Optional input data map (unused for motion descriptors)
  FdcMotion(const MatMap& data = MatMap()) { this->setData(data); }

  /// @brief Factory method to create a shared pointer instance.
  /// @return Shared pointer to a new FdcMotion
  static FdcPtr createFdc() { return FdcPtr(new FdcMotion<FT, GT>()); }

  using FdcObjI<FT, GT, descriptor_type>::create;

  /// @brief Create a single descriptor from a geometric object.
  /// Extracts the center point of the geometric object as the descriptor midpoint.
  /// @param input Source geometric object
  /// @param dst Output descriptor to populate
  virtual void create(const GT& input, descriptor_type& dst) { dst.midPoint = input.centerPoint(); }

  /// @brief Get the size of a single descriptor in bytes.
  /// @return Size of one descriptor
  virtual size_t size() const { return static_cast<size_t>(descriptor_type::size()); }

  /// @brief Set internal processing data.
  /// @param data Input data map (unused for motion descriptors)
  virtual void setData(const MatMap& data) {}
};
}  // namespace lsfm
