//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file LRDescriptor.hpp
/// @brief Left-Right feature descriptor for stereo matching.
/// Provides descriptor types and creators for left-right descriptor pairs.

#pragma once

#include <lfd/GenericDescriptor.hpp>

namespace lsfm {

/// @brief Left-Right feature descriptor for stereo matching.
/// Stores separate left and right descriptor halves, each of size cn.
/// Distance is computed as the product of L2 norms of both halves.
/// @tparam FT Float type for descriptor values
/// @tparam cn Number of channels per side (total size = cn * 2)
template <class FT, int cn>
struct LRDescritpor {
  LRDescritpor() {}

  /// @brief Construct from raw data pointer.
  /// @param d Pointer to source data (must contain cn * 2 elements)
  LRDescritpor(const FT* d) : data() { memcopy(data, d, sizeof(FT) * (cn * 2)); }

  FT data[static_cast<size_t>(cn * 2)]{};  ///< Raw descriptor data (left and right halves)

  /// @brief Get pointer to the left descriptor half.
  /// @return Pointer to the first cn elements
  inline FT* dataL() { return data; }

  /// @brief Get pointer to the right descriptor half.
  /// @return Pointer to the second cn elements
  inline FT* dataR() { return data + cn; }

  /// @brief Compute distance to another descriptor.
  /// Distance is the product of L2 norms of left and right halves.
  /// @param rhs The other descriptor
  /// @return Product of L2 distances for left and right halves
  inline FT distance(const LRDescritpor<FT, cn>& rhs) const {
    // return static_cast<FT>(norm(cv::_InputArray(data,cn*2), cv::_InputArray(rhs.data,cn*2), NORM_L2));
    return static_cast<FT>(norm(cv::_InputArray(data, static_cast<int>(cn)),
                                cv::_InputArray(rhs.data, static_cast<int>(cn)), cv::NORM_L2)) *
           static_cast<FT>(norm(cv::_InputArray(data + cn, static_cast<int>(cn)),
                                cv::_InputArray(rhs.data + cn, static_cast<int>(cn)), cv::NORM_L2));
  }

  /// @brief Compute distance between two descriptors (static version).
  /// @param lhs First descriptor
  /// @param rhs Second descriptor
  /// @return Product of L2 distances for left and right halves
  static inline FT distance(const LRDescritpor<FT, cn>& lhs, const LRDescritpor<FT, cn>& rhs) {
    // return static_cast<FT>(norm(cv::_InputArray(lhs.data,cn*2), cv::_InputArray(rhs.data,cn*2), NORM_L2));
    return static_cast<FT>(norm(cv::_InputArray(lhs.data, cn), cv::_InputArray(rhs.data, cn), cv::NORM_L2)) *
           static_cast<FT>(norm(cv::_InputArray(lhs.data + cn, cn), cv::_InputArray(rhs.data + cn, cn), cv::NORM_L2));
  }

  /// @brief Get total descriptor size.
  /// @return Number of elements (cn * 2)
  static inline int size() { return cn * 2; }

  /// @brief Get descriptor type name.
  /// @return String "LR"
  std::string name() const { return "LR"; }
};


/// @brief Generic left-right feature descriptor creator.
/// Creates LRDescritpor instances by sampling features on both sides of
/// a geometric object using a configurable helper strategy.
/// @tparam FT Float type for computations
/// @tparam GT Geometric type (default: LineSegment<FT>)
/// @tparam Helper Descriptor computation helper strategy (default: GchImgInterpolate<FT>)
template <class FT, class GT = LineSegment<FT>, class Helper = GchImgInterpolate<FT>>
class FdcGenericLR : public Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>> {
  /// @brief Create left and right descriptor halves from a geometric object.
  /// @param input Source geometric object
  /// @param dst Pointer to output descriptor data
  inline void doCreate(const GT& input, FT* dst) {
    Helper::create(data_.data(), input, dst, pos_, stepDir_, lstep_);
    dst += Helper::dscSize;
    Helper::create(data_.data(), input, dst, -pos_, -stepDir_, lstep_);
  }

 public:
  typedef typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr FdcPtr;           ///< Combined creator pointer
  typedef typename FdcObj<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr CustomFdcPtr;  ///< Object creator pointer
  typedef typename FdcMat<FT, GT>::Ptr SimpleFdcPtr;                                     ///< Matrix creator pointer
  typedef LRDescritpor<FT, Helper::dscSize> descriptor_type;                             ///< Descriptor type produced

  /// @brief Construct a left-right descriptor creator.
  /// @param data Input data map (images, gradients, etc.)
  /// @param pos Position offset from line center (default: -1)
  /// @param stepDir Step direction for sampling (default: 1)
  /// @param lstep Length step for sampling (default: 1)
  FdcGenericLR(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1)
      : data_(), pos_(pos), stepDir_(stepDir), lstep_(lstep) {
    data_.resize(Helper::inputData().size());
    this->setData(data);
  }

  /// @brief Factory method to create a shared pointer instance.
  /// @param data Input data map
  /// @param pos Position offset from line center
  /// @param stepDir Step direction for sampling
  /// @param lstep Length step for sampling
  /// @return Shared pointer to a new FdcGenericLR
  static FdcPtr createFdc(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1) {
    return FdcPtr(new FdcGenericLR<FT, GT, Helper>(data, pos, stepDir, lstep));
  }

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, descriptor_type>::create;

  /// @brief Create a single LR descriptor from a geometric object.
  /// @param input Source geometric object
  /// @param dst Output descriptor to populate
  virtual void create(const GT& input, descriptor_type& dst) { doCreate(input, dst.data); }

  /// @brief Create a single descriptor as a cv::Mat.
  /// @param input Source geometric object
  /// @param dst Output matrix (1 x size, allocated if needed)
  virtual void create(const GT& input, cv::Mat& dst) {
    if (dst.empty() || dst.cols != descriptor_type::size())
      dst.create(1, descriptor_type::size(), cv::DataType<FT>::type);
    doCreate(input, dst.template ptr<FT>());
  }

  /// @brief Get the size of a single descriptor.
  /// @return Total number of elements (cn * 2)
  virtual size_t size() const { return static_cast<size_t>(descriptor_type::size()); }

  /// @brief Set internal processing data.
  /// @param data Input data map with keys matching Helper::inputData()
  virtual void setData(const MatMap& data) {
    MatMap::const_iterator f;
    auto input = Helper::inputData();
    for (size_t i = 0; i != input.size(); ++i) {
      f = data.find(input[i]);
      if (f != data.end()) data_[i] = f->second;
    }
  }

  std::vector<cv::Mat> data_;  ///< Input image/gradient data
  FT pos_;                     ///< Position offset from line center
  FT stepDir_;                 ///< Step direction for sampling
  FT lstep_;                   ///< Length step for sampling along the line

 protected:
  /// @brief Create a descriptor into raw float pointer.
  /// @param input Source geometric object
  /// @param dst Pointer to output buffer
  virtual void create(const GT& input, FT* dst) { doCreate(input, dst); }
};

/// @brief Create a generic LR descriptor creator from an intensity image.
/// @tparam FT Float type
/// @tparam GT Geometric type (default: LineSegment<FT>)
/// @tparam Helper Descriptor computation helper (default: GchImgInterpolate<FT>)
/// @param img Input intensity image
/// @param pos Position offset from line center
/// @param stepDir Step direction for sampling
/// @param lstep Length step for sampling
/// @return Shared pointer to the created descriptor creator
template <class FT, class GT = LineSegment<FT>, class Helper = GchImgInterpolate<FT>>
typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr createGenericLRFdc(const cv::Mat& img,
                                                                                FT pos = 2,
                                                                                FT stepDir = 1,
                                                                                FT lstep = 1) {
  MatMap tmp;
  tmp["img"] = img;
  return typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGenericLR<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}

/// @brief Create a generic LR descriptor creator from gradient images.
/// @tparam FT Float type
/// @tparam GT Geometric type (default: LineSegment<FT>)
/// @tparam Helper Descriptor computation helper (default: GchGradInterpolate<FT>)
/// @param gx Horizontal gradient image
/// @param gy Vertical gradient image
/// @param pos Position offset from line center
/// @param stepDir Step direction for sampling
/// @param lstep Length step for sampling
/// @return Shared pointer to the created descriptor creator
template <class FT, class GT = LineSegment<FT>, class Helper = GchGradInterpolate<FT>>
typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr createGenericLRFdc(
    const cv::Mat& gx, const cv::Mat& gy, FT pos = 2, FT stepDir = 1, FT lstep = 1) {
  MatMap tmp;
  tmp["gx"] = gx;
  tmp["gy"] = gy;
  return typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGenericLR<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}

/// @brief Create a generic LR descriptor creator from gradient and intensity images.
/// @tparam FT Float type
/// @tparam GT Geometric type (default: LineSegment<FT>)
/// @tparam Helper Descriptor computation helper (default: GchGradImgInterpolate<FT>)
/// @param gx Horizontal gradient image
/// @param gy Vertical gradient image
/// @param img Input intensity image
/// @param pos Position offset from line center
/// @param stepDir Step direction for sampling
/// @param lstep Length step for sampling
/// @return Shared pointer to the created descriptor creator
template <class FT, class GT = LineSegment<FT>, class Helper = GchGradImgInterpolate<FT>>
typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr createGenericLRFdc(
    const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& img, FT pos = 2, FT stepDir = 1, FT lstep = 1) {
  MatMap tmp;
  tmp["gx"] = gx;
  tmp["gy"] = gy;
  tmp["img"] = img;
  std::cout << tmp.size() << std::endl;
  return typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGenericLR<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}
}  // namespace lsfm
