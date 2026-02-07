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

// Left Right Feature Descriptor
template <class FT, int cn>
struct LRDescritpor {
  LRDescritpor() {}
  LRDescritpor(const FT* d) : data() { memcopy(data, d, sizeof(FT) * (cn * 2)); }

  FT data[static_cast<size_t>(cn * 2)]{};

  inline FT* dataL() { return data; }

  inline FT* dataR() { return data + cn; }

  inline FT distance(const LRDescritpor<FT, cn>& rhs) const {
    // return static_cast<FT>(norm(cv::_InputArray(data,cn*2), cv::_InputArray(rhs.data,cn*2), NORM_L2));
    return static_cast<FT>(norm(cv::_InputArray(data, static_cast<int>(cn)),
                                cv::_InputArray(rhs.data, static_cast<int>(cn)), cv::NORM_L2)) *
           static_cast<FT>(norm(cv::_InputArray(data + cn, static_cast<int>(cn)),
                                cv::_InputArray(rhs.data + cn, static_cast<int>(cn)), cv::NORM_L2));
  }

  //! compute distance between two descriptors (static version)
  static inline FT distance(const LRDescritpor<FT, cn>& lhs, const LRDescritpor<FT, cn>& rhs) {
    // return static_cast<FT>(norm(cv::_InputArray(lhs.data,cn*2), cv::_InputArray(rhs.data,cn*2), NORM_L2));
    return static_cast<FT>(norm(cv::_InputArray(lhs.data, cn), cv::_InputArray(rhs.data, cn), cv::NORM_L2)) *
           static_cast<FT>(norm(cv::_InputArray(lhs.data + cn, cn), cv::_InputArray(rhs.data + cn, cn), cv::NORM_L2));
  }

  static inline int size() { return cn * 2; }

  std::string name() const { return "LR"; }
};


// Generic Feature Descriptor creator for gradient
template <class FT, class GT = LineSegment<FT>, class Helper = GchImgInterpolate<FT>>
class FdcGenericLR : public Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>> {
  inline void doCreate(const GT& input, FT* dst) {
    Helper::create(data_.data(), input, dst, pos_, stepDir_, lstep_);
    dst += Helper::dscSize;
    Helper::create(data_.data(), input, dst, -pos_, -stepDir_, lstep_);
  }

 public:
  typedef typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr FdcPtr;
  typedef typename FdcObj<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr CustomFdcPtr;
  typedef typename FdcMat<FT, GT>::Ptr SimpleFdcPtr;
  typedef LRDescritpor<FT, Helper::dscSize> descriptor_type;

  FdcGenericLR(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1)
      : data_(), pos_(pos), stepDir_(stepDir), lstep_(lstep) {
    data_.resize(Helper::inputData().size());
    this->setData(data);
  }

  static FdcPtr createFdc(const MatMap& data, FT pos = -1, FT stepDir = 1, FT lstep = 1) {
    return FdcPtr(new FdcGenericLR<FT, GT, Helper>(data, pos, stepDir, lstep));
  }

  using FdcMatI<FT, GT>::create;
  using FdcObjI<FT, GT, descriptor_type>::create;

  //! create single descriptor from single geometric object
  virtual void create(const GT& input, descriptor_type& dst) { doCreate(input, dst.data); }

  //! create single simple descriptor from geometric object
  virtual void create(const GT& input, cv::Mat& dst) {
    if (dst.empty() || dst.cols != descriptor_type::size())
      dst.create(1, descriptor_type::size(), cv::DataType<FT>::type);
    doCreate(input, dst.template ptr<FT>());
  }

  //! get size of single descriptor (cols in cv::Mat)
  virtual size_t size() const { return static_cast<size_t>(descriptor_type::size()); }

  //! allow to set internal processing data after init
  virtual void setData(const MatMap& data) {
    MatMap::const_iterator f;
    auto input = Helper::inputData();
    for (size_t i = 0; i != input.size(); ++i) {
      f = data.find(input[i]);
      if (f != data.end()) data_[i] = f->second;
    }
  }

  // input
  std::vector<cv::Mat> data_;
  FT pos_, stepDir_, lstep_;

 protected:
  virtual void create(const GT& input, FT* dst) { doCreate(input, dst); }
};

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

template <class FT, class GT = LineSegment<FT>, class Helper = GchGradInterpolate<FT>>
typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr createGenericLRFdc(
    const cv::Mat& gx, const cv::Mat& gy, FT pos = 2, FT stepDir = 1, FT lstep = 1) {
  MatMap tmp;
  tmp["gx"] = gx;
  tmp["gy"] = gy;
  return typename Fdc<FT, GT, LRDescritpor<FT, Helper::dscSize>>::Ptr(
      new FdcGenericLR<FT, GT, Helper>(tmp, pos, stepDir, lstep));
}

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
