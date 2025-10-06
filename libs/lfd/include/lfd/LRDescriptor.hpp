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

// C by Benjamin Wassermann
//


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
      : pos_(pos), stepDir_(stepDir), lstep_(lstep) {
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
