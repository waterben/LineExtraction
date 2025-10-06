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

#include "opencv2/imgproc/imgproc.hpp"
#include <imgproc/filter.hpp>


namespace lsfm {

//! Laplace base class
//! Use IT to define Image type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use LT to define Laplace type (int, float or double)
//! For 8 Bit images use GT = short, for 16 Bit images float.
//! For images with floating values, use IT and GT = image floating type (float or double)
template <class IT, class LT>
class LaplaceI : public FilterI<IT> {
  LaplaceI(const LaplaceI&);

 public:
  typedef IT img_type;
  typedef Range<IT> IntensityRange;
  typedef LT laplace_type;
  typedef Range<LT> LaplaceRange;

  LaplaceI() {}
  virtual ~LaplaceI() {}

  //! get magnitude
  virtual cv::Mat laplace() const = 0;

  //! get magnitude range for intensity range
  virtual LaplaceRange laplaceRange() const = 0;

  //! convert threshold between 0-1 to magnitude threshold
  virtual LT laplaceThreshold(double val) const { return static_cast<LT>(laplaceRange().size() * val); }
};

//! Laplace base class helper
template <class IT, class LT>
class Laplace : public LaplaceI<IT, LT> {
 protected:
  Range<IT> intRange_;

 public:
  typedef IT img_type;
  typedef Range<IT> IntensityRange;
  typedef LT laplace_type;
  typedef Range<LT> LaplaceRange;

  Laplace(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : intRange_(int_lower, int_upper) {}


  //! get image intensity range (for single channel)
  IntensityRange intensityRange() const { return intRange_; }

  //! generic interface to get processed data
  virtual FilterResults results() const {
    FilterResults ret;
    ret["laplace"] = FilterData(this->laplace(), this->laplaceRange());
    return ret;
  }
};

template <class IT, class LT>
class LaplaceSimple : public Laplace<IT, LT> {
 protected:
  cv::Mat laplace_;
  cv::Mat_<LT> k_;
  cv::Point anchor;
  using Laplace<IT, LT>::intRange_;

 public:
  typedef IT img_type;
  typedef Range<IT> IntensityRange;
  typedef LT laplace_type;
  typedef Range<LT> LaplaceRange;

  LaplaceSimple(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : Laplace<IT, LT>(int_lower, int_upper), anchor(-1, -1) {
    k_ = cv::Mat_<LT>::ones(3, 3);
    k_(1, 1) = -8;
  }


  //! process laplacian
  void process(const cv::Mat& img) {
    cv::filter2D(img, laplace_, cv::DataType<LT>::type, k_, anchor, 0, cv::BORDER_REFLECT_101);
  }

  //! get magnitude
  cv::Mat laplace() const { return laplace_; }

  //! get magnitude range for intensity range
  virtual LaplaceRange laplaceRange() const {
    LT val = intRange_.upper * 8;
    return LaplaceRange(-val, val);
  }

  //! get name of gradient operator
  std::string name() const { return "laplace"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Laplace<IT, LT>::intensityRange;
  using Laplace<IT, LT>::results;
};

template <class IT, class LT>
class LoG : public LaplaceSimple<IT, LT> {
  int ksize_;
  double kspace_, kscale_;
  using LaplaceSimple<IT, LT>::intRange_;
  Range<LT> laplaceRange_;

 public:
  static inline double exp_d2(double x, double y, double s) {
    double xy2 = x * x + y * y;
    return s * (xy2 - 1) * std::exp(-xy2);
  }

  static inline cv::Mat_<LT> createFilter(int width, double spacing, double scale) {
    width = width / 2;
    cv::Mat_<LT> kernel(width * 2 + 1, width * 2 + 1);
    for (int i = -width; i <= width; ++i)
      for (int j = -width; j <= width; ++j)
        kernel(j + width, i + width) = static_cast<LT>(exp_d2(i * spacing, j * spacing, scale));
        // zero dc
#ifndef DISABLE_DC_ZERO_FIX
    kernel -= cv::sum(kernel)[0] / (kernel.size().area());
#endif

    return kernel;
  }

 private:
  void create_kernel() {
    this->k_ = createFilter(ksize_, kspace_, kscale_);
    cv::Mat_<LT> tmp;
    this->k_.copyTo(tmp);
    tmp.setTo(0, tmp < 0);
    laplaceRange_.upper = static_cast<LT>(cv::sum(tmp)[0] * intRange_.upper);
    this->k_.copyTo(tmp);
    tmp.setTo(0, tmp > 0);
    laplaceRange_.lower = static_cast<LT>(cv::sum(tmp)[0] * intRange_.upper);
    // std::cout << this->k_ << std::endl << laplaceRange_.upper << std::endl << laplaceRange_.lower << std::endl;
  }


 public:
  typedef IT img_type;
  typedef Range<IT> IntensityRange;
  typedef LT laplace_type;
  typedef Range<LT> LaplaceRange;

  LoG(int kernel_size = 5,
      double kernel_spacing = 1.008,
      double kernel_scale = 1,
      IT int_lower = std::numeric_limits<IT>::lowest(),
      IT int_upper = std::numeric_limits<IT>::max())
      : LaplaceSimple<IT, LT>(int_lower, int_upper),
        ksize_(kernel_size),
        kspace_(kernel_spacing),
        kscale_(kernel_scale) {
    this->add("grad_kernel_size", std::bind(&LoG<IT, LT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for LoG-Operator.");
    this->add("grad_kernel_spacing", std::bind(&LoG<IT, LT>::valueKernelSpacing, this, std::placeholders::_1),
              "Spacing for a single step for LoG-Operator.");
    this->add("grad_kernel_scale", std::bind(&LoG<IT, LT>::valueKernelScale, this, std::placeholders::_1),
              "Upscale for LoG-Operator (e.g. for converting to short).");

    create_kernel();
  }

  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  //! get kernel size
  int kernelSize() const { return ksize_; }

  //! set kernel size (range 2-99, has to be odd, even will be corrected to ksize+1)
  //! Note: large kernels needs larger GT type like int or long long int
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    ksize_ = ks;
    create_kernel();
  }

  Value valueKernelSpacing(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSpacing(ks);
    return kspace_;
  }

  double kernelSpacing() const { return kspace_; }

  void kernelSpacing(double ks) {
    if (ks == kspace_ || ks <= 0) return;
    kspace_ = ks;
    create_kernel();
  }

  Value valueKernelScale(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelScale(ks);
    return kscale_;
  }

  double kernelScale() const { return kscale_; }

  void kernelScale(double ks) {
    if (ks == kscale_ || ks <= 0) return;
    kscale_ = ks;
    create_kernel();
  }

  cv::Mat kernel() const { return this->k_; }

  cv::Mat even() const { return this->laplace(); }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using LaplaceSimple<IT, LT>::process;
  using LaplaceSimple<IT, LT>::laplace;
  using LaplaceSimple<IT, LT>::intensityRange;
  using LaplaceSimple<IT, LT>::results;

  //! get magnitude range for intensity range
  virtual LaplaceRange laplaceRange() const { return laplaceRange_; }

  //! get name of gradient operator
  std::string name() const { return "LoG"; }
};

template <class IT, class LT>
class LaplaceCV : public Laplace<IT, LT> {
  Range<LT> laplaceRange_;
  cv::Mat laplace_;
  int ksize_;

  void calc_range() {
    // TODO
    laplaceRange_.lower = static_cast<LT>(-this->intRange_.upper * 2 * ksize_);
    laplaceRange_.upper = static_cast<LT>(this->intRange_.upper * 2 * ksize_);
  }

 public:
  typedef IT img_type;
  typedef Range<IT> IntensityRange;
  typedef LT laplace_type;
  typedef Range<LT> LaplaceRange;

  LaplaceCV(int ksize = 5,
            IT int_lower = std::numeric_limits<IT>::lowest(),
            IT int_upper = std::numeric_limits<IT>::max())
      : Laplace<IT, LT>(int_lower, int_upper), ksize_(ksize) {
    this->add("grad_kernel_size", std::bind(&LaplaceCV<IT, LT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Laplace-Operator.");
    calc_range();
  }

  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks.getInt());
    return ksize_;
  }

  //! Get kernel size
  int kernelSize() const { return ksize_; }


  //! Set kernel size (range 1-31, has to be odd, even will be corrected to ksize+1)
  //! Note: large kernels needs larger GT type like float or double
  void kernelSize(int ks) {
    if (ksize_ == ks) return;
    if (ks < 1) ks = 1;
    if (ks % 2 == 0) ++ks;
    if (ks > 31) ks = 31;
    ksize_ = ks;
    calc_range();
  }


  //! process laplacian
  void process(const cv::Mat& img) {
    cv::Laplacian(img, laplace_, cv::DataType<LT>::type, ksize_, 1, 0, cv::BORDER_REFLECT_101);
  }

  //! get magnitude
  cv::Mat laplace() const { return laplace_; }

  //! get magnitude range for intensity range
  LaplaceRange laplaceRange() const { return laplaceRange_; }

  //! get name of gradient operator
  std::string name() const { return "laplaceCV"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Laplace<IT, LT>::intensityRange;
  using Laplace<IT, LT>::results;
};

}  // namespace lsfm
