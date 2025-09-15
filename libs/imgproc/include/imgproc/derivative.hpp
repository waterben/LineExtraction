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

#ifndef _DERIVATIVE_HPP_
#define _DERIVATIVE_HPP_
#ifdef __cplusplus

#  include <utility/value_manager.hpp>
#  include <imgproc/gaussian.hpp>
#  include <opencv2/imgproc/imgproc.hpp>


namespace lsfm {

//! Store derivative max values for intensity change of 1
//! 1st max possible value for one direction
//! 2nd max possible value if both direction are equaly included
//! 3rd max possible value for second direction, if frist is max
template <class GT = short>
struct DerivativeMax {
  typedef GT grad_type;

  DerivativeMax(grad_type mf = 0, grad_type ms = 0, grad_type mt = 0) : max_1st(mf), max_2nd(ms), max_3rd(mt) {}
  grad_type max_1st, max_2nd, max_3rd;
};

//! Derivative base class
//! Use IT to define Image Type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use GT to define Derivative Type (short, float or double)
//! For 8 Bit images use GT = short, for 16 Bit images float.
//! For images with floating values, use IT and GT = image floating type (float or double)
template <class IT = uchar, class GT = short>
struct Derivative {
  //! Compute derivative gx and gy from image
  //! 8bit images should produce gx and gy of type short
  //! 16bit/32bit images should produce gx and gy of type float
  //! float images should produce gy and gy of type float/double
  virtual void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const = 0;

  //! get max derivative values for intensity change of 1
  //! 1st max possible value for one direction
  //! 2nd max possible value if both direction are equaly included
  //! 3rd max possible value for second direction, if frist is max
  virtual DerivativeMax<GT> max() const = 0;

  //! get name of derivative operator
  virtual std::string name() const = 0;
};


//! Roberts Derivative
template <class IT, class GT = short>
class RobertsDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  mutable cv::Mat da, bc;
  cv::Point anchor;

 public:
  typedef IT img_type;
  typedef GT grad_type;

  RobertsDerivative() : anchor(0, 0) {
    kx.create(2, 2, cv::DataType<GT>::type);
    kx.at<GT>(0, 0) = -1;
    kx.at<GT>(1, 1) = 1;
    kx.at<GT>(0, 1) = 0;
    kx.at<GT>(1, 0) = 0;

    ky.create(2, 2, cv::DataType<GT>::type);
    ky.at<GT>(0, 0) = 0;
    ky.at<GT>(1, 1) = 0;
    ky.at<GT>(0, 1) = 1;
    ky.at<GT>(1, 0) = -1;
  }

  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    cv::filter2D(img, da, cv::DataType<GT>::type, kx, anchor, 0, cv::BORDER_REFLECT_101);
    cv::filter2D(img, bc, cv::DataType<GT>::type, ky, anchor, 0, cv::BORDER_REFLECT_101);

    gx = da + bc;
    gy = da - bc;
  }


  DerivativeMax<GT> max() const { return DerivativeMax<GT>(2, 1, 0); }

  //! get name of derivative operator
  std::string name() const { return "roberts"; }
};

//! Prewitt Derivative
template <class IT, class GT = short>
class PrewittDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  cv::Point anchor;

 public:
  typedef IT img_type;
  typedef GT grad_type;

  PrewittDerivative() : anchor(-1, -1) {
    kx.create(3, 1, cv::DataType<GT>::type);
    kx.at<GT>(0) = -1;
    kx.at<GT>(1) = 0;
    kx.at<GT>(2) = 1;
    ky.create(3, 1, cv::DataType<GT>::type);
    ky.at<GT>(0) = 1;
    ky.at<GT>(1) = 1;
    ky.at<GT>(2) = 1;
  }


  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "prewitt: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  DerivativeMax<GT> max() const { return DerivativeMax<GT>(3, 2, 1); }

  //! get name of derivative operator
  std::string name() const { return "prewitt"; }
};


//! Sobel Derivative
template <class IT = uchar, class GT = short>
class SobelDerivative : public Derivative<IT, GT>, public ValueManager {
  cv::Mat kx, ky;
  DerivativeMax<GT> gm;
  cv::Point anchor;
  int ksize_;

 public:
  typedef IT img_type;
  typedef GT grad_type;

  SobelDerivative(int kernel_size = 3) : gm(4, 3, 2), anchor(-1, -1), ksize_(0) {
    this->add("grad_kernel_size", std::bind(&SobelDerivative<IT, GT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Sobel-Operator.");
    kernelSize(kernel_size);
  }

  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks.getInt());
    return ksize_;
  }

  //! Get kernel size
  int kernelSize() const { return ksize_; }


  //! Set kernel size (range 3-31, has to be odd, even will be corrected to ksize+1)
  //! Note: large kernels needs larger GT type like float or double
  void kernelSize(int ks) {
    if (ksize_ == ks) return;
    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 31) ks = 31;

    cv::getDerivKernels(kx, ky, 1, 0, ks, false, std::max<int>(CV_32F, cv::DataType<GT>::type));
    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
    GT s = static_cast<GT>(sum(ky)[0]);
    GT c = ky.at<GT>(ks / 2);
    gm.max_1st = gm.max_2nd = gm.max_3rd = 0;
    for (int i = ks / 2 + 1; i != ks; ++i) {
      GT v = kx.at<GT>(i);
      gm.max_1st += s * v;
      gm.max_3rd += c * v;
    }
    for (int i = 0; i != ks / 2; ++i) {
      GT sc = std::abs(kx.at<GT>(i));
      for (int j = i; j < ks - i - 1; ++j) gm.max_2nd += ky.at<GT>(j) * sc;
    }
    ksize_ = ks;
  }

  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "sobel: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }


  DerivativeMax<GT> max() const { return gm; }

  //! get name of derivative operator
  std::string name() const { return "sobel"; }
};

//! Scharr Derivative
template <class IT = uchar, class GT = short>
class ScharrDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  cv::Point anchor;

 public:
  typedef IT img_type;
  typedef GT grad_type;

  ScharrDerivative() : anchor(-1, -1) {
    cv::getDerivKernels(kx, ky, 1, 0, -1, false, std::max<int>(CV_32F, cv::DataType<GT>::type));
    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
  }

  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "scharr: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  DerivativeMax<GT> max() const { return DerivativeMax<GT>(16, 13, 10); }

  //! get name of derivative operator
  std::string name() const { return "scharr"; }
};


//! Gaussian Derivative
template <class IT = uchar, class GT = short>
class GaussianDerivative : public Derivative<IT, GT>, public ValueManager {
  cv::Mat kx, ky;
  DerivativeMax<GT> gm;
  cv::Point anchor;
  int ksize_;
  double range_, scale_;

  void create_kernel(int ksize, double range, double scale) {
    kx = gaussianD1<double>(ksize, range);
    kx *= std::sqrt(scale);
    cv::Mat kx2 = kx.t();
    ky = gaussian<double>(ksize, range).t() * sqrt(scale);
    // std::cout << kx * ky.t() << std::endl;

    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
    // std::cout << "gauss: " << kx << "; " << ky << std::endl;

    GT s = static_cast<GT>(sum(ky)[0]);
    GT c = ky.at<GT>(ksize / 2);
    gm.max_1st = gm.max_2nd = gm.max_3rd = 0;
    for (int i = ksize / 2 + 1; i != ksize; ++i) {
      GT v = kx.at<GT>(i);
      gm.max_1st += s * v;
      gm.max_3rd += c * v;
    }
    for (int i = 0; i != ksize / 2; ++i) {
      GT sc = std::abs(kx.at<GT>(i));
      for (int j = i; j < ksize - i - 1; ++j) gm.max_2nd += ky.at<GT>(j) * sc;
    }

    // std::cout << gm.max_1st << " " << gm.max_2nd << " " << gm.max_3rd << std::endl;
  }

 public:
  GaussianDerivative(int kernel_size = 5, double range = 3, double scale = 1)
      : gm(0, 0, 0), anchor(-1, -1), ksize_(0), range_(range), scale_(scale) {
    CV_Assert(kernel_size > 2 && range > 0 && scale > 0);

    this->add("grad_kernel_size", std::bind(&GaussianDerivative<IT, GT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Gaussian-Operator.");
    this->add("grad_range", std::bind(&GaussianDerivative<IT, GT>::valueRange, this, std::placeholders::_1),
              "Parameter range for Gaussian-Operator.");
    this->add("grad_scale", std::bind(&GaussianDerivative<IT, GT>::valueScale, this, std::placeholders::_1),
              "Upscale for Gaussian-Operator (max value in operator == upscale).");
    kernelSize(kernel_size);
  }

  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  //! get kernel size
  int kernelSize() const { return ksize_; }

  //! set kernel size (range 3-99, has to be odd, even will be corrected to ksize+1)
  //! Note: large kernels needs larger GT type like int or long long int
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    create_kernel(ks, range(), scale());
    ksize_ = ks;
  }


  Value valueRange(const Value& r = Value::NAV()) {
    if (r.type()) range(r);
    return range_;
  }

  //! get range
  double range() const { return range_; }
  //! set range (range 0-X), 0: determine range by kernel size and sigma
  void range(double r) {
    if (r <= 0 || r == range_) return;
    create_kernel(ksize_, r, scale_);
    range_ = r;
  }

  Value valueScale(const Value& s = Value::NAV()) {
    if (s.type()) scale(s);
    return scale_;
  }

  //! get scale
  double scale() const { return scale_; }
  //! set scale (range 0-X), 0: no scaling
  //! Will compute factor beween greates value in kernel and scale value and scale
  //! up the kernel by this factor
  void scale(double sc) {
    if (sc <= 0 || sc == scale_) return;
    create_kernel(ksize_, range_, sc);
    scale_ = sc;
  }

  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  DerivativeMax<GT> max() const { return gm; }

  //! get name of derivative operator
  std::string name() const { return "gauss"; }
};
}  // namespace lsfm
#endif
#endif
