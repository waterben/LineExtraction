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

#ifndef _IMAGE_OPERATOR_HPP_
#define _IMAGE_OPERATOR_HPP_
#ifdef __cplusplus

#  include <opencv2/core/core.hpp>
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/photo/photo.hpp>

#  include <memory>
#  include <string>

namespace lsfm {

class ImageOperator {
  std::string name_;
  ImageOperator(const ImageOperator&);

 public:
  virtual ~ImageOperator() {}

  inline std::string name() const { return name_; }

  virtual void apply(cv::Mat& img) = 0;

  inline void apply(const cv::Mat& in, cv::Mat& out) {
    in.copyTo(out);
    apply(out);
  }

  inline void operator()(cv::Mat& img) { this->apply(img); }

  inline void operator()(const cv::Mat& in, cv::Mat& out) {
    in.copyTo(out);
    apply(out);
  }

 protected:
  ImageOperator(const std::string& n) : name_(n) {}
};

typedef std::shared_ptr<ImageOperator> ImageOperatorPtr;
typedef std::vector<ImageOperatorPtr> ImageOperatorPtrVec;


class PipelineOperator : public ImageOperator {
 public:
  std::vector<ImageOperatorPtr> pipePtr;
  std::vector<ImageOperator*> pipe;

  PipelineOperator() : ImageOperator("Pipeline") {}
  // PipelineOperator(const PipelineOperator& op) : ImageOperator("Pipeline"), pipe(op.pipe) {}
  virtual ~PipelineOperator() {}

  void push(const ImageOperatorPtr& op) {
    pipePtr.push_back(op);
    pipe.push_back(op.get());
  }

  void push(ImageOperator* op) {
    pipePtr.push_back(ImageOperatorPtr());
    pipe.push_back(op);
  }

  void pop() {
    pipe.pop_back();
    pipePtr.pop_back();
  }

  void clear() {
    pipe.clear();
    pipePtr.clear();
  }

  virtual void apply(cv::Mat& img) {
    for_each(pipe.begin(), pipe.end(), [&img](ImageOperator* op) { op->apply(img); });
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create() { return ImageOperatorPtr(new PipelineOperator()); }
};

template <class T>
class RotateOperator : public ImageOperator {
  T angle_;
  cv::Point_<T> pivot_;
  int interp_;

 public:
  RotateOperator(T a, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR)
      : ImageOperator("Rotate"), angle_(a), pivot_(p), interp_(i) {}
  // PipelineOperator(const RotateOperator& op) : ImageOperator("Rotate"), angle_(op.angle), pivot_(op.pivot_) {}
  virtual ~RotateOperator() {}

  virtual void apply(cv::Mat& img) {
    T a = cos(angle_), b = sin(angle_), _1 = 1;
    cv::Mat tmp;
    // rotate image
    cv::Matx<T, 2, 3> m(a, b, (_1 - a) * pivot_.x - b * pivot_.y, -b, a, b * pivot_.x + (_1 - a) * pivot_.y);
    cv::warpAffine(img, tmp, m, img.size(), interp_, cv::BORDER_REPLICATE);
    tmp = img;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(T a, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new RotateOperator(a, p, i));
  }
};

template <class T>
class ScaleOperator : public ImageOperator {
  cv::Point_<T> scale_, pivot_;
  int interp_;

 public:
  ScaleOperator(const cv::Point_<T>& s, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR)
      : ImageOperator("Scale"), scale_(s), pivot_(p), interp_(i) {}
  ScaleOperator(T s, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR)
      : ImageOperator("Scale"), scale_(s, s), pivot_(p), interp_(i) {}
  virtual ~ScaleOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // scale image
    cv::Matx<T, 2, 3> m(scale_.x, 0, (1 - scale_.x) * pivot_.x, 0, scale_.y, (1 - scale_.y) * pivot_.y);
    cv::warpAffine(img, tmp, m, img.size(), interp_);
    img = tmp;
  }


  using ImageOperator::apply;
  using ImageOperator::name;
  static ImageOperatorPtr create(const cv::Point_<T>& s,
                                 const cv::Point_<T>& p = cv::Point_<T>(0, 0),
                                 int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new ScaleOperator(s, p, i));
  }

  static ImageOperatorPtr create(T s, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new ScaleOperator(s, p, i));
  }
};

template <class T>
class TranslateOperator : public ImageOperator {
  cv::Point_<T> trans_;
  int interp_;

 public:
  TranslateOperator(const cv::Point_<T>& t, int i = cv::INTER_LINEAR)
      : ImageOperator("Translate"), trans_(t), interp_(i) {}
  virtual ~TranslateOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // translate image
    cv::Matx<T, 2, 3> m(1, 0, trans_.x, 0, 1, trans_.y);
    cv::warpAffine(img, tmp, m, img.size(), interp_);
    img = tmp;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(const cv::Point_<T>& t, int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new TranslateOperator(t, i));
  }
};

template <class T>
class AffineOperator : public ImageOperator {
  cv::Matx<T, 2, 3> map_;
  int interp_;

 public:
  AffineOperator(const cv::Matx<T, 2, 3>& m, int i = cv::INTER_LINEAR) : ImageOperator("Affine"), map_(m), interp_(i) {}
  virtual ~AffineOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // warpAffine image
    cv::warpAffine(img, tmp, map_, img.size(), interp_);
    img = tmp;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(const cv::Matx<T, 2, 3>& m, int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new AffineOperator(m, i));
  }
};

template <class T>
class PerspectiveOperator : public ImageOperator {
  cv::Matx<T, 3, 3> map_;
  int interp_;

 public:
  PerspectiveOperator(const cv::Matx<T, 3, 3>& m, int i = cv::INTER_LINEAR)
      : ImageOperator("Perspective"), map_(m), interp_(i) {}
  virtual ~PerspectiveOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // warpPerspective image
    cv::warpPerspective(img, tmp, map_, img.size(), interp_);
    img = tmp;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(const cv::Matx<T, 3, 3>& m, int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new PerspectiveOperator(m, i));
  }
};

class NoOp : public ImageOperator {
 public:
  NoOp() : ImageOperator("Noop") {}
  virtual ~NoOp() {}

  virtual void apply(cv::Mat& /*img*/) {}

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create() { return ImageOperatorPtr(new NoOp); }
};

class ResizeOperator : public ImageOperator {
  int width_, height_, interp_;

 public:
  ResizeOperator(int w, int h, int i = cv::INTER_LINEAR) : ImageOperator("Resize"), width_(w), height_(h), interp_(i) {}
  virtual ~ResizeOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // resize image
    cv::resize(img, tmp, cv::Size(width_, height_), 0, 0, interp_);
    img = tmp;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(int w, int h, int i = cv::INTER_LINEAR) {
    return ImageOperatorPtr(new ResizeOperator(w, h, i));
  }
};

class BlurOperator : public ImageOperator {
  int blur_;

 public:
  BlurOperator(int b) : ImageOperator("Blur"), blur_(b) {}
  virtual ~BlurOperator() {}

  virtual void apply(cv::Mat& img) {
    // blur image
    cv::blur(img, img, cv::Size(blur_, blur_));
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(int b) { return ImageOperatorPtr(new BlurOperator(b)); }
};

class GaussianBlurOperator : public ImageOperator {
  double sigma_;
  cv::Size ksize_;

 public:
  GaussianBlurOperator(double s, const cv::Size& ks = cv::Size(0, 0))
      : ImageOperator("GaussionBlur"), sigma_(s), ksize_(ks) {}
  GaussianBlurOperator(int b) : ImageOperator("GaussionBlur"), sigma_(0), ksize_(b, b) {}
  virtual ~GaussianBlurOperator() {}

  virtual void apply(cv::Mat& img) {
    // gaussian blur image
    cv::GaussianBlur(img, img, ksize_, sigma_);
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(double s, const cv::Size& ks = cv::Size(0, 0)) {
    return ImageOperatorPtr(new GaussianBlurOperator(s, ks));
  }
  static ImageOperatorPtr create(int b) { return ImageOperatorPtr(new GaussianBlurOperator(b)); }
};

class MedianBlurOperator : public ImageOperator {
  int ksize_;

 public:
  MedianBlurOperator(int ks = 3) : ImageOperator("MedianBlur"), ksize_(ks) {}
  virtual ~MedianBlurOperator() {}

  virtual void apply(cv::Mat& img) {
    // gaussian blur image
    cv::medianBlur(img, img, ksize_);
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(int ks = 3) { return ImageOperatorPtr(new MedianBlurOperator(ks)); }
};

class BilateralOperator : public ImageOperator {
  double sCol_, sSpace_;
  int d_;

 public:
  BilateralOperator(int d = 5, double sigmaColor = 50, double sigmaSpace = 50)
      : ImageOperator("BilateralFilter"), d_(d), sCol_(sigmaColor), sSpace_(sigmaSpace) {}
  virtual ~BilateralOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat tmp;
    // Bilateral filter -> not inplace
    cv::bilateralFilter(img, tmp, d_, sCol_, sSpace_);
    img = tmp;
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(int d = 5, double sigmaColor = 50, double sigmaSpace = 50) {
    return ImageOperatorPtr(new BilateralOperator(d, sigmaColor, sigmaSpace));
  }
};

class FastNlMeansOperator : public ImageOperator {
  float h_;
  int tmpWinSize_, searchWinSize_;

 public:
  FastNlMeansOperator(float h = 3, int templateWindowSize = 7, int searchWindowSize = 21)
      : ImageOperator("FastNlMeansFilter"), h_(h), tmpWinSize_(templateWindowSize), searchWinSize_(searchWindowSize) {}
  virtual ~FastNlMeansOperator() {}

  virtual void apply(cv::Mat& img) {
    if (img.depth() == 3)
      cv::fastNlMeansDenoisingColored(img, img, h_, h_, tmpWinSize_, searchWinSize_);
    else
      cv::fastNlMeansDenoising(img, img, h_, tmpWinSize_, searchWinSize_);
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(float h = 3, int templateWindowSize = 7, int searchWindowSize = 21) {
    return ImageOperatorPtr(new FastNlMeansOperator(h, templateWindowSize, searchWindowSize));
  }
};


class UniformNoiseOperator : public ImageOperator {
  double lower_, upper_;

 public:
  UniformNoiseOperator(double l, double u) : ImageOperator("UniformNoise"), lower_(l), upper_(u) {}
  virtual ~UniformNoiseOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat noise(img.size(), CV_64F, cv::Scalar(0));
    int type = img.type();
    img.convertTo(img, CV_MAKETYPE(CV_64F, img.channels()));
    if (img.channels() > 1) {
      std::vector<cv::Mat> channels;
      cv::split(img, channels);
      for_each(channels.begin(), channels.end(), [&](cv::Mat& ch) {
        // uniform noise image
        cv::randu(noise, lower_, upper_);
        ch += noise;
      });
      cv::merge(channels, img);
    } else {
      cv::randu(noise, lower_, upper_);
      img += noise;
    }
    img.convertTo(img, type);
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(double l, double u) { return ImageOperatorPtr(new UniformNoiseOperator(l, u)); }
};


class GaussianNoiseOperator : public ImageOperator {
  double sigma_, mean_;

 public:
  GaussianNoiseOperator(double s, double m = 0) : ImageOperator("GaussionNoise"), sigma_(s), mean_(m) {}
  virtual ~GaussianNoiseOperator() {}

  virtual void apply(cv::Mat& img) {
    cv::Mat noise(img.size(), CV_64F, cv::Scalar(0));
    int type = img.type();
    img.convertTo(img, CV_MAKETYPE(CV_64F, img.channels()));
    if (img.channels() > 1) {
      std::vector<cv::Mat> channels;
      cv::split(img, channels);
      for_each(channels.begin(), channels.end(), [&](cv::Mat& ch) {
        // gaussian noise image
        cv::randn(noise, this->mean_, this->sigma_);
        ch += noise;
      });
      cv::merge(channels, img);
    } else {
      // gaussian noise image
      cv::randn(noise, this->mean_, this->sigma_);
      img += noise;
    }
    img.convertTo(img, type);
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  static ImageOperatorPtr create(double s, double m = 0) { return ImageOperatorPtr(new GaussianNoiseOperator(s, m)); }
};

}  // namespace lsfm
#endif
#endif
