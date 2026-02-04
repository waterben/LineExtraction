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

/// @file image_operator.hpp
/// @brief Image preprocessing operators for pipeline-based processing.
///
/// This file provides a framework for composable image operations including
/// geometric transforms (rotation, scaling, translation), filtering (blur,
/// Gaussian, bilateral, median), and noise generation. Operators can be
/// chained into processing pipelines.

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef HAVE_OPENCV_PHOTO
#  include <opencv2/photo/photo.hpp>
#endif

#include <memory>
#include <string>

namespace lsfm {

/// @brief Abstract base class for image processing operators.
///
/// Provides a common interface for image transformation operations that
/// can be applied in-place or with separate input/output images. Operators
/// can be composed into pipelines using PipelineOperator.
class ImageOperator {
  std::string name_;                    ///< Operator name for identification.
  ImageOperator(const ImageOperator&);  ///< Deleted copy constructor.

 public:
  virtual ~ImageOperator() {}

  /// @brief Get the operator name.
  /// @return Name string identifying the operator type.
  inline std::string name() const { return name_; }

  /// @brief Apply the operator to an image in-place.
  /// @param[in,out] img Image to transform.
  virtual void apply(cv::Mat& img) = 0;

  /// @brief Apply the operator with separate input/output images.
  /// @param[in] in Input image.
  /// @param[out] out Output image (copy of input with transformation applied).
  inline void apply(const cv::Mat& in, cv::Mat& out) {
    in.copyTo(out);
    apply(out);
  }

  /// @brief Function call operator for in-place application.
  inline void operator()(cv::Mat& img) { this->apply(img); }

  /// @brief Function call operator with separate input/output.
  inline void operator()(const cv::Mat& in, cv::Mat& out) {
    in.copyTo(out);
    apply(out);
  }

 protected:
  /// @brief Construct an operator with a name.
  /// @param[in] n Operator name string.
  ImageOperator(const std::string& n) : name_(n) {}
};

typedef std::shared_ptr<ImageOperator> ImageOperatorPtr;    ///< Shared pointer type.
typedef std::vector<ImageOperatorPtr> ImageOperatorPtrVec;  ///< Vector of operator pointers.


/// @brief Operator pipeline for chaining multiple image operations.
///
/// Allows building a sequence of image operators that are applied
/// in order to an input image.
class PipelineOperator : public ImageOperator {
 public:
  std::vector<ImageOperatorPtr> pipePtr;  ///< Owned operator pointers.
  std::vector<ImageOperator*> pipe;       ///< Raw pointers for execution.

  /// @brief Construct an empty pipeline.
  PipelineOperator() : ImageOperator("Pipeline"), pipePtr{}, pipe{} {}
  virtual ~PipelineOperator() {}

  /// @brief Add an owned operator to the pipeline.
  /// @param[in] op Shared pointer to operator (takes ownership).
  void push(const ImageOperatorPtr& op) {
    pipePtr.push_back(op);
    pipe.push_back(op.get());
  }

  /// @brief Add a non-owned operator to the pipeline.
  /// @param[in] op Raw pointer to operator (caller retains ownership).
  void push(ImageOperator* op) {
    pipePtr.push_back(ImageOperatorPtr());
    pipe.push_back(op);
  }

  /// @brief Remove the last operator from the pipeline.
  void pop() {
    pipe.pop_back();
    pipePtr.pop_back();
  }

  /// @brief Clear all operators from the pipeline.
  void clear() {
    pipe.clear();
    pipePtr.clear();
  }

  /// @brief Apply all operators in sequence.
  /// @param[in,out] img Image to process through the pipeline.
  virtual void apply(cv::Mat& img) {
    for_each(pipe.begin(), pipe.end(), [&img](ImageOperator* op) { op->apply(img); });
  }

  using ImageOperator::apply;
  using ImageOperator::name;

  /// @brief Factory method to create a pipeline operator.
  static ImageOperatorPtr create() { return ImageOperatorPtr(new PipelineOperator()); }
};

/// @brief Rotate an image around a pivot point.
/// @tparam T Numeric type for angle and coordinates (e.g., float, double).
template <class T>
class RotateOperator : public ImageOperator {
  T angle_;              ///< Rotation angle in radians.
  cv::Point_<T> pivot_;  ///< Center of rotation.
  int interp_;           ///< Interpolation method.

 public:
  /// @brief Construct a rotation operator.
  /// @param[in] a Rotation angle in radians.
  /// @param[in] p Pivot point (center of rotation, default: origin).
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
  RotateOperator(T a, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR)
      : ImageOperator("Rotate"), angle_(a), pivot_(p), interp_(i) {}
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

/// @brief Scale an image around a pivot point.
/// @tparam T Numeric type for scale factors and coordinates.
template <class T>
class ScaleOperator : public ImageOperator {
  cv::Point_<T> scale_;  ///< Scale factors (x, y).
  cv::Point_<T> pivot_;  ///< Center of scaling.
  int interp_;           ///< Interpolation method.

 public:
  /// @brief Construct with separate X/Y scale factors.
  /// @param[in] s Scale factors as (sx, sy).
  /// @param[in] p Pivot point (default: origin).
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
  ScaleOperator(const cv::Point_<T>& s, const cv::Point_<T>& p = cv::Point_<T>(0, 0), int i = cv::INTER_LINEAR)
      : ImageOperator("Scale"), scale_(s), pivot_(p), interp_(i) {}

  /// @brief Construct with uniform scale factor.
  /// @param[in] s Uniform scale factor.
  /// @param[in] p Pivot point (default: origin).
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
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

/// @brief Translate (shift) an image by a fixed offset.
/// @tparam T Numeric type for translation coordinates.
template <class T>
class TranslateOperator : public ImageOperator {
  cv::Point_<T> trans_;  ///< Translation offset (dx, dy).
  int interp_;           ///< Interpolation method.

 public:
  /// @brief Construct a translation operator.
  /// @param[in] t Translation offset as (dx, dy).
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
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

/// @brief Apply an affine transformation to an image.
/// @tparam T Numeric type for transformation matrix elements.
template <class T>
class AffineOperator : public ImageOperator {
  cv::Matx<T, 2, 3> map_;  ///< 2x3 affine transformation matrix.
  int interp_;             ///< Interpolation method.

 public:
  /// @brief Construct an affine transformation operator.
  /// @param[in] m 2x3 affine transformation matrix.
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
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

/// @brief Apply a perspective transformation to an image.
/// @tparam T Numeric type for transformation matrix elements.
template <class T>
class PerspectiveOperator : public ImageOperator {
  cv::Matx<T, 3, 3> map_;  ///< 3x3 perspective transformation matrix.
  int interp_;             ///< Interpolation method.

 public:
  /// @brief Construct a perspective transformation operator.
  /// @param[in] m 3x3 perspective (homography) matrix.
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
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

/// @brief No-operation placeholder operator.
///
/// Passes through images unchanged. Useful as a placeholder in pipelines.
class NoOp : public ImageOperator {
 public:
  /// @brief Construct a no-op operator.
  NoOp() : ImageOperator("Noop") {}
  virtual ~NoOp() {}

  /// @brief Does nothing to the image.
  virtual void apply(cv::Mat& /*img*/) {}

  using ImageOperator::apply;
  using ImageOperator::name;

  /// @brief Factory method.
  static ImageOperatorPtr create() { return ImageOperatorPtr(new NoOp); }
};

/// @brief Resize an image to specified dimensions.
class ResizeOperator : public ImageOperator {
  int width_;   ///< Target width.
  int height_;  ///< Target height.
  int interp_;  ///< Interpolation method.

 public:
  /// @brief Construct a resize operator.
  /// @param[in] w Target width in pixels.
  /// @param[in] h Target height in pixels.
  /// @param[in] i Interpolation method (default: cv::INTER_LINEAR).
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

/// @brief Apply box blur (uniform averaging) filter.
class BlurOperator : public ImageOperator {
  int blur_;  ///< Kernel size for blur.

 public:
  /// @brief Construct a blur operator.
  /// @param[in] b Kernel size (blur x blur).
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

/// @brief Apply Gaussian blur filter.
class GaussianBlurOperator : public ImageOperator {
  double sigma_;    ///< Gaussian sigma (standard deviation).
  cv::Size ksize_;  ///< Kernel size (computed from sigma if zero).

 public:
  /// @brief Construct with sigma and optional kernel size.
  /// @param[in] s Gaussian sigma.
  /// @param[in] ks Kernel size (if zero, computed from sigma).
  GaussianBlurOperator(double s, const cv::Size& ks = cv::Size(0, 0))
      : ImageOperator("GaussionBlur"), sigma_(s), ksize_(ks) {}

  /// @brief Construct with explicit kernel size.
  /// @param[in] b Kernel size (sigma computed automatically).
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

/// @brief Apply median blur filter for noise reduction.
///
/// Median filtering is effective for salt-and-pepper noise removal
/// while preserving edges.
class MedianBlurOperator : public ImageOperator {
  int ksize_;  ///< Kernel size (must be odd).

 public:
  /// @brief Construct a median blur operator.
  /// @param[in] ks Kernel size, must be odd (default: 3).
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

/// @brief Apply bilateral filter for edge-preserving smoothing.
///
/// Bilateral filtering smooths images while keeping edges sharp by considering
/// both spatial distance and color similarity.
class BilateralOperator : public ImageOperator {
  double sCol_;    ///< Filter sigma in color space.
  double sSpace_;  ///< Filter sigma in coordinate space.
  int d_;          ///< Diameter of pixel neighborhood.

 public:
  /// @brief Construct a bilateral filter operator.
  /// @param[in] d Diameter of neighborhood (if <= 0, computed from sigmaSpace).
  /// @param[in] sigmaColor Filter sigma in color space.
  /// @param[in] sigmaSpace Filter sigma in coordinate space.
  BilateralOperator(int d = 5, double sigmaColor = 50, double sigmaSpace = 50)
      : ImageOperator("BilateralFilter"), sCol_(sigmaColor), sSpace_(sigmaSpace), d_(d) {}
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

#ifdef HAVE_OPENCV_PHOTO
/// @brief Apply Fast Non-Local Means Denoising filter.
///
/// Advanced denoising filter that searches for similar patches in the image
/// to perform averaging, providing excellent noise removal while preserving
/// texture and detail.
/// @note Requires OpenCV photo module (opencv_photo).
class FastNlMeansOperator : public ImageOperator {
  float h_;            ///< Filter strength (higher h removes more noise).
  int tmpWinSize_;     ///< Template window size (should be odd).
  int searchWinSize_;  ///< Search window size (should be odd).

 public:
  /// @brief Construct a Fast NL Means denoising operator.
  /// @param[in] h Filter strength parameter (default: 3).
  /// @param[in] templateWindowSize Size of template patch (default: 7).
  /// @param[in] searchWindowSize Size of search area (default: 21).
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
#endif  // HAVE_OPENCV_PHOTO


/// @brief Add uniform random noise to an image.
///
/// Adds noise sampled from a uniform distribution U[lower, upper] to each pixel.
class UniformNoiseOperator : public ImageOperator {
  double lower_;  ///< Lower bound of uniform distribution.
  double upper_;  ///< Upper bound of uniform distribution.

 public:
  /// @brief Construct a uniform noise operator.
  /// @param[in] l Lower bound of noise range.
  /// @param[in] u Upper bound of noise range.
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


/// @brief Add Gaussian random noise to an image.
///
/// Adds noise sampled from a Gaussian (normal) distribution N(mean, sigma²)
/// to each pixel. Commonly used for simulating sensor noise.
class GaussianNoiseOperator : public ImageOperator {
  double sigma_;  ///< Standard deviation of Gaussian noise.
  double mean_;   ///< Mean of Gaussian noise.

 public:
  /// @brief Construct a Gaussian noise operator.
  /// @param[in] s Standard deviation (sigma) of noise.
  /// @param[in] m Mean of noise distribution (default: 0).
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
