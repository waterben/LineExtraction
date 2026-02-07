//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file gradient_adapter.hpp
/// @brief Adapter classes for using quadrature filters as gradient operators.
///
/// This file provides adapter classes that wrap quadrature filter implementations
/// to expose them through the standard Gradient interface. This allows quadrature-
/// based edge detection methods to be used interchangeably with traditional
/// gradient operators.

#pragma once

#include <imgproc/gradient.hpp>

namespace lsfm {

/// @brief Adapter to use the odd component of a quadrature filter as gradient.
///
/// Wraps a quadrature filter (QUAD) and exposes its odd filter response
/// (imaginary part) as gradient magnitude. The odd filter response corresponds
/// to edges in the image.
/// @tparam QUAD Quadrature filter type (e.g., Quadrature, QuadratureS).
template <class QUAD>
class GradientOdd : public GradientI<typename QUAD::img_type,
                                     typename QUAD::grad_type,
                                     typename QUAD::mag_type,
                                     typename QUAD::dir_type> {
  typedef GradientI<typename QUAD::img_type, typename QUAD::grad_type, typename QUAD::mag_type, typename QUAD::dir_type>
      GRAD;

 protected:
  QUAD quad_;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;  ///< Magnitude range type.
  typedef typename QUAD::DirectionRange DirectionRange;  ///< Direction range type.

  /// @brief Construct a gradient adapter with default quadrature parameters.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  GradientOdd(img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_({}, int_lower, int_upper) {
    this->addManager(quad_);
  }

  /// @brief Construct with named options.
  /// @param options Configuration options for the quadrature filter.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  GradientOdd(const ValueManager::NameValueVector& options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_(options, int_lower, int_upper) {
    this->addManager(quad_);
  }

  /// @brief Construct with initializer list options.
  /// @param options Configuration options for the quadrature filter.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  GradientOdd(ValueManager::InitializerList options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_(options, int_lower, int_upper) {
    this->addManager(quad_);
  }

  /// @brief Get gradient magnitude from odd filter response.
  /// @return Magnitude image from the odd (edge-detecting) filter.
  virtual cv::Mat magnitude() const { return quad_.odd(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  virtual MagnitudeRange magnitudeRange() const { return quad_.oddRange(); }

  /// @brief Convert normalized threshold to magnitude threshold.
  /// @param val Normalized threshold value in [0, 1].
  /// @return Absolute magnitude threshold value.
  virtual mag_type magnitudeThreshold(double val) const { return quad_.oddThreshold(val); }

  /// @brief Get norm type information.
  /// @return Norm type (typically NONE for quadrature-based methods).
  virtual NormType normType() const { return quad_.normType(); }

  /// @brief Get gradient direction image.
  /// @return Direction image from the quadrature filter.
  virtual cv::Mat direction() const { return quad_.direction(); }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  virtual DirectionRange directionRange() const { return quad_.directionRange(); }

  /// @brief Get gradient components from odd filter.
  /// @param grad_x X-component of the odd filter response.
  /// @param grad_y Y-component of the odd filter response.
  virtual void directionals(cv::Mat& grad_x, cv::Mat& grad_y) const { quad_.odd(grad_x, grad_y); }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  virtual GradientRange gradientRange() const { return quad_.oddGradRange(); }

  /// @brief Get image intensity range.
  /// @return Valid intensity range for input images.
  IntensityRange intensityRange() const { return quad_.intensityRange(); }

  /// @brief Process an image with the quadrature filter.
  /// @param img Input image to process.
  virtual void process(const cv::Mat& img) { quad_.process(img); }

  /// @brief Get all filter results as a map.
  /// @return Map of named result images including gx, gy, mag, dir.
  virtual FilterResults results() const {
    FilterResults ret = quad_.results();
    ret["gx"] = this->gx();
    ret["gy"] = this->gy();
    ret["mag"] = this->magnitude();
    ret["dir"] = this->direction();
    return ret;
  }

  /// @brief Get the name of the underlying quadrature filter.
  /// @return Filter name string.
  std::string name() const { return quad_.name(); }

  using GRAD::gx;
  using GRAD::gy;
  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
};

/// @brief Adapter to use local energy from a quadrature filter as gradient.
///
/// Wraps a quadrature filter (QUAD) and exposes its local energy response
/// (sqrt(even² + odd²)) as gradient magnitude. Local energy provides a
/// combined measure of edges and lines.
/// @tparam QUAD Quadrature filter type (e.g., Quadrature, QuadratureS).
template <class QUAD>
class GradientEnergy : public GradientOdd<QUAD> {
  typedef GradientOdd<QUAD> GRAD;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;
  typedef typename QUAD::DirectionRange DirectionRange;

  GradientEnergy(img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(int_lower, int_upper) {}

  GradientEnergy(const ValueManager::NameValueVector& options,
                 img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  GradientEnergy(ValueManager::InitializerList options,
                 img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  /// @brief Get gradient magnitude from local energy.
  /// @return Energy image (sqrt(even² + odd²)).
  virtual cv::Mat magnitude() const { return this->quad_.energy(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible energy values.
  virtual MagnitudeRange magnitudeRange() const { return this->quad_.energyRange(); }

  /// @brief Convert normalized threshold to energy threshold.
  /// @param val Normalized threshold value in [0, 1].
  /// @return Absolute energy threshold value.
  virtual mag_type magnitudeThreshold(double val) const { return this->quad_.energyThreshold(val); }

  using GRAD::direction;
  using GRAD::directionals;
  using GRAD::directionRange;
  using GRAD::gradientRange;
  using GRAD::gx;
  using GRAD::gy;
  using GRAD::intensityRange;
  using GRAD::name;
  using GRAD::normType;
  using GRAD::process;
  using GRAD::results;
  using GRAD::value;
  using GRAD::valuePair;
  using GRAD::values;
};


/// @brief Adapter to use phase congruency from a quadrature filter as gradient.
///
/// Wraps a quadrature filter (QUAD) and exposes its phase congruency response
/// as gradient magnitude. Phase congruency provides illumination-invariant
/// feature detection by measuring the consistency of phase across scales.
/// @tparam QUAD Quadrature filter type with phase congruency support
///              (e.g., PhaseCongruency, PhaseCongruencyLaplace).
template <class QUAD>
class GradientPC : public GradientOdd<QUAD> {
  typedef GradientOdd<QUAD> GRAD;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;
  typedef typename QUAD::DirectionRange DirectionRange;

  GradientPC(img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(int_lower, int_upper) {}

  GradientPC(const ValueManager::NameValueVector& options,
             img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  GradientPC(ValueManager::InitializerList options,
             img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  /// @brief Get gradient magnitude from phase congruency.
  /// @return Phase congruency image in range [0, 1].
  virtual cv::Mat magnitude() const { return this->quad_.phaseCongruency(); }

  /// @brief Get magnitude value range.
  /// @return Range of phase congruency values (typically [0, 1]).
  virtual MagnitudeRange magnitudeRange() const { return this->quad_.phaseCongruencyRange(); }

  /// @brief Convert normalized threshold to phase congruency threshold.
  ///
  /// For phase congruency, the normalized value is used directly since
  /// phase congruency is already normalized to [0, 1].
  /// @param val Normalized threshold value in [0, 1].
  /// @return Threshold value (same as input for phase congruency).
  virtual mag_type magnitudeThreshold(double val) const { return static_cast<mag_type>(val); }

  using GRAD::direction;
  using GRAD::directionals;
  using GRAD::directionRange;
  using GRAD::gradientRange;
  using GRAD::gx;
  using GRAD::gy;
  using GRAD::intensityRange;
  using GRAD::name;
  using GRAD::normType;
  using GRAD::process;
  using GRAD::results;
  using GRAD::value;
  using GRAD::valuePair;
  using GRAD::values;
};

}  // namespace lsfm
