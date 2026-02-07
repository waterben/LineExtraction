//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file quadrature.hpp
/// @brief Abstract interface for quadrature filters (local energy computation).

#pragma once

#include <imgproc/gradient.hpp>
#include <imgproc/laplace.hpp>

namespace lsfm {

/// @brief Abstract interface for quadrature filters (local energy computation).
///
/// Quadrature filters use pairs of filters that are in quadrature relationship
/// (90 degrees phase difference). The even filter detects lines/edges and the
/// odd filter detects steps/gradients. Combined they compute local energy and phase.
/// @tparam IT Input image pixel type (uchar, short, float, double)
/// @tparam GT Gradient/even-odd response type (short, float, double)
/// @tparam MT Magnitude of odd operators (int, float, double)
/// @tparam ET Energy type (int, float, double)
/// @tparam DT Direction/phase type (float, double)
template <class IT, class GT, class MT, class ET, class DT>
class QuadratureI : public LaplaceI<IT, GT> {
  QuadratureI(const QuadratureI&);

 protected:
  QuadratureI() {}

 public:
  typedef IT img_type;     ///< Input image pixel type
  typedef GT grad_type;    ///< Gradient/response type
  typedef MT mag_type;     ///< Magnitude type
  typedef ET energy_type;  ///< Energy type
  typedef DT dir_type;     ///< Direction type
  typedef DT phase_type;   ///< Phase type

  typedef Range<IT> IntensityRange;  ///< Range for intensity values
  typedef Range<GT> GradientRange;   ///< Range for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range for magnitude values
  typedef Range<ET> EnergyRange;     ///< Range for energy values
  typedef Range<DT> DirectionRange;  ///< Range for direction values
  typedef Range<DT> PhaseRange;      ///< Range for phase values

  virtual ~QuadratureI() {}

  /// @brief Get even response (alias for laplace()).
  /// @return Even filter response image
  virtual cv::Mat laplace() const override { return even(); }

  /// @brief Get even response range.
  /// @return Range of even response values
  virtual GradientRange laplaceRange() const override { return evenRange(); }

  /// @brief Convert normalized threshold to even threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding even threshold value
  virtual GT laplaceThreshold(double val) const override { return evenThreshold(val); }

  /// @brief Get even filter response (symmetric/line-detecting).
  /// @return Even filter response image
  virtual cv::Mat even() const = 0;

  /// @brief Get even response value range.
  /// @return Range of even response values
  virtual GradientRange evenRange() const = 0;

  /// @brief Convert normalized threshold to even threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding even threshold value
  virtual GT evenThreshold(double val) const { return static_cast<GT>(evenRange().size() * val); }

  /// @brief Get odd filter response magnitude (antisymmetric/edge-detecting).
  /// @return Odd filter magnitude image
  virtual cv::Mat odd() const = 0;

  /// @brief Get odd response magnitude range (combined x and y).
  /// @return Range of odd magnitude values
  virtual MagnitudeRange oddRange() const = 0;

  /// @brief Convert normalized threshold to odd magnitude threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding odd magnitude threshold value
  virtual MT oddThreshold(double val) const { return static_cast<MT>(oddRange().upper * val); }

  /// @brief Get separate odd responses in x and y directions.
  /// @param[out] ox X-direction odd response
  /// @param[out] oy Y-direction odd response
  virtual void odd(cv::Mat& ox, cv::Mat& oy) const = 0;

  /// @brief Get x-direction odd filter response.
  /// @return X-direction odd response image
  virtual cv::Mat oddx() const {
    cv::Mat ox, oy;
    odd(ox, oy);
    return ox;
  }

  /// @brief Get y-direction odd filter response.
  /// @return Y-direction odd response image
  virtual cv::Mat oddy() const {
    cv::Mat ox, oy;
    odd(ox, oy);
    return oy;
  }

  /// @brief Get odd response range for single direction.
  /// @return Range of single-direction odd values
  virtual GradientRange oddGradRange() const {
    double val = oddRange().upper * 0.707106781;
    return GradientRange(static_cast<GT>(-val), static_cast<GT>(val));
  }

  /// @brief Get norm type used for odd magnitude computation.
  /// @return Norm type (L1, L2, or L2_squared)
  virtual NormType normType() const { return NormType::NORM_L2; }

  /// @brief Get direction of odd filter response.
  /// @return Direction image in radians or degrees
  virtual cv::Mat direction() const = 0;

  /// @brief Get direction value range.
  /// @return Direction range ([-PI,PI], [0,2PI] or [0,360])
  virtual DirectionRange directionRange() const = 0;

  /// @brief Get local energy image.
  ///
  /// Local energy = sqrt(even² + odd²)
  /// @return Energy image
  virtual cv::Mat energy() const = 0;

  /// @brief Get energy value range.
  /// @return Range of energy values
  virtual EnergyRange energyRange() const = 0;

  /// @brief Convert normalized threshold to energy threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding energy threshold value
  virtual ET energyThreshold(double val) const { return static_cast<ET>(energyRange().upper * val); }

  /// @brief Get local phase image.
  ///
  /// Phase = atan2(odd, even)
  /// @return Phase image in radians or degrees
  virtual cv::Mat phase() const = 0;

  /// @brief Get phase value range.
  /// @return Phase range ([-PI,PI], [0,2PI] or [0,360])
  virtual PhaseRange phaseRange() const = 0;
};

/// @brief Base implementation class for quadrature filters.
///
/// Provides common functionality including intensity range management.
/// @tparam IT Input image pixel type
/// @tparam GT Gradient/even-odd response type
/// @tparam MT Magnitude type
/// @tparam ET Energy type
/// @tparam DT Direction/phase type
template <class IT, class GT, class MT, class ET, class DT>
class Quadrature : public QuadratureI<IT, GT, MT, ET, DT> {
  Quadrature();
  Quadrature(const Quadrature&);

 protected:
  Range<IT> intRange_;  ///< Input intensity range

  /// @brief Construct with intensity range.
  /// @param int_lower Lower bound of input intensity
  /// @param int_upper Upper bound of input intensity
  Quadrature(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;     ///< Input image pixel type
  typedef GT grad_type;    ///< Gradient/response type
  typedef MT mag_type;     ///< Magnitude type
  typedef ET energy_type;  ///< Energy type
  typedef DT dir_type;     ///< Direction type
  typedef DT phase_type;   ///< Phase type

  typedef Range<IT> IntensityRange;  ///< Range for intensity values
  typedef Range<GT> GradientRange;   ///< Range for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range for magnitude values
  typedef Range<ET> EnergyRange;     ///< Range for energy values
  typedef Range<DT> DirectionRange;  ///< Range for direction values
  typedef Range<DT> PhaseRange;      ///< Range for phase values

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  IntensityRange intensityRange() const { return intRange_; }

  /// @brief Get all filter outputs as named results.
  /// @return Map containing all quadrature outputs with ranges
  virtual FilterResults results() const {
    FilterResults ret;
    ret["even"] = FilterData(this->even(), this->evenRange());
    ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
    ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
    ret["odd"] = FilterData(this->odd(), this->oddRange());
    ret["dir"] = FilterData(this->direction(), this->directionRange());
    ret["energy"] = FilterData(this->energy(), this->energyRange());
    ret["phase"] = FilterData(this->phase(), this->phaseRange());
    return ret;
  }
};

/// @brief Simple quadrature filter combining gradient and Laplacian operators.
///
/// Combines a gradient operator (odd filter) with a Laplacian operator (even filter)
/// to compute quadrature responses. Note that this is not a true quadrature pair
/// since the signals don't satisfy the Hilbert constraint, so phase may be invalid.
/// @tparam IT Input image pixel type
/// @tparam GT Gradient type
/// @tparam FT Floating point type for energy/phase
/// @tparam GRAD Gradient operator class
/// @tparam LAPLACE Laplacian operator class
/// @tparam P Polar coordinate helper (default: Polar)
template <class IT, class GT, class FT, class GRAD, class LAPLACE, template <typename, typename> class P = Polar>
class QuadratureSimple : public Quadrature<IT, GT, FT, FT, FT> {
  QuadratureSimple();
  QuadratureSimple(const QuadratureSimple&);

  GRAD grad_;        ///< Gradient operator (odd filter)
  LAPLACE laplace_;  ///< Laplacian operator (even filter)

  mutable bool phase_done_, energy_done_;  ///< Lazy computation flags

  mutable cv::Mat_<FT> energy_, phase_;  ///< Cached results

  Range<FT> energyRange_;  ///< Energy value range

  /// @brief Compute maximum possible energy response.
  void max_response() {
    GradientRange lr = laplace_.laplaceRange();
    GradientRange gr = grad_.gradientRange();
    FT lmax = std::max(lr.lower * lr.lower, lr.upper * lr.upper);
    energyRange_.lower = 0;
    energyRange_.upper = std::sqrt(lmax + static_cast<FT>(gr.upper) * gr.upper);
  }

  /// @brief Initialize filter and register options.
  inline void init() {
    phase_done_ = false;
    energy_done_ = false;
    this->addManager(grad_);
    this->addManager(laplace_);

    max_response();
  }

 public:
  typedef IT img_type;     ///< Input image pixel type
  typedef GT grad_type;    ///< Gradient/response type
  typedef FT mag_type;     ///< Magnitude type
  typedef FT energy_type;  ///< Energy type
  typedef FT dir_type;     ///< Direction type
  typedef FT phase_type;   ///< Phase type

  typedef Range<IT> IntensityRange;  ///< Range for intensity values
  typedef Range<GT> GradientRange;   ///< Range for gradient values
  typedef Range<FT> MagnitudeRange;  ///< Range for magnitude values
  typedef Range<FT> EnergyRange;     ///< Range for energy values
  typedef Range<FT> DirectionRange;  ///< Range for direction values
  typedef Range<FT> PhaseRange;      ///< Range for phase values

  /// @brief Construct with default intensity range.
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  QuadratureSimple(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
  }

  /// @brief Construct with options from name-value vector.
  /// @param options Configuration options
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  QuadratureSimple(const ValueManager::NameValueVector& options,
                   IT int_lower = std::numeric_limits<IT>::lowest(),
                   IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
    this->value(options);
  }

  /// @brief Construct with options from initializer list.
  /// @param options Configuration options as initializer list
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  QuadratureSimple(ValueManager::InitializerList options,
                   IT int_lower = std::numeric_limits<IT>::lowest(),
                   IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
    this->value(options);
  }


  using Quadrature<IT, GT, FT, FT, FT>::intensityRange;
  using Quadrature<IT, GT, FT, FT, FT>::results;

  /// @brief Process image through quadrature filter.
  /// @param img Input grayscale image
  void process(const cv::Mat& img) {
    energy_done_ = false;
    phase_done_ = false;
    grad_.process(img);
    laplace_.process(img);
  }

  /// @brief Get even filter response.
  /// @return Laplacian response
  cv::Mat even() const { return laplace_.laplace(); }

  /// @brief Get even response range.
  /// @return Laplacian value range
  GradientRange evenRange() const { return laplace_.laplaceRange(); }

  /// @brief Convert normalized threshold to even threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding even threshold value
  GT evenThreshold(double val) const { return laplace_.laplaceThreshold(val); }

  /// @brief Get odd filter response magnitude.
  /// @return Gradient magnitude
  cv::Mat odd() const { return grad_.magnitude(); }

  /// @brief Get odd response range.
  /// @return Gradient magnitude range
  MagnitudeRange oddRange() const { return grad_.magnitudeRange(); }

  /// @brief Convert normalized threshold to odd magnitude threshold.
  /// @param val Normalized threshold [0,1]
  /// @return Corresponding odd threshold value
  FT oddThreshold(double val) { return grad_.magnitudeThreshold(val); }

  /// @brief Get separate odd responses in x and y directions.
  /// @param[out] ox X-direction gradient
  /// @param[out] oy Y-direction gradient
  void odd(cv::Mat& ox, cv::Mat& oy) const { return grad_.directionals(ox, oy); }

  /// @brief Get x-direction odd response.
  /// @return X-direction gradient
  cv::Mat oddx() const { return grad_.gx(); }

  /// @brief Get y-direction odd response.
  /// @return Y-direction gradient
  cv::Mat oddy() const { return grad_.gy(); }

  /// @brief Get single-direction odd response range.
  /// @return Gradient range for one direction
  GradientRange oddGradRange() const { return grad_.gradientRange(); }

  /// @brief Get norm type used for odd magnitude.
  /// @return Norm type from gradient operator
  NormType normType() const { return grad_.normType(); }

  /// @brief Get gradient direction.
  /// @return Direction image
  cv::Mat direction() const { return grad_.direction(); }

  /// @brief Get direction value range.
  /// @return Direction range from gradient operator
  DirectionRange directionRange() const { return grad_.directionRange(); }

  /// @brief Get local energy image.
  ///
  /// Computed lazily as sqrt(even² + odd²).
  /// @return Energy image
  cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), even(), energy_);
      energy_done_ = true;
    }
    return energy_;
  }

  /// @brief Get energy value range.
  /// @return Energy range
  EnergyRange energyRange() const { return energyRange_; }

  /// @brief Get local phase image.
  ///
  /// Computed lazily as atan2(odd, even).
  /// @return Phase image
  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), even(), phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /// @brief Get phase value range.
  /// @return Phase range from polar helper
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  /// @brief Get filter name.
  /// @return Combined name "quad_[grad]_[laplace]"
  std::string name() const { return "quad_" + grad_.name() + "_" + laplace_.name(); }
};

}  // namespace lsfm
