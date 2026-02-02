
#pragma once

#include <opencv2/core/core.hpp>
#include <utility/range.hpp>


namespace lsfm {

/// @brief Conversion constant from degrees to radians.
constexpr double DEG2RAD = CV_PI / 180;

/// @brief Conversion constant from radians to degrees.
constexpr double RAD2DEG = 180 / CV_PI;

/// @brief Polar coordinate conversion utilities.
///
/// Provides static methods for converting between Cartesian (x, y) and
/// polar (magnitude, phase) representations. Uses std::atan2 for phase
/// computation with range [-PI, PI].
/// @tparam GT Gradient/Cartesian coordinate type
/// @tparam FT Floating point type for polar values
template <class GT, class FT>
struct Polar {
  typedef GT grad_type;   ///< Cartesian coordinate type
  typedef FT float_type;  ///< Floating point type

  /// @brief Get phase value range.
  /// @return Range [-PI, PI]
  inline static const Range<FT> range() { return Range<FT>(static_cast<FT>(-CV_PI), static_cast<FT>(CV_PI)); }

  /// @brief Compute phase from x and y components.
  /// @param dx X-component (gradient dx or real part)
  /// @param dy Y-component (gradient dy or imaginary part)
  /// @param[out] phase Output phase image in radians
  static inline void phase(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& phase) {
    phase.create(dx.rows, dy.cols, cv::DataType<FT>::type);
    FT* pphase = phase.ptr<FT>();

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        pphase[j] = std::atan2(static_cast<FT>(pdy[j]), static_cast<FT>(pdx[j]));
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pphase += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pphase[j] = std::atan2(static_cast<FT>(pdy[j]), static_cast<FT>(pdx[j]));
        }
      }
    }
  }

  /// @brief Compute magnitude from x and y components.
  /// @param dx X-component
  /// @param dy Y-component
  /// @param[out] mag Output magnitude image (L2 norm)
  static inline void magnitude(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag) {
    mag.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    FT* pmag = mag.ptr<FT>();
    FT vdx, vdy;

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        vdx = static_cast<FT>(pdx[j]);
        vdy = static_cast<FT>(pdy[j]);
        pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pmag += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          vdx = static_cast<FT>(pdx[j]);
          vdy = static_cast<FT>(pdy[j]);
          pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
        }
      }
    }
  }

  /// @brief Convert Cartesian to polar coordinates.
  /// @param dx X-component input
  /// @param dy Y-component input
  /// @param[out] mag Output magnitude
  /// @param[out] phase Output phase in radians
  static inline void cart2Polar(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag, cv::Mat& phase) {
    mag.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    phase.create(dx.rows, dx.cols, cv::DataType<FT>::type);
    FT* pmag = mag.ptr<FT>();
    FT* pphase = phase.ptr<FT>();
    FT vdx, vdy;

    if (dx.isContinuous() && dy.isContinuous()) {
      const GT* pdx = dx.ptr<GT>();
      const GT* pdy = dy.ptr<GT>();
      int size = dx.rows * dx.cols;
      for (int j = 0; j != size; ++j) {
        vdx = static_cast<FT>(pdx[j]);
        vdy = static_cast<FT>(pdy[j]);
        pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
        pphase[j] = std::atan2(vdy, vdx);
      }
    } else {
      for (int i = 0; i != dx.rows; ++i, pmag += dx.cols, pphase += dx.cols) {
        const GT* pdx = dx.ptr<GT>(i);
        const GT* pdy = dy.ptr<GT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          vdx = static_cast<FT>(pdx[j]);
          vdy = static_cast<FT>(pdy[j]);
          pmag[j] = std::sqrt(vdx * vdx + vdy * vdy);
          pphase[j] = std::atan2(vdy, vdx);
        }
      }
    }
  }

  /// @brief Convert polar to Cartesian coordinates.
  /// @param mag Input magnitude
  /// @param phase Input phase in radians
  /// @param[out] dx Output x-component
  /// @param[out] dy Output y-component
  static inline void polar2Cart(const cv::Mat& mag, const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    dx.create(mag.rows, mag.cols, cv::DataType<GT>::type);
    dy.create(mag.rows, mag.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (mag.isContinuous() && phase.isContinuous()) {
      const FT* pmag = mag.ptr<FT>();
      const FT* pphase = phase.ptr<FT>();
      int size = mag.rows * mag.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(pmag[j] * std::cos(pphase[j]));
        pdy[j] = static_cast<GT>(pmag[j] * std::sin(pphase[j]));
      }
    } else {
      for (int i = 0; i != mag.rows; ++i, pdx += mag.cols, pdy += mag.cols) {
        const FT* pmag = mag.ptr<FT>(i);
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(pmag[j] * std::cos(pphase[j]));
          pdy[j] = static_cast<GT>(pmag[j] * std::sin(pphase[j]));
        }
      }
    }
  }

  /// @brief Convert phase angle to unit Cartesian components.
  ///
  /// Generates unit vectors (dx, dy) from phase angles alone, effectively
  /// treating magnitude as 1.0 for all pixels.
  /// @param[in] phase Input phase angle image in radians.
  /// @param[out] dx Output X-component (cos(phase)).
  /// @param[out] dy Output Y-component (sin(phase)).
  static inline void polar2Cart(const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    dx.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    dy.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (phase.isContinuous()) {
      const FT* pphase = phase.ptr<FT>();
      int size = phase.rows * phase.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(std::cos(pphase[j]));
        pdy[j] = static_cast<GT>(std::sin(pphase[j]));
      }
    } else {
      for (int i = 0; i != phase.rows; ++i, pdx += phase.cols, pdy += phase.cols) {
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(std::cos(pphase[j]));
          pdy[j] = static_cast<GT>(std::sin(pphase[j]));
        }
      }
    }
  }

  /// @brief Convert phase angle to scaled Cartesian components.
  ///
  /// Generates Cartesian vectors from phase angles with a uniform scale factor,
  /// resulting in dx = scale * cos(phase) and dy = scale * sin(phase).
  /// @param[in] phase Input phase angle image in radians.
  /// @param[in] scale Uniform scale factor applied to all output vectors.
  /// @param[out] dx Output X-component (scale * cos(phase)).
  /// @param[out] dy Output Y-component (scale * sin(phase)).
  static inline void polar2Cart(const cv::Mat& phase, FT scale, cv::Mat& dx, cv::Mat& dy) {
    dx.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    dy.create(phase.rows, phase.cols, cv::DataType<GT>::type);
    GT* pdx = dx.ptr<GT>();
    GT* pdy = dy.ptr<GT>();

    if (phase.isContinuous()) {
      const FT* pphase = phase.ptr<FT>();
      int size = phase.rows * phase.cols;
      for (int j = 0; j != size; ++j) {
        pdx[j] = static_cast<GT>(std::cos(pphase[j]) * scale);
        pdy[j] = static_cast<GT>(std::sin(pphase[j]) * scale);
      }
    } else {
      for (int i = 0; i != phase.rows; ++i, pdx += phase.cols, pdy += phase.cols) {
        const FT* pphase = phase.ptr<FT>(i);
        for (int j = 0; j != dx.cols; ++j) {
          pdx[j] = static_cast<GT>(std::cos(pphase[j]) * scale);
          pdy[j] = static_cast<GT>(std::sin(pphase[j]) * scale);
        }
      }
    }
  }
};

/// @brief Polar coordinate converter using OpenCV built-in functions.
///
/// This struct provides the same interface as Polar but delegates to OpenCV's
/// optimized `cv::phase()`, `cv::magnitude()`, `cv::cartToPolar()`, and
/// `cv::polarToCart()` functions. Suitable when OpenCV's implementations are
/// preferred or when consistency with other OpenCV operations is desired.
/// @note OpenCV's phase function returns angles in range [0, 2π).
/// @tparam GT Gradient component type (e.g., short, float).
/// @tparam FT Floating-point type for phase and magnitude (e.g., float, double).
template <class GT, class FT>
struct PolarCV {
  typedef GT grad_type;   ///< Type for gradient components.
  typedef FT float_type;  ///< Type for floating-point results.

  /// @brief Get the valid phase angle range.
  /// @return Range [0, 2π) for OpenCV phase convention.
  inline static const Range<FT> range() { return Range<FT>(0, static_cast<FT>(2 * CV_PI)); }

  /// @brief Compute phase angle from gradient components using OpenCV.
  ///
  /// Uses `cv::phase()` to compute atan2(dy, dx) for each pixel.
  /// @param[in] dx Gradient X-component image.
  /// @param[in] dy Gradient Y-component image.
  /// @param[out] phase Output phase angle image in radians [0, 2π).
  static inline void phase(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& phase) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::phase(tdx, tdy, phase);
  }

  /// @brief Compute gradient magnitude using OpenCV.
  ///
  /// Uses `cv::magnitude()` to compute sqrt(dx² + dy²) for each pixel.
  /// @param[in] dx Gradient X-component image.
  /// @param[in] dy Gradient Y-component image.
  /// @param[out] mag Output magnitude image.
  static inline void magnitude(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::magnitude(tdx, tdy, mag);
  }

  /// @brief Convert Cartesian gradients to polar coordinates using OpenCV.
  ///
  /// Uses `cv::cartToPolar()` to compute both magnitude and phase simultaneously,
  /// which may be more efficient than computing them separately.
  /// @param[in] dx Gradient X-component image.
  /// @param[in] dy Gradient Y-component image.
  /// @param[out] mag Output magnitude image.
  /// @param[out] phase Output phase angle image in radians [0, 2π).
  static inline void cart2Polar(const cv::Mat& dx, const cv::Mat& dy, cv::Mat& mag, cv::Mat& phase) {
    cv::Mat tdx, tdy;
    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      dx.convertTo(tdx, cv::DataType<FT>::type);
      dy.convertTo(tdy, cv::DataType<FT>::type);
    } else {
      tdx = dx;
      tdy = dy;
    }

    cv::cartToPolar(tdx, tdy, mag, phase);
  }

  /// @brief Convert polar coordinates to Cartesian gradients using OpenCV.
  ///
  /// Uses `cv::polarToCart()` to compute dx = mag * cos(phase) and
  /// dy = mag * sin(phase) for each pixel.
  /// @param[in] mag Input magnitude image.
  /// @param[in] phase Input phase angle image in radians.
  /// @param[out] dx Output X-component image.
  /// @param[out] dy Output Y-component image.
  static inline void polar2Cart(const cv::Mat& mag, const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(mag, phase, tdx, tdy);

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  /// @brief Convert phase angle to unit Cartesian components using OpenCV.
  ///
  /// Uses `cv::polarToCart()` with empty magnitude to generate unit vectors.
  /// @param[in] phase Input phase angle image in radians.
  /// @param[out] dx Output X-component (cos(phase)).
  /// @param[out] dy Output Y-component (sin(phase)).
  static inline void polar2Cart(const cv::Mat& phase, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(cv::Mat(), phase, tdx, tdy);

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  /// @brief Convert phase angle to scaled Cartesian components using OpenCV.
  ///
  /// Uses `cv::polarToCart()` and then scales the output vectors uniformly.
  /// @param[in] phase Input phase angle image in radians.
  /// @param[in] scale Uniform scale factor applied to all output vectors.
  /// @param[out] dx Output X-component (scale * cos(phase)).
  /// @param[out] dy Output Y-component (scale * sin(phase)).
  static inline void polar2Cart(const cv::Mat& phase, FT scale, cv::Mat& dx, cv::Mat& dy) {
    cv::Mat tdx, tdy;
    cv::polarToCart(cv::Mat(), phase, tdx, tdy);
    tdx *= scale;
    tdy *= scale;

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      tdx.convertTo(dx, cv::DataType<GT>::type);
      tdy.convertTo(dy, cv::DataType<GT>::type);
    } else {
      dx = tdx;
      dy = tdy;
    }
  }

  /// @brief Wrap phase angles from [0, 2π) to (-π, π] range.
  ///
  /// Converts angles greater than π to their equivalent negative angles,
  /// which is useful for algorithms that expect signed angle representation.
  /// @param[in] angle Input angle image in range [0, 2π).
  /// @param[out] output Output angle image in range (-π, π].
  static inline void wrap(const cv::Mat& angle, cv::Mat& output) {
    output = angle.clone();
    cv::Mat tmp = (static_cast<FT>(-CV_PI) - (static_cast<FT>(CV_PI) - angle));
    tmp.copyTo(output, (angle > static_cast<FT>(CV_PI)));
  }
};


}  // namespace lsfm
